#!/usr/bin/python
import rospkg
import roslib
import sys
import re
import os
import string

CATEGORIES = ['sub', 'pub', 'srv', 'srv_called', 'param', 'param_set', 
        'req_tf', 'prov_tf', 'goal', 'feedback', 'result', 'act_called']

CAT_5C = ['comm', "conf", "coord", "comput", "compo"] # Red, Blue, Green, Grey, Yellow

DEFAULT_ORDER = ['name', 'type', 'from', 'to', 'desc', 'default']

GENERIC = '<([^>]*)>'   # <something>
QSTRING = '"([^"]*)"'
PSTRING = '["\']([^"\']*)["\']'   # not quite right, will break on "can't"
NEXT_PARAM =  ',\s*([^\,]+)' # , something, 
FINAL_PARAM = ',\s*([^\)]+)\)' # , something)


PATTERNS = [
#[CATEGORY,     PATTERN,                                            FIELD_LIST], 
# Add empty patterns () for blank fields

# C++
 ['name',       'ros::init\([^\)]*' + QSTRING + '\)',               ['name']],  
 ['spin',       'ros::spin\(\)',                                    []],
 ['publ',       'publish\(\s*',                                    []],  
 ['sub',        'subscribe' + GENERIC + '\(' + QSTRING,             ["type", "name"]], 
 ['sub',        'subscribe\(' + QSTRING + '\s*',                            ["name"]], 
 ['pub',        'advertise' + GENERIC + '\s*\(' + QSTRING,             ['type', 'name']],
 ['param',      'param()\(' + QSTRING + ', [^,]+, ([^\)]+)\)',      ['type', 'name', 'default']], 
 ['param',      'param' + GENERIC + '\(' + QSTRING + ', [^,]+' + FINAL_PARAM, ['type', 'name', 'default']],
 ['param',      'getParam\(' + QSTRING,                             ['name']], 
 ['param',      'param::get\(' + QSTRING,                           ['name']], 
 ['param_set',  'setParam\(' + QSTRING,                             ['name']], 
 ['param_set',  'param::set\(' + QSTRING,                           ['name']], 
 ['srv',        'advertiseService\(' + QSTRING,                     ['name']], #C++ service *SHOULD SET TYPE
 ['srv_called', 'advertiseService' + GENERIC + '\(' + QSTRING,      ['type', 'name']],
 ['nodehandle',       'ros::NodeHandle \s*',                        []],
 ['subscriber_def',       'ros::Subscriber \s*',                        []],
 ['publisher_def',       'ros::Publisher \s*',                        []],


# Python
 ['name',       'rospy.init_node\(' + PSTRING,                      ['name']], 
 ['sub',        'rospy.Subscriber\('+ PSTRING + NEXT_PARAM + ',',   ['name', 'type']], 
 ['pub',        'rospy.Publisher\(' + PSTRING + FINAL_PARAM,        ['name', 'type']], 
 ['param',      'rospy.get_param\(' + PSTRING + '\)',               ['name']],  
 ['param',      'rospy.get_param\(' + PSTRING + FINAL_PARAM,        ['name', 'default']], 
 ['param_set',  'rospy.set_param\(' + PSTRING,                      ['name']], 
 ['srv',        'rospy.Service\(' + PSTRING + NEXT_PARAM + ',',     ['name', 'type']],
 ['srv_called', 'rospy.ServiceProxy\(' + PSTRING + FINAL_PARAM,     ['name', 'type']],

# Python or C++
 ['req_tf',     'lookupTransform\(' + PSTRING + ', ' + PSTRING,     ['from', 'to']],
 ['prov_tf',    'sendTransform\(.*' + NEXT_PARAM + FINAL_PARAM,     ['from', 'to']], # DOES NOT WORK
]



class RosWiki:
    def __init__(self):
        self.fields = {}

    def parse_line_by_line(self, filename):
        f = open(filename, 'r')
        lines = f.readlines()
        f.close()
        mmap = {}
        line_number = 0
        parsed_lines = []
        for line in lines:
            found = 0
            for (cat, pattern, fields) in PATTERNS:
                for match in re.findall(pattern, line.strip()):
                    print "###", line_number, match, fields, cat
                    found = cat
                    mmap = {}
                    if len(fields) > 1:
                        for key, value in zip(fields, match):
                            mmap[key] = self.clean(value)
                            #print "key, value: ", key, value
                            #print "###########" 
            parsed_lines.append([line_number, found, line.rstrip(), mmap])
            line_number+=1
        return parsed_lines

    def parse(self, filename):
        f = open(filename, 'r')
        contents = f.read()

        for (cat, pattern, fields) in PATTERNS:
            for match in re.findall(pattern, contents):
                mmap = {}
                if len(fields) > 1:
                    for key, value in zip(fields, match):
                        mmap[key] = self.clean(value)
                else:
                    mmap[ fields[0] ] = match
                self.add(cat, mmap)
        

    def add(self, category, mmap):
        if category in self.fields:
            self.fields[category].append(mmap)
        else:
            self.fields[category] = [mmap]

    def clean(self, s):
        needles = [['::', '/'], ['\n', ' ']]
        for needle, replacement in needles:
            s = string.replace(s, needle, replacement)
        return s

    def get_value(self, field):
        if field in self.fields:
            first = self.fields[field][0]
            return first.values()[0]
        return ''

    def model_print(self, code):
        f = open('model/generated.ros_package', 'w')
        #Create HEADER
        f.write( "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n")
        f.write( "<ros:Package xmi:version=\"2.0\" xmlns:xmi=\"http://www.omg.org/XMI\" xmlns:ros=\"http://ros/1.0\" name=\"parsing_of_manifest_missing\" author=\"Alexander Bubeck\" description=\"missing\">\n")
        #Create Node
        for line in code:
            if(line[1] == "name"):
                f.write( "\tTUT: "+ line[2]+ str(line[3])+"\n")
        #Create parameters
        for line in code:
            if(line[1] == "param"):
                f.write( "\t\t<parameter name=\"" + line[3]['name']+ "\" value=" + line[3]['default'] + " type=\"" + line[3]['type']+ "/>\n")
        #Create publishers
        for line in code:
            if(line[1] == "pub"):
                f.write( "\t\t<publisher name=\"" + line[3]['name'] + "\" eventHandler=\"\" msg=\"" + line[3]['type'] + "\"/>\n")
        #Create subscribers
        for line in code:
            if(line[1] == "sub"):
                f.write( "sub " + str(line[3])+"\n")
        #       f.write( "<subscriber name=\"" + line[3]['name'] + "\" msg=\"" + line[3]['type'] + "\"/>")
        f.write( "\t</node>\n")
        f.write( "</ros:Package>\n")
        f.close()

    def pretty_print(self, code):
        f = open('code_colored.html', 'w')
        print "<html>"
        print "<body>"
        print """    <table summary=\"code examples\"> 
        <tr>
        <td><pre style="border: 1px solid #888;padding: 2px">"""

        for line in code:
            if(line[1] != 0):
                if(line[1] == "param"):
                    print "<font color=\"#00FF00\">" , line[2], "</font>" , line[3]
                else:
                    print "<font color=\"#FF0000\">" , line[2], "</font>" , line[3]
            else:
                print line[2]

        print "</pre>"
        print "</body>"
        print "</html>"

    #Erstelle Grafisches Model des XMI
    def create_model_graph(self, code):
        pass

    #Berechne Metrik fuer Separierung der 5C
    def analyse_separation_metric(self, code):
        for line in code:
            pass

    #Zaehle lines of code fuer die verschiedenen Aspekte um Nachzuweisen, wie viel Code man schreiben muesste ohne MDE
    def analyse_code_ratios(self, code):
        pass

    #Triggere model comparison um aehnliche Modelle in der Modelldatenbank zu finden 
    #(TODO: definiere aehnlichkeit im ROS Sinne)
    #(TODO: checke ob das ganze zeitmaessig ueberhaupt geht, skaliert mit der Anzahl der Modelle in der Datenbank, vielleicht Kategoriebasierend)
    def analyse_code_similarity(self, code):
        pass

        
if __name__ == '__main__':
    wiki = RosWiki()
    for arg in sys.argv[1:]:
        wiki.pretty_print(wiki.parse_line_by_line(arg))

    #print wiki.code()



