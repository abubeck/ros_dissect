#!/usr/bin/python
import sys
import os
import string
from ros_parser import Parser




class RosWiki:
    def __init__(self):
        self.fields = {}        

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

    # Creates XMI File based on parser output
    def model_print(self, code):
        f = open('generated.ros_package', 'w')
        #Create HEADER
        f.write( "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n")
        f.write( "<ros:Package xmi:version=\"2.0\" xmlns:xmi=\"http://www.omg.org/XMI\" xmlns:ros=\"http://ros/1.0\" name=\"parsing_of_manifest_missing\" author=\"Alexander Bubeck\" description=\"missing\">\n")
        #Create Node
        for line in code:
            if(line[1] == "name"):
                f.write( "\tTUT: "+ line[2]+ str(line[3])+"\n")
        #Create parameters
        for line in code:
            if(line[1] == "param_val"):
                f.write( "\t\t<parameter name=\"" + line[3]['name']+ "\" value=" + line[3]['default'] + " type=\"" + line[3]['type']+ "/>\n")
        #Create publishers
        for line in code:
            if(line[1] == "pub_val"):
                f.write( "\t\t<publisher name=\"" + line[3]['name'] + "\" eventHandler=\"\" msg=\"" + line[3]['type'] + "\"/>\n")
        #Create subscribers
        for line in code:
            if(line[1] == "sub"):
                f.write( "sub " + str(line[3])+"\n")
        #       f.write( "<subscriber name=\"" + line[3]['name'] + "\" msg=\"" + line[3]['type'] + "\"/>")
        f.write( "\t</node>\n")
        f.write( "</ros:Package>\n")
        f.close()

    # Creates a colorized HTML page based on parser output
    def pretty_print(self, code):
        f = open('code_colored.html', 'w')
        f.write( "<html>\n")
        f.write( "<body>\n")
        f.write( """    <table summary=\"code examples\">\n 
        <tr>\n
        <td><pre style="border: 1px solid #888;padding: 2px">\n""")
        line_counter = 0
        for line in code:
            line_counter += 1
            if(line[1] != 0):
                if(line[1] == "param" or line[1] == "param_val"):
                    f.write(str(line_counter)+":"+ "<font color=\"#00FF00\">" + str(line[2])+ "</font>" + str(line[3]) + "\n")
                else:
                    f.write(str(line_counter)+":"+ "<font color=\"#FF0000\">" + str(line[2])+ "</font>" + str(line[3]) + "\n")
            else:
                f.write(str(line_counter)+":"+ line[2] +"\n")

        f.write( "</pre>\n" )
        f.write( "</body>\n")
        f.write( "</html>\n")
        f.close()

    #Erstelle Grafisches Model des XMI
    def create_model_graph(self, code):
        pass

    #Berechne Metrik fuer Separierung der 5C
    def analyse_separation_metric(self, code, num_of_param, num_of_comm, num_of_user):       
        print "Expecting", num_of_param, "lines of parameters,", num_of_comm, "lines of communication and", num_of_user, "lines of user code"
        expecting = ["param", "user", "comm"] #First we expect any concern
        counters = {"param": num_of_param, "comm": num_of_comm, "user": num_of_user}
        penalty = 0
        line_counter = 0
        for line in code:
            line_counter += 1
            if(line[1] == "param" or line[1] == "param_val"):
                line_is = "param"
            elif(line[1] == 0):
                line_is = "user"
            else:
                line_is = "comm"
            counters[line_is] -= 1
            if(line_is in expecting and counters[line_is] > 0): #We look if the current line is what we expect
                expecting = [line_is] #Next line we expect the same type of concern
            else:
                if(counters[line_is] > 0): #we count that as penalty
                    print "Penalty at line", line_counter
                    penalty += 1
                expecting = []
                for key, value in counters.iteritems():
                    if(value != 0):
                        expecting.append(key) # we expect everything we have left
        print "Overall penalty: ", penalty
        return float(penalty)/len(code)


    #Zaehle lines of code fuer die verschiedenen Aspekte um Nachzuweisen, wie viel Code man schreiben muesste ohne MDE
    def analyse_code_ratios(self, code):
        pass

    #Triggere model comparison um aehnliche Modelle in der Modelldatenbank zu finden 
    #(TODO: definiere aehnlichkeit im ROS Sinne)
    #(TODO: checke ob das ganze zeitmaessig ueberhaupt geht, skaliert mit der Anzahl der Modelle in der Datenbank, vielleicht Kategoriebasierend)
    def analyse_code_similarity(self, code):
        pass

    def code_to_ros_ratio(self, code):
        param_counter = 0
        comm_counter = 0
        user_counter = 0
        for line in code:
            if(line[1] == "param" or line[1] == "param_val"):
                param_counter +=1
            elif(line[1] == 0):
                user_counter +=1
            else:
                comm_counter +=1  
        print "Found", param_counter+comm_counter , "lines of ROS Code and", user_counter, "lines of common code in total" , len(code), " lines of code" 
        return [len(code), param_counter, comm_counter]

if __name__ == '__main__':
    if(len(sys.argv) == 2):
        p = Parser(sys.argv[1])
    w = RosWiki()
    w.pretty_print(p.parsed_lines)
    [lloc, num_of_param, num_of_comm] = w.code_to_ros_ratio(p.parsed_lines)
    w.model_print(p.parsed_lines)
    print "Sep: ", w.analyse_separation_metric(p.parsed_lines, num_of_param, num_of_comm, lloc-num_of_param-num_of_comm)

