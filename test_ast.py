import ast
import collections

BaseFunc = collections.namedtuple('Function', ['name', 'lineno', 'col_offset',
                                               'is_method', 'classname',
                                               'clojures', 'complexity'])
BaseClass = collections.namedtuple('Class', ['name', 'lineno', 'col_offset',
                                             'methods', 'real_complexity'])

Node = {}
Node['Name'] = ""
Node['Publishers'] = []
Node['ServiceServers'] = []
Node['Subscribers'] = []
Node['ServiceClients'] = []


class Function(BaseFunc):
    '''Base object represeting a function.
    '''

    @property
    def letter(self):
        '''The letter representing the function. It is `M` if the function is
        actually a method, `F` otherwise.
        '''
        return 'M' if self.is_method else 'F'

    @property
    def fullname(self):
        '''The full name of the function. If it is a method, then the full name
        is:
                {class name}.{method name}
        Otherwise it is just the function name.
        '''
        if self.classname is None:
            return self.name
        return '{0}.{1}'.format(self.classname, self.name)

    def __str__(self):
        return '{0} {1}:{2} {3} - {4}'.format(self.letter, self.lineno,
                                              self.col_offset, self.fullname,
                                              self.complexity)


class Class(BaseClass):

    letter = 'C'

    @property
    def fullname(self):
        '''The full name of the class. It is just its name. This attribute
        exists for consistency (see :data:`Function.fullname`).
        '''
        return self.name

    @property
    def complexity(self):
        '''The average complexity of the class. It corresponds to the average
        complexity of its methods plus one.
        '''
        if not self.methods:
            return self.real_complexity
        return int(self.real_complexity / float(len(self.methods))) + 1

    def __str__(self):
        return '{0} {1}:{2} {3} - {4}'.format(self.letter, self.lineno,
                                              self.col_offset, self.name,
                                              self.complexity)


#Taken from radon project
class CodeVisitor(ast.NodeVisitor):
    '''Base class for every NodeVisitors in `radon.visitors`. It implements a
    couple utility class methods and a static method.
    '''

    @staticmethod
    def get_name(obj):
        '''Shorthand for ``obj.__class__.__name__``.'''
        return obj.__class__.__name__

    @classmethod
    def from_code(cls, code, **kwargs):
        '''Instanciate the class from source code (string object). The
        `**kwargs` are directly passed to the `ast.NodeVisitor` constructor.
        '''
        return cls.from_ast(ast.parse(code), **kwargs)

    @classmethod
    def from_ast(cls, ast_node, **kwargs):
        '''Instanciate the class from an AST node. The `**kwargs` are
        directly passed to the `ast.NodeVisitor` constructor.
        '''
        visitor = cls(**kwargs)
        visitor.visit(ast_node)
        return visitor


#Taken from radon project
#TODO: has to be rewritten to ROSVisitor
class ComplexityVisitor(CodeVisitor):
    '''A visitor that keeps track of the cyclomatic complexity of
    the elements.

    :param to_method: If True, every function is treated as a method. In this
        case the *classname* parameter is used as class name.
    :param classname: Name of parent class.
    :param off: If True, the starting value for the complexity is set to 1,
        otherwise to 0.
    '''

    def __init__(self, to_method=False, classname=None, off=True):
        self.off = off
        self.complexity = 1 if off else 0
        self.functions = []
        self.classes = []
        self.to_method = to_method
        self.classname = classname

        self.node = []

    @property
    def functions_complexity(self):
        '''The total complexity from all functions (i.e. the total number of
        decision points + 1).
        '''
        return sum(map(GET_COMPLEXITY, self.functions)) - len(self.functions)

    @property
    def classes_complexity(self):
        '''The total complexity from all classes (i.e. the total number of
        decision points + 1).
        '''
        return sum(map(GET_REAL_COMPLEXITY, self.classes)) - len(self.classes)

    @property
    def total_complexity(self):
        '''The total complexity. Computed adding up the class complexity, the
        functions complexity, and the classes complexity.
        '''
        return (self.complexity + self.functions_complexity +
                self.classes_complexity + (not self.off))

    @property
    def nodes(self):
        return self.node

    @property
    def blocks(self):
        '''All the blocks visited. These include: all the functions, the
        classes and their methods. The returned list is not sorted.
        '''
        blocks = self.functions
        for cls in self.classes:
            blocks.append(cls)
            blocks.extend(cls.methods)
        return blocks

    def generic_visit(self, node):
        '''Main entry point for the visitor.'''
        # Get the name of the class
        name = self.get_name(node)
        #print "generic_visit: ", name
        # The Try/Except block is counted as the number of handlers
        # plus the `else` block.
        # In Python 3.3 the TryExcept and TryFinally nodes have been merged
        # into a single node: Try
        if name in ('Try', 'TryExcept'):
            self.complexity += len(node.handlers) + len(node.orelse)
        elif name == 'BoolOp':
            self.complexity += len(node.values) - 1
        # Lambda functions, ifs, with and assert statements count all as 1.
        elif name in ('Lambda', 'With', 'If', 'IfExp', 'Assert'):
            self.complexity += 1
        # The For and While blocks count as 1 plus the `else` block.
        elif name in ('For', 'While'):
            self.complexity += len(node.orelse) + 1
        # List, set, dict comprehensions and generator exps count as 1 plus
        # the `if` statement.
        elif name == 'comprehension':
            self.complexity += len(node.ifs) + 1

        super(ComplexityVisitor, self).generic_visit(node)

    def visit_FunctionDef(self, node):
        # The complexity of a function is computed taking into account
        # the following factors: number of decorators, the complexity
        # the function's body and the number of clojures (which count
        # double).
        clojures = []
        body_complexity = 1
        for child in node.body:
            visitor = ComplexityVisitor(off=False)
            visitor.visit(child)
            clojures.extend(visitor.functions)
            # Add general complexity and clojures' complexity
           	# body_complexity += (visitor.complexity +
            #                    visitor.functions_complexity)

        #print "Function: ", node.name, " with args ", node.args
        func = Function(node.name, node.lineno, node.col_offset,
                        self.to_method, self.classname, clojures,
                        body_complexity)
        self.functions.append(func)

    def parse_Service(self, node):
        print "ServiceServer: Name:", node.args[0].s, " Type: ", node.args[1].id
        Node['ServiceServers'].append({'Name': str(node.args[0].s), 'Type': str(node.args[1].id) })

    def parse_ServiceProxy(self, node):
        print "ServiceClient: Name:", node.args[0].s, " Type: ", node.args[1].id
        Node['ServiceClients'].append({'Name': str(node.args[0].s), 'Type': str(node.args[1].id) })

    def parse_Subscriber(self, node):
        print "Subscriber: Name:", node.args[0].s, " Type: ", node.args[1].id
        Node['Subscribers'].append({'Name': str(node.args[0].s), 'Type': str(node.args[1].id) })

    def parse_Publisher(self, node):
        print "Publisher: Name:", node.args[0].s, " Type: ", node.args[1].id
        Node['Publishers'].append({'Name': str(node.args[0].s), 'Type': str(node.args[1].id) })

    def parse_init_node(self, node):
        print "Node: ", node.args[0].s
        #n = {'Node': str(node.args[0].s)} 
        Node['Name'] = str(node.args[0].s)

    def parse_default(self, node):
        pass

    def visit_Call(self, node):
        if (str(type(node.func))=="<class '_ast.Attribute'>"):
            #print "Call : ", ast.dump(node.func)
            if (str(type(node.func.value))=="<class '_ast.Name'>"):
                if(node.func.value.id == "rospy"):
                    print "ROSCALL at ", node.lineno, " Type: ", node.func.attr
                    getattr(self, "parse_"+str(node.func.attr), self.parse_default)(node)
                    print "===================================="
        ast.NodeVisitor.generic_visit(self, node)

    def visit_ClassDef(self, node):
        # The complexity of a class is computed taking into account
        # the following factors: number of decorators and the complexity
        # of the class' body (which is the sum of all the complexities).
        methods = []
        # According to Cyclomatic Complexity definition it has to start off
        # from 1.
        body_complexity = 1
        classname = node.name
        for child in node.body:
            visitor = ComplexityVisitor(True, classname, off=False)
            visitor.visit(child)
            methods.extend(visitor.functions)
            #body_complexity += (visitor.complexity +
            #                    visitor.functions_complexity)

        cls = Class(classname, node.lineno, node.col_offset,
                    methods, body_complexity)
        self.classes.append(cls)

def toXMI(node):
    f = open('generated.ros_package', 'w')
    #Create HEADER
    f.write( "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n")
    f.write( "<ros:Package xmi:version=\"2.0\" xmlns:xmi=\"http://www.omg.org/XMI\" xmlns:ros=\"http://ros/1.0\" name=\"" + node['Name'] + "\" author=\"Alexander Bubeck\" description=\"missing\">\n")
    #Create Node
    f.write( "\t<node name=\"" + node['Name']+ "\">\n")
    #Create parameters
    #for line in code:
    #    if(line[1] == "param_val"):
    #        f.write( "\t\t<parameter name=\"" + line[3]['name']+ "\" value=" + line[3]['default'] + " type=\"" + line[3]['type']+ "/>\n")
    #Create publishers
    for pub in node["Publishers"]:
        f.write( "\t\t<publisher name=\"" + pub['Name'] + "\" eventHandler=\"\" msg=\"" + pub['Type'] + "\"/>\n")
    #Create subscribers
    for sub in node["Subscribers"]:
        f.write( "\t\t<subscriber name=\"" + sub['Name'] + "\" msg=\"" + sub['Type'] + "\"/>\n")
    #Create serviceclients
    for client in node["ServiceClients"]:
        f.write( "\t\t<serviceClient name=\"" + client['Name'] + "\" msg=\"" + client['Type'] + "\"/>\n")
    #Create serviceservers
    for server in node["ServiceServers"]:
        f.write( "\t\t<serviceServer name=\"" + server['Name'] + "\" msg=\"" + server['Type'] + "\"/>\n")
    f.write( "\t</node>\n")
    f.write( "</ros:Package>\n")
    f.close()


if __name__ == "__main__":
    #filename = "test/pose_transformer.py"
    #filename = "test/cob_hwboard.py"
    filename = "test/tactile_filters.py"
    fobj = open(filename)
    code = fobj.read()
    parsed_ast = ast.parse(code)
    #print ast.dump(parsed_ast)
    cv = ComplexityVisitor()
    cv.from_ast(parsed_ast)
    print Node
    toXMI(Node)


