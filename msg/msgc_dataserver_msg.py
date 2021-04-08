#!/usr/bin/evn python
import os, sys, traceback
path = os.path.split('C:\\Users\\HXW\\Desktop\\version\\agv-combine\\msg')[0]
#path = os.path.split(os.path.realpath(__file__))[0]
#os.chdir(path)
path = path.split( os.sep  )
path[-1] = 'bin'
os.environ['PATH'] = os.environ['PATH'] + ';' + os.sep.join( path )
compiler =  'protoc2 '
def compile( file ):
    cwd = os.getcwd()
    file = os.path.realpath( file )
    os.chdir(  os.sep.join( file.split( os.sep  )[:-1] ) )
    file = file.split( os.sep  )[-1]
    os.system( compiler + file + ' --cpp_out=. --python_out=.' )
    
    file = os.path.splitext(file)[0]
    try: os.remove(file + '.cc')
    except:pass
    os.rename( file + '.pb.cc', file + '.cc' )
    
    try: os.remove(file + '.hh')
    except:pass
    os.rename( file + '.pb.h', file + '.hh' )
    
    try: os.remove(file + '.py')
    except:pass
    os.rename( file + '_pb2.py', file + '.py' )
    
    src = open(file + '.cc').read()
    src = src.replace('%s.pb.h'%file, '%s.hh'%file)
    f = open(file + '.cc', 'w')
    f.write(src)
    f.close()
    os.chdir(cwd)
    
    
try:   
    #compile(sys.argv[1])
    compile('dataserver_msg.proto')
except:
    traceback.print_exc()
    print('build proto msg error')
    sys.exit( -1 )
raw_input("Press <enter>")
