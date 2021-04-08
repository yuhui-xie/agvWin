from distutils.core import setup
import py2exe

opt = {"py2exe": {
            "includes": ["sip", "PyQt4.QtGui", "stdmsg", "google.protobuf", "component" ],
             'bundle_files': 1, 
             'compressed': True  
       } }
setup(
    windows=['gui.py'], 
    options= opt,
    data_files = ['robot.png', 'navigation.ui', ]
)
