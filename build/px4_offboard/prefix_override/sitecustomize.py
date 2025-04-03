import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/guilhermekohl/UAV-Window-Detection/install/px4_offboard'
