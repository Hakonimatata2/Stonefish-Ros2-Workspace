from pathlib import Path
from rosbags.highlevel import AnyReader

bagdir = Path('sintef_dataset_ros2/data_bag')

with AnyReader([bagdir]) as reader:
    print('Topics and message types:')
    for c in sorted(reader.connections, key=lambda x: x.topic):
        print(f'{c.topic:45s}  {c.msgtype}')
