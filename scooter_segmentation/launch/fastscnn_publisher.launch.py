import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, launch_description_sources
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
import launch_ros.actions


def generate_launch_description():
    depthai_examples_path = get_package_share_directory('scooter_segmentation')
    urdf_launch_dir = os.path.join(get_package_share_directory('depthai_descriptions'), 'launch')

    default_resources_path = os.path.join(depthai_examples_path,
                                'resources')

    mxId         = LaunchConfiguration('mxId',      default = 'x')


    tf_prefix    = LaunchConfiguration('tf_prefix',     default = 'oak')

    lrcheck        = LaunchConfiguration('lrcheck', default = True)
    extended       = LaunchConfiguration('extended', default = False)
    subpixel       = LaunchConfiguration('subpixel', default = True)
    resourceBaseFolder      = LaunchConfiguration('resourceBaseFolder', default = default_resources_path)


    stereo_fps            = LaunchConfiguration('stereo_fps', default = 15)
    confidence            = LaunchConfiguration('confidence', default = 200)
    LRchecktresh          = LaunchConfiguration('LRchecktresh', default = 5)
    
    previewWidth            = LaunchConfiguration('previewWidth',   default = 640)
    previewHeight           = LaunchConfiguration('previewHeight',  default = 360)
    
    
    


    urdf_launch = IncludeLaunchDescription(
                            launch_description_sources.PythonLaunchDescriptionSource(
                                    os.path.join(urdf_launch_dir, 'urdf_launch.py')),
                            launch_arguments={'tf_prefix'   : tf_prefix}.items())


    fastscnn_node = launch_ros.actions.Node(
            package='scooter_segmentation', executable='fastscnn_node',
            output='screen',
            parameters=[{'mxId':                    mxId},
                        {'resourceBaseFolder':      resourceBaseFolder},

                        {'tf_prefix':               tf_prefix},

                        {'lrcheck':                 lrcheck},
                        {'extended':                extended},
                        {'subpixel':                subpixel},

                        {'stereo_fps':              stereo_fps},
                        {'confidence':              confidence},
                        {'LRchecktresh':            LRchecktresh},

                        {'previewWidth':            previewWidth},
                        {'previewHeight':           previewHeight},
                        ])
    

    ld = LaunchDescription()

    ld.add_action(urdf_launch)
    ld.add_action(fastscnn_node)

    # if LaunchConfigurationEquals('depth_aligned', 'True') and LaunchConfigurationEquals('rectify', 'True'):
    #     ld.add_action(point_cloud_container)
    
    # # ld.add_action(point_cloud_node)
    # if LaunchConfigurationEquals('enableRviz', 'True') and rviz_node is not None:
    #     ld.add_action(rviz_node)
    return ld

