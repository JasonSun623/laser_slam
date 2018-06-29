#!/usr/bin/env python

import time
import json
import os
import subprocess
import rospy
from std_msgs.msg import String
import roslaunch

global process
global service_pub

bag_path = None
inited = False

mount_dir = '/mnt'
home_dir = '/home/robot'
local_bags_dir = home_dir + '/mr_bags'
map_server_dir = home_dir + '/catkin_ws/install/share/map_server'
local_maps_dir = map_server_dir + '/maps'
udisk_maps_dir = mount_dir + '/mr_maps'
udisk_bags_dir = mount_dir + '/mr_bags'
udisk_logs_dir = mount_dir + '/mr_logs'
all_maps_name = 'all_maps.tar.gz'

dot_ros_dir = home_dir
dot_ros_name = 'dot_ros.tar.gz'

agent_log_dir = home_dir + '/agent'
agent_log_name = 'agent.tar.gz'

residual_bags = set()


def status_pub(status):
    data = {'status': status}
    status = json.dumps(
        {'name': 'service_desk_status', 'data': data})
    service_pub.publish(status)


def log_status_pub(level, status):
    if level == 'info':
        rospy.loginfo(status)
    elif level == 'err':
        rospy.logerr(status)
    elif level == 'warn':
        rospy.logwarn(status)
    status_pub(status)


def mount_udisk():
    disk_info = subprocess.check_output('sudo fdisk -l', shell=True)
    disk_count = disk_info.count('Disk /dev/')
    if disk_count > 1:
        log_status_pub('info', 'find udisk.')
        dev_index = disk_info.rindex('/dev/')
        space_index = disk_info.index(' ', dev_index)
        dev_path = disk_info[dev_index: space_index]
        rospy.loginfo('dev_path: ' + dev_path)
    else:
        log_status_pub('err', 'no udisk found.')
        return False

    mount_result = subprocess.call(
        'sudo mount ' + dev_path + ' ' + mount_dir, shell=True)
    if mount_result == 0:
        log_status_pub('info', 'mount udisk success')
        return True
    else:
        log_status_pub('err', 'mount udisk fail')
        return False


def umount_udisk():
    umount_result = subprocess.call('sudo umount ' + mount_dir, shell=True)
    if umount_result == 0:
        log_status_pub('info', 'umount udisk success')
        return True
    else:
        log_status_pub('err', 'umount udisk fail')
        return False


def compress_bag(bag_path):
    if bag_path is None:
        log_status_pub('warn', 'no valid bag.')
        return False
    bag_name_full = bag_path + '.bag'
    log_status_pub('info', 'start compress: ' + bag_name_full)
    bag_name = bag_name_full.replace(local_bags_dir + '/', '', 1)

    compress_result = subprocess.call(
        'tar -C ' + local_bags_dir + ' -zcv --file=' + bag_name_full + '.tar.gz ' + bag_name, shell=True)
    if compress_result == 0:
        log_status_pub('info', 'compress bag ' + bag_name_full + ' success')
        return True
    else:
        log_status_pub('err', 'compress bag ' + bag_name_full + ' fail')
        return False


def move_bag_to_udisk(bag_path):
    if bag_path is None:
        log_status_pub('warn', 'no valid bag.')
        return False
    file_name = bag_path + '.bag.tar.gz'
    log_status_pub('info', 'start move: ' + file_name)

    if not os.path.exists(udisk_bags_dir):
        os.makedirs(udisk_bags_dir)

    copy_result = subprocess.call(
        'mv ' + file_name + ' ' + udisk_bags_dir, shell=True)
    if copy_result == 0:
        log_status_pub('info', 'move ' + file_name + ' to udisk success')
        residual_bags.discard(bag_path)
        return True
    else:
        log_status_pub('err', 'move ' + file_name + ' to udisk fail')
        return False


def compress_and_copy_latest_bag():
    if process.is_alive():
        log_status_pub('warn', 'one bag is recording, please stop it first.')
        return
    if not mount_udisk():
        umount_udisk()
        return
    if not compress_bag(bag_path):
        umount_udisk()
        return
    if not move_bag_to_udisk(bag_path):
        umount_udisk()
        return
    umount_udisk()


def compress_and_copy_resudial_bags():
    if process.is_alive():
        log_status_pub('warn', 'one bag is recording, please stop it first.')
        return
    if not mount_udisk():
        umount_udisk()
        return
    if not residual_bags:
        log_status_pub('warn', 'no residual bags.')
        umount_udisk()
        return
    # can not change the container's size during iter, so do a shallow copy here
    residual_bags_copy = residual_bags.copy()
    for bag_path in residual_bags_copy:
        compress_bag(bag_path)
        move_bag_to_udisk(bag_path)
    umount_udisk()


def delete_all_bags():
    if process.is_alive():
        log_status_pub('warn', 'one bag is recording, please stop it first.')
        return
    delete_result = subprocess.call(
        'rm -r ' + local_bags_dir + '/', shell=True)
    if delete_result == 0:
        log_status_pub('info', 'delete all bags success')
        return True
    else:
        log_status_pub('err', 'delete all bags fail')
        return False


def init_process():
    global process
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    node = roslaunch.core.Node('rosbag', 'record', args='/none -O None')
    process = launch.launch(node)
    rospy.loginfo('init process')


def getCurrentTime():
    return time.strftime('%Y-%m-%d-%H-%M-%S', time.localtime(time.time()))


def start_rosbag_record(bag_name, params='', tag=''):
    if not os.path.exists(local_bags_dir):
        os.makedirs(local_bags_dir)

    if process.is_alive():
        log_status_pub('warn', 'one bag is recording, please stop it first.')
        return
    global bag_path
    current_time = getCurrentTime()

    bag_path = local_bags_dir + "/" + current_time + '-' + tag
    if bag_name != '':
        bag_path = bag_path + '-' + bag_name

    residual_bags.add(bag_path)

    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    node = roslaunch.core.Node(
        'rosbag', 'record', args=params + ' -O ' + bag_path)
    global process
    process = launch.launch(node)
    log_status_pub('info', 'start record ' + params)


def start_rosbag_record_all(bag_name):
    start_rosbag_record(bag_name, params='-a', tag='all')


def start_rosbag_record_map_build(bag_name):

    kill_result = subprocess.call('rosnode kill /amcl', shell=True)
    if kill_result == 0:
        log_status_pub('info', 'kill amcl success')
    else:
        log_status_pub('err', 'kill amcl fail')

    subprocess.call('rosnode kill /slam_gmapping', shell=True)
    if kill_result == 0:
        log_status_pub('info', 'kill gmapping success')
    else:
        log_status_pub('err', 'kill gmapping fail')

    subprocess.call('rosnode kill /slam_karto', shell=True)
    if kill_result == 0:
        log_status_pub('info', 'kill karto success')
    else:
        log_status_pub('err', 'kill karto fail')

    map_build_param = rospy.get_param(
        '/service_desk/rosbag_record_map_build', '/tf /scan /odom /odometry/filtered')
    start_rosbag_record(
        bag_name, params=map_build_param, tag='mapBuild')


def start_rosbag_record_custom(bag_name):
    custom_param = rospy.get_param('/service_desk/rosbag_record_custom', '-a')
    start_rosbag_record(bag_name, params=custom_param, tag='custom')


def start_rosbag_record_common(bag_name):
    common_param = rospy.get_param('/service_desk/rosbag_record_common', '-a')
    start_rosbag_record(bag_name, params=common_param, tag='common')


def stop_rosbag_record():
    if process.is_alive():
        process.stop()
        log_status_pub('info', 'stop record success')
    else:
        log_status_pub('warn', 'no recording bag')


def simple_stop_record():
    if process.is_alive():
        process.stop()
    global inited
    inited = True


def get_rosbag_name(json_obj):
    data = json_obj.get('data')
    if data is None:
        return ''
    bag_name = data.get('bag_name')
    if bag_name is None:
        return ''
    else:
        return bag_name


def copy_single_map(json_obj):
    data = json_obj.get('data')
    scene_name = data.get('scene_name')
    map_name = data.get('map_name')

    log_status_pub('info', 'start copy: ' + scene_name + '/' + map_name)

    map_path = local_maps_dir + '/' + scene_name + '/map/' + map_name + '.pgm'

    if not os.path.exists(map_path):
        log_status_pub('err', 'map file not exist: ' + map_path)
        return

    udisk_scene_dir = udisk_maps_dir + '/' + scene_name

    if not os.path.exists(udisk_scene_dir):
        os.makedirs(udisk_scene_dir)

    copy_result = subprocess.call(
        'cp ' + map_path + ' ' + udisk_scene_dir, shell=True)
    if copy_result == 0:
        log_status_pub('info', 'copy ' + scene_name +
                       '/' + map_name + ' to udisk success')
        return True
    else:
        log_status_pub('err', 'copy ' + scene_name +
                       '/' + map_name + ' to udisk fail')
        return False


def copy_map_to_udisk(json_obj):
    if not mount_udisk():
        umount_udisk()
        return
    if not copy_single_map(json_obj):
        umount_udisk()
        return
    umount_udisk()


def compress_maps():
    log_status_pub('info', 'start compress maps.')
    compress_result = subprocess.call(
        'tar -C ' + map_server_dir + ' -zcv --file=' + map_server_dir + '/' + all_maps_name + ' maps', shell=True)
    if compress_result == 0:
        log_status_pub('info', 'compress maps success')
        return True
    else:
        log_status_pub('err', 'compress maps fail')
        return False


def move_maps_udisk():
    log_status_pub('info', 'start move ' + all_maps_name + ' to udisk')

    if not os.path.exists(udisk_maps_dir):
        os.makedirs(udisk_maps_dir)

    move_result = subprocess.call(
        'mv ' + map_server_dir + '/' + all_maps_name + ' ' + udisk_maps_dir, shell=True)
    if move_result == 0:
        log_status_pub('info', 'move ' + all_maps_name + ' to udisk success')
        return True
    else:
        log_status_pub('err', 'move ' + all_maps_name + ' to udisk fail')
        return False


def copy_maps_out():
    if not mount_udisk():
        umount_udisk()
        return
    compress_maps()
    move_maps_udisk()
    umount_udisk()


def copy_maps_robot():
    log_status_pub('info', 'start copy ' + all_maps_name + ' to robot')

    if not os.path.exists(udisk_maps_dir):
        os.makedirs(udisk_maps_dir)

    move_result = subprocess.call(
        'cp ' + udisk_maps_dir + '/' + all_maps_name + ' ' + map_server_dir, shell=True)
    if move_result == 0:
        log_status_pub('info', 'copy ' + all_maps_name + ' to robot success')
        return True
    else:
        log_status_pub('err', 'copy ' + all_maps_name + ' to robot fail')
        return False


def extract_maps():
    log_status_pub('info', 'start extract maps.')
    extract_result = subprocess.call(
        'tar -C ' + map_server_dir + ' -zxv --file=' + map_server_dir + '/' + all_maps_name, shell=True)
    if extract_result == 0:
        log_status_pub('info', 'extract maps success')
        return True
    else:
        log_status_pub('err', 'extract maps fail')
        return False


def delete_compressed_maps():
    log_status_pub('info', 'start delete compressed maps.')
    delete_result = subprocess.call(
        'rm ' + map_server_dir + '/' + all_maps_name, shell=True)
    if delete_result == 0:
        log_status_pub('info', 'delete compressed maps success')
        return True
    else:
        log_status_pub('err', 'delete compressed maps fail')
        return False


def copy_maps_in():
    if not mount_udisk():
        umount_udisk()
        return
    if not copy_maps_robot():
        umount_udisk()
        return
    umount_udisk()
    extract_maps()
    delete_compressed_maps()


def compress_dot_ros():
    log_status_pub('info', 'start compress .ros.')
    compress_result = subprocess.call(
        'tar -C ' + dot_ros_dir + ' -zcv --file=' + dot_ros_dir + '/' + dot_ros_name + ' .ros', shell=True)
    if compress_result == 0:
        log_status_pub('info', 'compress .ros success')
        return True
    else:
        log_status_pub('err', 'compress .ros fail')
        return False


def move_dot_ros_udisk():
    log_status_pub('info', 'start move ' + dot_ros_name + ' to udisk')

    if not os.path.exists(udisk_logs_dir):
        os.makedirs(udisk_logs_dir)

    move_result = subprocess.call(
        'mv ' + dot_ros_dir + '/' + dot_ros_name + ' ' + udisk_logs_dir, shell=True)
    if move_result == 0:
        log_status_pub('info', 'move ' + dot_ros_name + ' to udisk success')
        return True
    else:
        log_status_pub('err', 'move ' + dot_ros_name + ' to udisk fail')
        return False


def copy_log_dot_ros():
    if not mount_udisk():
        umount_udisk()
        return
    compress_dot_ros()
    move_dot_ros_udisk()
    umount_udisk()


def compress_agent():
    log_status_pub('info', 'start compress agent logs.')
    compress_result = subprocess.call(
        'tar -C ' + agent_log_dir + ' -zcv --file=' + agent_log_dir + '/' + agent_log_name + ' logs', shell=True)
    if compress_result == 0:
        log_status_pub('info', 'compress agent logs success')
        return True
    else:
        log_status_pub('err', 'compress agent logs fail')
        return False


def move_agent_udisk():
    log_status_pub('info', 'start move ' + agent_log_name + ' to udisk')

    if not os.path.exists(udisk_logs_dir):
        os.makedirs(udisk_logs_dir)

    move_result = subprocess.call(
        'mv ' + agent_log_dir + '/' + agent_log_name + ' ' + udisk_logs_dir, shell=True)
    if move_result == 0:
        log_status_pub('info', 'move ' + agent_log_name + ' to udisk success')
        return True
    else:
        log_status_pub('err', 'move ' + agent_log_name + ' to udisk fail')
        return False


def copy_log_agent():
    if not mount_udisk():
        umount_udisk()
        return
    compress_agent()
    move_agent_udisk()
    umount_udisk()


def service_desk_request_callback(data):
    if not inited:
        log_status_pub('err', 'not inited, please try later.')
        return
    json_obj = json.loads(data.data)
    name = json_obj.get('name')
    if name is None:
        rospy.logerr('name is null')
        return

    if name == 'rosbag_record_all':
        bag_name = get_rosbag_name(json_obj)
        start_rosbag_record_all(bag_name)

    elif name == 'rosbag_record_map_build':
        bag_name = get_rosbag_name(json_obj)
        start_rosbag_record_map_build(bag_name)

    elif name == 'rosbag_record_custom':
        bag_name = get_rosbag_name(json_obj)
        start_rosbag_record_custom(bag_name)

    elif name == 'rosbag_record_common':
        bag_name = get_rosbag_name(json_obj)
        start_rosbag_record_common(bag_name)

    elif name == 'rosbag_record_stop':
        stop_rosbag_record()

    elif name == 'rosbag_compress_copy_latest':
        compress_and_copy_latest_bag()

    elif name == 'rosbag_compress_copy_residual':
        compress_and_copy_resudial_bags()

    elif name == 'rosbag_delete_all':
        delete_all_bags()

    elif name == 'map_copy_udisk':
        copy_map_to_udisk(json_obj)

    elif name == 'maps_copy_out':
        copy_maps_out()

    elif name == 'maps_copy_in':
        copy_maps_in()

    elif name == 'log_copy_dot_ros':
        copy_log_dot_ros()

    elif name == 'log_copy_agent':
        copy_log_agent()


if __name__ == '__main__':
    rospy.init_node('service_desk', anonymous=False)
    service_pub = rospy.Publisher('service_desk_pub', String, queue_size=10)
    rospy.Subscriber('service_desk_sub',
                     String, service_desk_request_callback)

    init_process()
    time.sleep(1)
    simple_stop_record()
    rospy.spin()
