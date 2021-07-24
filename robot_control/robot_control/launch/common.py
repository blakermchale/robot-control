#!/usr/bin/env python3
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import ast
import jinja2
import xacro
import os


def get_launch_arguments(launch_args: list):
    """Converts dictionary of launch args into list of DeclareLaunchArgument's"""
    validate_launch_args(launch_args)
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description'], choices=param.get("choices")) for param in launch_args]


def convert_type(value, atype):
    """Converts string using type identifier."""
    if atype == "int": return int(value)
    elif atype == "bool": return value == "true"
    elif atype == "list": return ast.literal_eval(value)
    else: return value


def get_local_arguments(launch_args: dict, context):
    """Stores launch arguments in dictionary using RCL context."""
    validate_launch_args(launch_args)
    return {param["name"]: convert_type(LaunchConfiguration(param["name"]).perform(context), param.get("type")) for param in launch_args}


def validate_launch_args(launch_args: list):
    """Validates launch args list does not contain any duplicate names."""
    names = []
    for arg in launch_args:
        name = arg["name"]
        if name in names:
            raise ValueError(f"Repeated name '{name}' found in launch arguments. Please make sure to remove the duplicate or add namespacing.")
        names.append(name)


def parse_model_file(file_path, mappings):
    namespace = mappings["namespace"]
    robot_desc = None
    if file_path.split('.')[-1] == "xacro":
        tmp_path = f'/tmp/{namespace}.urdf'
        doc = xacro.process_file(file_path, mappings=mappings)
        robot_desc = doc.toprettyxml(indent='  ')
        tmp_file = open(tmp_path, 'w')
        tmp_file.write(robot_desc)
        tmp_file.close()
    elif file_path.split('.')[-1] == "erb":
        tmp_path = f'/tmp/{namespace}.sdf'
        cmd = "erb"
        for (key, val) in mappings.items():
            cmd += f" {key}={val}"
        cmd += f" {file_path} > {tmp_path}"
        os.system(cmd)
    elif file_path.split('.')[-1] == "jinja":
        tmp_path = f'/tmp/{namespace}.sdf'
        templateFilePath = jinja2.FileSystemLoader(os.path.dirname(file_path))
        jinja_env = jinja2.Environment(loader=templateFilePath)
        j_template = jinja_env.get_template(os.path.basename(file_path))
        output = j_template.render(mappings)
        with open(tmp_path, 'w') as sdf_file:
            sdf_file.write(output)
    else:
        tmp_path = file_path
    return tmp_path, robot_desc

def combine_names(l: list, sep: str):
    l = list(filter(None, l))  # Remove empty strings
    return sep.join(l)
