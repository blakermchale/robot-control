#!/usr/bin/env python3
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import ast


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
