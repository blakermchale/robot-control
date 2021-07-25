#!/usr/bin/env python
import os
import textwrap
from argparse import ArgumentParser, RawDescriptionHelpFormatter


def get_arguments():
    parser = ArgumentParser(prog='micrortps_agent',
      formatter_class=RawDescriptionHelpFormatter,
      epilog=textwrap.dedent('''\
         Example:
             python micrortps_agent.py -i 1 -d "~/logs/" -n iris_1/RTPS
         '''))
    parser.add_argument("-i", "--instance", default=0, type=int, help="Instance of vehicle.")
    parser.add_argument("-n", "--namespace", default=None, type=str, help="ROS namespace")
    parser.add_argument("-d", "--log-directory", type=str, help="Directory to store log files.")
    parser.add_argument("-s", "--serial-device", default="/dev/ttyACM0", type=str, help="UART device.")
    parser.add_argument("-fc", "--flight-controller", action="store_true", default=False, help="Starts up agent for PX4 board.")
    args, _ = parser.parse_known_args()
    return args


def main():
    args = get_arguments()
    # Create folder for storing vehicle data
    flight_controller = args.flight_controller
    namespace = args.namespace
    instance = args.instance
    log_directory = args.log_directory
    os.system(f'[ ! -d "{log_directory}" ] && mkdir -p "{log_directory}"')

    rtps_name = f"{namespace}/RTPS"
    if flight_controller:
        os.system(f"micrortps_agent -n {rtps_name} -d {args.serial_device} -b 1000000")
    else:
        os.system(f"micrortps_agent -t UDP -r {2020+instance*2} -s {2019+instance*2} -n {rtps_name} >{log_directory}/agent_out.log 2>{log_directory}/agent_err.log")


if __name__=="__main__":
    main()
