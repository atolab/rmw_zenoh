# Copyright 2025 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
import em
from io import StringIO
import os
import pathlib
import sys
from ament_index_python.packages import get_package_share_directory

class ZenohSecutiryConfigGenerator:
    """ZenohSecutiryConfigGenerator generates Zenoh secutiry configurations."""

    def __init__(self):
        """
        Construct NEXUSConfigGenerator.
        """
        self.zenoh_cfg_file_extension = "json5"

    def generate_router_config(self, data):
        try:
            output = StringIO()
            interpreter = em.Interpreter(
                output=output,
                options={
                    em.BUFFERED_OPT: True,
                    em.RAW_OPT: True,
                },
            )

            template_path = pathlib.Path(os.path.join(
                get_package_share_directory("zenoh_security_configuration"),
                "templates",
                "router.json5"
            ))


            with template_path.open('r') as h:
                template_content = h.read()
            interpreter.string(template_content, locals=data)
            return output.getvalue()
        except:
            print('lol')
        finally:
            if interpreter is not None:
                interpreter.shutdown()
            interpreter = None

    def generate_zenoh_config(self, output_dir, router_config, encoding: str = 'utf-8'):
        """
        Generate Zenoh bridge configs and output to directory 'output_dir'.

        Parameters
        ----------
        output_dir : str
            Output directory for Zenoh configurations

        """
        write_filepath = os.path.join(
            output_dir, "router" + "."
            + self.zenoh_cfg_file_extension,
        )
        output_file = pathlib.Path(write_filepath)
        print(f"Generated Zenoh secutiry configuration at {write_filepath}")
        if output_file.exists():
            existing_content = output_file.read_text(encoding=encoding)
            if existing_content == router_config:
                return
        elif output_file.parent:
            os.makedirs(str(output_file.parent), exist_ok=True)

        output_file.write_text(router_config, encoding=encoding)

def main(argv=sys.argv):
    """Entrypoint."""
    parser = argparse.ArgumentParser(
        description="Generate Zenoh configurations from a NEXUS Network \
            and REDF Configuration",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    parser.add_argument(
        "-o",
        "--output",
        required=True,
        type=str,
        help="Output directory for Zenoh bridge configurations",
    )

    parser.add_argument(
        "-p",
        "--protocols",
        nargs='*',
        required=False,
        choices=["tcp", "tls"],
        default=['tls'],
        help="Protocols chooices",
    )

    args = parser.parse_args(argv[1:])

    data = {'protocols': args.protocols}

    zenoh_sec_gen = ZenohSecutiryConfigGenerator()
    router_config = zenoh_sec_gen.generate_router_config(data)
    zenoh_sec_gen.generate_zenoh_config(args.output, router_config)

if __name__ == "__main__":
    main(sys.argv)
