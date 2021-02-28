#!/usr/bin/env python3

import argparse
import glob
import os
import subprocess


def _format_python3(root_path):
    yapf_cmd = ['yapf3', '-i', root_path]
    docformat_cmd = [
        'docformatter', '-i', '--wrap-summaries', '80', '--wrap-descriptions',
        '0', root_path
    ]

    if os.path.isdir(root_path):
        py_files = glob.glob(os.path.join(root_path, '**/*.py'), rescsive=True)
        if not py_files:
            return
        yapf_cmd.append('-r')
        docformat_cmd.append('-r')

    subprocess.check_call(yapf_cmd)
    # subprocess.check_call(docformat_cmd)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Format python files")
    parser.add_argument(
        'root_path', help='Input root path to directory or file')
    args = parser.parse_args()

    _format_python3(args.root_path)
