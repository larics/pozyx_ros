#!/usr/bin/env python

import os
import csv
import yaml
import argparse

parser = argparse.ArgumentParser(description="Arguments for the pozyx csv-yaml conversion script.")
parser.add_argument('csv', nargs='?', default='../config/anchors.csv')
parser.add_argument('yaml', nargs='?', default='../config/anchors.yaml')
args = parser.parse_args()

if args.csv[0] in ['/', '~']:
    in_path = args.csv
else:
    in_path = os.getcwd() + '/' + args.csv


if args.yaml[0] in ['/', '~']:
    out_path = args.yaml
else:
    out_path = os.getcwd() + '/' + args.yaml



with open(in_path, mode='r') as in_file:
    reader = csv.DictReader(in_file, delimiter=';')
    anchors = []
    for row in reader:
        if row['x'] and row['y'] and row['z']:
            anchors.append({'id': row['hexId'], 'coordinates': map(int, [row['x'], row['y'], row['z']])})
    with open(out_path, mode='w') as out_file:
        yaml.dump({'anchors': anchors}, out_file, default_flow_style=None, sort_keys=False)

