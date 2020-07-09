#!/usr/bin/env python

# A script to check network hostname sanity
# It's useful to setup robot system
# w/o DNS.

import sys
from socket import gethostbyname, gaierror
from yaml import load
try:
  from colorama import Fore, Style
except:
  print("Please install colorama by pip install colorama")
  sys.exit(1)


def usage():
  print("check_host_sanity.py yaml-file")

def checkHostSanity(yaml_file_path):
  with open(yaml_file_path, "r") as f:
    host_names = load(f)
    for p in host_names:
      hostname = p["hostname"]
      ip_should_be = p["ip"]
      try:
        real_ip = gethostbyname(hostname)
        if real_ip == "127.0.0.1" or real_ip == "127.0.1.1" or real_ip == ip_should_be:
          # Always localhost is OK
          print(Fore.GREEN, "[OK!]    ", hostname, "is", real_ip)
        else:
          print(Fore.RED, "[ERROR!] ", hostname, "is", real_ip, "but it should be", ip_should_be)
      except gaierror:
        print(Fore.RED, "[ERROR!] ", hostname, "is not defined but it should be", ip_should_be)
        
if __name__ == "__main__":
  if len(sys.argv) != 2:
    usage()
    sys.exit(1)
  else:
    checkHostSanity(sys.argv[1])
