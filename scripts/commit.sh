#!/bin/bash

# Parse command line arguments
while getopts "b:" opt; do
  case $opt in
    b)
      optional_arg=$OPTARG
      ;;
    \?)
      echo "Invalid option: -$OPTARG" >&2
      exit 1
      ;;
    :)
      echo "Option -$OPTARG requires an argument." >&2
      exit 1
      ;;
  esac
done

shift $((OPTIND-1))

# Check that the required argument is present
if [ -z "$1" ]; then
  echo "Usage: ./commit.sh [-b branch-name] <commit_msg>"
  exit 1
fi

required_arg=$1

# Do something with the arguments

git add .
git commit -m "$required_arg"
git push -u origin $optional_arg
