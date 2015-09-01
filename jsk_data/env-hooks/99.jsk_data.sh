#!/bin/sh

# -------------------------
# jsk_data
# ~~~~~~~~
# Easier way to sync files.
# -------------------------


# -------
# command
# -------

usage () {
  if [ $# -eq 0 ]; then
    cat <<EOF
usage: jsk_data <command>

commands:

  get    download data from remote
  ls     list data at remote
EOF
    return 0
  fi
  case "$1" in
    get|ls)
      cat <<EOF
usage: jsk_data $1 <remote> <filename>

  <remote>    small or large (alias: s,sm,l,lg)
  <filename>  allow regex

example:

  $ jsk_data $1 large pr2-rosbag.bag
  $ jsk_data $1 small baxter-*.bag
EOF
      ;;
    *) ;;
  esac
}


_check_remote_dest () {
  if [ $# -eq 0 ]; then
    echo "Please specify remote" >&2
    return 1
  fi
  case "$1" in
    l|lg|large)
      echo large ;;
    s|sm|small)
      echo small ;;
    *)
      echo "Unexpected remote" >&2
      return 1 ;;
  esac
}


_jsk_data_get () {
  local remote
  remote=`_check_remote_dest $1`
  [ $? -eq 0 ] || {
    echo; usage get; return 1
  }
  (roscd jsk_data && make $remote KEYWORD=$2)
}


_jsk_data_ls () {
  local remote
  remote=`_check_remote_dest $1`
  [ $? -eq 0 ] || {
    echo; usage ls; return 1
  }
  (roscd jsk_data && make $remote-list KEYWORD=$2)
}


jsk_data () {
  if [ $# -eq 0 ]; then
    usage && return 1
  fi

  case "$1" in
    get)
      shift
      _jsk_data_get $@ && return 0 ;;
    ls)
      shift
      _jsk_data_ls $@ && return 0 ;;
    *)
      echo "Error: unexpected command" >&2
      usage && return 1 ;;
  esac
}
