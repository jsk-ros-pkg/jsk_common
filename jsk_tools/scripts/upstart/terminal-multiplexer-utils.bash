#!/bin/bash

SESSION_NAME=`hostname`
DEFAULT_NAME=orig
BACKEND=byobu-screen

if [ `echo $BACKEND | grep 'tmux'` ]; then
    function create-session() {
        $BACKEND new-session -d -s $SESSION_NAME -n $DEFAULT_NAME
        $BACKEND set-option prefix C-t
        # $BACKEND set-window-option aggressive-resize on
        $BACKEND set-option mouse-select-window on
        $BACKEND set-window-option -g mode-mouse copy-mode
    }
    function new-window() {
        $BACKEND new-window -k -n $1 -t $SESSION_NAME
        $BACKEND send-keys -t $SESSION_NAME:$1 "${@:2}" C-m # how to pass whtie space ?
        $BACKEND select-window -t 0
    }
    function kill-window() {
        $BACKEND kill-window -t $SESSION_NAME:$1
    }
elif [ `echo $BACKEND | grep 'screen'` ]; then
    function create-session() {
        $BACKEND -S $SESSION_NAME -d -m
        $BACKEND -S $SESSION_NAME -p 0 -X title $DEFAULT_NAME
    }
    function new-window() {
        $BACKEND -S $SESSION_NAME -X screen -t $1
        $BACKEND -S $SESSION_NAME -p $1 -X stuff "${@:2}$(printf \\r)"
    }
    function kill-window() {
        $BACKEND -S $SESSION_NAME -p $1 -X kill
        sleep 2                 # wait to kill
    }
fi
