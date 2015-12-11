restart\_travis
---------------
Restart a travis job for specified github repository.
Firstly, add below in your `.bashrc`:

```sh
# you can get SLACK_TOKEN at https://api.slack.com/web.
export SLACK_TOKEN=xoxp-XXXXXXXX-XXXXXXXX-XXXXXXXX
```

To restart travis, need the repository slug (ex: `jsk-ros-pkg/jsk_common`) and job id (ex: `2019.6`):

```sh
$ restart_travis jsk-ros-pkg/jsk_common 2019.6  # usage: restart_travis <repo_slug> <job_id>
sending... 'restart travis jsk-ros-pkg/jsk_common 2019.6' -> #travis
```