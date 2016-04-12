config_switch
=============

Seamlessly switch user configuration with dotfiles and shell.

Currently supports below dotfiles:

  - .bashrc
  - .zshrc
  - .vimrc
  - .vim
  - .gitconfig

Usage
-----

.. code-block:: bash

  $ gh_user=wkentaro
  $ vim ~/.zshrc.$gh_user
  $ vim ~/.vimrc.$gh_user

  # set user and shell
  $ config_switch wkentaro /usr/local/bin/zsh
  Switching user: -> wkentaro
  Linked .bashrc -> .bashrc.wkentaro
  WARNING: .zshrc is not symlink, so skipping
  Logging in as 'wkentaro' with '/usr/local/bin/zsh'

  $ ls -la ~/.vimrc
  ~/.vimrc -> ~/.vimrc.wkentaro

  $ cat ~/.ros/jsk_tools/current_config
  wkentaro
  /usr/local/bin/zsh

  $ config_switch
  Current user: wkentaro
  Current shell: /usr/local/bin/zsh
