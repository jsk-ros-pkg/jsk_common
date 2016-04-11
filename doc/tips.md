# Tips & FAQ


## Speed-up compilation by ccache

You can use ccache to speed-up compilation.

```bash
sudo apt-get install ccache
sudo ln -sf /usr/bin/ccache /usr/local/bin/gcc
sudo ln -sf /usr/bin/ccache /usr/local/bin/cc
sudo ln -sf /usr/bin/ccache /usr/local/bin/g++
sudo ln -sf /usr/bin/ccache /usr/local/bin/c++
ccache -M 10G
```

And all of your compilation should be done by ccache.
