#!/bin/bash
#\file    sync.sh
#\brief   Sync files from original (private use only).
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Jan.28, 2016
rsync -azv -L ${HOME}/ros_ws/lfd_sim/{CMakeLists.txt,Makefile,config,include,launch,mainpage.dox,manifest.xml,msg,src,srv} .
