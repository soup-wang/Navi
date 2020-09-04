#!/bin/bash
ps -ef|grep Navigator |grep -v grep|cut -c 9-15|xargs kill -9
echo "完成"
