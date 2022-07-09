rocker --network host --privileged --nvidia --x11 --user --name prius_av \
       --volume "$PWD/:${HOME}/prius_av" \
       --volume /snap/clion/current:/opt/clion \
       --volume /snap/pycharm-professional/current:/opt/pycharm-professional \
       -- amadeuszsz/prius_av:master
