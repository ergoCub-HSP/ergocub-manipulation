FROM ar0s/yarp-gazebo

RUN git clone https://github.com/robotology/idyntree.git && \
    cd idyntree && git checkout v8.1.0 && mkdir build && cd build && \
    cmake .. && make && make install

RUN apt update && apt install --no-install-recommends -y -qq tmux && \
    cd /root && wget https://github.com/tmux-plugins/tmux-sensible/blob/master/sensible.tmux -O .sensible.tmux && \
    echo "set -g mouse on\nrun-shell ~/.sensible.tmux" > .tmux.conf

ENTRYPOINT []