FROM ubuntu

MAINTAINER Ryota Sakamoto <saka_ro@yahoo.co.jp>


COPY . /usr/src/security_box
WORKDIR /usr/src/security_box

RUN mv /bin/sh /bin/sh_tmp && ln -s /bin/bash /bin/sh

RUN apt-get update && apt-get upgrade -y && \
    apt-get install python3-dev -y && \
    apt-get install python3-pip -y && \
    pip3 install pipenv RPi.GPIO && \
    pipenv install --system
