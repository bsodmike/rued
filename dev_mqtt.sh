#!/bin/bash

set -ex

docker run -p 8080:8080 -p 1883:1883 hivemq/hivemq4&