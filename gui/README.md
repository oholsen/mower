# README

A web service on the robot, communicating directly to edge-control via WebSocket, showing the robot status and allowing simple control of the robot. 

## Installation

Copy or symlink the appropriate site .json configuration from ./sites to ./sites/site.json.

Static content is served by Apache on the RPi by linking /var/www/html/gui -> .

## Development

Running locally with auto refresh on http://localhost:4000:

    npm install -g light-server 
    light-server -s . -w *
