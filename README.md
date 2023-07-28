# robot-autonomy

Stand-alone autonomous robot stack for a DIY lawn mower.

# Quick start

Start the GUI and edge-control running the robot simulation in separate terminals:

    cd gui
    npm install -g light-server 
    light-server -s . -w *

    cd edge-control
    poetry install
    poetry run mqtt

Open http://localhost:4000. Check the Control box and manually steer the robot with forward, left, right, stop. Observe the robot moving in the map. Press Stop, uncheck the Control box, type "Mowing" in the mission box, and press Start. The robot will slowly start mowing the complex lawn area around the perimenter and step inwards on subsequent laps.
