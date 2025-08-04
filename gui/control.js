
  $( function() {

    var robot_status;

    $("button").button();

    const Speed = 0.1;
    const Omega = 0.1;
    var speed = 0;
    var omega = 0;
    var cut_power = 0;

    $('#forward').on('click', function (e) { speed = Speed; });
    $('#backward').on('click', function (e) { speed = -Speed; });
    $('#left').on('click', function (e) { omega = Omega; });
    $('#right').on('click', function (e) { omega = -Omega; });
    $('#cut').on('click', function (e)  { cut_power = 1; });
    $('#stop').on('click', function (e) { stop(); });

    $('#mission_start').on('click', function (e) { mission_start(); });
    $('#mission_abort').on('click', function (e) { mission_abort(); });

    function stop() {
      speed = 0;
      omega = 0;
      cut_power = 0;
      post("move/stop", null);
    }

    const config = document.getElementById("log");

    function debug(message) {
      if (message.startsWith("Battery"))
        config.value = "";
      config.value += message;
      config.scrollTop = config.scrollHeight;
    }

    var s;

    function post(topic, message) {
      // console.log("post", s)
      console.log("post", topic, message);
      envelope = { topic: topic, message: message };
      try {
        s.send(JSON.stringify(envelope));
      } catch (ex) {
        console.log("Failed to post:", ex.message);
      }
    }

    function mission_start() {
        let name = $('#mission_name').val();
        if (!name.trim()) {
            alert('Please enter a mission name');
            return;
        }
        post("mission/start", name);
    }

    function mission_abort() {
        post("mission/abort", null);
    }

    function updateTime() {
        const now = new Date();
        const timeString = now.toLocaleTimeString();
        $('#time').text(timeString);
    }

    function updateWebSocketStatus(isConnected) {
        const statusElement = $('#websocket');
        const indicator = statusElement.prev('.status-indicator');
        
        if (isConnected) {
            statusElement.text('Connected');
            indicator.removeClass('offline').addClass('online');
        } else {
            statusElement.text('Disconnected');
            indicator.removeClass('online').addClass('offline');
        }
    }

    function connect() {
      try {
        const host = "ws://" + window.location.hostname + ":18888";
        console.log("Host:", host);
        
        s = new WebSocket(host);
        
        var heartbeatTimer;

        function heartbeat() {
          let timeout = robot_status.time.robot_time + 2.5;
          post("move", {timeout: timeout, speed: speed, omega: omega});
          post("cut", {timeout: timeout, power: cut_power});
        }

        s.onopen = function (e) {
          console.log("Websocket open");
          updateWebSocketStatus(true);
          heartbeatTimer = setInterval(() => $("#heartbeat").prop("checked") && heartbeat(), 1000);
        };
        
        s.onclose = function (e) {
          console.log("Websocket closed", e);
          updateWebSocketStatus(false);
          clearInterval(heartbeatTimer);
          setTimeout(connect, 5000);
        };
        
        s.onmessage = function (e) {
          const message = e.data;
          // console.log("WS message:", message);
          let envelope = JSON.parse(message);

          if (envelope.topic == "map") {
              let map = envelope.message;
              console.log("map", map);
              canvas_clear();
              drawBackground(map);
              return;
          }

          if (envelope.topic == "robot_tracking") {
              let message = envelope.message;
              if (Array.isArray(message)) {
                  // Draw all previous positions in current mission when connection is opened
                  drawTrail(message);
              }
              else {
                  let state = message;
                  robot_at(state.x, state.y, state.theta);
              }
              return;
          }

          if (envelope.topic == "status") {
              robot_status = envelope.message;
              // console.log("STATUS", robot_status);
              $("#status").text(JSON.stringify(robot_status, null, 2));
              return;
          }
        };
        
        s.onerror = function (e) {
          console.log("Socket error:", e);
          updateWebSocketStatus(false);
        };
        
      } catch (ex) {
        console.log("Socket exception:", ex);
        updateWebSocketStatus(false);
      }
    }

    // Initialize time display and update every second
    updateTime();
    setInterval(updateTime, 1000);

    connect();

  });
