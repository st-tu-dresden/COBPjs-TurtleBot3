var scanDataEventSet = bp.EventSet("", function (e) {
  return e.name.equals("ScanData");
});

var scanEventSet = bp.EventSet("", function (e) {
  return e.name.equals("/scan");
});

var updateVelocityEventSet = bp.EventSet("", function (e) {
  return e.name.equals("UpdateVelocity");
});

var forwardEventSet = bp.EventSet("", function (e) {
  return e.name.equals("UpdateVelocity") && e.data.linear.x > 0;
});

var leftEventSet = bp.EventSet("", function (e) {
  return e.name.equals("UpdateVelocity") && e.data.angular.z > 0;
});

var rightEventSet = bp.EventSet("", function (e) {
  return e.name.equals("UpdateVelocity") && e.data.angular.z < 0;
});

var backwardsEventSet = bp.EventSet("", function (e) {
  return e.name.equals("UpdateVelocity") && e.data.linear.x < 0;
});


bthread("init", function () {
    var properties = {"queue_length": 10, "throttle_rate": 125}; //125
    ros.addTopic("/cmd_vel", "geometry_msgs/Twist", properties);
    ros.addTopic("/scan", "sensor_msgs/LaserScan", properties);
    ros.subscribe("/scan");
    ros.advertise("/cmd_vel", "UpdateVelocity");
});

ctx.bthread("update scan data", "Robot", function (entity) {
  while (true) {
      var e = bp.sync({waitFor: scanEventSet});
      var data = JSON.parse(e.data);
      var scan_data = {ranges: data.ranges, range_max:data.range_max};
     
      sync({request: Event("ScanData", scan_data)});
      sync({request: Event("ScanCompleted")});
  }
}); 

ctx.bthread("toggle scan and move", "Robot", function (entity) {
  while(true) {
    sync({waitFor: scanDataEventSet, block: updateVelocityEventSet});
    sync({waitFor: updateVelocityEventSet, block: scanDataEventSet});
  }
});

ctx.bthread("move forward", "Robot", function (entity) {
  while (true) {
    sync({request: Event("UpdateVelocity", {"linear": {"x": 0.3}, "angular": {"z": 0}})}, 100);
  }
});

ctx.bthread("turn left", "Robot", function (entity) {
  while(true) {
    sync({request: Event("UpdateVelocity", {"linear": {"x": 0}, "angular": {"z": 1.5}})}, 50);
  }
});

ctx.bthread("turn right", "Robot", function (entity) {
  while(true) {
    sync({request: Event("UpdateVelocity", {"linear": {"x": 0}, "angular": {"z": -1.5}})}, 50);
  }
});

ctx.bthread("move backwards", "Robot", function (entity) {
  while (true) {
    sync({request: Event("UpdateVelocity", {"linear": {"x": -0.3}, "angular": {"z": 0}})}, 10);
  }
});

ctx.bthread("avoid walls ahead", "ObstacleAhead", function (entity) {
  sync({block: forwardEventSet}) 
});

ctx.bthread("avoid walls on the left", "ObstacleLeft", function (entity) {
  sync({block: leftEventSet})
});

ctx.bthread("avoid walls on the right", "ObstacleRight", function (entity) {
  sync({block: rightEventSet})
});

ctx.bthread("avoid walls on the front left", "ObstacleFrontLeft", function (entity) {
  sync({block: [leftEventSet, forwardEventSet]})
});

ctx.bthread("avoid walls on the front right", "ObstacleFrontRight", function (entity) {
  sync({block: [rightEventSet, forwardEventSet]})
});

ctx.bthread("avoid walls behind", "ObstacleBehind", function (entity) {
  sync({block: backwardsEventSet})
});
