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

var positionEventSet = bp.EventSet("", function (e) {
  return e.name.equals("PositionData");
});

var odomEventSet = bp.EventSet("", function (e) {
  return e.name.equals("/odom");
});

var storeTargetEventSet = bp.EventSet("", function (e) {
  return e.name.equals("StoreTarget");
});

var addDeliveryEventSet = bp.EventSet("", function (e) {
  return e.name.equals("/add_del");
});


bthread("init", function () {
    var properties = {"queue_length": 10, "throttle_rate": 125}; 
    ros.addTopic("/cmd_vel", "geometry_msgs/Twist", properties);
    ros.addTopic("/odom", "nav_msgs/Odometry", properties);
    ros.addTopic("/scan", "sensor_msgs/LaserScan", properties);
    ros.subscribe("/scan");
    ros.subscribe("/odom");
    ros.advertise("/cmd_vel", "UpdateVelocity");
    
    ros.addTopic("/delivery_info", "std_msgs/String", properties);
    ros.advertise("/delivery_info", "PubInfo");
    ros.addTopic("/add_del", "geometry_msgs/PoseArray", properties);
    ros.subscribe("/add_del");
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

ctx.bthread("update position", "Robot", function (entity) {
  while (true) {
    var e = bp.sync({waitFor: odomEventSet});
    var position = JSON.parse(e.data).pose.pose.position;

    var orientation = JSON.parse(e.data).pose.pose.orientation;
    var w = orientation.w;
    var x = orientation.x;
    var y = orientation.y;
    var z = orientation.z;
    var siny = 2.0 * (w * z + x * y);
    var cosy = 1.0 - 2.0 * (y * y + z * z);
    var angle = Math.atan2(siny, cosy);

    var position_data = {x_coor: position.x, y_coor: position.y, orientation: angle}

    sync({request: Event("PositionData", position_data)});
    sync({request: Event("PositionUpdateCompleted")});
  }
});

ctx.bthread("toggle scan and move", "Robot", function (entity) {
  while(true) {
    sync({waitFor: scanDataEventSet, block:[updateVelocityEventSet, positionEventSet]});
    sync({waitFor: updateVelocityEventSet, block:[scanDataEventSet, positionEventSet]});
    sync({waitFor: positionEventSet, block:[updateVelocityEventSet, scanDataEventSet]});
  }
});

ctx.bthread("move forward", "Obstacle", function (entity) {
  while (true) {
    sync({request: Event("UpdateVelocity", {"linear": {"x": 0.3}, "angular": {"z": 0}})}, 100);
  }
});

ctx.bthread("turn left", "Obstacle", function (entity) {
  while(true) {
    sync({request: Event("UpdateVelocity", {"linear": {"x": 0}, "angular": {"z": 1.5}})}, 50);
  }
});

ctx.bthread("turn right", "Obstacle", function (entity) {
  while(true) {
    sync({request: Event("UpdateVelocity", {"linear": {"x": 0}, "angular": {"z": -1.5}})}, 50);
  }
});

ctx.bthread("move backwards", "Obstacle", function (entity) {
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

ctx.bthread("prioritise obstacle avoidance", "Obstacle", function (entity) {
  sync({block: Event("AdvanceToTarget")});
});


ctx.bthread("move to target", "Target", function (entity) {
  function rad(x){
    return (x >= 0) ? x : (2*Math.PI + x);
  }
  
  function angle_difference(source, target){
    diff = target - source;
    if(diff > Math.PI){diff = diff - 2*Math.PI};
    if(diff < -Math.PI){diff = diff + 2*Math.PI};
    return diff;
  }

  while(true) {
    sync({request: Event("AdvanceToTarget")});

    var angle_of_robot = entity.rob_orientation;
    var angle_to_target = Math.atan2(entity.tar_y_coor-entity.rob_y_coor, entity.tar_x_coor - entity.rob_x_coor);
   
    var turn_angle = angle_difference(rad(angle_of_robot), rad(angle_to_target))
    var turn_angle_abs = Math.abs(turn_angle);
    
    if((Math.abs(entity.tar_x_coor - entity.rob_x_coor) < 0.1 ) && (Math.abs(entity.tar_y_coor - entity.rob_y_coor) < 0.1 )){
      sync({request: Event("UpdateVelocity", {"linear": {"x": 0.0}, "angular": {"z": 0}})});
      sync({request: Event("PubInfo", {data: "Reached target at coordinates: x: " + String(entity.tar_x_coor) + " y: " + String(entity.tar_y_coor)})}, 1000);

      sync({request: Event("TargetReached", entity.id)}, 1000);
    }
    
    if(turn_angle_abs > 0.2) {
      sync({request: Event("UpdateVelocity", {"linear": {"x": 0}, "angular": {"z": turn_angle/2}})});
    } else {
      sync({request: Event("UpdateVelocity", {"linear": {"x": 0.3}, "angular": {"z": turn_angle/2}})});
    }
  }
});

ctx.bthread("perform delivery", "Delivery", function (entity) {
  while(entity.queueList.length > 0){
    var source = entity.queueList[0].source;
    var goal = entity.queueList[0].goal;
    
    source.id = String(source.x_coor) + ", " + String(source.y_coor);
    goal.id = String(goal.x_coor) + ", " + String(goal.y_coor);
    
    sync({request: Event("SetTarget", source)}, 1000);
    sync({waitFor: Event("TargetReached", source.id)});
      sync({request: Event("PubInfo", {data: "Pick up package."})}, 1000);
    sync({request: Event("SetTarget", goal)}, 1000);
    sync({waitFor: Event("TargetReached", goal.id)}); 
      sync({request: Event("PubInfo", {data: "Package delivered."})}, 1000);
    sync({request: Event("DeliveryComplete", entity.id)}, 1000);
  }
});


ctx.bthread("react to battery", "Battery", function (entity) {
  sync({request: Event("PubInfo", {data: "Battery low. Move to hub for charging."})}, 1000);
    
  var TarId = String(entity.hub_x_coor) + ", " + String(entity.hub_y_coor);
  var hub = {id: TarId, x_coor: entity.hub_x_coor, y_coor: entity.hub_y_coor};

  sync({request: Event("StoreTarget")}, 800); 
  sync({request: Event("SetTarget", hub)}, 1000); 
  sync({waitFor: Event("TargetReached", hub.id)}); 
  while(entity.battery_charge < entity.battery_capacity){
    sync({request: Event("ChargeBattery", entity.id), block: updateVelocityEventSet})
  }
  sync({request: Event("RestoreTarget")}, 1000); 
  sync({request: Event("PubInfo", {data: "Charging complete."})}, 1000);
  sync({request: Event("ChargingComplete", entity.id), block: updateVelocityEventSet})
  
});

ctx.bthread("store target", "TargetStore", function (entity) {
  while(true){    
    var target = sync({waitFor: storeTargetEventSet});
    
    sync({request: Event("PutTargetInStore", {storeId: entity.id, content: target})}, 1000);
    sync({waitFor: Event("RestoreTarget")});
    sync({request: Event("LoadTargetFromStore", {storeId: entity.id})}, 1000);
  }
});


ctx.bthread("add delivery", "DeliveryQueue", function (entity) {
  while(true){
    var e = sync({waitFor: addDeliveryEventSet});
    var del_coors = JSON.parse(e.data).poses;

    if (del_coors.length == 2){
      var delivery = {source: {x_coor: del_coors[0].position.x, y_coor: del_coors[0].position.y}, 
        goal: {x_coor: del_coors[1].position.x, y_coor: del_coors[1].position.y}};
      var del_data = {delivery_list: entity.id, delivery: delivery};
      
      sync({request: Event("EnqueueDelivery", del_data)}, 1000);
        sync({request: Event("PubInfo", {data: "Delivery enqueued."})}, 1000);
    } else {
      bp.log.info("Delivery must consist of two coordinate pairs.")
      sync({request: Event("PubInfo", {data: "Delivery must consist of two coordinate pairs."})}, 1000);
    }
  }  
});
