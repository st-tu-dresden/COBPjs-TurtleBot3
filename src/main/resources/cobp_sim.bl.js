/* This file contains the behavioral code of the COBPjs application (behavioral layer).
It consists mainly of 
EventSets (bp.EventSet(name, function(event){})), 
B-Threads (bthread(name, function(){}))
and Context-aware B-Threads (CBTs) (ctx.bthread(name, query_name, function(entity){})).
*/



/* the following lines define EventSets. They are used to group Events 
and reference them.   
In this case they are manly used to group Events with the same name but different data.
The first argument is the name of the set (they are left as empty strings)
The second argument is a function that takes an event as an parameter and returns true 
when the event is part of the Event set or false otherwise.
*/
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

/* This is a "normal" B-Thread (not a context aware one). 
It is executed once when the application is started and defines the topics that are used to
communicate with ROS. The "ros" object on which the methods are called is an instance of the 
RosBridge class which is defined in the "RosBridge.java" file. It is made accessible by being 
put in the global scope of the COBPjs application and realizes the actual communication with ROS 
and the TurtleBot.
ros.addTopic specifies the topics that are used and their expected message type.
The subscribed topics create external Events that have the name of the topic and its corresponding data.
Advertised topics wait for the selection of events with the name that is specified in the second argument 
of the advertise method and publish its data to the specified topic.
*/
bthread("init", function () {
    var properties = {"queue_length": 10, "throttle_rate": 125}; 
    ros.addTopic("/cmd_vel", "geometry_msgs/Twist", properties);
    ros.addTopic("/odom", "nav_msgs/Odometry", properties);
    ros.addTopic("/scan", "sensor_msgs/LaserScan", properties);
    ros.subscribe("/scan");
    ros.subscribe("/odom");
    ros.advertise("/cmd_vel", "UpdateVelocity");
    
    // additional topics that allow to interact with the TurtleBot
    ros.addTopic("/delivery_info", "std_msgs/String", properties);
    ros.advertise("/delivery_info", "PubInfo");
    ros.addTopic("/add_del", "geometry_msgs/PoseArray", properties);
    ros.subscribe("/add_del");
});

/*
This is a CBT that is bound to the query "Robot". 
It waits for the scan data (distance to obstacles) of the turtleBot and parses it. 
It then requests an Event that contains this data. 
The data is added to the context by the effect of the "ScanData" which is defined 
in the data-access layer. 
*/
ctx.bthread("update scan data", "Robot", function (entity) {
  while (true) {
      var e = bp.sync({waitFor: scanEventSet});
      var data = JSON.parse(e.data);
      // array with 360 values corresponding to the distance at that angle
      var scan_data = {ranges: data.ranges, range_max:data.range_max};
     
      sync({request: Event("ScanData", scan_data)});
      sync({request: Event("ScanCompleted")});
  }
}); 

// Similar to the "update scan data" CBT but for the odometry data.
ctx.bthread("update position", "Robot", function (entity) {
  while (true) {
    var e = bp.sync({waitFor: odomEventSet});
    var position = JSON.parse(e.data).pose.pose.position;


    /* transforms the quaternion data into "radiants"
      front = 0
      back = pi
      left = pi/2
      right = -pi/2
    */
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

/* enforces the alternation between scans, odometry and movement events.
This is necessary because the movement events are internal while the scan and 
odometry events are external. Since internal event are prioritized over external once,
the movement events might prevent the processing of scan data. 
It also guarantees an equal occurrence of these events.
 */
ctx.bthread("toggle scan and move", "Robot", function (entity) {
  while(true) {
    sync({waitFor: scanDataEventSet, block:[updateVelocityEventSet, positionEventSet]});
    sync({waitFor: positionEventSet, block:[updateVelocityEventSet, scanDataEventSet]});
    sync({waitFor: updateVelocityEventSet, block:[scanDataEventSet, positionEventSet]});
  }
});

/* ####################################################################
The following code is responsible for the collision avoidance.

The following 4 CBTs realize the movement during the collision avoidance 
by requesting movement events into their respective directions.
They are bound to the "Obstacle" so their Live-Copies are only active when 
there is an obstacle that needs to be avoided. 
The second argument of their sync statements is an integer (100, 50, 10) and
defines the priority of the events (higher number means higher priority). 
So the event selection will always select the forward movement unless it is 
blocked. The turn events are equally likely to be selected while the backwards 
movement can only occur when the other options are all blocked. 
*/
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

/* These next Cbts are bound to queries that check whether their is an obstacle 
in a specific direction. If that is the case, Live-Copies of these CBTs are 
spawned that block the movement in that direction. Once the obstacle is no 
longer around, the query has no more results and the Live-Copies are 
automatically terminated.  
*/
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

// blocks the the movement towards the target while collision avoidance is active.
ctx.bthread("prioritise obstacle avoidance", "Obstacle", function (entity) {
  sync({block: Event("AdvanceToTarget")});
});


/* ####################################################################
The following code is responsible for performing deliveries.

The following CBT realizes the movement towards the target.
Its seed is accessible through the "entity" parameter which consists of an 
object that contains the id of the target, the coordinates of the target and 
the robot as well as the orientation of the robot.
*/
ctx.bthread("move to target", "Target", function (entity) {
  // converts the values of the atan2 function into proper radiants 
  function rad(x){
    return (x >= 0) ? x : (2*Math.PI + x);
  }
  
  /* calculates the difference between the current orientation of the robot 
  and the orientation that is needed to face the target. Takes radiants as arguments 
  and return an radiant.
  */ 
  function angle_difference(source, target){
    diff = target - source;
    if(diff > Math.PI){diff = diff - 2*Math.PI};
    if(diff < -Math.PI){diff = diff + 2*Math.PI};
    return diff;
  }

  while(true) {
    sync({request: Event("AdvanceToTarget")});

    // calculations to determine how the robot needs to turn
    var angle_of_robot = entity.rob_orientation;
    var angle_to_target = Math.atan2(entity.tar_y_coor-entity.rob_y_coor, entity.tar_x_coor - entity.rob_x_coor);
   
    var turn_angle = angle_difference(rad(angle_of_robot), rad(angle_to_target))
    var turn_angle_abs = Math.abs(turn_angle);
    
    // if the robot reached the target coordinates (+/-0.1) 
    if((Math.abs(entity.tar_x_coor - entity.rob_x_coor) < 0.1 ) && (Math.abs(entity.tar_y_coor - entity.rob_y_coor) < 0.1 )){
      // robot is stopped
      sync({request: Event("UpdateVelocity", {"linear": {"x": 0.0}, "angular": {"z": 0}})});
        // This event is used to publish information about the current action of the robot. It does not effect the behavior of the program and is indented to signify that.
        sync({request: Event("PubInfo", {data: "Reached target at coordinates: x: " + String(entity.tar_x_coor) + " y: " + String(entity.tar_y_coor)})}, 1000);
      // the effect of this event removes the target-entity from the context and thereby terminates this Live-Copy 
      sync({request: Event("TargetReached", entity.id)}, 1000);
    }
    
    /* If the robot is not facing the target (+/-0.2), the robot turns towards it. 
    Otherwise it moves towards it.
    The angular momentum is determined by the angle difference between the robot and the target. 
    So the robot turns faster or slower depending of how far it has to turn. 
    This allows for a smooth turn and prevents overswinging.
    While moving towards the target, the robot also uses angular movement to correct the trajectory. 
     */
    if(turn_angle_abs > 0.2) {
      sync({request: Event("UpdateVelocity", {"linear": {"x": 0}, "angular": {"z": turn_angle/2}})});
    } else {
      sync({request: Event("UpdateVelocity", {"linear": {"x": 0.3}, "angular": {"z": turn_angle/2}})});
    }
  }
});

// CBT performs the deliveries in the delivery list 
ctx.bthread("perform delivery", "Delivery", function (entity) {
  // could also be while(true) since the Live-Copy terminates when there are no more delivery orders
  while(entity.queueList.length > 0){
    // selects the first entry in the delivery list
    var source = entity.queueList[0].source;
    var goal = entity.queueList[0].goal;
    
    /* Creates an id (string) that is used for the target entity that is added to the context 
    when the "SetTarget" event is selected. It is used to reference the specific target. 
    */
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

/* ####################################################################
The following code is responsible for reacting to a low battery. 
The charge of the battery is simulated and is decreased at every movement event.
When the battery is low the current target is stored in another context entity 
and the new target is set to the coordinates of an designated hub. 
At the hub the robot is recharged and once the charging is completed the old target is restored
*/
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

/* CBT that realizes the storing and restoring of the target.
It has access to the target store Entity through its seed / query. 
*/
ctx.bthread("store target", "TargetStore", function (entity) {
  while(true){    
    var target = sync({waitFor: storeTargetEventSet});
    
    sync({request: Event("PutTargetInStore", {storeId: entity.id, content: target})}, 1000);
    sync({waitFor: Event("RestoreTarget")});
    sync({request: Event("LoadTargetFromStore", {storeId: entity.id})}, 1000);
  }
});


/* This CBT allows to add delivery orders through a ROS-Topic. 
It waits for the an event that contains the delivery data and adds that to the 
end of the delivery queue. 
Since the used message type is an array of Poses it is required that it ahs a 
length of 2, one pair of coordinates for the start and one for the end point.
*/
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
