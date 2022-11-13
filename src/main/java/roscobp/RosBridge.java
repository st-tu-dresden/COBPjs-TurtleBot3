package roscobp;

import com.google.common.collect.Maps;
import edu.wpi.rail.jrosbridge.JRosbridge;
import edu.wpi.rail.jrosbridge.Ros;
import edu.wpi.rail.jrosbridge.Service;
import edu.wpi.rail.jrosbridge.Topic;
import edu.wpi.rail.jrosbridge.callback.TopicCallback;
import edu.wpi.rail.jrosbridge.messages.Message;
import edu.wpi.rail.jrosbridge.services.ServiceRequest;
import edu.wpi.rail.jrosbridge.services.ServiceResponse;
import il.ac.bgu.cs.bp.bpjs.execution.BProgramRunner;
import il.ac.bgu.cs.bp.bpjs.execution.listeners.BProgramRunnerListenerAdapter;
import il.ac.bgu.cs.bp.bpjs.model.BEvent;
import il.ac.bgu.cs.bp.bpjs.model.BProgram;
import org.mozilla.javascript.Context;
import org.mozilla.javascript.NativeJSON;

import java.util.Map;
import java.util.Properties;

public class RosBridge {
    private final BProgram bProgram;
    private final BProgramRunner rnr; 
    private Ros ros;
    private Map<String, Topic> topics = Maps.newHashMap();
    private Map<String, Service> services = Maps.newHashMap();

    public RosBridge(BProgram bProgram, BProgramRunner rnr) {
        this.bProgram = bProgram;
        this.rnr = rnr;
        connect();
    }


    public void publish(String topic, String message) {
        topics.get(topic).publish(new Message(message));
    }

    // adds event listeners for every advertised event and publishes their data to the specified topics when they are selected
    public void advertise(String topic, String eventName) {
        rnr.addListener(new BProgramRunnerListenerAdapter() {
            @Override
            public void eventSelected(BProgram bp, BEvent event) {
                if (event.name.equals(eventName)) {
                    Context ct = Context.enter();
                    try {
                        //publish("/cmd_vel", (String) NativeJSON.stringify(ct, ct.initStandardObjects(), event.getData(), null, null));
                        publish(topic, (String) NativeJSON.stringify(ct, ct.initStandardObjects(), event.getData(), null, null));
                    } finally {
                        Context.exit();
                    }
                }
            }
        });
        topics.get(topic).advertise();
    }

    private void subscribe(String topic, TopicCallback callback) {
        topics.get(topic).subscribe(callback);
    }

    // waits for messages from the subscribed topics and creates events with the topic as the name and the message's contents as the data
    // then enqueues these events into the external event queue of the COBPjs-program
    public void subscribe(String topic) {
        topics.get(topic).subscribe(message -> bProgram.enqueueExternalEvent(new BEvent(topic, message.toString())));
    }

    public ServiceResponse callServiceAndWait(String service, String request) {
        return services.get(service).callServiceAndWait(new ServiceRequest(request));
    }

    public void addTopic(String name, String type) {
        addTopic(name, type, new Properties());
    }

    public void addTopic(String name, String type, Map<?, ?> additionalParameters) {
        Properties props = new Properties();
        additionalParameters.forEach(props::put);
        addTopic(name, type, props);
    }

    public void addTopic(String name, String type, Properties additionalParameters) {
        int throttle_rate = (int)((double) additionalParameters.getOrDefault("throttle_rate", 0.0));
        int queue_length = (int) ((double)additionalParameters.getOrDefault("queue_length", 0.0));
        JRosbridge.CompressionType compression_value = (JRosbridge.CompressionType) additionalParameters.getOrDefault("compression_value", JRosbridge.CompressionType.none);
        topics.put(name, new Topic(ros, name, type, compression_value, throttle_rate, queue_length));
    }

    public void addService(String name, String service) {
        services.put(name, new Service(ros, name, service));
    }

    private void connect() {
        System.out.println("Trying to connect rosbridge_websocket server...");
        ros = new Ros("localhost");
        if (!ros.connect()) {
            System.err.println("No connection made. Verify rosbridge_websocket is running.");
            System.exit(-1);
        }
    }

    public void disconnect() {
        ros.disconnect();
    }

    public void sampleData() {
        addTopic("/echo", "std_msgs/String");
        addTopic("/echo_back", "std_msgs/String");
        addService("/add_two_ints", "rospy_tutorials/AddTwoInts");

        subscribe("/echo_back", message -> System.out.println("From ROS: " + message.toString()));

        publish("echo", "{\"data\": \"hello, world!\"}");
        System.out.println(callServiceAndWait("/add_two_ints", "{\"a\": 10, \"b\": 20}").toString());
    }
}
