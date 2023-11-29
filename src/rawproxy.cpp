#include <dds/ddsc/dds_public_impl.h>
#include <functional>
#include <memory>
#include <stdio.h>

#include "rclcpp/rclcpp.hpp"
#include "dds/dds.h"

using std::placeholders::_1;

dds_entity_t writer;
dds_entity_t participant;

#include "Ddstype.h" // CHANGE
#include "tutorial_interfaces/msg/ddstype.hpp" // CHANGE

#define NAMESPACE tutorial_interfaces::msg
#define DDSTYPE Ddstype
#define DDSTYPE_DESC tutorial_interfaces_msg_Ddstype_desc

#define MAX_SAMPLES 1

#define DOMAINID 0

#define ROS2DDS_FROM "/MQTTCMD/topic1"
#define ROS2DDS_TO "topic1"

#define DDS2ROS_FROM "topic2"
#define DDS2ROS_TO "/MQTT/topic1"

#define DDS_SUB_PARTITION "testpart"
#define DDS_PUB_PARTITION "testpart"

static const char *partitionpub[1] = {DDS_PUB_PARTITION};
static const char *partitionsub[1] = {DDS_SUB_PARTITION};

const dds_topic_descriptor_t *ddsdesc = &DDSTYPE_DESC;

class DDSRawProxy: public rclcpp::Node
{
public:
  DDSRawProxy()
  : Node("ROS2_DDS_Raw_Proxy")
  {
    subscription_ = this->create_subscription<NAMESPACE::DDSTYPE>(
      ROS2DDS_FROM, 10, std::bind(&DDSRawProxy::topic_callback, this, _1));
      // "/MQTT/topic1", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().transient_local(), std::bind(&DDSRawProxy::topic_callback, this, _1));    // QOS
    publisher_ = this->create_publisher<NAMESPACE::DDSTYPE>(DDS2ROS_TO, 10);
    RCLCPP_INFO_STREAM(this->get_logger(), "ROS2 Node Inited");
  }

  void sendmsg(const NAMESPACE::DDSTYPE &msg) {
    publisher_->publish(msg);
  }

private:
  void topic_callback(const NAMESPACE::DDSTYPE & msg) const
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "ROS2 ===> DDS. int32_test " << msg.int32_test);
	// ROS2 to DDS
	void *sample = malloc(sizeof(msg));
    memcpy(sample, &msg, sizeof(msg));
    dds_write(writer, sample);
	free(sample);
  }
  rclcpp::Subscription<NAMESPACE::DDSTYPE>::SharedPtr subscription_;
  rclcpp::Publisher<NAMESPACE::DDSTYPE>::SharedPtr publisher_;
};

static void
dds_data_available(dds_entity_t rd, void *arg)
{
  int rc = 0;
  DDSRawProxy *rawproxy = reinterpret_cast<DDSRawProxy *>(arg);
  dds_sample_info_t infos[MAX_SAMPLES];

  void *samples[MAX_SAMPLES];
  samples[0] = malloc(sizeof(NAMESPACE::DDSTYPE));

  rc = dds_take(rd, samples, infos, MAX_SAMPLES, MAX_SAMPLES);
  if (rc < 0)
    DDS_FATAL("dds_take: %s\n", dds_strretcode(-rc));

  if ((rc > 0) && (infos[0].valid_data)) {
    // DDS to ROS2
	NAMESPACE::DDSTYPE msg;
    memcpy(&msg, samples[0], sizeof(msg));
    fprintf(stderr, "ROS2 <=== DDS. int32_test %d\n", msg.int32_test);
 
    rawproxy->sendmsg(msg);
  }
}

int main(int argc, char * argv[])
{
  uint32_t rc = 0;
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto rawproxy = std::make_shared<DDSRawProxy>();

  participant = dds_create_participant(DOMAINID, NULL, NULL);
  if (participant < 0) {
    DDS_FATAL("dds_create_participant: %s\n", dds_strretcode(-participant));
    return participant;
  }

  dds_listener_t   *listener;
  // Create a listener
  listener = dds_create_listener(NULL);
  dds_lset_data_available_arg(listener, dds_data_available, &(*rawproxy), true);

  dds_entity_t      waitSet;
  // Create waitSet
  waitSet = dds_create_waitset(participant);

  dds_qos_t        *qossub;
  dds_entity_t      subscriber;
  // Qos for Subscriber
  qossub = dds_create_qos();
  dds_qset_partition(qossub, 1, partitionsub);

  // Create the Subscriber
  subscriber = dds_create_subscriber(participant, qossub, NULL);
  if (subscriber < 0)
    DDS_FATAL("dds_create_subscriber: %s\n", dds_strretcode(-subscriber));
  dds_delete_qos(qossub);

  // Set wait set
  int status = dds_waitset_attach(waitSet, waitSet, waitSet);
  if (status < 0)
    DDS_FATAL("dds_waitset_attach: %s\n", dds_strretcode(-status));

  dds_entity_t topicr;
  // Topic for reader
  topicr = dds_create_topic(
    participant, ddsdesc, DDS2ROS_FROM, NULL, NULL);
  if (topicr < 0) {
    DDS_FATAL("dds_create_topic: %s\n", dds_strretcode(-topicr));
    return topicr;
  }

  dds_qos_t *qosr;
  dds_entity_t reader;
  // Qos for Reader.
  qosr = dds_create_qos();
  dds_qset_reliability(qosr, DDS_RELIABILITY_RELIABLE, DDS_SECS(10));

  // Create the Reader
  reader = dds_create_reader(subscriber, topicr, qosr, listener);
  if (reader < 0) {
    DDS_FATAL("dds_create_reader: %s\n", dds_strretcode(-reader));
    return reader;
  }
  dds_delete_qos(qosr);

  dds_entity_t publisher;
  dds_qos_t *  qospub;

  // Qos for Publisher
  qospub = dds_create_qos();
  dds_qset_partition(qospub, 1, partitionpub);

  // Create the Publisher.
  publisher = dds_create_publisher(participant, qospub, NULL);
  if (publisher < 0) {
    DDS_FATAL("dds_create_publisher: %s\n", dds_strretcode(-publisher));
    return publisher;
  }
  dds_delete_qos(qospub);

  dds_entity_t topicw;
  dds_qos_t *qosw;

  // Topic for writer
  topicw = dds_create_topic(
    participant, ddsdesc, ROS2DDS_TO, NULL, NULL);
  if (topicw < 0) {
    DDS_FATAL("dds_create_topic: %s\n", dds_strretcode(-topicw));
    return topicw;
  }

  // Qos for Writer
  qosw = dds_create_qos();
  dds_qset_reliability(qosw, DDS_RELIABILITY_RELIABLE, DDS_SECS(10));

  // Create a Writer
  writer = dds_create_writer(publisher, topicw, qosw, NULL);
  if (writer < 0) {
    DDS_FATAL("dds_create_writer: %s\n", dds_strretcode(-writer));
    return writer;
  }
  dds_delete_qos(qosw);

  rc = dds_set_status_mask(writer, DDS_PUBLICATION_MATCHED_STATUS);
  if (rc != DDS_RETCODE_OK) {
    DDS_FATAL("dds_set_status_mask: %s\n", dds_strretcode(-rc));
    return rc;
  }
  fprintf(stderr, "DDS Node Inited!\n");

  executor.add_node(rawproxy);
  executor.spin();
  rclcpp::shutdown();

  return 0;
}
