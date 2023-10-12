#include <dds/ddsc/dds_public_impl.h>
#include <functional>
#include <memory>
#include <stdio.h>

#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/msg/num.hpp"                                       // CHANGE
#include "tutorial_interfaces/msg/ddstype.hpp"                                       // CHANGE

#include "dds/dds.h"

#include "Ddstype.h"

using std::placeholders::_1;

dds_entity_t writer;
dds_entity_t participant;

#define DOMAINID 0

#define ROS2DDS_FROM "topic1"
#define ROS2DDS_TO "topic1"

#define DDS2ROS_FROM "topic2"
#define DDS2ROS_TO "topic2"

#define DDS_SUB_PARTITION "testpart"
#define DDS_PUB_PARTITION "testpart"

static const char *partitionpub[1] = {DDS_PUB_PARTITION};
static const char *partitionsub[1] = {DDS_SUB_PARTITION};

const dds_topic_descriptor_t *ddsdesc = &tutorial_interfaces_msg_Ddstype_desc;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<tutorial_interfaces::msg::Ddstype>(    // CHANGE
      ROS2DDS_FROM, 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));    // CHANGE
      // "/MQTT/topic1", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().transient_local(), std::bind(&MinimalSubscriber::topic_callback, this, _1));    // CHANGE
  }

private:
  void topic_callback(const tutorial_interfaces::msg::Ddstype & msg) const  // CHANGE
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Get int32_test: '" << msg.int32_test << "'");     // CHANGE
	// ROS2 to DDS
	void *sample = malloc(sizeof(msg));
    memcpy(sample, &msg, sizeof(msg));
    dds_write(writer, sample);
	free(sample);
  }
  rclcpp::Subscription<tutorial_interfaces::msg::Ddstype>::SharedPtr subscription_;  // CHANGE
};

static void
dds_data_available(dds_entity_t rd, void *arg)
{
}

int main(int argc, char * argv[])
{
  uint32_t rc = 0;
  rclcpp::init(argc, argv);

  participant = dds_create_participant(DOMAINID, NULL, NULL);
  if (participant < 0)
    fprintf(stderr, "Error");
  fprintf(stderr, "YES!");

  dds_listener_t   *listener;
  /* Create a listener */
  listener = dds_create_listener(NULL);
  dds_lset_data_available_arg(listener, dds_data_available, NULL, true);

  dds_entity_t      waitSet;
  // Create waitSet
  waitSet = dds_create_waitset(participant);

  dds_qos_t        *qossub;
  dds_entity_t      subscriber;
  /* Qos for Subscriber */
  qossub = dds_create_qos();
  dds_qset_partition(qossub, 1, partitionsub);

  /* Create the Subscriber */
  subscriber = dds_create_subscriber(participant, qossub, NULL);
  if (subscriber < 0)
    DDS_FATAL("dds_create_subscriber: %s\n", dds_strretcode(-subscriber));
  dds_delete_qos(qossub);

  // Set wait set
  int status = dds_waitset_attach(waitSet, waitSet, waitSet);
  if (status < 0)
    DDS_FATAL("dds_waitset_attach: %s\n", dds_strretcode(-status));

  dds_entity_t topicr;
  /* Topic for reader */
  topicr = dds_create_topic(
    participant, ddsdesc, DDS2ROS_FROM, NULL, NULL);
  if (topicr < 0) {
    DDS_FATAL("dds_create_topic: %s\n", dds_strretcode(-topicr));
    return topicr;
  }

  dds_qos_t *qosr;
  dds_entity_t reader;
  /* Qos for Reader. */
  qosr = dds_create_qos();
  dds_qset_reliability(qosr, DDS_RELIABILITY_RELIABLE, DDS_SECS(10));

  /* Create the Reader */
  reader = dds_create_reader(subscriber, topicr, qosr, listener);
  if (reader < 0) {
    DDS_FATAL("dds_create_reader: %s\n", dds_strretcode(-reader));
    return reader;
  }
  dds_delete_qos(qosr);

  dds_entity_t publisher;
  dds_qos_t *  qospub;

  /* Qos for Publisher */
  qospub = dds_create_qos();
  dds_qset_partition(qospub, 1, partitionpub);

  /* Create the Publisher. */
  publisher = dds_create_publisher(participant, qospub, NULL);
  if (publisher < 0)
    fprintf(stderr, "Error");
  dds_delete_qos(qospub);
  fprintf(stderr, "YES!");

  dds_entity_t topicw;
  dds_qos_t *qosw;

  /* Topic for writer */
  topicw = dds_create_topic(
    participant, ddsdesc, ROS2DDS_TO, NULL, NULL);
  if (topicw < 0) {
    DDS_FATAL("dds_create_topic: %s\n", dds_strretcode(-topicw));
    return topicw;
  }

  /* Qos for Writer */
  qosw = dds_create_qos();
  dds_qset_reliability(qosw, DDS_RELIABILITY_RELIABLE, DDS_SECS(10));

  /* Create a Writer */
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
  fprintf(stderr, "YES!");

  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();

  return 0;
}
