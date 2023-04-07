# Chapter 5: Network Protocols and Industrial Bus Systems - Implementing MODBUS TCP

Industrial mechatronics systems often rely on various network protocols and industrial bus systems to facilitate communication between components, such as sensors, actuators, and controllers. One widely used protocol in industrial automation is MODBUS, particularly in its TCP variant, which enables efficient and reliable data exchange over Ethernet networks. This chapter will provide an in-depth understanding of network protocols and industrial bus systems, focusing on the implementation of MODBUS TCP for seamless integration and communication in industrial mechatronics applications.

## 5.1 Industrial Bus Systems Overview
Industrial bus systems are essential for enabling communication between devices and components in industrial mechatronics applications. They provide the necessary infrastructure for transmitting data, control signals, and power, facilitating the integration and coordination of various system elements. This section presents an overview of commonly used industrial bus systems and their characteristics, highlighting the essential features and benefits they offer in industrial mechatronics.
1.	Fieldbus: Fieldbus systems are widely used for connecting sensors, actuators, and controllers in industrial automation applications. Several fieldbus standards have been developed, each with specific features and benefits, including:
    - PROFIBUS: A popular, high-speed fieldbus system that supports both cyclic and acyclic communication, making it suitable for a wide range of industrial automation applications.
    - DeviceNet: Based on the Common Industrial Protocol (CIP), DeviceNet provides robust and reliable communication for factory automation systems and is often used for device-level communication.
    - CANopen: Built on the Controller Area Network (CAN) protocol, CANopen enables efficient communication between devices in various industries, such as automotive, medical, and automation systems.
2.	Ethernet-based Protocols: With the growing demand for high-speed communication and increased data transfer requirements, Ethernet-based protocols have become increasingly popular in industrial mechatronics applications. Some notable Ethernet-based protocols include:
    - EtherCAT: A high-performance, real-time Ethernet protocol designed for industrial automation, EtherCAT provides fast and deterministic communication, making it suitable for applications requiring precise motion control.
    - PROFINET: An extension of the PROFIBUS standard, PROFINET leverages the speed and flexibility of Ethernet for communication between devices in industrial automation systems.
    - Ethernet/IP: Based on the Common Industrial Protocol (CIP), Ethernet/IP enables seamless integration of industrial devices over standard Ethernet networks, providing a unified communication solution for industrial automation systems.
3.	Wireless Protocols: In some industrial mechatronics applications, wireless protocols offer advantages such as increased flexibility, reduced wiring complexity, and support for mobile devices. Commonly used wireless protocols in industrial environments include:
    - WirelessHART: An adaptation of the HART protocol for wireless communication, WirelessHART is often used in process automation systems, providing secure and reliable communication between field devices and control systems.
    - ISA100.11a: A wireless protocol designed for industrial automation, ISA100.11a supports various communication patterns and offers robust performance in challenging industrial environments.
    - ZigBee: A low-power, mesh networking protocol, ZigBee is used in a variety of applications, including building automation, smart energy, and industrial automation.

In conclusion, a comprehensive understanding of industrial bus systems is crucial for engineers and developers working with industrial mechatronics systems. By selecting the appropriate bus system based on the specific requirements of their applications, such as speed, reliability, and network topology, engineers can optimize communication performance and ensure seamless integration of system components.

## 5.2 MODBUS Protocol and its Applications
Understanding the MODBUS protocol and its applications is essential when designing and implementing control and automation systems. MODBUS is a widely-used, simple, and robust protocol for communication between industrial devices. Developed in the late 1970s, it has become the de facto standard for many industries, such as manufacturing, energy, building automation, and transportation. This section will provide an overview of the MODBUS protocol and discuss its applications in various industrial mechatronics contexts.
1.	MODBUS Protocol Overview: MODBUS is an application-layer protocol that defines a common structure for data exchange between devices. It operates on a master-slave (or client-server) architecture, in which one master device sends requests to one or more slave devices, which then respond accordingly. The protocol supports two primary transmission modes:
    - MODBUS RTU: A binary, serial communication mode that uses compact data representation and is typically implemented over RS-232 or RS-485 physical interfaces.
    - MODBUS TCP: An extension of the MODBUS protocol for Ethernet networks, enabling communication over TCP/IP networks, providing increased speed, flexibility, and network scalability.
2.	Data Representation and Addressing: MODBUS uses a simple and consistent data representation model based on four primary data types:
    - Coils: Single-bit, discrete outputs, typically used for controlling actuators, such as relays or solenoids.
    - Discrete Inputs: Single-bit, read-only inputs, often used for reading the state of sensors or switches.
    - Input Registers: 16-bit, read-only registers, commonly used for storing analog input values from sensors or other devices.
    - Holding Registers: 16-bit, read-write registers, typically used for storing configuration parameters or setpoints.
Devices are addressed using unique identifiers within the MODBUS network, allowing for direct communication between the master and the desired slave device.
3.	Applications of MODBUS in Industrial Mechatronics: Due to its simplicity, reliability, and widespread adoption, MODBUS has been applied in various industrial mechatronics applications, including:
    - Manufacturing: In assembly lines, MODBUS enables communication between programmable logic controllers (PLCs), sensors, and actuators to coordinate the production process.
    - Energy Management: MODBUS is commonly used for communication between energy meters, power distribution units, and supervisory control and data acquisition (SCADA) systems to monitor and control energy consumption.
    - Building Automation: MODBUS facilitates communication between HVAC systems, lighting controllers, and other building management systems to optimize building performance and energy efficiency.
    - Transportation: In railway and traffic control systems, MODBUS enables communication between control systems, signaling equipment, and monitoring devices to ensure safe and efficient transportation operations.
In conclusion, the MODBUS protocol is a critical component in many industrial mechatronics applications, offering a simple and reliable method for communication between devices. By understanding the fundamentals of the protocol and its applications, engineers and developers can effectively leverage MODBUS to design and implement efficient and robust industrial mechatronics systems.

## 5.3 Implementing MODBUS TCP

MODBUS TCP is an extension of the MODBUS protocol that operates over Ethernet networks, leveraging the advantages of TCP/IP for communication. In this section, we will discuss the implementation of MODBUS TCP in the C programming language, providing examples for both client (master) and server (slave) devices.
1.	Prerequisites: To implement MODBUS TCP in C, the libmodbus library is required. This library provides a comprehensive set of functions for implementing MODBUS communication and can be installed on most systems using package managers or by building from source.
2.	MODBUS TCP Client (Master) Example:
```c
#include <stdio.h>
#include <modbus/modbus.h>

int main(void) {
    modbus_t *ctx;
    uint16_t tab_reg[32];
    int rc;

    ctx = modbus_new_tcp("192.168.1.10", 502); // Connect to server (slave) at IP 192.168.1.10 and port 502

    if (modbus_connect(ctx) == -1) {
        fprintf(stderr, "Connection failed: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        return -1;
    }

    // Read 10 holding registers starting from address 0
    rc = modbus_read_registers(ctx, 0, 10, tab_reg);

    if (rc == -1) {
        fprintf(stderr, "%s\n", modbus_strerror(errno));
        return -1;
    }

    for (int i = 0; i < 10; i++) {
        printf("reg[%d]=%d (0x%X)\n", i, tab_reg[i], tab_reg[i]);
    }

    modbus_close(ctx);
    modbus_free(ctx);

    return 0;
}
```
3.	MODBUS TCP Server (Slave) Example:
```c
#include <stdio.h>
#include <modbus/modbus.h>

int main(void) {
    modbus_t *ctx;
    int socket;

    ctx = modbus_new_tcp("127.0.0.1", 502); // Bind to IP 127.0.0.1 and port 502
    modbus_set_slave(ctx, 1); // Set the slave ID to 1

    socket = modbus_tcp_listen(ctx, 1); // Listen for incoming connections

    for (;;) {
        modbus_tcp_accept(ctx, &socket);

        uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];
        int rc;

        rc = modbus_receive(ctx, query);

        if (rc > 0) {
            modbus_reply(ctx, query, rc, NULL); // Reply to the client (master) using the default mapping
        } else if (rc == -1) {
            break;
        }
    }

    modbus_close(ctx);
    modbus_free(ctx);

    return 0;
}
```

These examples illustrate the basic structure of MODBUS TCP client and server implementations in C using the libmodbus library. By modifying and extending these examples, developers can implement custom MODBUS TCP communication for their specific industrial mechatronics applications.
