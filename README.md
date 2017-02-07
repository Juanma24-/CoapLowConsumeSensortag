# CoapLowConsumeSensortag
Developing of multicast coap over Contiki adding very low consuming characteristics on Sensortag 2.0 (cc2650).    

An effort for developing an app joining Coap and Rest engine with Ipv6 multicasting. The two funcionalities are developed in
[Contiki's examples] (URL "https://github.com/contiki-os/contiki/tree/master/examples"), however, due to the kind of application I want to develop I need to join the two examples in one adding all necesary to work properly.  
This project has been developed under [Contiki 3.x] (URL "https://github.com/contiki-os/contiki/releases/tag/3.x"), . It seems that for Sensortag there is no problem using the most recent version of Contiki however it does not happened the same in gateway. Due to this incompatibility a stable version will be released with the app.
Moreover, It will be also added OTA firmware updating. Due to the goal of this project is to manage a lot of nodes, it becomes very important to develop a method to update all nodes in a short time. OAT is the solution to this problem, in addition, it has been develop under Coap, reduccing incompatibilites, complexity and firmware size. 
