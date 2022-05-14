# Robocar_car_stm
System directly controlling the car hardware in Robocar project

# Main project
More information on the project available at:
https://github.com/juanma-rm/Robocar_SWP

# About
The application runs on top of FreeRTOS, splitting the process in several tasks:
* SPI: handles SPI communication as slave.
* Data center: collects information from the incoming SPI messages and the sensors and redirect it as needed to the robot control task and the outgoing SPI messages. It also prints status out via serial port for debugging.
* hcsr04: driver for hcsr04 ultrasonics sensors.
* Robot control: handles the motor control (setting speed as PWM, processing incoming orders from the user and calculating linear and angular speed).

Tasks run under round-robin policy with 1 ms of period.

# Contact <a name="Contact"></a>

[![LinkedIn][linkedin-shield]][linkedin-url]

<!-- MARKDOWN LINKS & IMAGES -->

[linkedin-shield]: https://img.shields.io/badge/LinkedIn-0077B5?style=for-the-badge&logo=linkedin&logoColor=white
[linkedin-url]: https://www.linkedin.com/in/juan-manuel-reina-mu%C3%B1oz-56329b130/
