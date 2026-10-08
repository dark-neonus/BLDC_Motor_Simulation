Hello, I want to build complete bldc motor simulator. I dont knwo much about them, so for example if there is a field or a selections, it should have queston mark near it to which at hower will give short but comprehensive explanation with link at the end to docs which explain it better.

Things I want to see in and use for simulator:

1) Learn about bldc motor behavior, learn their parameters and how they affect motor behavior

2) Be able to reproduce behaviour of motor based on its specifications or parameters

3) Play around with motor with some basic controll configurations: see how it behaves under certain voltages(despite it rated voltage), current limits, rotate with different controlls, hold angles with different controlls, hold torque and angle with different load at different distance from center. Things like inertia of rotor/stator should be accounted, as well as inertia of load. Power supply need to be customable too.

4) Have some basics motor presets and ability to create new one or copy and modify existing. Parameters of them must be cross-restrained, so it will be impossible to input unmatching parameters. Presets and user models must be saved in convinient format on drive(in different locations). There should be both inner and outer rotor motors, AC and DC motors. Also presets should contain basic motos geometry like 8010, 10015, 6020, etc. Focus more on torque and low rpm motors.

5) Interface of this simulator must be convinient, nice, have nice real time motor visualization(including visualization of motor geometry, coils, which coisl are enabled, current flow size in coils visualized as moving particles, motor rotation, laod shoulder and load weight dynamically, also aability to show live plotting of different values). I think for this will best suit some sort of moder web interface. Also interface should be build that way that it would be very easy and convinient for agent to travel through and work with(in case of when I will have an instance of this web app open and I will ask you to enable something or check something, you need to do this in ease).

6) Also give ability to configure controll loop. E.g. make interface like diagrams in controll theory, where you have system input, controller, action, feedback(I dont know if they are right, so you can name other way, just conventional). Each of that type need to be configurable and if possible be able to be disabled. Some of those block(like decision making e.g. microcontroller, action maker e.g. transistors, feedback e.g. magnetic encoder) have working freqencies and they should also be configurable. For controll loop there should be a presets for usual types of controll loops with different parameters each. And for magnetic encoder for example, there should be parameter like discrete precision(e.g. number of angles it can see).

7) Since this is simulation software, it need to strictly follow algorythms and formulas, so use dedicated libraries and frameworks, find way to double check and validate formulas, maybe write some tests.

8) As I have mentioned in the beggining there should be full docs, which explain interface, math, physics, controll theory and other related stuff which can be accessed as another page or redirected via link under quick help under question mark near each field I can change. Those docs should also be saved and deployed in conventional format for such type of docs and they should also be able to easy reachable for AI agents. 

9) Design should have claude like color pallete(dark/light + orange and smooth and great UI/UX)
