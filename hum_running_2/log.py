filename = "log.txt"
with open("log.txt", 'w') as file_object:
    file_object.write("self.delay_pwm",self.delay_pwm)
    file_object.write("\n")
    file_object.write("left_back_dist",left_back_dist)
    file_object.write("\n")
    file_object.write("left_front_dist",left_front_dist)
    file_object.write("\n")
    file_object.write("self.parallel_yaw",self.parallel_yaw)
    file_object.write("\n")
