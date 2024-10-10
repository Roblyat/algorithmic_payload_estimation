float64[] wrench_input  # Array of predicted efforts for each joint []1x18 feature vector, [ [pos, vel, Ypred(effort)_Joint_1] , ... , [pos, vel, Ypred(effort)_Joint_6] 
float64 variance  # Array of variances for each predicted effort
float64 timestamp  # Timestamp of the prediction