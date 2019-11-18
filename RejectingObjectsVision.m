vrep=remApi('remoteApi');
disp('program started');
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1',19999,true,false,10000,5);
if(clientID>-1)
    disp('connected to remote API server');
end

[res1,joint]=vrep.simxGetObjectHandle(clientID, 'Prismatic_joint',vrep.simx_opmode_oneshot_wait);
[res3,visionSensor]=vrep.simxGetObjectHandle(clientID, 'Vision_sensor',vrep.simx_opmode_oneshot_wait);
[res4,cylinderOuter]=vrep.simxGetObjectHandle(clientID, 'Cylinder3',vrep.simx_opmode_oneshot_wait);
[res5,cylinderInner]=vrep.simxGetObjectHandle(clientID, 'Cylinder2',vrep.simx_opmode_oneshot_wait);

while 1
    while 1

        [returnCode,resolution,image]=vrep.simxGetVisionSensorImage2(clientID,visionSensor,0,vrep.simx_opmode_oneshot_wait);
        if max(max(image(:,:,3)))>200
            
            I=rgb2gray(image);
            kx = [-1 0 1;-2 0 2; -1 0 1];
            ky = kx';

            Gx = conv2(kx,I); 
            Gx = Gx./max(Gx(:));
            
            Gy = conv2(ky,I);
            Gy = Gy./max(Gy(:));
            

            G = sqrt(Gx.^2+Gy.^2);
            

            I1 = G>0.1;
            I2=I1(:,(3:32));
            %imshow(I2)
            
            
            if max((sum(I2)))>30;
            
                [returnCode]=vrep.simxSetJointPosition(clientID,joint,1,vrep.simx_opmode_oneshot_wait);
                [returnCode]=vrep.simxSetJointPosition(clientID,joint,0,vrep.simx_opmode_oneshot_wait);
                break
            end
        end
    end
end
