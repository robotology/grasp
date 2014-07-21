
% Copyright: (C) 2010 RobotCub Consortium
% Authors: Vadim Tikhanoff
% CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

LoadYarp;
import yarp.Port
import yarp.BufferedPortBottle
import yarp.Bottle

clear
done=0;
counter=1;

port=BufferedPortBottle;        %port for reading "quit" signal
port.close;                     %first close the port just in case

disp('Opening port: /matlab/read');
port.open('/matlab/read');
disp('The program closes when ''quit'' is received');

while(~done)
    b=Bottle;
    b = port.read( false );%use false to have a non blocking port
    if (sum(size(b)) ~= 0) %check size of bottle 
        disp('received command: ');
        %disp(b.toString_c());
        if (strcmp(b.toString_c(), 'quit'))
            disp('Closing matlab ports...');
            done=1;
        elseif (strcmp(b.toString_c(), 'plot'))
            disp ('should save:');
            disp (counter-1);
            disp ('sequences');
            for j=1:counter-1
                drawResults(joints(j,:), ee(j,:), axisangle(j,:), center, c1(j,:), c2(j,:), c3(j,:), n1(j,:), n2(j,:), n3(j,:), hand(j,:), cloud);
            end
            
            drawResults(best_joints, best_ee, best_axisangle, center, best_c1, best_c2, best_c3, best_n1, best_n2, best_n3, best_hand, cloud);
            
            counter = 1;
            clear joints ee axisangle center dim c1 c2 c3 n1 n2 n3 rotmat hand cloud;
        elseif (strcmp(b.get(0).asString(), 'best'))
            main=Bottle;
            main=b.get(1).asList();
            
            best_joints=zeros(1,size(main.find('joints').asList()));
            for j=0:size(main.find('joints').asList())-1
               best_joints(j+1)=main.find('joints').asList().get(j).asDouble();
            end
            
            best_ee=zeros(1,size(main.find('ee').asList()));
            for j=0:size(main.find('ee').asList())-1
                best_ee(j+1)=main.find('ee').asList().get(j).asDouble();
            end
            
            best_axisangle=zeros(1,size(main.find('axisangle').asList()));
            for j=0:size(main.find('axisangle').asList())-1
                best_axisangle(j+1)=main.find('axisangle').asList().get(j).asDouble();
            end
            
            best_c1=zeros(1,size(main.find('c1').asList()));
            for j=0:size(main.find('c1').asList())-1
                best_c1(j+1)=main.find('c1').asList().get(j).asDouble();
            end

            best_c2=zeros(1,size(main.find('c2').asList()));
            for j=0:size(main.find('c2').asList())-1
                best_c2(j+1)=main.find('c2').asList().get(j).asDouble();
            end

            best_c3=zeros(1,size(main.find('c3').asList()));
            for j=0:size(main.find('c3').asList())-1
                best_c3(j+1)=main.find('c3').asList().get(j).asDouble();
            end

            best_n1=zeros(1,size(main.find('n1').asList()));
            for j=0:size(main.find('n1').asList())-1
                best_n1(j+1)=main.find('n1').asList().get(j).asDouble();
            end

            best_n2=zeros(1,size(main.find('n2').asList()));
            for j=0:size(main.find('n2').asList())-1
                best_n2(j+1)=main.find('n2').asList().get(j).asDouble();
            end

            best_n3=zeros(1,size(main.find('n3').asList()));
            for j=0:size(main.find('n3').asList())-1
                best_n3(j+1)=main.find('n3').asList().get(j).asDouble();
            end
            
            tmp=Bottle;
            tmp=main.find('hand').asList().get(0).asString();
            best_hand = tmp.toString;
        else
            disp('now processing bottle:');
            main=Bottle;
            main=b.get(0).asList();
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            for j=0:size(main.find('joints').asList())-1
                joints(counter,j+1)=main.find('joints').asList().get(j).asDouble();
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            for j=0:size(main.find('ee').asList())-1
                ee(counter,j+1)=main.find('ee').asList().get(j).asDouble();
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            for j=0:size(main.find('axisangle').asList())-1
                axisangle(counter,j+1)=main.find('axisangle').asList().get(j).asDouble();
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            for j=0:size(main.find('c1').asList())-1
                c1(counter,j+1)=main.find('c1').asList().get(j).asDouble();
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            for j=0:size(main.find('c2').asList())-1
                c2(counter,j+1)=main.find('c2').asList().get(j).asDouble();
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            for j=0:size(main.find('c3').asList())-1
                c3(counter,j+1)=main.find('c3').asList().get(j).asDouble();
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            for j=0:size(main.find('n1').asList())-1
                n1(counter,j+1)=main.find('n1').asList().get(j).asDouble();
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            for j=0:size(main.find('n2').asList())-1
                n2(counter,j+1)=main.find('n2').asList().get(j).asDouble();
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            for j=0:size(main.find('n3').asList())-1
                n3(counter,j+1)=main.find('n3').asList().get(j).asDouble();
            end

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            tmp=Bottle;
            tmp=main.find('hand').asList().get(0).asString();
            hand(counter) = tmp.toString;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            if (counter == 1)

                x3=size(main.find('center').asList());
                center=zeros(1,x3);
                for j=0:x3-1
                    center(j+1)=main.find('center').asList().get(j).asDouble();
                end

                x4=size(main.find('dim').asList());
                dim=zeros(1,x4);
                for j=0:x4-1
                    dim(j+1)=main.find('dim').asList().get(j).asDouble();
                end
                
                x11=size(main.find('rotmat').asList());
                vec=zeros(1,x11);
                for j=0:x11-1
                    vec(1,j+1)=main.find('rotmat').asList().get(j).asDouble();
                end
                rotmat = vec2mat(vec,3);
                
                x12 =size(main.find('cloud').asList());
                index =size(main.find('cloud').asList().get(0).asList());
                cloudVec=zeros(x12,index);
                for j=0:x12-1
                    for i=0:index-1
                        cloudVec(j+1,i+1)=main.find('cloud').asList().get(j).asList().get(i).asDouble();
                    end
                end
                cloud = vec2mat(cloudVec,6);
            end
            disp (counter);
            counter = counter + 1;
            
            disp('finished processing yarp list...');
        end
    else
        pause(0.001);
    end
end
port.close;
disp('done...');