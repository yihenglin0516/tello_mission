classdef drone_qr 
    properties(SetAccess = private)
        droneobj
        cameraobj  
        flag
        id
        im
        BW
        SE
        stats
        msg
    end 
    methods 
        function  obj=connect_drone()
            clear;
            obj.droneobj=ryze("Tello");
            obj.cameraobj=camera(obj.droneobj);
            obj.flag=0;
            obj.id=0;
            obj.im=[];
            obj.BW=[];
            obj.SE = strel('square',20);
            obj.msg="";
        end
        function obj=image_processing(obj)
 
            if ~isempty(obj.im)
                obj.BW=createMask_red_cbcr(obj.im);
                obj.BW=obj.BW(1:540,:);
                obj.BW=imfill(obj.BW,'holes');                
                obj.BW=imopen(obj.BW,obj.SE);
                obj.BW=bwareafilt(obj.BW,1);
                obj.stats=regionprops(obj.BW,'basic');
            end 
        end 
        function obj=find_red(obj)
            obj.im=snapshot(obj.cameraobj);
            obj=obj.image_processing(obj);
            if ~isempty(obj.stats)
                if obj.id==8
                    if obj.stats.Area>400
                        obj.flag=1;
                        disp("running in state 1");
                        rc(obj.droneobj,0,0,0,0);
                        figure(1)
                        imshow(obj.im);
                        figure(2)
                        imshow(obj.BW);
                    end 
                     else
                        if obj.stats.Area>2000
                            obj.flag=1; 
                            disp("running in state 1");
                            rc(obj.droneobj,0,0,0,0);
                            figure(1)
                            imshow(obj.im);
                            figure(2)
                            imshow(obj.BW);
                        end
                end 
            end 
        end 
        function obj=find_qr(obj)
            obj.im=snapshot(obj.cameraobj);
            if ~isempty(obj.im)
                [~, ~, loc] = readBarcode(obj.im);
                if ~isempty(loc)
                    obj.flag=2;
                    disp("running in state 2");
                    rc(obj.droneobj,0,0,0,0);
                else
                    obj=obj.image_processing(obj);
                    if ~isempty(obj.stats)
                        x=obj.stats.Centroid(1);
                        y=obj.stats.Centroid(2);
                        error_x=480-x;
                        error_y=200-y;
                        z=60;
                        if abs(obj.stats.BoundingBox(3)-obj.stats.BoundingBox(4))<10
                            l=obj.stats.BoundingBox(3);
                            z=930*13/l;
                            fprintf("find rectangle from %f away",z);   
                        end
                        rc(obj.droneobj,-set_limit(error_x),-set_limit(60-z),set_limit(error_y),0);
                    end 
                end 
            
            end 
        end 
        function obj=qr_stable(obj)
            obj.im=snapshot(obj.cameraobj);
            if ~isempty(obj.im)
                [~, ~, loc] = readBarcode(obj.im);
                if ~isempty(loc)
                    [error_x,error_y,error_z]=find_error(loc);
                    if (abs(error_x)<8) && (abs(error_y)<8 && (abs(error_z)<8))
                        obj.flag=3;
                        disp("running in state 3");
                    else 
                        rc(obj.droneobj,-set_limit(error_x),-set_limit(error_z),set_limit(error_y),0);
                    end 
                end 
            else 
                rc(obj.droneobj,0,0,0,0);
            end 
        end 
        function obj=do_rotation(obj)
            obj.im=snapshot(obj.cameraobj);
            if ~isempty(obj.im)
                [obj.msg, ~, ~] = readBarcode(obj.im);
                if ~isempty(obj.msg)
                    obj.msg=split(obj.msg,",");
                    if obj.msg(4)=="HL"
                        rc(obj.droneobj,0,0,0,0);
                        obj.flag=4;
                        disp("running in state 4");
                    elseif obj.msg(4)=="CW"
                        turn(obj.droneobj,deg2rad(str2double(msg(5))));
                        obj.flag=4;
                        disp("running in state 4");
                    elseif msg(4)=="CCW"
                        turn(obj.droneobj,-deg2rad(str2double(msg(5))));
                        obj.flag=4;
                        disp("running in state 4");
                    end 
                end 
            end 
        end 
        function obj=move_next(obj)
            if obj.msg(3)=="L"
            rc(obj.droneobj,-10,0,0,0);
            elseif obj.msg(3)=="U"
                rc(obj.droneobj,0,0,10,0);
            elseif obj.msg(3)=="F"
                rc(obj.droneobj,0,10,0,0);
            end 
            obj.flag=5;
            disp("5");
        end 
        function obj=leave(obj)
            obj.im=snapshot(obj.cameraobj);
            obj=image_processing(obj);
            if ~isempty(obj.stats)
                if obj.stats.Area<1000
                    obj.flag=0;
                    disp("running in state 0"); 
                end  
            end 
        end 
    end 
end
    
