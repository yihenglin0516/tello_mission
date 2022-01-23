classdef drone_map
    properties
        droneobj
        cameraobj
        flag
        im
        BW
        SE
        stats
        param
        direction
    end
    methods
        function  obj=connect_drone()
            clear;
            obj.droneobj=ryze("Tello");
            obj.cameraobj=camera(obj.droneobj);
            obj.flag=0;
            obj.im=[];
            obj.BW=[];
            obj.SE = strel('square',20);
            obj.param=[0 0 0 0 ];
            obj.direction=1;
            obj.outerwall=4;
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
        function obj=control_in_map(obj)
            obj.im=snapshot(obj.cameraobj);
            if ~isempty(obj.im)
                obj.BW=createMask_map(obj.im);
                obj.BW=bwareafilt(obj.BW,1); 
                [diff,x1,x2]=find_length_map_x(obj.BW);
                z=get_depth(diff,15,1);
                center=(x1+x2)/2;
                error=480-center;  % in pixle 
                error_real=z*error/930;
                if abs(error_real)<4  & abs(80-z)<15
                    obj.flag=5;
                else 
                    disp("X");
                    disp(error_real);
                    disp("Z");
                    disp(80-z);
                    obj.param=[-set_limit(error_real),-set_limit(80-z),0,0];
                end 
            end 
        end 
        function obj=control_in_map_y(obj)
            obj.im=snapshot(obj.cameraobj);
            if ~isempty(obj.im)
                obj.BW=createMask_map(obj.im);
                obj.BW=bwareafilt(obj.BW,1); 
                [diff,y1,y2]=find_length_map_y(obj.BW);
                z=get_depth(diff,15,1);
                center=(y1+y2)/2;
                error=360-center;  % in pixle 
                error_real=z*error/930;
                if abs(error_real)<4  & abs(80-z)<15
                    obj.flag=5;
                else 
                    disp("Y");
                    disp(error_real);
                    disp("Z");
                    disp(80-z);
                    obj.param=[-set_limit(error_real),-set_limit(80-z),0,0];
                end 
            end 
        end 
        function obj=stabilize(obj)
            if obj.direction<3
                obj=obj.control_in_map(obj);
            else
                obj=obj.control_in_map_y(obj);
            end 
            rc(obj.droneobj,obj.param(1),obj.param(2),obj.param(3),obj.param(4));
        end 
        function obj=run_in_map(obj)
            obj.im=snapshot(obj.cameraobj);
            if ~isempty(obj.im)
                obj.BW=createMask_map(obj.im);
                
            end 
        end 
    end 
end 