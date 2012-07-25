function [dpR,dxR,P1,P2,dpL,dxL] = fcn(   pH, pTOR, rTOR, ...
                                pLA1, rLA1, pLA2, rLA2, ...
                                pRA1, rRA1, pRA2, rRA2, ...
                                pRL1, rRL1, pLL1, rLL1)
%#eml

    pTypeCuboid=struct( 'dim',      [0 0 0]',...             %dimensions X Y Z
                        'pos',      [0 0 0]',...             %tranlation
                        'R',        [1 0 0; 0 1 0; 0 0 1]); %rotation
    pTypeCylinder=struct('height',  0,...                   %dimensions R H
                        'radius',   0, ...
                        'pos',      [0 0 0]',...             %tranlation
                        'R',        [1 0 0; 0 1 0; 0 0 1]); %rotation
    pTypeSphere=struct( 'radius',   0, ...
                        'pos',      [0 0 0]');               %tranlation


      
    Body=pTypeCuboid;
    Body.dim=[0.205 0.144 0.18]';
    Body.pos=pTOR;
    Body.R= rTOR;
    
    
    
    Head=pTypeCylinder;
    Head.radius=0.08;
    Head.height=0.001;
    Head.pos=pH;
    
    %arms
    RA1=pTypeCylinder;
    RA1.pos=pRA1;
    RA1.R=  rRA1;
    RA1.height=0.111;
    RA1.radius=0.035;
    
    RA2=pTypeCylinder;
    RA2.pos=pRA2;
    RA2.R=  rRA2;
    RA2.height=0.131;
    RA2.radius=0.035;
    
    LA1=pTypeCylinder;
    LA1.pos=pLA1;
    LA1.R=  rLA1;
    LA1.height=0.111;
    LA1.radius=0.035;
    
    LA2=pTypeCylinder;
    LA2.pos=pLA2;
    LA2.R=  rLA2;
    LA2.height=0.131;
    LA2.radius=0.035;
    
    %legs
    RL1=pTypeCuboid;
    RL1.dim=[0.18 0.1 0.05]';
    RL1.pos=pRL1;
    RL1.R=  rRL1;
    
    LL1=pTypeCuboid;
    LL1.dim=[0.18 0.1 0.05]';
    LL1.pos=pLL1;
    LL1.R=  rLL1;
    
    % closest point on first object's surface
    criticals=zeros(3,10);
    % closest point on second object's surface
    remotes=zeros(3,10);
    % distances ()
    dist=ones(1,4);
    
   
    % % % % % % % % % % RIGHT LOWER ARM
    index=1;
    [remotes(:,index) criticals(:,index)]=Cylinder2Cylinder(LA2,RA2);  index=index+1;
    [remotes(:,index) criticals(:,index)]=Cylinder2Cylinder(Head,RA2);  index=index+1;
    [~, remotes(:,index) criticals(:,index) ]=Cuboid2Cylinder(Body,RA2);index=index+1;
    [~, remotes(:,index) criticals(:,index)]=Cuboid2Cylinder(RL1,RA2); index=index+1;
    
    %   calc distances  
    for z=1:4
        dist(z)=norm(criticals(:,z)-remotes(:,z));
    end
    
    %     find the closest
    [~, dminindx]=min(dist(1:4));
    Point1=criticals(:,dminindx);
    Point2=remotes(:,dminindx);
    
    if norm(Point1-Point2)>0
        dpR=((Point1-Point2)/norm(Point1-Point2))';
        dmin=0.02;
        dxR=(dmin/dist(dminindx))^5;
        dxR=dxR/100;
    else
        % shouldn't happen (stop simulation here ?)
        dpR=[0 0 0];
        dxR=0.0001;
    end
    
    %animation output
    P1=Point1;
    P2=Point2;
    
    % % % % % % % % % LEFT ARM
    index=1;
    [remotes(:,index) criticals(:,index)]=Cylinder2Cylinder(RA2,LA2);  index=index+1;
    [remotes(:,index) criticals(:,index)]=Cylinder2Cylinder(Head,LA2);  index=index+1;
    [~, remotes(:,index) criticals(:,index) ]=Cuboid2Cylinder(Body,LA2);index=index+1;
    [~, remotes(:,index) criticals(:,index)]=Cuboid2Cylinder(LL1,LA2); index=index+1;
%     
    for z=1:4
        dist(z)=norm(criticals(:,z)-remotes(:,z));
    end
%     dist(2)=dist(2)*1.3;
    [~, dminindx]=min(dist(1:4));
    Point1=criticals(:,dminindx);
    Point2=remotes(:,dminindx);
    
    if norm(Point1-Point2)>0
        dpL=((Point1-Point2)/norm(Point1-Point2))';
        dmin=0.02;
        dxL=(dmin/dist(dminindx))^5;
        dxL=dxL/100;
    else
        % shouldn't happen (stop simulation here ?)
        dpL=[0 0 0];
        dxL=0.0001;
    end
    
    %animation output
    if norm(P1-P2)>norm(Point1-Point2)
        P1=Point1;
        P2=Point2;
    end
    
 
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
function [Corners1 Point Remote]=Cuboid2Cylinder(cuboid,cylinder)
    % prototypes
    pTypeCorner=struct( 'position',  [0 0 0]);
    pTypeEdge=struct(   'cID1',      0,...
                        'cID2',      0,...
                        't1',        [0 0 0],...
                        't2',        [0 0 0],...
                        'critical',  [0 0 0],...
                        'isInCorner', 0,...
                        'remote',    [0 0 0]);
    pTypeFace=struct(   'cornerIDs',    [0 0 0 0],... 
                        'edgeIDs',      [0 0 0 0],... 
                        'conEdges',     [0 0 0 0 1],... 4 edges touching the face and a direction bit 
                        'Edges',        [pTypeEdge pTypeEdge pTypeEdge pTypeEdge],...
                        'R',            eye(3,3),...
                        'LocalInCorners', 0,...
                        'OtherInCorners', 0);
    % initialization
    Corners=[pTypeCorner,pTypeCorner,pTypeCorner,pTypeCorner,pTypeCorner,pTypeCorner,pTypeCorner,pTypeCorner]';                
    Edges=[pTypeEdge,pTypeEdge,pTypeEdge,pTypeEdge,pTypeEdge,pTypeEdge,pTypeEdge,pTypeEdge,pTypeEdge,pTypeEdge,pTypeEdge,pTypeEdge]';
    Faces=[pTypeFace,pTypeFace,pTypeFace,pTypeFace,pTypeFace,pTypeFace]';

    % % % get 8 corners
    ijk=zeros(8,3);
    cornerID=0;
    for i=-1:2:1
        for j=-1:2:1
            for k=-1:2:1
                cornerID=cornerID+1;
                ijk(cornerID,:)=[i j k];
                Corners(cornerID).position = ...
                    (cuboid.pos+cuboid.R*(ijk(cornerID,:)'.*cuboid.dim/2))';
    %             cuboidPrint(cornerID,:)= Corners(cornerID).position;
            end
        end
    end
    % % % get 12 edges
    edgeID=0;
    for i=[1 2 3 4]
        j=i+4;
        edgeID=edgeID+1;
        Edges(edgeID).t1=Corners(i).position;
        Edges(edgeID).t2=Corners(j).position;
        Edges(edgeID).cID1=i;
        Edges(edgeID).cID2=j;  
    end
    for i=[1 5 2 6]
        j=i+2;
        edgeID=edgeID+1;
        Edges(edgeID).t1=Corners(i).position;
        Edges(edgeID).t2=Corners(j).position;
        Edges(edgeID).cID1=i;
        Edges(edgeID).cID2=j;  
    end
    for i=[1 3 5 7]
        j=i+1;
        edgeID=edgeID+1;
        Edges(edgeID).t1=Corners(i).position;
        Edges(edgeID).t2=Corners(j).position;
        Edges(edgeID).cID1=i;
        Edges(edgeID).cID2=j;  
    end
    % find distances
    for i=1:12
        [Edges(i).critical ...
         Edges(i).remote ...
         Edges(i).isInCorner]=line2line(...
                [Edges(i).t1;Edges(i).t2],getCylinderSegment(cylinder));
    end

    % % % assemble 6 faces
    % there actually is some logic in this :)
    FaceCornerIdx=  [1  3 4 2; 6  8 7  5; 1 2  6 5;  7 8  4 3;  1  5  7  3;  4  8  6 2];
    FaceEdgeIdx=    [5 10 7 9; 8 12 6 11; 9 2 11 1; 12 4 10 3;  1  6  3  5;  4  8  2 7];
    FaceConEdgeIdx= [1  3 4 2; 2  4 3  1; 5 7  8 6;  6 8  7 5;  9 11 12 10; 10 12 11 9];
    FaceConEdgeDir= [1 -1 1 -1 1 -1];

    for i=1:6
        for j=1:4
            Faces(i).cornerIDs(j)=  FaceCornerIdx(i,j);
            Faces(i).edgeIDs(j)=    FaceEdgeIdx(i,j);     
            Faces(i).conEdges(j)=   FaceConEdgeIdx(i,j);
            Faces(i).Edges(j)=      Edges(FaceEdgeIdx(i,j));
        end
        Faces(i).conEdges(5)=FaceConEdgeDir(i);
    end
    Faces(1).R=eye(3,3);
    Faces(2).R=-Faces(1).R;
    Faces(3).R=[0 1 0; 0 0 1; 1 0 0];
    Faces(4).R=-Faces(3).R;
    Faces(5).R=[0 0 1; 1 0 0; 0 1 0];
    Faces(6).R=-Faces(5).R;

    % % % FINDING THE PAIR OF POINTS
    Point=[0 0 0]';
    Remote=[0 0 0]';

    dmin=1e6*ones(6,1);
    dminindx=1e6*ones(6,1);

    objectsRparalel=true;

    % % % LOOP THROUGH FACES
    for i=1:6
        Faces(i).LocalInCorners=0;
        Faces(i).OtherInCorners=0;
        for j=1:4
            if Edges(Faces(i).edgeIDs(j)).isInCorner==0 ...
            || Edges(Faces(i).edgeIDs(j)).isInCorner==1
                Faces(i).LocalInCorners=Faces(i).LocalInCorners+1;
            end
            if (Edges(Faces(i).conEdges(j)).isInCorner==0 && Faces(i).conEdges(5)==-1) ...
            || (Edges(Faces(i).conEdges(j)).isInCorner==1 && Faces(i).conEdges(5)==1)
                Faces(i).OtherInCorners=Faces(i).OtherInCorners+1;
            end
        end

    % % % Check if the objects colide
    %     remember the shortest distance for all faces
        dists=1e6*ones(4,1);
        for d=1:4
            dir=Faces(i).Edges(d).critical-Faces(i).Edges(d).remote;
            dists(d)=norm(dir);
        end
        [dmin(i), dminindx(i)]=min(dists);
        if dmin(i)<cylinder.radius
        else        
    % % % Find the closest pair of points
            if Faces(i).LocalInCorners==0 && Faces(i).OtherInCorners==4
        % % % % Its on a face
                objectsRparalel=false;              
                faceR=Faces(i).R;
                t1=faceR*cuboid.R'*(Faces(i).Edges(1).critical-cuboid.pos')';
                t2=faceR*cuboid.R'*(Faces(i).Edges(2).critical-cuboid.pos')';
                Point=[t1(1) t1(2) t2(3)]';
                Point=cuboid.R*Faces(i).R'*Point;
                Point=Point+cuboid.pos;
                [~, temp, ~]=...
                    line2line(...
                    [Point';cuboid.pos'],getCylinderSegment(cylinder));
                Remote=temp';             
                dir=(Point-Remote)';
                Remote=Remote+(dir/norm(dir)*cylinder.radius')';
            end

             if Faces(i).LocalInCorners==2 && Faces(i).OtherInCorners==4
        % % % % Its on an edge    
                objectsRparalel=false;
                dists=1e6*ones(4,1);
                for d=1:4
                    dir=Faces(i).Edges(d).critical-Faces(i).Edges(d).remote;
                    dists(d)=norm(dir);
                end
                [~, d]=min(dists);
                Remote=Faces(i).Edges(d).remote';
                Point=Faces(i).Edges(d).critical';
                dir=(Point-Remote)';
                Remote=Remote+(dir/norm(dir)*cylinder.radius')';
             end

            if Faces(i).LocalInCorners>2 && Faces(i).OtherInCorners==4
        % % % % Its in a corner       
                objectsRparalel=false;
                dists=1e6*ones(4,1);
                for d=1:4
                    dir=Faces(i).Edges(d).critical-Faces(i).Edges(d).remote;
                    dists(d)=norm(dir);
                end
                [~, d]=min(dists);
                Remote=Faces(i).Edges(d).remote';
                Point=Faces(i).Edges(d).critical';
                dir=(Point-Remote)';
                Remote=Remote+(dir/norm(dir)*cylinder.radius)';
            end
        end
        % % % objects R paralel        
        if objectsRparalel
            [~, faceNO]=min(dmin);
            Remote=Faces(faceNO).Edges(dminindx(faceNO)).remote';
            Point=Faces(faceNO).Edges(dminindx(faceNO)).critical';
            dir=(Point-Remote)';
            Remote=Remote+(dir/norm(dir)*cylinder.radius)';
        end
    %     disp(msg);
    end
    %  disp('------------');

    Corners1=zeros(3,12);
    Corners2=zeros(3,12);
    for i=1:12
        Corners1(:,i)=Edges(i).critical;
    end

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
function [Point1 Point2] = Sphere2Cylinder( sphere, cylinder )
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
    [Point1, Point2]=line2line(...
                getSphereSegment(sphere),getCylinderSegment(cylinder));
    vect=Point2-Point1;
    vect=vect/norm(vect);

    Point1=Point1+vect*sphere.radius;
    Point2=Point2-vect*cylinder.radius;

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %   
function [Point1 Point2]=Cylinder2Cylinder(cylinder1,cylinder2)
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
    [Point1, Point2, ~]=line2line(...
                getCylinderSegment(cylinder1),getCylinderSegment(cylinder2));
    vect=Point2-Point1;
    vect=vect/norm(vect);

    Point1=Point1+vect*cylinder1.radius;
Point2=Point2-vect*cylinder2.radius;

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
function TwoPoints=getCylinderSegment(cylinder)
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
    t1=cylinder.pos+cylinder.R*[0  cylinder.height/2 0]';
    t2=cylinder.pos+cylinder.R*[0 -cylinder.height/2 0]';
    TwoPoints=[t1';t2'];
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
function TwoPoints=getSphereSegment(sphere)
    t1=sphere.pos;
    t2=sphere.pos+[rand(1); rand(1); rand(1)]*1e-6;
    TwoPoints=[t1';t2'];

    
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
function [Pc, Qc, where]=line2line(L1,L2)
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
%   Pc - point on segment L1
%   Pq - point on segment L2
%   where - Pc normalized to length of segment L1
    P0=L1(1,:);
    P1=L1(2,:);
    Q0=L2(1,:);
    Q1=L2(2,:);
    
    u=(P1-P0); v=(Q1-Q0); w0=P0-Q0; w=w0;
    a=dot(u,u); b=dot(u,v); c=dot(v,v); d=dot(u,w0); e=dot(v,w0);

    D=a*c - b*b;
    sD = D;
    tD = D;
    SMALL_NUM=1e-10;
    if (D < SMALL_NUM) 
         sN = 0.0;  
         sD = 1.0; 
         tN = e;
         tD = c;
    else
        sN = (b*e - c*d);
        tN = (a*e - b*d);
        if sN < 0.0
            sN = 0.0;
            tN = e;
            tD = c;
        elseif sN > sD
            sN = sD;
            tN = e + b;
            tD = c;
        end
    end
    
    if (tN < 0.0)
        tN = 0.0;
        if (-d < 0.0)
            sN = 0.0;
        elseif -d > a
            sN = sD;
        else
            sN = -d;
            sD = a;
        end
    elseif tN > tD
        tN = tD;
        if ((-d + b) < 0.0)
             sN = 0;
        elseif ((-d + b) > a)
             sN = sD;
        else
            sN = (-d + b);
            sD = a;
        end
    end

    if abs(sN) < SMALL_NUM
        sc=0;
    else sc = sN/sD;
    end
    
    if abs(tN) < SMALL_NUM
        tc=0;
    else tc = tN/tD;
    end
        
    dP = w + (sc * u) - (tc * v);
    Pc=P0 + (sc * u);
    Qc=Q0 + (tc * v);
    
    LESS_SMALL_NUM=1e-3;
    where=norm(P1-Pc)/norm(Pc-Qc);
    if norm(P1-Pc) < LESS_SMALL_NUM
        where=0;
    end   
    if norm(P0-Pc) < LESS_SMALL_NUM
        where=1;
    end