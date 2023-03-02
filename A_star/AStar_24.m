function [path,close,open]=AStar_24(obstacle,map)
% obstacle ：障碍物坐标，mx2大小数组

% map ：包含以下元素的结构体
%       map.start   [x1,y1]  起点
%       map.goal    [x2,y2]  终点

% 用于存储路径
path=[];

close=[];

% open集合初始化
%    [节点X坐标   ,节点Y坐标   ,代价值F=G+H                      ,代价值G,父节点X     ,父节点Y      ]
open=[map.start(1),map.start(2),0+10*sum(abs(map.start-map.goal)),0      ,map.start(1), map.start(2)];

% 邻域集合
xyMat=ones(5,5);
xyMat(3,3)=0;
[x,y]=find(xyMat);
next=[x,y]-3;
next=[next,sqrt(next(:,1).^2+next(:,2).^2).*10];


while true
    
    % 若open集合被取完，则无路径
    if isempty(open(:,1))
        disp('No path to goal!!');
        return;
    end
    
    % 判断目标点是否出现在open列表中
    [~,~,IndexGoal]=intersect(map.goal,open(:,1:2),'rows');
    if ~isempty(IndexGoal)
        disp('Find Goal!!');
        close=[open(IndexGoal,:);close];
        break;
    end
    
    [~,IndexSort] = sort(open(:,3)); % 获取OpenList中第三列大小顺序
    open=open(IndexSort,:);          % 将open集合按照Index排序
    
    % 将最小open移至close集合并作为当前点
    close=[open(1,:);close];
    current=open(1,:);
    open(1,:)=[];
    
    
    for i=1:size(next,1)
        newNode=[current(1)+next(i,1),current(2)+next(i,2),0,0,0,0]; 
        newNode(4)=current(4)+next(i,3);                                % 相邻节点G值
        newNode(3)=10*sum(abs(newNode(1:2)-map.goal))+newNode(4);       % 相邻节点F值
        
        % 如果它不可达，忽略它
        if ~isempty(intersect(newNode(1:2),obstacle,'rows'))
            continue;
        end
        
         % 如果它与父节点之间有障碍物，忽略它
        midPnt=(newNode(1:2)+current(1:2))./2;
        if any(midPnt-round(midPnt)==0)&&any(midPnt-round(midPnt)~=0)
            tempPnt1=ceil(midPnt);
            tempPnt2=floor(midPnt);
            tempBool1=~isempty(intersect(tempPnt1,obstacle,'rows'));
            tempBool2=~isempty(intersect(tempPnt2,obstacle,'rows'));
            if tempBool1&&tempBool2
                continue;
            end 
        end
        
        if ~isempty(intersect(midPnt,obstacle,'rows'))
                continue;
        end 
        
        % 如果它在close集合中，忽略它
        if ~isempty(intersect(newNode(1:2),close(:,1:2),'rows'))
            continue;
        end
        
        % 如果它不在open集合中
        if isempty(intersect(newNode(1:2),open(:,1:2),'rows'))
            newNode(5:6)=current(1:2);          % 将当前节点作为其父节点
            open=[open;newNode];                % 将此相邻节点加放OpenList中
        else % 若在
            [~,~,IndexInOpen]=intersect(newNode(1:2),open(:,1:2),'rows');
            if newNode(3)<open(IndexInOpen,3)
                % 若F更小，则将此相邻节点的父节点设置为当前节点，否则不作处理
                newNode(5:6)=current(1:2);      % 将当前节点作为其父节点
                open(IndexInOpen,:)=newNode;    % 将此相邻节点在OpenList中的数据更新
            end
        end
%         plot_map(map,obstacle,open,close);
    end
%     plot_map(map,obstacle,open,close);
end

path = GetPath(close, map.start);
%追溯路径
% Index=1;
% while 1
%     path=[path;close(Index,1:2)];
%     if isequal(close(Index,1:2),map.start)   
%         break;
%     end
%     [~,IndexInClose,~]=intersect(close(:,1:2),close(Index,5:6),'rows');
%     
%     % 以下为找到中间点的过程
%     midPnt=(close(Index,1:2)+close(Index,5:6))./2;
%     
%     if ~all(midPnt~=round(midPnt))% 若中点两个数值均不是整数，则说明两节点间中间点
%         
%         % 若有一个数字不是整数，或两个数字均为整数
%         % 下向上向下取整
%         tempPnt1=floor(midPnt);
%         tempPnt2=ceil(midPnt);
%         tempPntSet=[tempPnt1;tempPnt2];
%         
%         % 判断取整后节点是否和原节点重合
%         [~,tempIndex1,~]=intersect(tempPntSet,close(Index,1:2),'rows');
%         tempPntSet(tempIndex1,:)=[];
%         [~,tempIndex2,~]=intersect(tempPntSet,close(Index,5:6),'rows');
%         tempPntSet(tempIndex2,:)=[];
%         
%         % 判断中间点是否为障碍物，并选择F值最小的中间点
%         openAndCloseSet=[open;close];
%         [~,~,tempIndex]=intersect(tempPntSet,openAndCloseSet(:,1:2),'rows');
%         tempF=openAndCloseSet(tempIndex,3);
%         if ~isempty(tempF)
%             tempIndex3=find(tempF==min(tempF));
%             tempIndex3=tempIndex3(1);
%             midPnt=openAndCloseSet(tempIndex(tempIndex3),:);
%             path=[path;midPnt(1:2)];
%         end
%         
%     end
%     Index=IndexInClose;
%     
% end


end
