function flag=is_critical_robot(d,R,factor)
%判断机器人是否为关键机器人，根据距离划分，距离小于threshold的为近邻，大于threshold的为远邻；
%如果i的远邻为空，i非关键机器人；如果i所有远邻的邻居中，均存在至少一个为i的近邻，则不为关键机器人
%输出：flag为nx1向量，元素表示：1为关键，0为非关键
%输入：agent：机器人编号，d：距离矩阵
n=size(d,1);
flag=ones(n,1);
threshold=R*factor;
for agent=1:n
    a=0;
    if isempty(d(agent,d(agent,:)>threshold&d(agent,:)<=R))%远邻为空
        flag(agent)=0;
    else
        ind_if=find(d(agent,:)>threshold&d(agent,:)<=R);%agent的远邻索引
        ind_in=find(d(agent,:)<=threshold);%agent的近邻索引
        for j=1:length(ind_if)
            for k=1:length(ind_in) 
                ind_ck=find(d(ind_in(k),:)<=threshold);%近邻ind_in的近邻索引
                if ismember(ind_if(j),ind_ck)
                    a=a+1;
                    break
                end
            end
        end
        if a==length(ind_if)
            flag(agent)=0;
        end
    end
end
end
