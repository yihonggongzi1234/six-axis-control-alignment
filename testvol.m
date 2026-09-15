T = 0.5/0.8;
V_record = zeros(100,6);
A_record = zeros(99,6);
for i = 1:100
    for j = 2:7
        V_record(i,j-1) = 100 / (0.5/0.8) * (Length_record(i+1,j) - Length_record(i,j));
    end
end
for i = 1:99
    for j = 1:6
         A_record(i,j) = 100 / (0.5/0.8) * (V_record(i+1,j) - V_record(i,j));
    end
end

figure(1)
for k = 1:6
    plot(V_record(:,k),'linewidth',3,'color',[0.15*k,1-0.15*k,0]);
    hold on
end

figure(2)
for k = 1:6
    plot(A_record(:,k),'linewidth',3,'color',[0.15*k,1-0.15*k,0]);
    hold on
end
