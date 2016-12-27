function showDisparityMap( )
width  = 640; % ����ط���Ҫָ��Ϊ���Լ��ľ���Ŀ���
height = 480; % ����E�Ҫָ��Ϊ���Լ��ľ���ĸ߶�
channels = 1; % ͨ���?
%fs = fopen('disparityMap24.txt', 'rb');
im=importdata('disparityMap25.txt');
%db = fread(fs, 'int8'); % ע�⣬����Eõ���unsigned int8
%fclose(fs);
%size(db)
%ou = reshape(db, width, channels*height); % ������ʾ��ʽ

%im(:,:) = ou(3:end, :)'; % Bͨ��
min = inf;
for i=1:height
    for j=1:640
        value = im(i,j);
        if value~=0 && value<min
            min=value;
        end
    end
end
for i=1:height
    for j=1:640
        value = im(i,j);
        if value~=0
            im(i,j) = value - min;
        end
    end
end
maxIm = max(max(im));
fig2=figure;
plotHandles(1) = pcolor( flipud(im) );
set( plotHandles(1) , 'EdgeColor' , 'none');
caxis([0,maxIm])
colormap jet;
%figure; image(im);caxis([0,maxIm]) colormap jet; % һ��Ҫ�ǵ�ת��Ϊuint8
%figure; imshow((mapminmax(im)+1)*0.5);
end

