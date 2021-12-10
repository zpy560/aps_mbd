classdef Spline

    properties(Access=public)
        a
        b
        c
        d
        w
        
        x
        y
        
        nx
    end
    methods(Access=public)
        
        function obj =Spline(x,y) % x代表每个点对应的累加弧长s
            obj.b=[];
            obj.c=[];
            obj.d=[];
            obj.w=[];
            obj.x=x;
            obj.y=y;            
            obj.nx=length(x);   %dimension of x 轨迹点的个数
            h=diff(x); % 对应的每段弧长
            
            %% cal coefficient c and a , 
            obj.a=y; 
            A=obj.calc_A(h);
            B=obj.calc_B(h);
            obj.c=A\B; % 对线性方程组 Ax = B 求解 x
            obj.c=(obj.c)';
            % 由N+1个点N段样条方程组成 方程 f(s)=ai+bi*(s-si)+ci*(s-si)^2+di*(s-si)^3；
            % yi = ai;经过(si,yi)
            % 导数方程
            % d_f(s)=bi+2*ci*(s-si)+3*di*(s-si)；
            % bi+2*ci*(s_i+1-s_i)+3*di*(s_i+1-s_i)^2 = b_i+1
            % 求二阶导 dd_f(s)=2*ci+6*di(s-si); 
            % ci + 3*di*(s_i+1-s_i) = c_i+1;
            % 中间N-1个点一二阶导连续
            % hi = s_i+1-s_i;消除bi、di先求系数c
            %% cal coefficient b and d
            for i = 1:(obj.nx-1)
                obj.d(end+1)=(obj.c(i+1)-obj.c(i)) / (3.0*h(i)); % 分段轨迹平滑，加速度连续；计算中间点系数di;2*ci+6*di*hi=2*c(i+1);
                tb= (obj.a(i+1) - obj.a(i) ) / h(i)- h(i)* (obj.c(i+1)+2.0*obj.c(i))/ 3.0; 
                % bi的计算；由上，系数di已知；根据三次样条分段轨迹方程,其中，hi = s_i+1-s_i;
                % Xend(i+1) = Xstanrt(i) + bi*(s_i+1-s_i) + ci * (s_i+1-s_i)^2 + di*(s_i+1-s_i)^3计算
                obj.b(end+1)=tb;
            end
        end
        
        function result=calc(obj,t)
            %% calculate position
            %if t is outside of the input x, return None
            if (t<obj.x(1))
                result= NaN;
                return;
            elseif ( t>obj.x(end))
                result=NaN;
                return;
            end
             i = obj.research_index(t);
             dx=t-obj.x(i); % result为三次样条方程f(s)的值；设方程 f(s)=ai+bi*(s-si)+ci*(s-si)^2+di*(s-si)^3；
             result=obj.a(i) + obj.b(i)*dx + obj.c(i)*dx.^2.0 + obj.d(i)*dx.^3.0; % 3次样条曲线
        end
                
        function result = calcd(obj, t)
            %% calculate frist derivative
            %if t is outside of the input x, return None
            if (t<obj.x(1))
                result= NaN;
                return;
            elseif ( t>obj.x(end))
                result=NaN;
                return;
            end
           i = obj.research_index(t);
             dx=t-obj.x(i);
             result=obj.b(i) +2.0* obj.c(i)*dx+3.0* obj.d(i)*dx.^2.0;
             % 方程 f(s)=ai+bi*(s-si)+ci*(s-si)^2+di*(s-si)^3；
             % 求导 d_f(s)=bi+2*ci*(s-si)+3*di*(s-si)^2
        end
        
        function result =calcdd(obj,t)
            %% calculate second derivative
            %if t is outside of the input x, return None
            if (t<obj.x(1))
                result= NaN;
                return;
            elseif ( t>obj.x(end))
                result=NaN;
                return;
            end
             i = obj.research_index(t);
             dx=t-obj.x(i);
             result=2.0* obj.c(i)+6.0* obj.d(i)*dx;
             % 方程 f(s)=ai+bi*(s-si)+ci*(s-si)^2+di*(s-si)^3；
             % 求导 d_f(s)=bi+2*ci*(s-si)+3*di*(s-si)
             % 求二阶导 dd_f(s)=2*ci+6*di(s-si)
        end
        
        function idx = research_index(obj, x)
            %research date segment index
            idx=obj.bisect(obj.x, x, 1, length(obj.x))-1;
        end
        
        function lo = bisect(~, list, x, lo, hi)
        %Return the index where to insert item x in list a, assuming a is sorted.
        % The return value i is such that all e in a[:i] have e <= x, and all e in
        % a[i:] have e > x.  So if x already appears in the list, a.insert(x) will
        % insert just after the rightmost x already there.
        % Optional args lo (default 0) and hi (default len(a)) bound the
        % slice of a to be searched.
            if lo < 1
                error('lo must be positive integer');
            end
            if isnan(hi)
                hi =length(list);
            end
            while lo < hi 
                mid = floor((lo+hi)/2); % 二分查找法
                if x < list(mid) 
                    hi=mid;
                else 
                    lo=mid+1;
                end
            end
        end
            
        function A = calc_A(obj, h)
            %% calculate matrix A for spline coefficient c
            A= zeros(obj.nx, obj.nx);
            A(1, 1)= 1.0;
            for i = 1:(obj.nx - 1)
                if i ~= (obj.nx - 1)
                    A(i+1,i+1) = 2.0*(h(i) + h(i + 1)); % 对角线元素第2-5个
                end
             A(i + 1,i) = h(i);
             A(i, i + 1)= h(i);
            end
            A(1, 2)=0.0;
            A(obj.nx, obj.nx-1) = 0.0;
            A(obj.nx, obj.nx) = 1.0;
        end
            
        function B = calc_B(obj, h)
            %% calc matrix B for spline coefficient c
            B= zeros(obj.nx,1);
            for i = 1:(obj.nx-2) % 第2-6个元素
                B(i+1)=3.0*(obj.a(i+2)-obj.a(i+1))/ ...
                      h(i+1)-3.0*(obj.a(i+1)-obj.a(i))/h(i) ;
            end         
        end
        
    end
end