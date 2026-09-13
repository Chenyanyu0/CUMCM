# B 题问题二：第二探测点的面积与直径期望

本文给出问题二的一套完整计算流程。目标是在第一次探测得到一个
`2°` 扇形后，选择第二个探测点 `S2=(x,y)`，使两次测向得到的候选区域尽可能小。
这里分别使用候选区域的面积和直径作为定位效果指标。

文中给出的是一个可直接数值实现的模型，同时给出在交集始终为四边形时面积期望的闭式表达式。

## 1. 建模假设

### 1.1 坐标系

把第一次探测点设为

$$
 S_1=(0,0).
$$

将第一次测得的 `2°` 扇形下边界旋转到 `x` 轴，令

$$
 \varepsilon=1^\circ=\frac{\pi}{180},
 \qquad \beta=2\varepsilon=2^\circ,
 \qquad k=\tan\beta.
$$

因此第一次测向区域为

$$
 W_1=\{(X,Y):Y\ge 0,\; kX-Y\ge 0\}.
$$

由于 `Y>=0` 且 `kX-Y>=0`，该区域自动满足 `X>=0`。

### 1.2 干扰源的位置分布

第一次扇形内的干扰源记为 `G=(a,b)`。由于干扰源已经在 `S1` 处被接收到，而接收半径上限为
`1500 m`，对这次条件分布可取扇形最大半径 `R=1500 m`。若题目设定的全局先验圆盘半径
`1800 m` 还要同时用于源位置，则应把 `R` 改为相应的几何上限，或在积分后再与该圆盘求交。
若考虑干扰源距离 `S1` 小于 `5 m` 时无法正常测向，取内半径
`r0=5 m`；若暂时忽略这个小孔洞，取 `r0=0`。

干扰源用极坐标表示为

$$
 a=r\cos\psi,\qquad b=r\sin\psi,
$$

其中

$$
 r_0<r\le R,
 \qquad 0\le\psi\le\beta.
$$

假设干扰源在扇形内按**平面面积均匀分布**。由于极坐标面积元为
`r dr dpsi`，因此半径不能按普通的一维均匀分布处理。这里 `psi` 只表示源点相对于
`S1` 的极角，不表示第二次测得的中心线方向；后文的中心线角仍记为 `theta`。

扇形面积为

$$
 |C_1|=\frac{\beta}{2}(R^2-r_0^2).
$$

### 1.3 第二次测向误差

真实方向是从 `S2` 指向 `G` 的方向。设实际测得的中心方向相对于真实方向的误差为 `delta`，采用

$$
 \delta\sim U[-\varepsilon,\varepsilon]
$$

作为误差模型。于是第二个 `2°` 扇形的两条边界方向为

$$
 \phi+\delta-\varepsilon,
 \qquad
 \phi+\delta+\varepsilon,
$$

其中

$$
 \phi=\operatorname{atan2}(b-y,a-x).
$$

也就是说，第二个扇形保持宽度 `2°`，但会在总共 `2°` 的范围内旋转。

这里的误差是一个单次测向误差参数。若同一地点重复测量，按照题目说明仍使用同一个误差，不另行引入独立噪声。

### 1.4 用 `x,y,theta` 表示四个交点

如果只计算某一次实际测量形成的第二扇形，可以直接用第二扇形中心线的方向角 `theta`，不必显式写出干扰源坐标 `a,b`。本节中的 `theta` 表示第二次测得的中心线角；若真实方向角为 `phi`、测向误差为 `delta`，则

$$
\theta=\phi+\delta.
$$

为避免和第一扇形内源位置的极角混淆，后文对源位置积分时可将源极角另记为 `psi`。令

$$
\eta=1^\circ=\frac{\pi}{180},\qquad
\beta=2^\circ,\qquad
k=\tan\beta.
$$

第二扇形的两条边界角度为

$$
\theta_s=\theta+s\eta,qquad s\in\{-1,+1\}.
$$

第 `s` 条边界射线的参数方程为

$$
(X,Y)=(x,y)+t(\cos\theta_s,\sin\theta_s),\qquad t\ge0.
$$

#### 1.4.1 与第一扇形下边界相交

第一扇形下边界为 `Y=0`。令参数方程中的 `Y=0`，得到

$$
t=-\frac{y}{\sin\theta_s}.
$$

代回 X 坐标：

$$
p_s=x-y\cot\theta_s
    =\frac{x\sin\theta_s-y\cos\theta_s}{\sin\theta_s}.
$$

所以交点为

$$
\boxed{P_s=(p_s,0)}.
$$

两个下边界交点为

$$
P_-=(x-y\cot(\theta-\eta),0),\qquad
P_+=(x-y\cot(\theta+\eta),0).
$$

这里要求 `sin(theta_s)!=0`。若交点必须位于边界射线而不是其反向延长线上，还应检查

$$
-\frac{y}{\sin\theta_s}\ge0.
$$

#### 1.4.2 与第一扇形上边界相交

第一扇形上边界为 `Y=kX`。代入参数方程：

$$
y+t\sin\theta_s=k\left(x+t\cos\theta_s\right).
$$

解得

$$
t=\frac{kx-y}{\sin\theta_s-k\cos\theta_s}.
$$

令

$$
q_s=
\frac{x\sin\theta_s-y\cos\theta_s}
     {\sin\theta_s-k\cos\theta_s}.
$$

则交点为

$$
\boxed{Q_s=(q_s,kq_s)}.
$$

因此

$$
Q_-=\left(
\frac{x\sin(\theta-\eta)-y\cos(\theta-\eta)}
     {\sin(\theta-\eta)-k\cos(\theta-\eta)},
 kq_-\right),
$$

$$
Q_+=\left(
\frac{x\sin(\theta+\eta)-y\cos(\theta+\eta)}
     {\sin(\theta+\eta)-k\cos(\theta+\eta)},
 kq_+\right).
$$

这里要求 `sin(theta_s)-k cos(theta_s)!=0`，并检查射线参数

$$
\frac{kx-y}{\sin\theta_s-k\cos\theta_s}\ge0.
$$

#### 1.4.3 四个交点的统一表达式

综上，令

$$
p_s=x-y\cot(\theta+s\eta),
\qquad
q_s=
\frac{x\sin(\theta+s\eta)-y\cos(\theta+s\eta)}
     {\sin(\theta+s\eta)-k\cos(\theta+s\eta)},
\qquad s=\pm1.
$$

则四个候选交点统一为

$$
\boxed{
P_s=(p_s,0),\qquad
Q_s=(q_s,kq_s),\qquad s=\pm1.
}
$$

当四个点均满足两扇形的半平面约束，并且对应的射线参数均非负时，实际交集通常按以下顺序排列：

$$
P_-,\quad P_+,\quad Q_+,\quad Q_-.
$$

此时面积和直径已经完全写成 `x,y,theta` 的函数：

$$
\boxed{
A(x,y,\theta)=
\frac{k}{2}\left|p_+q_+-p_-q_-\right|.
}
$$

$$
\boxed{
D(x,y,\theta)=
\max\left\{
|p_+-p_-|,
\sqrt{1+k^2}|q_+-q_-|,
\max_{s,t=\pm1}
\sqrt{(p_s-q_t)^2+k^2q_t^2}
\right\}.
}
$$

上述面积、直径公式只适用于四边形情形。若某个点不满足全部半平面约束，实际交集可能是三角形或退化图形，此时应保留实际可行顶点后再用鞋带公式和顶点对最大距离计算。

### 1.4.4 将第二探测点改用极坐标表示

为了避免和源点极坐标中的 `r,psi` 以及第二次测得的中心线角 `theta` 重名，先令

$$
S_2=(\rho\cos\vartheta,\rho\sin\vartheta),
\qquad \rho\ge0.
$$

如果希望直接使用用户所说的 `(r,theta)`，只需在本节最后作替换
`rho -> r`、`vartheta -> theta`；源点半径和测量中心线角应分别改用 `s` 和 `mu`。

源点写成

$$
G=(s\cos\psi,s\sin\psi),
\qquad r_0\le s\le R,
\qquad 0\le\psi\le\beta.
$$

从 `S2` 指向源点的真实方向角、实际测得的中心线角和两条边界角分别为

$$
\phi_g=\operatorname{atan2}
\left(s\sin\psi-\rho\sin\vartheta,
      s\cos\psi-\rho\cos\vartheta\right),
$$

$$
\mu=\phi_g+\delta,
\qquad
\alpha_\sigma=\mu+\sigma\eta,
\qquad \sigma\in\{-1,+1\}.
$$

将

$$
x=\rho\cos\vartheta,qquad y=\rho\sin\vartheta
$$

代入前面的直角坐标交点公式，利用恒等式

$$
x\sin\alpha_\sigma-y\cos\alpha_\sigma
=\rho\sin(\alpha_\sigma-\vartheta),
$$

得到

$$
\boxed{
p_\sigma=
\rho\frac{\sin(\alpha_\sigma-\vartheta)}{\sin\alpha_\sigma},
\qquad
q_\sigma=
\rho\frac{\sin(\alpha_\sigma-\vartheta)}
{\sin\alpha_\sigma-k\cos\alpha_\sigma}.
}
$$

又因为 `k=tan(beta)`，

$$
\sin\alpha_\sigma-k\cos\alpha_\sigma
=\frac{\sin(\alpha_\sigma-\beta)}{\cos\beta},
$$

所以也可以写为

$$
q_\sigma=
\rho\cos\beta\,
\frac{\sin(\alpha_\sigma-\vartheta)}
{\sin(\alpha_\sigma-\beta)}.
$$

因此四个交点为

$$
P_\sigma=\left(
\rho\frac{\sin(\alpha_\sigma-\vartheta)}{\sin\alpha_\sigma},0
\right),
$$

$$
Q_\sigma=\left(q_\sigma,kq_\sigma\right).
$$

这说明极坐标推导与直角坐标推导逐项相同，并不是另一套近似模型。

在交集确实为四边形时，定义

$$
\pi_\sigma=
\frac{\sin(\alpha_\sigma-\vartheta)}{\sin\alpha_\sigma},
\qquad
\chi_\sigma=
\frac{\sin(\alpha_\sigma-\vartheta)}
{\sin\alpha_\sigma-k\cos\alpha_\sigma}.
$$

则

$$
p_\sigma=\rho\pi_\sigma,qquad q_\sigma=\rho\chi_\sigma,
$$

面积变为

$$
\boxed{
A_4(\rho,\vartheta;\alpha_-,\alpha_+)
=\frac{k\rho^2}{2}
\left|\pi_+\chi_+-\pi_-\chi_-\right|.
}
$$

展开后也可写成

$$
A_4=\frac{\rho^2\sin\beta}{2}
\left|
\frac{\sin^2(\alpha_+-\vartheta)}
{\sin\alpha_+\sin(\alpha_+-\beta)}
-
\frac{\sin^2(\alpha_--\vartheta)}
{\sin\alpha_-\sin(\alpha_--\beta)}
\right|.
$$

直径变为

$$
\boxed{
D_4=\rho\max\left\{
|\pi_+-\pi_-|,
\sqrt{1+k^2}|\chi_+-\chi_-|,
\max_{\sigma,\tau=\pm1}
\sqrt{(\pi_\sigma-\chi_\tau)^2+k^2\chi_\tau^2}
\right\}.
}
$$

这里出现的 `rho^2` 和 `rho` 只是固定角度几何量的缩放关系。完整模型中 `alpha_sigma` 通过 `phi_g` 依赖于源点和 `S2`，所以总体期望不能简单地说成纯粹的 `rho^2` 或 `rho` 函数。若发生三角形、近顶点或退化情形，仍必须使用

$$
\mathcal V=\{S_1,S_2,P_-,P_+,Q_-,Q_+\}
$$

逐点筛选后计算鞋带面积和顶点对最大距离；极坐标代换不会消除拓扑检查。

第二探测点属于第一扇形时，在 `rho>0` 下有

$$
\sin\vartheta\ge0,qquad
\sin(\beta-\vartheta)\ge0,
$$

等价于

$$
0\le\vartheta\le\beta.
$$

接收保证圆盘条件也可直接改写为

$$
\rho\le1000,\qquad
\rho\le2000\cos\vartheta,qquad
\rho\le2000\cos(\vartheta-\beta),
$$

其中应同时满足原来的平方不等式，避免在余弦为负时错误取平方根。

全局有限分支条件则变为

$$
\boxed{
\rho\sin(\vartheta+\beta)>R\sin(2\beta)
\quad\text{或}\quad
\rho\sin(2\beta-\vartheta)>R\sin(2\beta).
}
$$

它们分别就是直角坐标条件

$$
x\sin\beta+y\cos\beta>R\sin(2\beta),
$$

$$
y\cos(2\beta)-x\sin(2\beta)<-R\sin(2\beta)
$$

在 `x=rho cos(vartheta), y=rho sin(vartheta)` 下的结果。

如果 `theta=phi+delta`，固定真实源位置后的误差平均可直接改写为

$$
\overline A(x,y;a,b)=
\frac{1}{2\eta}
\int_{\phi-\eta}^{\phi+\eta}
A(x,y,\theta)\,d\theta,
$$

$$
\overline D(x,y;a,b)=
\frac{1}{2\eta}
\int_{\phi-\eta}^{\phi+\eta}
D(x,y,\theta)\,d\theta,
$$

其中

$$
\phi=\operatorname{atan2}(b-y,a-x).
$$

这与使用误差变量 `delta` 的积分完全等价，只是把积分变量从误差角换成了实际中心线角。
### 1.5 直接使用直线斜截式

上一节使用的是边界射线的参数方程。也可以完全改用直线

$$
y=aX+b
$$

的形式。为避免与干扰源坐标中的 a,b 混淆，本节把第二扇形第 s 条边界直线的斜率和截距记为 a_s,b_s。

令

$$
\theta_s=\theta+s\eta,\qquad s\in\{-1,+1\}.
$$

假设边界线不是竖直线，即

$$
\cos\theta_s\ne0.
$$

因为边界线经过 S2=(x_0,y_0)，其斜率和截距分别为

$$
a_s=\tan\theta_s,\qquad
b_s=y_0-a_sx_0.
$$

因此第二扇形的两条边界直线为

$$
\boxed{
y=a_sX+b_s,\qquad
a_s=\tan(\theta+s\eta),\qquad
b_s=y_0-a_sx_0.
}
$$

这里的 a_s,b_s 只是直线的斜率和截距，不是干扰源坐标。

斜截式表示的是整条无限直线，而测向边界实际上是从 S2 向外延伸的射线。因此，计算出直线交点后仍要检查交点位于正向射线上。等价地，对下边界交点检查

$$
t_{P_s}=-\frac{y_0}{\sin\theta_s}\ge0,
$$

对上边界交点检查

$$
t_{Q_s}=
\frac{kx_0-y_0}{\sin\theta_s-k\cos\theta_s}\ge0.
$$

#### 1.5.1 与第一扇形下边界相交

第一扇形下边界为 y=0。联立

$$
0=a_sX+b_s
$$

得到

$$
X=-\frac{b_s}{a_s}
  =x_0-\frac{y_0}{a_s}
  =x_0-y_0\cot\theta_s.
$$

所以

$$
\boxed{
P_s=
\left(-\frac{b_s}{a_s},0\right)
=
\left(x_0-y_0\cot\theta_s,0\right).
}
$$

取 s=-1,+1：

$$
P_-=
\left(x_0-y_0\cot(\theta-\eta),0\right),
$$

$$
P_+=
\left(x_0-y_0\cot(\theta+\eta),0\right).
$$

#### 1.5.2 与第一扇形上边界相交

第一扇形上边界为

$$
y=kX,\qquad k=\tan\beta.
$$

与 y=a_sX+b_s 联立：

$$
kX=a_sX+b_s.
$$

因此

$$
X=\frac{b_s}{k-a_s}
  =\frac{a_sx_0-y_0}{a_s-k}.
$$

令

$$
q_s=\frac{a_sx_0-y_0}{a_s-k}.
$$

则

$$
\boxed{Q_s=(q_s,kq_s)}.
$$

将 a_s=tan(theta_s) 代入，可得

$$
q_s
=\frac{x_0\tan\theta_s-y_0}{\tan\theta_s-k}
=\frac{x_0\sin\theta_s-y_0\cos\theta_s}
       {\sin\theta_s-k\cos\theta_s}.
$$

因此

$$
Q_-=
\left(
\frac{x_0\sin(\theta-\eta)-y_0\cos(\theta-\eta)}
     {\sin(\theta-\eta)-k\cos(\theta-\eta)},
kq_-
\right),
$$

$$
Q_+=
\left(
\frac{x_0\sin(\theta+\eta)-y_0\cos(\theta+\eta)}
     {\sin(\theta+\eta)-k\cos(\theta+\eta)},
kq_+
\right).
$$

#### 1.5.3 证明两种写法相等

参数方程

$$
(X,Y)=(x_0,y_0)+t(\cos\theta_s,\sin\theta_s)
$$

在 cos(theta_s) 不为零时，消去 t：

$$
t=\frac{X-x_0}{\cos\theta_s},
$$

于是

$$
Y-y_0=(X-x_0)\tan\theta_s,
$$

即

$$
Y=\tan\theta_s X+
\left(y_0-x_0\tan\theta_s\right)
=a_sX+b_s.
$$

所以参数方程和斜截式描述的是同一条边界直线。

对于下边界 Y=0，参数方程给出

$$
t=-\frac{y_0}{\sin\theta_s},
\qquad
X=x_0+t\cos\theta_s
=x_0-y_0\cot\theta_s
=-\frac{b_s}{a_s}.
$$

斜截式给出的横坐标也是 -b_s/a_s，因此

$$
P_s^{(\mathrm{slope})}=P_s^{(\mathrm{param})}.
$$

对于上边界 Y=kX，参数方程给出

$$
t=\frac{kx_0-y_0}
        {\sin\theta_s-k\cos\theta_s},
$$

从而

$$
X=
\frac{x_0\sin\theta_s-y_0\cos\theta_s}
     {\sin\theta_s-k\cos\theta_s}
=\frac{a_sx_0-y_0}{a_s-k}
=\frac{b_s}{k-a_s}.
$$

这与斜截式结果完全相同，所以

$$
\boxed{
Q_s^{(\mathrm{slope})}=Q_s^{(\mathrm{param})}.
}
$$

斜截式在 cos(theta_s)=0 时不能直接使用，此时边界为竖直线 X=x_0，应改用参数方程或一般式直线。

### 1.6 近顶点相交情形

前面的四边形公式默认四条跨边界交点都是真实可行顶点。当第二个扇形靠近第一个扇形的顶点，或者两个扇形的一个顶点落入另一个扇形时，交集的顶点集合会发生变化，必须把两个扇形顶点本身也纳入检查。

仍记第一扇形顶点为

$$
S_1=(0,0)
$$

第二扇形顶点为

$$
S_2=(x_0,y_0).
$$

第二扇形两条边界方向为

$$
\theta_-=\theta-\eta,\qquad
\theta_+=\theta+\eta.
$$

用叉积定义

$$
C_s(Z)=
\cos\theta_s\,(Y-y_0)
-\sin\theta_s\,(X-x_0),
\qquad Z=(X,Y).
$$

第二扇形内部的判定为

$$
C_-(Z)\ge0,\qquad C_+(Z)\le0.
$$

#### 1.6.1 第二扇形是否包含 S1

将 $S_1=(0,0)$ 代入上式：

$$
C_s(S_1)=x_0\sin\theta_s-y_0\cos\theta_s.
$$

因此

$$
\boxed{
S_1\in W_2
\Longleftrightarrow
x_0\sin\theta_- - y_0\cos\theta_-\ge0,
\quad
x_0\sin\theta_+ - y_0\cos\theta_+\le0.
}
$$

等价地，只要 $S_1\ne S_2$，令

$$
\alpha_{21}=\operatorname{atan2}(-y_0,-x_0),
$$

则条件是从 $\theta$ 到 $\alpha_{21}$ 的有向最小角差满足

$$
\operatorname{wrap}(\alpha_{21}-\theta)\in[-\eta,\eta].
$$

如果一个等号成立，$S_1$ 位于第二扇形的一条边界上；如果两个不等式均为严格不等式，$S_1$ 位于第二扇形内部。此时 $S_1$ 必须作为交集候选顶点加入。

#### 1.6.2 第一扇形是否包含 S2

第一扇形的两个半平面约束为

$$
Y\ge0,\qquad kX-Y\ge0.
$$

因此

$$
\boxed{
S_2\in W_1
\Longleftrightarrow
y_0\ge0,\qquad
kx_0-y_0\ge0.
}
$$

等号分别对应 $S_2$ 位于第一扇形下边界或上边界；两个不等式严格时，$S_2$ 位于第一扇形内部。此时 $S_2$ 也是两扇形交集的真实候选顶点。

#### 1.6.3 两个顶点都要加入通用半平面交

四条边界直线的两两交点包括：

$$
S_1=L_{1,0}\cap L_{1,\beta},\qquad
S_2=L_{2,-}\cap L_{2,+},
$$

以及四个跨边界交点

$$
P_-,\ P_+,\ Q_-,\ Q_+.
$$

因此完整候选集合应写成

$$
\mathcal V=
\{S_1,S_2,P_-,P_+,Q_-,Q_+\}.
$$

对 $\mathcal V$ 中每个点同时检查四个半平面约束，并删除重复点，才能得到实际交集顶点。于是：

- 若 $S_1$ 或 $S_2$ 满足约束，它们会自动进入实际顶点集；
- 若某个 $P_s$ 或 $Q_s$ 不满足约束，必须删除，不能继续套用四边形公式；
- 若保留下来的顶点数为 3，交集是三角形；
- 若顶点数为 2 或 1，交集退化为线段或点；
- 若存在共同无界方向，交集无界，原始面积和直径均为 $+\infty$。

这也解释了为什么在近顶点情况下，面积应统一使用实际顶点的鞋带公式，直径应统一枚举实际顶点对，而不能只计算 $P_\pm,Q_\pm$ 四点。

可以用两个布尔条件快速判断常见拓扑：

$$
I_1=\mathbf 1_{\{S_1\in W_2\}},
\qquad
I_2=\mathbf 1_{\{S_2\in W_1\}}.
$$

在没有平行线和重合边界的普通情况下：

| 条件 | 典型交集顶点 |
|---|---|
| $I_1=0,\ I_2=0$ | 四个跨边界点 $P_-,P_+,Q_-,Q_+$，通常为四边形 |
| $I_1=1,\ I_2=0$ | $S_1$ 加上部分跨边界点，通常为三角形 |
| $I_1=0,\ I_2=1$ | $S_2$ 加上部分跨边界点，通常为三角形 |
| $I_1=1,\ I_2=1$ | 两个顶点都可能进入交集，可能为四边形，也可能因边界顺序变成三角形 |

等号情形表示顶点落在边界上，交集可能退化；因此实际程序不能只根据 $I_1,I_2$ 直接指定顶点数，仍需逐点检查四个半平面约束。

#### 1.6.4 两个顶点重合或非常接近

当

$$
\rho=\|S_2-S_1\|=\sqrt{x_0^2+y_0^2}
$$

很小时，$S_1$ 和 $S_2$ 几乎重合，四个跨边界交点可能出现严重的消去误差。计算时应：

1. 直接把 $S_1,S_2$ 纳入候选点集合；
2. 对 $\rho\le\texttt{tol}$ 的情形单独标记为近重合；
3. 使用一般式直线或叉积形式判断半平面，不用斜率比较；
4. 对重复点做距离去重。

当 $S_1=S_2$ 时，若两个扇形的角区间有正的重叠，原始无限扇形交集仍是一个以共同顶点为顶点的无限扇形，面积和直径为 $+\infty$。只有加入有限先验裁剪后，才会得到有限指标。

#### 1.6.5 边界方向平行和扇形只接触

斜截式中的分母为零对应明确的几何边界：

$$
\sin\theta_s=0
\Longleftrightarrow
L_{2,s}\parallel (Y=0),
$$

$$
\sin\theta_s-k\cos\theta_s=0
\Longleftrightarrow
L_{2,s}\parallel (Y=kX).
$$

第一种情况下，第二扇形边界与第一扇形下边界平行；若两条直线又重合，则不能把它们当作一个普通交点处理，而应保留整条公共边并由其他约束截取。第二种情况同理。算法中遇到这些分母小于容差的情形，应改用一般式直线

$$
AX+BY+C=0
$$

做平行和重合判定，不能直接计算 -b_s/a_s 或 b_s/(k-a_s)。

再定义两个扇形的方向区间

$$
I_1=[0,\beta],\qquad
I_2=[\theta-\eta,\theta+\eta]\pmod{2\pi}.
$$

它们的交集决定无穷远处的共同方向：

- 若 $I_1\cap I_2$ 含有正长度区间，且两个扇形有一个公共点，则交集存在二维无界部分，面积和直径均为 $+\infty$；
- 若 $I_1\cap I_2$ 只有一个方向，交集的无界方向只有这一条射线；若截面宽度为正，则交集是无界带状区域，面积和直径均为 $+\infty$；只有当截面也退化为零宽度时，交集才是一条无界射线，此时面积为 0、直径为 $+\infty$；
- 若 $I_1\cap I_2=\varnothing$，非空交集的多边形没有共同无界方向，因而是有界的；
- 若两个扇形没有任何公共点，则交集为空，面积和直径按空集处理，不能把它当作退化小区域。

在本题 $\beta=2\eta=2^\circ$ 的情况下，忽略角度的模运算后，两个方向区间无共同方向的充分条件可写成

$$
\theta\bmod 2\pi\in(\beta+\eta,\ 2\pi-\eta).
$$

代入 $\beta=2^\circ,\eta=1^\circ$ 后，该区间就是

$$
\theta\bmod 2\pi\in(3^\circ,\ 359^\circ).
$$

端点 $\theta=3^\circ$ 或 $\theta=359^\circ$ 对应两个扇形方向区间只在一个边界方向上接触，必须单独判断是否真的存在公共点以及交集是否退化。

当 $S_1=S_2$ 时，分类尤其简单。令

$$
J=I_1\cap I_2.
$$

则

$$
\begin{array}{c|c|c}
J & \text{交集} & (A,D)\\
\hline
\varnothing & \{S_1\} & (0,0)\\
\text{单点} & \text{一条无界射线} & (0,+\infty)\\
\text{正长度区间} & \text{无界扇形} & (+\infty,+\infty)
\end{array}
$$

但在真实测向模型中 $S_2=S_1$ 时，第二次测向方向通常没有定义，且若干扰源也位于该点则无法用普通方位观测处理。因此这是几何极限情形，不能直接作为正常探测方案。

#### 1.6.6 边界情形下的面积和直径

设经过半平面筛选和去重后，实际有界交集的顶点按逆时针排列为

$$
V_1,V_2,\ldots,V_m,\qquad V_{m+1}=V_1.
$$

统一面积公式为鞋带公式

$$
\boxed{
A=\frac12\left|
\sum_{i=1}^{m}
\left(
V_{i,x}V_{i+1,y}
-V_{i,y}V_{i+1,x}
\right)
\right|.
}
$$

统一直径公式为

$$
\boxed{
D=\max_{1\le i<j\le m}\|V_i-V_j\|.
}
$$

例如，若近顶点情形下实际交集是三角形 $\{S_1,P_s,Q_t\}$，则

$$
A=\frac{k}{2}|p_s q_t|,
$$

$$
D=\max\left\{
|p_s|,
\sqrt{1+k^2}|q_t|,
\sqrt{p_s^2+k^2q_t^2}
\right\}.
$$

若三角形顶点为 $\{S_2,P_s,Q_t\}$，则

$$
A=
\frac12\left|
(p_s-x_0)(kq_t-y_0)
+y_0(q_t-x_0)
\right|,
$$

$$
D=\max\left\{
\sqrt{(p_s-x_0)^2+y_0^2},
\sqrt{(q_t-x_0)^2+(kq_t-y_0)^2},
\sqrt{(p_s-q_t)^2+(kq_t)^2}
\right\}.
$$

如果顶点全部共线，鞋带公式给出面积 0；直径仍按最远的两个实际顶点计算。若存在二维无界部分，则上述有限多边形公式不适用，应返回无界状态。

## 2. 第一次探测的可行区域

这一步是确定 `S2` 候选位置的先验约束，与后面计算两扇形交集的面积和直径是两个不同层次的问题。

### 2.1 一个简单的接收保证区域

若要求第二个探测点对第一扇形内的所有可能干扰源都能接收到信号，并采用最保守的 `1000 m` 接收半径，可以得到如下区域。

先只看第一扇形的一条边界射线。令该射线上的源为 `G=(r,0)`，并令
`S2=(x,y)`。第一次能在距离 `r` 处收到信号，说明该干扰源的实际接收半径至少为

$$
 R_{\rm sig}\ge\max(1000,r).
$$

当 `0<=r<=1000` 时，由距离函数关于线段参数的凸性，只需限制线段两个端点：

$$
 x^2+y^2\le1000^2,
 \qquad
 (x-1000)^2+y^2\le1000^2.
$$

当 `1000<=r<=1500` 时，保证接收所需的不等式为

$$
 (x-r)^2+y^2\le r^2,
$$

即

$$
 x^2+y^2\le2rx.
$$

在候选区域内 `x>=0`，右端随 `r` 增大，因此 `r=1000` 是这一段最严格的条件，而它已经包含在第二个端点圆盘条件中。故一条射线对应

$$
 D((0,0),1000)\cap D((1000,0),1000).
$$

再让射线方向从 `0` 旋转到 `beta`。当 `S2` 位于第一扇形的前向区域（候选区域通常满足这一条件）时，内积 `x cos(theta)+y sin(theta)` 在该窄角区间的最小值由两个端点取得，因此保留两条边界射线的 `1000 m` 端点圆盘即可。如果允许 `S2` 位于扇形后方，则必须先对连续角度 `theta` 求最小内积，不能无条件地只检查两个端点。

令第一扇形两条边界方向上的两个点为

$$
 P_0=(1000,0),
 \qquad
 P_\beta=(1000\cos\beta,1000\sin\beta).
$$

忽略 `5 m` 小孔洞时，一个常用的保证区域是

$$
 \mathcal F
 =D(S_1,1000)\cap D(P_0,1000)\cap D(P_\beta,1000),
$$

其中 `D(C,r)` 表示以 `C` 为圆心、半径 `r` 的闭圆盘。

若需要保留距离 `5 m` 的近场限制，则边界方向上还要加入半径 `5 m` 的端点条件，形成相应的四圆盘交集。这个区域只表达“能够接收”，并不保证两次扇形交集一定有界，也不保证定位精度达到最优。

### 2.2 无界问题

如果直接计算两个无限延伸的测向扇形的交集，某些 `S2` 位置可能产生无界区域。对于无界区域，面积和直径都定义为 `+infinity`。

在当前坐标系下，对源域 `r0<=r<=R`、`0<=psi<=beta` 和所有误差角，下面两个严格不等式分别给出下方和上方的全局有界分支。它们描述的是“所有源位置和所有误差下都统一有界”的定义域：

$$
 x\sin\beta+y\cos\beta>R\sin(2\beta),
$$

或

$$
 y\cos(2\beta)-x\sin(2\beta)<-R\sin(2\beta).
$$

当两个不等式都严格失败时，存在一组具有正概率的源位置和误差使两个方向区间重叠，交集无界；若不加入其他裁剪，总体面积和直径期望为无穷大。等号是方向区间刚好接触的临界边界，通常只对应源域边界点和误差端点的零测度参数集，不能仅凭等号就断言总体期望为无穷大；应对其邻域做局部可积性检查。另一种做法是用全局半径 `1800 m` 的先验圆盘裁剪候选区域；此时交集必然有限，但计算的面积和直径已经是“裁剪后区域”的指标，必须在模型中明确写出。

## 3. 固定 `G` 和 `delta` 时的几何计算

本节给出单个干扰源位置、单个误差角度下的精确交集计算。

### 3.1 第二扇形的边界方向

记

$$
 u=a-x,\qquad v=b-y.
$$

对 `s=+1` 或 `s=-1`，定义

$$
 \gamma_s=\delta+s\varepsilon,
$$

并令

$$
\begin{aligned}
 U_s&=u\cos\gamma_s-v\sin\gamma_s,\\
 V_s&=v\cos\gamma_s+u\sin\gamma_s.
\end{aligned}
$$

向量 `(U_s,V_s)` 与真实方向向量 `(u,v)` 的夹角为 `gamma_s`，因此它表示第二扇形第 `s` 条边界的方向。

### 3.2 半平面不等式

第一扇形的两个半平面为

$$
 Y\ge0,
 \qquad kX-Y\ge0.
$$

第二扇形以 `S2` 为顶点。下边界的可行侧和上边界的可行侧分别写成

$$
 U_-(Y-y)-V_-(X-x)\ge0,
$$

$$
 U_+(Y-y)-V_+(X-x)\le0.
$$

因此总候选区域为四个半平面的交集

$$
 W(x,y;a,b,\delta)=W_1\cap W_2.
$$

这个表达式是后续程序实现的核心。它不依赖角度是否跨过 `0°/360°`，比直接比较角度更稳定。

### 3.3 四条边界的交点

定义

$$
 N_s=xV_s-yU_s.
$$

展开后也可写成

$$
 N_s=(xv-yu)\cos\gamma_s+(xu+yv)\sin\gamma_s.
$$

第 `s` 条边界射线可以参数化为

$$
 Z_s(\lambda)=S_2+\lambda(U_s,V_s),\qquad \lambda\ge0.
$$

与 `Y=0` 相交时，`y+lambda V_s=0`，所以

$$
 \lambda=-\frac{y}{V_s},qquad
 X=x+\lambda U_s=x-\frac{yU_s}{V_s}
   =\frac{xV_s-yU_s}{V_s}.
$$

第二扇形第 `s` 条边界与第一扇形下边界 `Y=0` 的交点为

$$
 P_s=(p_s,0),
 \qquad
 p_s=\frac{N_s}{V_s},
$$

其中要求 `V_s\ne0`。

与 `Y=kX` 相交时，

$$
 y+\lambda V_s=k(x+\lambda U_s),
$$

从而

$$
 \lambda=\frac{kx-y}{V_s-kU_s},qquad
 X=x+\lambda U_s
   =\frac{xV_s-yU_s}{V_s-kU_s}.
$$

与第一扇形上边界 `Y=kX` 的交点为

$$
 Q_s=(q_s,kq_s),
 \qquad
 q_s=\frac{N_s}{V_s-kU_s},
$$

其中要求 `V_s-kU_s\ne0`。

如果这四个交点都满足四个半平面不等式，并且都位于相应的实际边界射线上，则交集是四边形。其一个可能的循环顺序为

$$
 P_-,\ P_+,\ Q_+,\ Q_-.
$$

由于不同参数下方向可能反转，程序中应重新按凸包顺序排列，并使用绝对值计算面积。

### 3.4 四边形面积

对上述顺序使用鞋带公式，可得

$$
\boxed{
 A(x,y;a,b,\delta)=
 \frac{k}{2}\left|p_+q_+-p_-q_-\right|.
 }
$$

具体地，只有两项叉积不为零：

$$
 P_+\times Q_+=kp_+q_+,
 \qquad
 Q_-\times P_-=-kp_-q_-.
$$

所以鞋带公式给出

$$
 A=\frac12\left|kp_+q_+-kp_-q_-\right|.
$$

这条公式只在四个点确实构成可行四边形时使用。

### 3.5 四边形直径

凸多边形的直径等于顶点对之间的最大距离。因此四边形的直径是六个点对距离的最大值：

$$
\begin{aligned}
D(x,y;a,b,\delta)=\max\Bigl\{&
|p_+-p_-|,\;
\sqrt{1+k^2}\,|q_+-q_-|,\\
&\max_{s,t\in\{-1,+1\}}
\sqrt{(p_s-q_t)^2+k^2q_t^2}
\Bigr\}.
\end{aligned}
$$

不能简单地认为直径一定是两个对角线中的某一条，六个顶点对都应检查。

## 4. 三角形、退化和一般实现方式

当某一个交点不满足所有半平面约束时，实际交集可能是三角形；边界平行时还可能出现退化线段或点。因此完整算法不能盲目使用四边形公式。

推荐对四条边界做通用半平面交：

1. 把四个约束统一写成 `n_i dot (Z-c_i) >= 0`。
2. 枚举任意两条边界直线的交点，共最多六个。
3. 保留同时满足全部四个半平面不等式的交点。
4. 删除重复点，并按凸包或相对质心的极角排序。
5. 顶点数 `m>=3` 时用鞋带公式求面积，枚举顶点对求直径。
6. 顶点数为 `2` 时面积为零、直径为两点距离；顶点数为 `1` 时两者都为零。
7. 通过半平面交算法的无界性检测判断是否存在无限延伸方向。

无界性的代数判定如下。把四个约束写成

$$
 n_i\cdot Z\ge c_i,\qquad i=1,\ldots,4.
$$

若存在非零方向向量 `d` 满足

$$
 n_i\cdot d\ge0\quad\text{对所有 }i,
$$

且某个可行点 `Z0` 已存在，则 `Z0+t d` 对所有 `t>=0` 都可行，交集无界。程序可以把四条边界法向量的方向区间排序，或直接检查这些线性不等式是否存在非零解。

由于这里没有先验裁剪时最多只有四条边，直接枚举边界对的计算量为常数。若加入全局圆盘或其他多边形裁剪，仍可使用同样的半平面交框架。

对于由真实干扰源和误差范围生成的观测，真实点 `G` 位于两次 `2°` 扇形交集中，因此理论上交集不会为空。若实际输入存在空集，应把它作为数据或模型不一致处理，而不是默认为一个小面积区域。

## 5. 固定干扰源时对角度误差求期望

固定 `x,y,a,b` 后，面积和直径仍随 `delta` 变化。定义条件期望

$$
\boxed{
 \overline A(x,y;a,b)=
 \frac{1}{2\varepsilon}
 \int_{-\varepsilon}^{\varepsilon}
 A(x,y;a,b,\delta)\,d\delta,
 }
$$

$$
\boxed{
 \overline D(x,y;a,b)=
 \frac{1}{2\varepsilon}
 \int_{-\varepsilon}^{\varepsilon}
 D(x,y;a,b,\delta)\,d\delta.
 }
$$

直径中的 `max` 必须在积分号内：

$$
 E[\max_i L_i(\delta)]
 \ne \max_i E[L_i(\delta)].
$$

因此直径期望通常采用一维数值积分。

### 5.1 面积期望的闭式（仅适用于全程四边形）

如果在整个 `delta in [-epsilon,epsilon]` 区间内，交集始终是同一拓扑类型的非退化四边形，令

$$
 L^2=(a-x)^2+(b-y)^2,
$$

$$
 h=(b-y)\cos\beta-(a-x)\sin\beta,
 \qquad
 c=x\sin\beta-y\cos\beta.
$$

把四边形面积公式中的 `p_s,q_s` 代入，并使用正切代换积分，可以得到

$$
\boxed{
\overline A(x,y;a,b)=
\frac{1}{4\varepsilon}
\left|
 c^2\ln\left|1-\frac{L^2\sin^2\beta}{h^2}\right|
 -y^2\ln\left|1-\frac{L^2\sin^2\beta}{(b-y)^2}\right|
\right|.
}
$$

使用该式时必须检查：

- `V_s` 和 `V_s-kU_s` 在误差区间内不为零；
- 四个交点始终是实际可行顶点；
- 没有经过三角形、退化或无界状态；
- 有向面积 `A_sgn(delta)` 在整个误差区间内不改变符号；
- 对数中的量不在奇异点上。

闭式的推导如下。令

$$
 \alpha=\phi+\delta,
 \qquad L=\sqrt{u^2+v^2},
$$

则 `u=L cos(phi), v=L sin(phi)`，从而

$$
 U_s=L\cos(\alpha+s\varepsilon),
 \qquad V_s=L\sin(\alpha+s\varepsilon).
$$

代入 `p_s,q_s` 后，公共因子 `L` 消去，得到

$$
 p_sq_s=f(\alpha+s\varepsilon),
$$

其中

$$
 f(t)=
 \frac{(x\sin t-y\cos t)^2}
 {\sin t\,[\sin t-k\cos t]}.
$$

面积的有向形式为

$$
 A_{\rm sgn}(\delta)=
 \frac{k}{2}\bigl[f(\phi+\delta+\varepsilon)-
 f(\phi+\delta-\varepsilon)\bigr].
$$

对 `f` 作部分分式分解。记

$$
 A_0=\frac{x^2-y^2+2kxy}{1+k^2},
 \qquad B_0=-\frac{y^2}{k},
 \qquad C_0=\frac{x^2-A_0}{k}.
$$

则

$$
 f(t)=A_0+B_0\cot t
 +C_0\frac{\cos t+k\sin t}{\sin t-k\cos t},
$$

其原函数为

$$
 F(t)=A_0t+B_0\ln|\sin t|
 +C_0\ln|\sin t-k\cos t|.
$$

在 `delta` 上积分时，两个平移项相减，线性项 `A0 t` 完全抵消：

$$
 \int_{-\varepsilon}^{\varepsilon} A_{\rm sgn}(\delta)\,d\delta
 =\frac{k}{2}\left[F(\phi+\beta)-2F(\phi)+F(\phi-\beta)\right].
$$

使用 `k=tan(beta)`、`u=L cos(phi)`、`v=L sin(phi)`，并注意

$$
 kB_0=-y^2,
 \qquad
 kC_0=\frac{(kx-y)^2}{1+k^2}=c^2,
$$

即可化为第一个对数项和第二个对数项，最终得到前述闭式。这个推导也说明了分母为零时会出现几何奇异性。

将第二探测点改用极坐标

$$
x=\rho\cos\vartheta,
\qquad y=\rho\sin\vartheta
$$

后，闭式中的三个几何量应写成

$$
\begin{aligned}
L^2&=s^2+\rho^2-2s\rho\cos(\psi-\vartheta),\\
h&=s\sin(\psi-\beta)-\rho\sin(\vartheta-\beta),\\
c&=\rho\sin(\beta-\vartheta),\\
v&=s\sin\psi-\rho\sin\vartheta.
\end{aligned}
$$

因此，同一闭式在极坐标下为

$$
\boxed{
\overline A(\rho,\vartheta;s,\psi)=
\frac{1}{4\varepsilon}
\left|
\rho^2\sin^2(\beta-\vartheta)
\ln\left|1-\frac{L^2\sin^2\beta}{h^2}\right|
-\rho^2\sin^2\vartheta
\ln\left|1-\frac{L^2\sin^2\beta}{v^2}\right|
\right|.
}
$$

这里第二项的系数是 `y^2=\rho^2\sin^2\vartheta`；`v^2=(b-y)^2` 只出现在该项对数的分母中。这个区分是由部分分式中的 `kB_0=-y^2` 决定的，也是直角坐标和极坐标闭式一致的必要条件。

其中用到的两个三角恒等式是

$$
 \frac{\sin(\phi+\beta)\sin(\phi-\beta)}{\sin^2\phi}
 =1-\frac{\sin^2\beta}{\sin^2\phi}
 =1-\frac{L^2\sin^2\beta}{v^2},
$$

以及

$$
 \frac{[\sin(\phi+\beta)-k\cos(\phi+\beta)]
       [\sin(\phi-\beta)-k\cos(\phi-\beta)]}
      {[\sin\phi-k\cos\phi]^2}
 =1-\frac{L^2\sin^2\beta}{h^2}.
$$

只要误差区间内发生拓扑变化，就应按变化点分段，并对每一段使用对应的顶点公式；最稳妥的实现仍是直接对通用半平面交结果做数值积分。

## 6. 干扰源遍历整个第一扇形

将 `a,b` 换成极坐标表达式

$$
 a=r\cos\psi,\qquad b=r\sin\psi,
$$

即可得到只关于第二探测点 `(x,y)` 的总体目标函数。

### 6.1 总体面积期望

$$
\boxed{
 F_A(x,y)=
 \frac{2}{\beta(R^2-r_0^2)}
 \int_0^\beta\int_{r_0}^{R}
 \overline A(x,y;r\cos\psi,r\sin\psi)
 \,r\,dr\,d\psi.
 }
$$

### 6.2 总体直径期望

$$
\boxed{
 F_D(x,y)=
 \frac{2}{\beta(R^2-r_0^2)}
 \int_0^\beta\int_{r_0}^{R}
 \overline D(x,y;r\cos\psi,r\sin\psi)
 \,r\,dr\,d\psi.
 }
$$

把第一层期望展开后，面积和直径可以统一写成三重积分：

$$
\boxed{
 F_M(x,y)=
 \frac{1}{\varepsilon\beta(R^2-r_0^2)}
 \int_0^\beta\int_{r_0}^{R}
 \int_{-\varepsilon}^{\varepsilon}
 M(x,y;r\cos\psi,r\sin\psi,\delta)
 \,r\,d\delta\,dr\,d\psi,
 }
$$

其中 `M` 可以取 `A` 或 `D`。

这就是关于 `x,y,a,b` 的条件期望，以及进一步对整个第一扇形平均后的最终表达式。

### 6.3 直接以 `a,b` 为变量的完整期望

设源点在第一扇形环域

$$
\mathcal C_1=
\left\{(a,b):r_0\le\sqrt{a^2+b^2}\le R,
\quad 0\le\operatorname{atan2}(b,a)\le\beta\right\}
$$

内按平面面积均匀分布，则

$$
|\mathcal C_1|=\frac{\beta}{2}(R^2-r_0^2),
\qquad
f_{A,B}(a,b)=\frac{1}{|\mathcal C_1|}
\mathbf 1_{\{(a,b)\in\mathcal C_1\}}.
$$

对于固定的源点 `(a,b)`，先对测向误差取平均：

$$
\overline M(x,y;a,b)=
\frac{1}{2\varepsilon}
\int_{-\varepsilon}^{\varepsilon}
M(x,y;a,b,\delta)\,d\delta,
\qquad M\in\{A,D\}.
$$

因此完整的总体期望可以直接写成

$$
\boxed{
F_M(x,y)=
\frac{1}{2\varepsilon|\mathcal C_1|}
\iint_{\mathcal C_1}
\int_{-\varepsilon}^{\varepsilon}
M(x,y;a,b,\delta)\,d\delta\,da\,db.
}
$$

取 `M=A` 得到面积期望 `F_A(x,y)`，取 `M=D` 得到直径期望 `F_D(x,y)`。这个表达式说明了计算顺序：对每个可能的源位置和误差计算一次交集指标，再按联合概率密度平均，最后把结果看成 `(x,y)` 的函数。不能让 `a`、`b` 在一个矩形内独立均匀取样，因为这样会给第一扇形外的点分配概率。

### 6.4 极坐标形式与归一化因子

令

$$
a=r\cos\psi,\qquad b=r\sin\psi,
\qquad r_0\le r\le R,\quad 0\le\psi\le\beta.
$$

由于 `da db = r dr dpsi`，上式等价于

$$
\boxed{
F_M(x,y)=
\frac{1}{\varepsilon\beta(R^2-r_0^2)}
\int_0^\beta\int_{r_0}^{R}
\int_{-\varepsilon}^{\varepsilon}
M\left(x,y;r\cos\psi,r\sin\psi,\delta\right)
\,r\,d\delta\,dr\,d\psi.
}
$$

归一化因子来自

$$
\frac{1}{2\varepsilon}\cdot
\frac{1}{|\mathcal C_1|}
=
\frac{1}{2\varepsilon}\cdot
\frac{2}{\beta(R^2-r_0^2)}
=
\frac{1}{\varepsilon\beta(R^2-r_0^2)}.
$$

也可以先定义固定源点的误差平均

$$
\overline M\left(x,y;r\cos\psi,r\sin\psi\right)=
\frac{1}{2\varepsilon}
\int_{-\varepsilon}^{\varepsilon}
M\left(x,y;r\cos\psi,r\sin\psi,\delta\right)d\delta,
$$

再使用二重积分

$$
\boxed{
F_M(x,y)=
 \frac{2}{\beta(R^2-r_0^2)}
\int_0^\beta\int_{r_0}^{R}
\overline M\left(x,y;r\cos\psi,r\sin\psi\right)
\,r\,dr\,d\psi.
 }
$$

如果把第二探测点也用极坐标作为优化变量，则只需作坐标代换

$$
x=\rho\cos\vartheta,\qquad y=\rho\sin\vartheta.
$$

因此

$$
\boxed{
F_M^{\rm pol}(\rho,\vartheta)
=F_M(\rho\cos\vartheta,\rho\sin\vartheta).
}
$$

展开为源点积分就是

$$
\boxed{
F_M^{\rm pol}(\rho,\vartheta)=
\frac{1}{\varepsilon\beta(R^2-r_0^2)}
\int_0^\beta\int_{r_0}^{R}
\int_{-\varepsilon}^{\varepsilon}
M\left(
\rho,\vartheta;
 s\cos\psi,s\sin\psi,\delta
\right)
\,s\,d\delta\,ds\,d\psi.
}
$$

这里 `s` 是源点半径，`rho` 是第二探测点到 `S1` 的距离。对第二探测点位置进行优化时，不能再给 `F_M^{pol}` 额外乘一个 `rho`；`rho d rho d vartheta` 只有在题目要求对第二探测点位置本身再取平均时才是面积元。两种坐标下的优化问题完全等价：

$$
\arg\min_{(x,y)\in\mathcal F}F_M(x,y)
\quad\Longleftrightarrow\quad
\arg\min_{(\rho,\vartheta)\in\mathcal F_{\rm pol}}
F_M^{\rm pol}(\rho,\vartheta).
$$

在每个积分点中，真实方向角和实际中心线角分别为

$$
\phi(a,b;x,y)=\operatorname{atan2}(b-y,a-x),
\qquad
\theta=\phi+\delta.
$$

因此 `psi`、`phi`、`theta`、`delta` 的含义分别是源点相对 `S1` 的极角、从 `S2` 指向源点的真实方向角、第二次测得的中心线角和测量误差。第二扇形的两条边界方向是 `theta-epsilon` 与 `theta+epsilon`。

### 6.5 近场和无界情形

如果距离 `|G-S2|` 不超过 `5 m` 时普通测向失效，不能仍把该样本当作普通的 `2°` 扇形。记

$$
\mathcal C_{\rm near}(x,y)=
\mathcal C_1\cap B((x,y),5),
\qquad
\mathcal C_{\rm far}(x,y)=
\mathcal C_1\setminus B((x,y),5).
$$

若近场采用另一指标 `M_opt`，则总体期望应写成

$$
F_M(x,y)=\frac{1}{|\mathcal C_1|}
\left[
\frac{1}{2\varepsilon}
\iint_{\mathcal C_{\rm far}(x,y)}
\int_{-\varepsilon}^{\varepsilon}M_{\rm bearing}\,d\delta\,da\,db
 +
\iint_{\mathcal C_{\rm near}(x,y)}M_{\rm opt}\,da\,db
\right].
$$

若只研究成功进行普通测向的条件期望，分母应改成 `|C_far(x,y)|`。此外，若不加入有限先验裁剪而某个 `(x,y)` 导致无界交集在参数空间中持续存在，则该目标函数应记为 `+infinity`；若仅在孤立边界点无界，则仍需检查其邻域积分是否收敛，不能只因为单点概率为零就直接忽略。

### 6.6 源点落在第一扇形边界上的处理

第一扇形环域的边界包括

$$
\psi=0,\qquad \psi=\beta,\qquad r=r_0,\qquad r=R.
$$

在“按平面面积均匀分布”的假设下，这些边界都是二维面积为零的一维曲线，因此

$$
\Pr\{G\in\partial\mathcal C_1\}=0.
$$

所以在总体期望中，写成 `0<=psi<=beta`、`r0<=r<=R`，或者写成严格不等式，积分值完全相同；不需要为边界另加一项。极坐标积分中的端点也不会改变理论期望。

但对某个确定的边界源点，几何交集仍按闭区域处理。第一扇形的约束使用

$$
Y\ge0,\qquad kX-Y\ge0,
$$

而不是严格不等式。这样，当 `b=0` 或 `b=a tan(beta)` 时，源点仍被保留在第一扇形中，交集面积和直径由通用半平面交正常计算；源点在边界上可能使某一约束成为活动约束，或者使交集出现三角形、线段等退化拓扑，但不应被误判为空集。

同理，对于固定源点，若 `delta=+epsilon` 或 `delta=-epsilon`，真实方向恰好落在第二扇形的一条边界上。由于第二扇形也采用闭半平面，源点仍属于交集。这两个误差端点对连续均匀误差分布的概率同样为零，但在确定性求积或边界测试中应保留并使用容差判断。

数值实现可用带容差的非严格判断，例如对半平面函数 `C_i(P)` 接受

$$
C_i(P)\ge-\tau,
$$

其中 `tau` 根据坐标尺度和浮点精度设置。若使用包含区间端点的求积规则，端点样本只代表数值近似中的一个节点；提高求积阶数或采用分段加密即可使其对积分的影响收敛到零。

上述“边界概率为零”的结论依赖于源点对面积测度绝对连续。若题目另行规定源点有概率质量集中在某条边界上，则必须改用混合分布。例如边界质量为 `p_b`、内部面积分布质量为 `1-p_b` 时，应写成

$$
F_M(x,y)=(1-p_b)F_M^{\rm area}(x,y)
 +p_bF_M^{\rm boundary}(x,y),
$$

其中 `F_M^{boundary}` 由相应边界曲线上的线积分或离散概率加权得到。

### 6.7 靠近 `S1` 和外圆弧的源点

需要区分“恰好位于边界”和“位于边界附近”。前者在面积均匀分布下概率为零；后者是有正概率的带状区域，必须通过积分保留。

若用 `rho` 表示靠近 `S1` 的半径阈值，且 `r0<=rho<=R`，则近 `S1` 区域的概率为

$$
p_{S_1}(\rho)=
\Pr(r\le\rho)=
\frac{\rho^2-r_0^2}{R^2-r_0^2}.
$$

对应的期望可以拆成

$$
F_M(x,y)=
p_{S_1}(\rho)F_M^{\rm near\,S_1}(x,y;\rho)
 +[1-p_{S_1}(\rho)]F_M^{\rm far\,S_1}(x,y;\rho),
$$

其中两个 `F` 是分别在 `r0<=r<=rho` 和 `rho<r<=R` 上归一化后的条件期望。等价地，也可以直接把三重积分的半径区间拆成这两段，不需要添加额外的边界项。

如果题目规定源点距离 `S1` 不超过 `r_{S1}` 时第一处测向不可用，推荐将这部分从源分布中剔除，即令 `r0=r_{S1}`，并用新的环域面积重新归一化。若仍要保留这部分样本，则必须给出近 `S1` 的替代观测模型 `M_{S1,near}`：

$$
M(x,y;a,b,\delta)=
\begin{cases}
M_{S_1,\rm near}(x,y;a,b), & r\le r_{S_1},\\
M_{\rm bearing}(x,y;a,b,\delta), & r>r_{S_1}.
\end{cases}
$$

在 `r0=0`、`R=1500 m`、`r_{S1}=5 m` 且按面积均匀分布的例子中，

$$
p_{S_1}(5)=\frac{25}{1500^2}\approx1.11\times10^{-5}.
$$

因此这部分的概率约为 `0.0011%`，在 `M_{S1,near}` 有界且量级正常时数值影响通常很小；但“概率小”不能替代物理模型，若近场观测失效或指标发散，仍必须单独处理。

若“圆弧末端”指外圆弧 `r=R`，则恰好 `r=R` 的概率也是零，但外圆弧附近的带宽 `Delta` 有概率

$$
p_{\rm out}(\Delta)=
\Pr(R-\Delta\le r\le R)=
\frac{R^2-(R-\Delta)^2}{R^2-r_0^2},
\qquad 0\le\Delta\le R-r_0.
$$

例如 `R=1500 m`、`r0=0`、`Delta=5 m` 时，

$$
p_{\rm out}(5)=\frac{1500^2-1495^2}{1500^2}
\approx0.00666,
$$

即外圆弧向内 `5 m` 的环带约占 `0.666%`。这部分不应删除，而应使用原积分中的 `r` 权重正常计算。如果 `M` 在 `r=R` 附近连续且有界，将带宽缩小到 `Delta` 时，其贡献按上述概率相应缩小；如果外圆弧处会发生拓扑变化，则应在该半径附近加密求积并使用通用半平面交。

若用户所说的“扇形末端”是两条角边 `psi=0` 或 `psi=beta`，则角宽为 `h` 的边缘带概率为

$$
p_{\rm side}(h)=\frac{h}{\beta},
$$

只要径向范围仍为整个 `[r0,R]`。同样，边界射线本身概率为零，边缘带则必须保留在积分中。

## 7. 数值计算步骤

由于 `F_A` 和 `F_D` 含有几何拓扑变化和直径的最大值，实际求解建议使用确定性数值积分。

### 7.1 单个积分节点 `(r,psi,delta)`

给定 `x,y,r,psi,delta`：

1. 计算 `a=r cos(psi), b=r sin(psi)`。
2. 计算 `u,v,U_s,V_s,N_s`。
3. 构造四个半平面约束。
4. 求任意两条边界线的交点，并筛选满足全部约束的点。
5. 判断空集、退化、有限或无界。
6. 对有限交集排序顶点，计算面积和直径。

### 7.2 三重积分

可以使用 Gauss--Legendre 求积、Clenshaw--Curtis 求积或自适应 Simpson 积分。一个简单的张量积实现是：

```text
sumArea = 0
sumDiameter = 0
for each psi quadrature node in [0, beta]:
    for each r quadrature node in [r0, R]:
        for each delta quadrature node in [-epsilon, epsilon]:
            G = (r*cos(psi), r*sin(psi))
            (area, diameter) = intersectionMetric(x, y, G, delta)
            weight = w_psi * w_r * w_delta * r
            sumArea += weight * area
            sumDiameter += weight * diameter

FA = sumArea / (epsilon * beta * (R*R-r0*r0))
FD = sumDiameter / (epsilon * beta * (R*R-r0*r0))
```

这里的归一化来自

$$
 \frac{1}{2\varepsilon}\cdot
 \frac{2}{\beta(R^2-r_0^2)}
 =\frac{1}{\varepsilon\beta(R^2-r_0^2)}.
$$

若某个采样点对应无界交集，先检查它是否属于一段具有正测度的参数区域，而不是把孤立的边界点直接当成总体无穷期望。数值实现中更稳妥的做法是：对检测到无界的节点进行邻域加密；若无界状态在 `delta`、`r` 或 `psi` 方向上持续存在，则将该 `(x,y)` 的目标函数记为 `+infinity`。为了避免奇异点处理，也可以在优化前直接把 `S2` 限制在可保证所有误差和所有源位置都有界的候选区域内。

需要注意，孤立的无界参数点虽然概率为零，但其附近的面积或直径可能按 `1/|delta-delta0|` 等形式发散，从而使积分本身仍不收敛。因此遇到分母趋近零时，必须进行局部渐近检查或使用自适应积分，不能仅依据单个点的概率为零就忽略。

对于半径端点，数值积分可以采用两种稳妥做法。第一种是在 `r0` 附近和 `R` 附近分别增加求积节点；第二种是做变量代换

$$
u=r^2,qquad r\,dr=\frac12\,du,
$$

把径向积分改写为

$$
\int_{r_0}^{R}g(r)r\,dr
=\frac12\int_{r_0^2}^{R^2}g(\sqrt u)\,du.
$$

这种代换直接对应面积均匀分布，尤其适合 `r0=0` 的情形。若 `r=R` 附近存在拓扑变化，则可进一步把 `[r0^2,R^2]` 在 `R^2` 附近分段，并对最后一段使用更高阶或自适应求积。

### 7.3 Monte Carlo 计算

如果使用随机抽样估计 `F_A(x,y)` 或 `F_D(x,y)`，令 `U1,U2,U3` 独立服从 `U[0,1]`，取

$$
\psi=\beta U_1,
\qquad
r=\sqrt{r_0^2+(R^2-r_0^2)U_2},
\qquad
\delta=(2U_3-1)\varepsilon.
$$

然后令

$$
a=r\cos\psi,qquad b=r\sin\psi.
$$

这样得到的 `(a,b)` 才是在第一扇形内按平面面积均匀分布的样本。特别是，`r` 不能直接取为 `r0+(R-r0)U2`，因为那对应半径均匀而不是面积均匀。

对第 `i` 个样本计算通用交集指标

$$
M_i=M(x,y;a_i,b_i,\delta_i),
$$

则

$$
\widehat F_M(x,y)=\frac{1}{N}\sum_{i=1}^{N}M_i,
\qquad M\in\{A,D\}.
$$

在独立抽样且方差有限时，Monte Carlo 误差的典型阶数为 `O(N^{-1/2})`。因此增加样本数只能以平方根速度降低随机误差；优化时应固定同一批随机样本比较不同 `(x,y)`，以减少目标函数的抽样噪声。

### 7.4 对 `(x,y)` 优化

分别求解

$$
 (x_A^*,y_A^*)=\arg\min_{(x,y)\in\mathcal F}F_A(x,y),
$$

$$
 (x_D^*,y_D^*)=\arg\min_{(x,y)\in\mathcal F}F_D(x,y).
$$

推荐先在候选区域上做规则网格扫描，再以最优网格点为初值进行局部优化。由于目标函数可能在拓扑变化处不可导，不能只依赖梯度法；网格加局部无导数搜索更稳妥。

如果既关心面积又关心直径，可以分别报告两个最优点，也可以定义归一化加权目标：

$$
 J_\lambda(x,y)=
 \lambda\frac{F_A(x,y)}{A_0}
 +(1-\lambda)\frac{F_D(x,y)}{D_0},
 \qquad 0\le\lambda\le1,
$$

其中 `A0,D0` 是用于无量纲化的参考值。若没有额外偏好，面积和直径最好分别优化，避免人为选择权重影响结论。

## 8. 一个数值核对例子

### 8.1 关于 `(x,y)` 的总体函数形状

下面给出一个用于观察函数形状的数值扫描。参数取

$$
R=1500\ \mathrm m,\qquad r_0=5\ \mathrm m,
\qquad \beta=2^\circ,\qquad \varepsilon=1^\circ.
$$

第二探测点先限制在接收保证候选域

$$
\mathcal F_{\rm recv}=
D((0,0),1000)\cap D((1000,0),1000)
\cap D((1000\cos\beta,1000\sin\beta),1000).
$$

未加入 `1800 m` 先验圆盘裁剪时，为使所有源位置和误差下的两个扇形交集都有界，需要把第二扇形的全部可能方向与第一扇形方向区间分离。对当前源域（包含外圆弧 `r=R`）和误差区间，这给出文档第 2.2 节的两个分支；在此模型下，两个严格不等式是“对所有源位置和误差统一有界”的充分且必要条件。等号只表示临界接触，不能直接等同于总体期望发散。有限目标函数的定义域为

$$
\mathcal F_{\rm finite}
=\mathcal F_{\rm recv}\cap
\left[
\left\{x\sin\beta+y\cos\beta>R\sin(2\beta)\right\}
\cup
\left\{y\cos(2\beta)-x\sin(2\beta)<-R\sin(2\beta)\right\}
\right].
$$

它由上下两个互不相连的弧形分支组成。对不满足有界条件的中间区域，至少有一部分源位置和误差使两个扇形的无穷延伸方向重叠，因此原始（未裁剪）模型给出

$$
F_A(x,y)=F_D(x,y)=+\infty.
$$

条件的来源如下。下方分支要求所有真实方向满足 `phi<-2°`，因为误差取到 `+1°` 后第二扇形的上边界仍必须低于第一扇形下边界 `0°`。令 `alpha=-2°`，对任意源点 `G=(a,b)` 有

$$
\operatorname{cross}\bigl((\cos\alpha,\sin\alpha),G-S_2\bigr)\le0.
$$

在第一扇形内，该表达式关于源点的最大值在外圆弧端点 `r=R, psi=beta` 取得，因而

$$
R\sin(2\beta)-x\sin\beta-y\cos\beta<0,
$$

即

$$
x\sin\beta+y\cos\beta>R\sin(2\beta).
$$

上方分支要求所有真实方向满足 `phi>4°`，因为误差取到 `-1°` 后第二扇形的下边界仍必须高于第一扇形上边界 `beta=2°`。令 `alpha=4°=2beta`，对任意源点要求相应的叉积为正；其最不利源点在 `r=R, psi=0`，得到

$$
y\cos(2\beta)-x\sin(2\beta)<-R\sin(2\beta).
$$

等号意味着某个源点和误差端点下两个方向区间刚好接触，单个观测参数可能出现无界方向；但该源点和误差端点通常是零测度集合，不能直接把总体期望定义为 `+infinity`。有限域使用严格不等式可以保证每一个源位置和误差都得到有界交集，是便于优化的保守定义。若加入 `1800 m` 圆盘裁剪，等号处也会得到有限指标，但那时计算的是另一种裁剪模型。

下面两幅图是早期的定性扫描：在有限分支内部，对源极角 `psi`、面积变量 `r^2` 和误差 `delta` 做中点求积，并直接使用四边形公式。扫描步长为 `10 m`，求积节点数为

$$
N_\psi=16,\qquad N_{r^2}=32,\qquad N_\delta=16.
$$

得到的等值线图如下：

![关于 `(x,y)` 的面积期望等值线图](problem2_area_landscape.png)

![关于 `(x,y)` 的直径期望等值线图](problem2_diameter_landscape.png)

图中黑线是接收保证候选域，红线是采用严格有界条件得到的保守有限分支；两条红线之间的开区域是原始无穷扇形模型下具有正概率无界样本的区域，不应把其中的颜色理解为有限数值。红线本身是临界边界，严格数学处理需另做局部可积性检查；图中为便于扫描将其并入白色区域。颜色使用对数刻度，便于同时显示靠近无界边界的高值和内部的低值。

这两幅图只能用于观察总体函数的大致形状，不能作为严格的最终数值结果。四边形公式要求四个跨边界交点全部可行；当源点靠近 `S1`、第二扇形靠近第一扇形顶点、边界平行或交集退化时，实际交集可能是三角形或线段。严格计算必须对

$$
\mathcal V=\{S_1,S_2,P_-,P_+,Q_-,Q_+\}
$$

逐点进行四个半平面筛选，并对筛选后的凸多边形计算鞋带面积和顶点对最大距离。因而下述网格最小值应标记为“四边形近似扫描值”，不能当作通用半平面交算法的精确最优值。

数值结果表明：

1. `F_A(x,y)` 和 `F_D(x,y)` 都在有限分支的有界性边界附近迅速增大，理论上靠近方向区间刚好接触的位置会出现发散趋势。
2. 从左侧 `x` 较小的位置向右侧移动，期望整体下降；但由于接收候选域在 `x=1000` 附近收缩，最优点不在几何域的端点，而在右侧内部。
3. 问题关于第一扇形的角平分线 `psi=beta/2` 具有反射对称性。图中上下两个分支看起来不关于 `x` 轴完全对称，是因为角平分线相对 `x` 轴旋转了 `1°`。

在上述网格和求积精度下，面积期望的网格最小值约为

$$
F_A\approx8.34\times10^2\ \mathrm{m^2},
$$

位置约为

$$
(x,y)\approx(960,-280)\ \mathrm m,
$$

其关于角平分线的对称点约为 `(951,310) m`。直径期望的网格最小值约为

$$
F_D\approx63.4\ \mathrm m,
$$

位置约为

$$
(x,y)\approx(860,-510)\ \mathrm m,
$$

其对称点约为 `(851,524) m`。

这些数值是默认分布、未裁剪无穷扇形模型下的四边形近似扫描值，不能代替通用半平面交、提高求积阶数和更细网格后的最终优化结果。若把半径 `1800 m` 的先验圆盘加入候选区域裁剪，则中间的 `+infinity` 区域会被消除，目标函数的形状和最优点也会改变，必须另行计算。

### 8.2 `+infinity` 区域的来源

这里的 `+infinity` 不是数值溢出，而是模型中的扩展实数结果。第一扇形的无穷延伸方向区间为

$$
I_1=[0,\beta].
$$

若固定源点的真实方向角为 `phi`，第二次误差为 `delta`，则第二扇形的方向区间为

$$
I_2(\phi,\delta)=
[\phi+\delta-\varepsilon,\ \phi+\delta+\varepsilon].
$$

只要 `I1` 与 `I2` 有共同方向，两个扇形的半平面交就存在共同的无穷延伸方向。由于真实源点 `G` 本身位于两个扇形交集中，交集非空；沿共同方向继续前进，仍满足四个半平面约束，所以交集无界，面积和直径均为 `+infinity`。

对固定 `phi`，存在某个 `delta in [-epsilon,epsilon]` 使两个方向区间相交，当且仅当

$$
-2\varepsilon\le\phi\le\beta+2\varepsilon,
$$

即当前参数下

$$
-2^\circ\le\phi\le4^\circ.
$$

如果一组源位置具有 `phi in (-2°,4°)`，则对应的误差 `delta` 不是单个端点，而是一个正长度区间。因此 `M=+infinity` 发生在 `(a,b,delta)` 参数空间的正测度集合上，进而

$$
\Pr\{M=+\infty\}>0
\quad\Longrightarrow\quad
E[M]=+\infty.
$$

在接收保证候选域内，所有源点方向都低于 `-2°` 的下方有限分支为

$$
x\sin\beta+y\cos\beta>R\sin(2\beta),
$$

所有源点方向都高于 `4°` 的上方有限分支为

$$
y\cos(2\beta)-x\sin(2\beta)<-R\sin(2\beta).
$$

因此两条边界之间的 `+infinity` 带为

$$
\boxed{
\frac{x\sin(2\beta)-R\sin(2\beta)}{\cos(2\beta)}
< y <
\frac{R\sin(2\beta)-x\sin\beta}{\cos\beta}
}
$$

再与 `F_recv` 求交。取 `R=1500 m`、`beta=2°`，它约为

$$
0.06993x-104.89<y<104.70-0.03492x.
$$

例如在 `x=500 m` 处，中间无穷带约为

$$
-87.4<y<87.2\ \mathrm m;
$$

在 `x=100 m` 处约为 `-97.9<y<101.2 m`，但还要同时满足三个接收圆盘约束。按圆盘交集的几何面积估算，该无穷带在接收候选域中约占 `12.9%`，其位置就是等值线图中间的白色区域。

等号边界需要单独说明：等号时通常只有某个源点和某个误差端点使两个方向区间刚好接触，单个参数点本身是零概率；有限交集的顶点会随着角度间隙趋近于零而趋向无穷，面积或直径通常具有 `1/gap` 型发散，但在源点和误差的联合积分中是否发散必须通过局部渐近或自适应积分判断，不能无条件断言。数值扫描将等号并入 `+infinity` 区域，只是为了使用“所有情形逐点有界”的保守定义。若采用 `1800 m` 先验圆盘裁剪，交集不再无界，临界带会变成有限但通常较大的高值带，必须按裁剪后的几何区域重新计算。

下面的数值用于检查公式和程序实现，不代表最终最优探测点。取

$$
 S_2=(500,500),
 \qquad
 G=(1000\cos1^\circ,1000\sin1^\circ).
$$

对每个 `delta` 使用四边形公式，并在 `[-1°,1°]` 上作均匀积分，得到

$$
 \overline A\approx1200.3189\ \mathrm{m^2},
 \qquad
 \overline D\approx77.7000\ \mathrm m.
$$

若固定为无误差情形 `delta=0`，则约为

$$
 A\approx1199.4687\ \mathrm{m^2},
 \qquad
 D\approx77.6686\ \mathrm m.
$$

这个例子说明，角度误差平均后的面积和直径一般会与零误差值不同。正式计算前仍要用通用半平面交检查每个采样点的拓扑类型。

### 8.3 直角坐标与极坐标的一致性验证

为了检查坐标替换是否引入代数错误，对同一批随机样本分别使用两套公式计算。样本满足：第二探测点位于接收候选域内，源点写成

$$
G=(s\cos\psi,s\sin\psi),
$$

并且样本被限制在四个跨边界点均可行、分母不接近零的非退化四边形分支。直角坐标版本使用

$$
x=\rho\cos\vartheta,qquad y=\rho\sin\vartheta,
$$

极坐标版本使用

$$
p_\sigma=\rho\frac{\sin(\alpha_\sigma-\vartheta)}{\sin\alpha_\sigma},qquad
q_\sigma=\rho\frac{\sin(\alpha_\sigma-\vartheta)}
{\sin\alpha_\sigma-k\cos\alpha_\sigma}.
$$

在 `100000` 个随机样本上，双精度计算得到的最大绝对差为：

| 比较量 | 最大绝对差 |
| --- | ---: |
| 四个交点坐标 | `2.37e-8 m` |
| 四边形面积 | `5.96e-7 m^2` |
| 四边形直径 | `1.30e-8 m` |

这些误差处于浮点舍入和三角函数求值误差的量级。代数上，一致性来自

$$
x\sin\alpha_\sigma-y\cos\alpha_\sigma
=\rho\sin(\alpha_\sigma-\vartheta),
$$

以及

$$
(a-x)^2+(b-y)^2
=s^2+\rho^2-2s\rho\cos(\psi-\vartheta).
$$

因此，直角坐标版和极坐标版不是两个不同的近似模型，而是同一个几何模型的两种参数化。坐标变换不会消除半平面筛选、退化判断和无界判断；这些判断仍必须在两套实现中一致执行。

对前面的固定例子，直接对 `delta` 作数值积分得到

$$
\overline A=1200.318901\ \mathrm{m^2},
\qquad
\overline D\approx77.7000\ \mathrm m.
$$

把同一例子代入极坐标闭式，其中

$$
\rho=\sqrt{500^2+500^2},
\qquad
\vartheta=45^\circ,
$$

仍得到 `1200.318901 m^2`。这里必须使用闭式中的系数
`\rho^2\sin^2\vartheta=y^2`；若误将它替换成
`(b-y)^2`，结果会变成约 `579.3703 m^2`，与直接积分不一致。

## 9. 边界和物理条件的处理

### 9.1 `5 m` 近场

题目规定距离探测点不超过 `5 m` 时信号过强，无法正常取得方位。计算时有两种一致的处理方式：

- 将源区域内 `r<=5` 的部分删除，使用 `r0=5`；
- 保留该部分，但把对应观测改成光学定位，并使用单独的误差模型。

不能在积分中把近场样本随意当成普通的 `2°` 测向样本。

### 9.2 `20 m` 光学清除半径

若采用光学清除，应明确它是对候选区域施加的安全约束，还是另一个定位观测。一个 `20 m` 乘 `20 m` 的方形区域直径为

$$
20\sqrt 2\approx28.28\text{ m},
$$

不能把其直径写成 `20 m`。如果使用圆形清除区，则直径才是 `40 m`。

### 9.3 有限先验裁剪

题目给出的干扰源全局圆盘半径 `1800 m` 是对干扰源位置的先验约束。是否把它用于裁剪“定位候选区域”，需要在模型中单独声明：

- 不裁剪：计算两个无限扇形的原始交集；可能出现无界，面积和直径为无穷大；
- 裁剪：计算 `W1 ∩ W2 ∩ D((0,0),1800)` 或其他明确的先验区域，结果始终有限，但指标会改变。

两种结果不能混用。

## 10. 结果应如何报告

对于每个候选点 `(x,y)`，至少报告：

1. `F_A(x,y)`：对源位置和第二次角度误差平均后的候选区域面积，单位 `m^2`；
2. `F_D(x,y)`：同样平均后的候选区域直径，单位 `m`；
3. 无界样本比例；
4. 使用的源位置分布、误差分布、`r0` 和是否进行 `1800 m` 先验裁剪；
5. 面积最优点和直径最优点，必要时给出两者在另一指标下的数值。

这样得到的结论才具有可复现性：改变误差分布、近场处理或先验裁剪方式时，可以清楚地知道是哪一个建模假设导致了结果变化。
