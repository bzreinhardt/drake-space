function dxdot = inspectorDynamics(theta,u1,u2,x,y)
%INSPECTORDYNAMICS
%    DXDOT = INSPECTORDYNAMICS(THETA,U1,U2,X,Y)

%    This function was generated by the Symbolic Math Toolbox version 6.0.
%    05-Dec-2014 10:22:51

t2 = sqrt(2.0);
t4 = cos(theta);
t5 = t2.*t4.*(1.0./2.0e1);
t6 = sin(theta);
t7 = t2.*t6.*(1.0./2.0e1);
t3 = t5+t7+x;
t8 = -t5+t7+y+5.0;
t9 = t3.^2;
t10 = t8.^2;
t11 = t9+t10;
t12 = u1.^2;
t13 = -t5+t7+x;
t14 = t5+t7-y-5.0;
t15 = t13.^2;
t16 = t5+t7-y-5.0;
t17 = u2.^2;
t18 = sqrt(t11);
t19 = t5+t7-y-5.0;
t20 = t5+t7-y-5.0;
t21 = x.*2.0;
t22 = t2.*t4.*(1.0./1.0e1);
t23 = t2.*t6.*(1.0./1.0e1);
t57 = t18.*4.41463e1;
t24 = -t57+2.207315e2;
t25 = exp(t24);
t26 = t21+t22+t23;
t27 = u1.*4.7160845866e-4;
t28 = t12.*3.844372009e-11;
t59 = t12.*u1.*3.485746657e-13;
t29 = t27+t28-t59;
t30 = t5+t7-y-5.0;
t31 = t5+t7-y-5.0;
t32 = t21-t22+t23;
t33 = u2.*4.7160845866e-4;
t34 = t17.*3.844372009e-11;
t62 = t17.*u2.*3.485746657e-13;
t35 = t33+t34-t62;
t49 = t18.*3.89901e1;
t36 = -t49+1.730911e2;
t37 = exp(t36);
t38 = 1.0./t11;
t39 = t12.*6.301449255e1;
t40 = t12.*u1.*7.66709574e-7;
t41 = t12.^2;
t51 = u1.*1.0074136425e1;
t52 = t41.*1.964786265e-7;
t42 = t39+t40-t51-t52;
t43 = t5+t7-y-5.0;
t44 = t5+t7-y-5.0;
t45 = t17.*6.301449255e1;
t46 = t17.*u2.*7.66709574e-7;
t47 = t17.^2;
t55 = u2.*1.0074136425e1;
t56 = t47.*1.964786265e-7;
t48 = t45+t46-t55-t56;
t50 = 1.0./t11.^(3.0./2.0);
t53 = t5+t7-y-5.0;
t54 = t5+t7-y-5.0;
t58 = 1.0./sqrt(t11);
t60 = t5+t7-y-5.0;
t61 = t5+t7-y-5.0;
t63 = t5+t7-y-5.0;
t64 = t5+t7-y-5.0;
t65 = y.*2.0;
t66 = -t22+t23+t65+1.0e1;
t67 = t5+t7-y-5.0;
t68 = t5+t7-y-5.0;
t69 = t5+t7-y-5.0;
t70 = t5+t7-y-5.0;
t71 = t5+t7-y-5.0;
t72 = t5+t7-y-5.0;
t73 = t5+t7-y-5.0;
t74 = t5+t7-y-5.0;
t75 = t5+t7;
t76 = t5+t7-y-5.0;
t77 = t5+t7-y-5.0;
t78 = t5-t7;
t79 = t5+t7-y-5.0;
t80 = t5+t7-y-5.0;
t81 = t3.*t78.*2.0;
t82 = t8.*t75.*2.0;
t83 = t81+t82;
t84 = t5+t7-y-5.0;
t85 = t5+t7-y-5.0;
t86 = t13.*t75.*2.0;
t87 = t14.*t78.*2.0;
t88 = t86+t87;
t89 = t5+t7-y-5.0;
t90 = t5+t7-y-5.0;
t91 = t5+t7-y-5.0;
t92 = t5+t7-y-5.0;
t93 = t5+t7-y-5.0;
t94 = t5+t7-y-5.0;
t95 = t5+t7-y-5.0;
t96 = t5+t7-y-5.0;
t97 = t5+t7-y-5.0;
t98 = t5+t7-y-5.0;
t99 = t5+t7-y-5.0;
t100 = t5+t7-y-5.0;
t101 = t5+t7-y-5.0;
t102 = t5+t7-y-5.0;
t103 = t5+t7-y-5.0;
t104 = t5+t7-y-5.0;
t105 = t5+t7-y-5.0;
t106 = t5+t7-y-5.0;
t107 = t5+t7-y-5.0;
t108 = t5+t7-y-5.0;
t109 = t5+t7-y-5.0;
t110 = t5+t7-y-5.0;
t111 = t5+t7-y-5.0;
t112 = t5+t7-y-5.0;
t113 = t5+t7-y-5.0;
t114 = t5+t7-y-5.0;
t115 = t22+t23-t65-1.0e1;
t116 = t5+t7-y-5.0;
t117 = t5+t7-y-5.0;
t118 = t5+t7-y-5.0;
t119 = t5+t7-y-5.0;
t120 = t5+t7-y-5.0;
t121 = t5+t7-y-5.0;
t122 = t5+t7-y-5.0;
t123 = t5+t7-y-5.0;
t124 = t5+t7-y-5.0;
t125 = t5+t7-y-5.0;
t126 = t5+t7-y-5.0;
t127 = t5+t7-y-5.0;
t128 = t5+t7-y-5.0;
t129 = t5+t7-y-5.0;
t130 = t78.*(t5+t7-y-5.0).*2.0;
t131 = t86+t130;
t132 = u1.*1.260289851e2;
t133 = t12.*2.300128722e-6;
t242 = t12.*u1.*7.85914506e-7;
t134 = t132+t133-t242-1.0074136425e1;
t135 = u1.*7.688744017999999e-11;
t244 = t12.*1.0457239971e-12;
t136 = t135-t244+4.7160845866e-4;
t137 = t5+t7-y-5.0;
t138 = t5+t7-y-5.0;
t139 = u2.*1.260289851e2;
t140 = t17.*2.300128722e-6;
t251 = t17.*u2.*7.85914506e-7;
t141 = t139+t140-t251-1.0074136425e1;
t142 = t5+t7-y-5.0;
t143 = t5+t7-y-5.0;
t144 = u2.*7.688744017999999e-11;
t254 = t17.*1.0457239971e-12;
t145 = t144-t254+4.7160845866e-4;
t146 = t25.*t29.*t58;
t147 = t5+t7-y-5.0;
t148 = t5+t7-y-5.0;
t149 = t5+t7-y-5.0;
t150 = t5+t7-y-5.0;
t151 = t5+t7-y-5.0;
t152 = t5+t7-y-5.0;
t153 = t5+t7-y-5.0;
t154 = t5+t7-y-5.0;
t155 = t5+t7-y-5.0;
t156 = t5+t7-y-5.0;
t157 = t37.*t58.*(t39+t40-t51-t52);
t158 = t8.*t25.*t26.*t29.*t38.*2.207315e1;
t159 = t8.*t25.*t26.*t29.*t50.*(1.0./2.0);
t160 = t5+t7-y-5.0;
t161 = t5+t7-y-5.0;
t162 = t5+t7-y-5.0;
t163 = t5+t7-y-5.0;
t164 = t5+t7-y-5.0;
t165 = t5+t7-y-5.0;
t166 = t5+t7-y-5.0;
t167 = t5+t7-y-5.0;
t168 = t5+t7-y-5.0;
t169 = t5+t7-y-5.0;
t170 = t5+t7-y-5.0;
t171 = t5+t7-y-5.0;
t172 = t5+t7-y-5.0;
t173 = t5+t7-y-5.0;
t174 = -t5+t7+y;
t175 = t8.*t25.*t29.*t38.*t66.*2.207315e1;
t176 = t8.*t25.*t29.*t50.*t66.*(1.0./2.0);
t177 = t37.*t42.*t58;
t178 = t5+t7-y-5.0;
t179 = t5+t7-y-5.0;
t180 = t5+t7-y-5.0;
t181 = t5+t7-y-5.0;
t182 = t5+t7-y-5.0;
t183 = t5+t7-y-5.0;
t184 = t5+t7-y-5.0;
t185 = t5+t7-y-5.0;
t186 = t5+t7-y-5.0;
t187 = t5+t7-y-5.0;
t188 = t5+t7-y;
t189 = t5+t7-y-5.0;
t190 = t5+t7-y-5.0;
t191 = t5+t7-y-5.0;
t192 = t5+t7-y-5.0;
t193 = t5+t7-y-5.0;
t194 = t5+t7-y-5.0;
t195 = t5+t7-y-5.0;
t196 = t5+t7-y-5.0;
t197 = t5+t7-y-5.0;
t198 = t5+t7-y-5.0;
t199 = t5+t7-y-5.0;
t200 = t5+t7-y-5.0;
t201 = t5+t7-y-5.0;
t202 = t5+t7-y-5.0;
t203 = t8.*t25.*t29.*t58;
t204 = t3.*t25.*t29.*t58;
t205 = t8.*t37.*t58.*(t39+t40-t51-t52);
t206 = t5+t7-y-5.0;
t207 = t5+t7-y-5.0;
t208 = t5+t7-y-5.0;
t209 = t5+t7-y-5.0;
t210 = t5+t7-y-5.0;
t211 = t5+t7-y-5.0;
t212 = t5+t7-y-5.0;
t213 = t5+t7-y-5.0;
t214 = t5+t7-y-5.0;
t215 = t5+t7-y-5.0;
t216 = t5+t7-y-5.0;
t217 = t5+t7-y-5.0;
t218 = t5+t7-y-5.0;
t219 = t5+t7-y-5.0;
t220 = t5+t7-y-5.0;
t221 = t5+t7-y-5.0;
t222 = t5+t7-y-5.0;
t223 = t5+t7-y-5.0;
t224 = t5+t7-y-5.0;
t225 = t5+t7-y-5.0;
t226 = t25.*t29.*t58.*t78;
t227 = t5+t7-y-5.0;
t228 = t5+t7-y-5.0;
t229 = t5+t7-y-5.0;
t230 = t5+t7-y-5.0;
t231 = t5+t7-y-5.0;
t232 = t5+t7-y-5.0;
t233 = t5+t7-y-5.0;
t234 = t5+t7-y-5.0;
t235 = t5+t7-y-5.0;
t236 = t5+t7-y-5.0;
t237 = t5+t7-y-5.0;
t238 = t5+t7-y-5.0;
t239 = t37.*t58.*t78.*(t39+t40-t51-t52);
t240 = t8.*t25.*t29.*t38.*t83.*2.207315e1;
t241 = t8.*t25.*t29.*t50.*t83.*(1.0./2.0);
t243 = t8.*t37.*t58.*t134;
t245 = t3.*t25.*t58.*t136;
t246 = t243+t245;
t247 = t3.*t37.*t58.*t134;
t248 = t247-t8.*t25.*t58.*t136;
t249 = t5+t7-y-5.0;
t250 = t5+t7-y-5.0;
t252 = t5+t7-y-5.0;
t253 = t5+t7-y-5.0;
t255 = t5+t7-y-5.0;
t256 = t5+t7-y-5.0;
t257 = t5+t7-y-5.0;
t258 = t5+t7-y-5.0;
dxdot = reshape([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t158+t159+t177+t48.*exp(sqrt(t15+t14.^2).*(-3.89901e1)+1.730911e2).*1.0./sqrt(t15+t16.^2)-t3.*t26.*t37.*t38.*t42.*1.949505e1-t3.*t26.*t37.*t42.*t50.*(1.0./2.0)-(t14.*t32.*t35.*exp(sqrt(t15+t19.^2).*(-4.41463e1)+2.207315e2).*2.207315e1)./(t15+t20.^2)-t14.*t32.*t35.*exp(sqrt(t15+t30.^2).*(-4.41463e1)+2.207315e2).*1.0./(t15+t31.^2).^(3.0./2.0).*(1.0./2.0)-(t13.*t32.*t48.*exp(sqrt(t15+t43.^2).*(-3.89901e1)+1.730911e2).*1.949505e1)./(t15+t44.^2)-t13.*t32.*t48.*exp(sqrt(t15+t53.^2).*(-3.89901e1)+1.730911e2).*1.0./(t15+t54.^2).^(3.0./2.0).*(1.0./2.0),t146+t35.*exp(sqrt(t15+t97.^2).*(-4.41463e1)+2.207315e2).*1.0./sqrt(t15+t98.^2)+(t32.*exp(sqrt(t15+t103.^2).*(-3.89901e1)+1.730911e2).*(t5+t7-y-5.0).*(t45+t46-t55-t56).*1.949505e1)./(t15+t104.^2)+t32.*exp(sqrt(t15+t105.^2).*(-3.89901e1)+1.730911e2).*1.0./(t15+t106.^2).^(3.0./2.0).*(t5+t7-y-5.0).*(t45+t46-t55-t56).*(1.0./2.0)-t3.*t25.*t26.*t29.*t38.*2.207315e1-t3.*t25.*t26.*t29.*t50.*(1.0./2.0)-t8.*t26.*t37.*t38.*t42.*1.949505e1-t8.*t26.*t37.*t42.*t50.*(1.0./2.0)-(t13.*t32.*t35.*exp(sqrt(t15+t99.^2).*(-4.41463e1)+2.207315e2).*2.207315e1)./(t15+t100.^2)-t13.*t32.*t35.*exp(sqrt(t15+t101.^2).*(-4.41463e1)+2.207315e2).*1.0./(t15+t102.^2).^(3.0./2.0).*(1.0./2.0),t204+t205-t174.*(t157+t158+t159-t3.*t26.*t37.*t38.*t42.*1.949505e1-t3.*t26.*t37.*t42.*t50.*(1.0./2.0))-t188.*(-t48.*exp(sqrt(t15+t160.^2).*(-3.89901e1)+1.730911e2).*1.0./sqrt(t15+t161.^2)+(t14.*t32.*t35.*exp(sqrt(t15+t162.^2).*(-4.41463e1)+2.207315e2).*2.207315e1)./(t15+t163.^2)+t14.*t32.*t35.*exp(sqrt(t15+t164.^2).*(-4.41463e1)+2.207315e2).*1.0./(t15+t165.^2).^(3.0./2.0).*(1.0./2.0)+(t13.*t32.*t48.*exp(sqrt(t15+t166.^2).*(-3.89901e1)+1.730911e2).*1.949505e1)./(t15+t167.^2)+t13.*t32.*t48.*exp(sqrt(t15+t168.^2).*(-3.89901e1)+1.730911e2).*1.0./(t15+t169.^2).^(3.0./2.0).*(1.0./2.0))+t13.*(t35.*exp(sqrt(t15+t147.^2).*(-4.41463e1)+2.207315e2).*1.0./sqrt(t15+t148.^2)+(t32.*exp(sqrt(t15+t153.^2).*(-3.89901e1)+1.730911e2).*(t5+t7-y-5.0).*(t45+t46-t55-t56).*1.949505e1)./(t15+t154.^2)+t32.*exp(sqrt(t15+t155.^2).*(-3.89901e1)+1.730911e2).*1.0./(t15+t156.^2).^(3.0./2.0).*(t5+t7-y-5.0).*(t45+t46-t55-t56).*(1.0./2.0)-(t13.*t32.*t35.*exp(sqrt(t15+t149.^2).*(-4.41463e1)+2.207315e2).*2.207315e1)./(t15+t150.^2)-t13.*t32.*t35.*exp(sqrt(t15+t151.^2).*(-4.41463e1)+2.207315e2).*1.0./(t15+t152.^2).^(3.0./2.0).*(1.0./2.0))-t3.*(-t146+t3.*t25.*t26.*t29.*t38.*2.207315e1+t3.*t25.*t26.*t29.*t50.*(1.0./2.0)+t8.*t26.*t37.*t38.*t42.*1.949505e1+t8.*t26.*t37.*t42.*t50.*(1.0./2.0))+t13.*t35.*exp(sqrt(t15+t170.^2).*(-4.41463e1)+2.207315e2).*1.0./sqrt(t15+t171.^2)-t14.*t48.*exp(sqrt(t15+t172.^2).*(-3.89901e1)+1.730911e2).*1.0./sqrt(t15+t173.^2),0.0,0.0,0.0,t175+t176-t25.*t29.*t58-t35.*exp(sqrt(t15+t60.^2).*(-4.41463e1)+2.207315e2).*1.0./sqrt(t15+t61.^2)+(t35.*t115.*exp(sqrt(t15+t63.^2).*(-4.41463e1)+2.207315e2).*(t5+t7-y-5.0).*2.207315e1)./(t15+t64.^2)+t35.*t115.*exp(sqrt(t15+t67.^2).*(-4.41463e1)+2.207315e2).*1.0./(t15+t68.^2).^(3.0./2.0).*(t5+t7-y-5.0).*(1.0./2.0)+(t13.*t115.*exp(sqrt(t15+t69.^2).*(-3.89901e1)+1.730911e2).*(t45+t46-t55-t56).*1.949505e1)./(t15+t70.^2)+t13.*t115.*exp(sqrt(t15+t71.^2).*(-3.89901e1)+1.730911e2).*1.0./(t15+t72.^2).^(3.0./2.0).*(t45+t46-t55-t56).*(1.0./2.0)-t3.*t37.*t38.*t42.*t66.*1.949505e1-t3.*t37.*t42.*t50.*t66.*(1.0./2.0),t157+exp(sqrt(t15+t107.^2).*(-3.89901e1)+1.730911e2).*1.0./sqrt(t15+t108.^2).*(t45+t46-t55-t56)-t3.*t25.*t29.*t38.*t66.*2.207315e1-t3.*t25.*t29.*t50.*t66.*(1.0./2.0)-t8.*t37.*t38.*t42.*t66.*1.949505e1-t8.*t37.*t42.*t50.*t66.*(1.0./2.0)+(t13.*t35.*t115.*exp(sqrt(t15+t109.^2).*(-4.41463e1)+2.207315e2).*2.207315e1)./(t15+t110.^2)+t13.*t35.*t115.*exp(sqrt(t15+t111.^2).*(-4.41463e1)+2.207315e2).*1.0./(t15+t112.^2).^(3.0./2.0).*(1.0./2.0)-(t14.*t48.*t115.*exp(sqrt(t15+t113.^2).*(-3.89901e1)+1.730911e2).*1.949505e1)./(t15+t114.^2)-t14.*t48.*t115.*exp(sqrt(t15+t116.^2).*(-3.89901e1)+1.730911e2).*1.0./(t15+t117.^2).^(3.0./2.0).*(1.0./2.0),t203+t13.*(t48.*exp(sqrt(t15+t178.^2).*(-3.89901e1)+1.730911e2).*1.0./sqrt(t15+t179.^2)+(t13.*t35.*t115.*exp(sqrt(t15+t180.^2).*(-4.41463e1)+2.207315e2).*2.207315e1)./(t15+t181.^2)+t13.*t35.*t115.*exp(sqrt(t15+t182.^2).*(-4.41463e1)+2.207315e2).*1.0./(t15+t183.^2).^(3.0./2.0).*(1.0./2.0)-(exp(sqrt(t15+t184.^2).*(-3.89901e1)+1.730911e2).*(t22+t23-t65-1.0e1).*(t5+t7-y-5.0).*(t45+t46-t55-t56).*1.949505e1)./(t15+t185.^2)-exp(sqrt(t15+t186.^2).*(-3.89901e1)+1.730911e2).*1.0./(t15+t187.^2).^(3.0./2.0).*(t22+t23-t65-1.0e1).*(t5+t7-y-5.0).*(t45+t46-t55-t56).*(1.0./2.0))+t188.*(-t35.*exp(sqrt(t15+t189.^2).*(-4.41463e1)+2.207315e2).*1.0./sqrt(t15+t190.^2)+(t13.*exp(sqrt(t15+t195.^2).*(-3.89901e1)+1.730911e2).*(t22+t23-t65-1.0e1).*(t45+t46-t55-t56).*1.949505e1)./(t15+t196.^2)+t13.*exp(sqrt(t15+t197.^2).*(-3.89901e1)+1.730911e2).*1.0./(t15+t198.^2).^(3.0./2.0).*(t22+t23-t65-1.0e1).*(t45+t46-t55-t56).*(1.0./2.0)+(t35.*exp(sqrt(t15+t191.^2).*(-4.41463e1)+2.207315e2).*(t22+t23-t65-1.0e1).*(t5+t7-y-5.0).*2.207315e1)./(t15+t192.^2)+t35.*exp(sqrt(t15+t193.^2).*(-4.41463e1)+2.207315e2).*1.0./(t15+t194.^2).^(3.0./2.0).*(t22+t23-t65-1.0e1).*(t5+t7-y-5.0).*(1.0./2.0))+t174.*(t146-t175-t176+t3.*t37.*t38.*t42.*t66.*1.949505e1+t3.*t37.*t42.*t50.*t66.*(1.0./2.0))-t3.*(-t177+t3.*t25.*t29.*t38.*t66.*2.207315e1+t3.*t25.*t29.*t50.*t66.*(1.0./2.0)+t8.*t37.*t38.*t42.*t66.*1.949505e1+t8.*t37.*t42.*t50.*t66.*(1.0./2.0))-t3.*t37.*t42.*t58-t14.*t35.*exp(sqrt(t15+t199.^2).*(-4.41463e1)+2.207315e2).*1.0./sqrt(t15+t200.^2)-t13.*t48.*exp(sqrt(t15+t201.^2).*(-3.89901e1)+1.730911e2).*1.0./sqrt(t15+t202.^2),0.0,0.0,0.0,t239+t240+t241-t25.*t29.*t58.*t75+t35.*t78.*exp(sqrt(t15+t76.^2).*(-4.41463e1)+2.207315e2).*1.0./sqrt(t15+t77.^2)+t75.*exp(sqrt(t15+t73.^2).*(-3.89901e1)+1.730911e2).*1.0./sqrt(t15+t74.^2).*(t45+t46-t55-t56)-t3.*t37.*t38.*t42.*t83.*1.949505e1-t3.*t37.*t42.*t50.*t83.*(1.0./2.0)-(t14.*t35.*t88.*exp(sqrt(t15+t79.^2).*(-4.41463e1)+2.207315e2).*2.207315e1)./(t15+t80.^2)-t14.*t35.*t88.*exp(sqrt(t15+t84.^2).*(-4.41463e1)+2.207315e2).*1.0./(t15+t85.^2).^(3.0./2.0).*(1.0./2.0)-(t13.*t48.*t88.*exp(sqrt(t15+t89.^2).*(-3.89901e1)+1.730911e2).*1.949505e1)./(t15+t90.^2)-t13.*t48.*t88.*exp(sqrt(t15+t91.^2).*(-3.89901e1)+1.730911e2).*1.0./(t15+t92.^2).^(3.0./2.0).*(1.0./2.0),t226+t35.*t75.*exp(sqrt(t15+t120.^2).*(-4.41463e1)+2.207315e2).*1.0./sqrt(t15+t121.^2)-t48.*t78.*exp(sqrt(t15+t118.^2).*(-3.89901e1)+1.730911e2).*1.0./sqrt(t15+t119.^2)+t37.*t58.*t75.*(t39+t40-t51-t52)+(t131.*exp(sqrt(t15+t126.^2).*(-3.89901e1)+1.730911e2).*(t5+t7-y-5.0).*(t45+t46-t55-t56).*1.949505e1)./(t15+t127.^2)+t131.*exp(sqrt(t15+t128.^2).*(-3.89901e1)+1.730911e2).*1.0./(t15+t129.^2).^(3.0./2.0).*(t5+t7-y-5.0).*(t45+t46-t55-t56).*(1.0./2.0)-t3.*t25.*t29.*t38.*t83.*2.207315e1-t3.*t25.*t29.*t50.*t83.*(1.0./2.0)-t8.*t37.*t38.*t42.*t83.*1.949505e1-t8.*t37.*t42.*t50.*t83.*(1.0./2.0)-(t13.*t35.*t88.*exp(sqrt(t15+t122.^2).*(-4.41463e1)+2.207315e2).*2.207315e1)./(t15+t123.^2)-t13.*t35.*t88.*exp(sqrt(t15+t124.^2).*(-4.41463e1)+2.207315e2).*1.0./(t15+t125.^2).^(3.0./2.0).*(1.0./2.0),t78.*(t204+t205)-t3.*(-t226-t37.*t42.*t58.*t75+t3.*t25.*t29.*t38.*t83.*2.207315e1+t3.*t25.*t29.*t50.*t83.*(1.0./2.0)+t8.*t37.*t38.*t42.*t83.*1.949505e1+t8.*t37.*t42.*t50.*t83.*(1.0./2.0))-t188.*(-t35.*t78.*exp(sqrt(t15+t216.^2).*(-4.41463e1)+2.207315e2).*1.0./sqrt(t15+t217.^2)-t48.*t75.*exp(sqrt(t15+t214.^2).*(-3.89901e1)+1.730911e2).*1.0./sqrt(t15+t215.^2)+(t14.*t35.*t131.*exp(sqrt(t15+t218.^2).*(-4.41463e1)+2.207315e2).*2.207315e1)./(t15+t219.^2)+t14.*t35.*t131.*exp(sqrt(t15+t220.^2).*(-4.41463e1)+2.207315e2).*1.0./(t15+t221.^2).^(3.0./2.0).*(1.0./2.0)+(t13.*t48.*t131.*exp(sqrt(t15+t222.^2).*(-3.89901e1)+1.730911e2).*1.949505e1)./(t15+t223.^2)+t13.*t48.*t131.*exp(sqrt(t15+t224.^2).*(-3.89901e1)+1.730911e2).*1.0./(t15+t225.^2).^(3.0./2.0).*(1.0./2.0))+t13.*(t35.*t75.*exp(sqrt(t15+t229.^2).*(-4.41463e1)+2.207315e2).*1.0./sqrt(t15+t230.^2)-t48.*t78.*exp(sqrt(t15+t227.^2).*(-3.89901e1)+1.730911e2).*1.0./sqrt(t15+t228.^2)+(t131.*exp(sqrt(t15+t235.^2).*(-3.89901e1)+1.730911e2).*(t5+t7-y-5.0).*(t45+t46-t55-t56).*1.949505e1)./(t15+t236.^2)+t131.*exp(sqrt(t15+t237.^2).*(-3.89901e1)+1.730911e2).*1.0./(t15+t238.^2).^(3.0./2.0).*(t5+t7-y-5.0).*(t45+t46-t55-t56).*(1.0./2.0)-(t13.*t35.*t131.*exp(sqrt(t15+t231.^2).*(-4.41463e1)+2.207315e2).*2.207315e1)./(t15+t232.^2)-t13.*t35.*t131.*exp(sqrt(t15+t233.^2).*(-4.41463e1)+2.207315e2).*1.0./(t15+t234.^2).^(3.0./2.0).*(1.0./2.0))-t174.*(t239+t240+t241-t25.*t29.*t58.*t75-t3.*t37.*t38.*t42.*t83.*1.949505e1-t3.*t37.*t42.*t50.*t83.*(1.0./2.0))+t78.*(t14.*t35.*exp(sqrt(t15+t206.^2).*(-4.41463e1)+2.207315e2).*1.0./sqrt(t15+t207.^2)+t13.*t48.*exp(sqrt(t15+t208.^2).*(-3.89901e1)+1.730911e2).*1.0./sqrt(t15+t209.^2))+t75.*(t13.*t35.*exp(sqrt(t15+t210.^2).*(-4.41463e1)+2.207315e2).*1.0./sqrt(t15+t211.^2)-t14.*t48.*exp(sqrt(t15+t212.^2).*(-3.89901e1)+1.730911e2).*1.0./sqrt(t15+t213.^2))+t75.*(t203-t3.*t37.*t42.*t58),1.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,t248,t246,t3.*t246-t174.*t248,0.0,0.0,0.0,t13.*t141.*exp(sqrt(t15+t93.^2).*(-3.89901e1)+1.730911e2).*1.0./sqrt(t15+t94.^2)+t145.*exp(sqrt(t15+t95.^2).*(-4.41463e1)+2.207315e2).*1.0./sqrt(t15+t96.^2).*(t5+t7-y-5.0),t13.*t145.*exp(sqrt(t15+t142.^2).*(-4.41463e1)+2.207315e2).*1.0./sqrt(t15+t143.^2)-t14.*t141.*exp(sqrt(t15+t137.^2).*(-3.89901e1)+1.730911e2).*1.0./sqrt(t15+t138.^2),t188.*(t13.*t141.*exp(sqrt(t15+t255.^2).*(-3.89901e1)+1.730911e2).*1.0./sqrt(t15+t256.^2)+t145.*exp(sqrt(t15+t257.^2).*(-4.41463e1)+2.207315e2).*1.0./sqrt(t15+t258.^2).*(t5+t7-y-5.0))+t13.*(t13.*t145.*exp(sqrt(t15+t252.^2).*(-4.41463e1)+2.207315e2).*1.0./sqrt(t15+t253.^2)-t14.*t141.*exp(sqrt(t15+t249.^2).*(-3.89901e1)+1.730911e2).*1.0./sqrt(t15+t250.^2))],[6, 9]);
