attribute highp vec4 i_vPosition;								
attribute lowp vec4 i_vColour;									
attribute mediump vec2 i_vTex0;								
																
uniform highp mat4 g_matWorldViewProj_VSC;						
															
varying lowp vec4 v_color;										
varying mediump vec2 v_texCoord;							
															
void main()													
{															
	gl_Position = g_matWorldViewProj_VSC * i_vPosition;		
	gl_Position = gl_Position * i_vPosition.z;					
	gl_Position.w = i_vPosition.z;							
	v_color = i_vColour;										
	v_texCoord = i_vTex0;                                     
}															

