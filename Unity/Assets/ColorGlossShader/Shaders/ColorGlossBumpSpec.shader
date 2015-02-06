Shader "ColorGloss/Bumped Specular" {
	Properties {
		_Color ("Main Color", Color) = (1,1,1,1)
		_Shininess ("Shininess", Range (0.03, 1)) = 0.078125
		_Gloss ("Gloss Power", Float) = 1
		_MainTex ("Base (RGB) Gloss (A)", 2D) = "white" {}
		_BumpMap ("Normalmap", 2D) = "bump" {}
		_SpecMap ("SpecMap", 2D) = "white" {}
	}
	SubShader {
		Tags { "RenderType" = "Opaque" }
		LOD 400

			
	Pass {
		Name "FORWARD"
		Tags { "LightMode" = "ForwardBase" }
Program "vp" {
// Vertex combos: 9
//   opengl - ALU: 6 to 79
//   d3d9 - ALU: 6 to 82
//   d3d11 - ALU: 1 to 39, TEX: 0 to 0, FLOW: 1 to 1
//   d3d11_9x - ALU: 1 to 36, TEX: 0 to 0, FLOW: 1 to 1
SubProgram "opengl " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Bind "vertex" Vertex
Bind "tangent" ATTR14
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 13 [_WorldSpaceCameraPos]
Vector 14 [_WorldSpaceLightPos0]
Vector 15 [unity_SHAr]
Vector 16 [unity_SHAg]
Vector 17 [unity_SHAb]
Vector 18 [unity_SHBr]
Vector 19 [unity_SHBg]
Vector 20 [unity_SHBb]
Vector 21 [unity_SHC]
Matrix 5 [_Object2World]
Matrix 9 [_World2Object]
Vector 22 [unity_Scale]
Vector 23 [_MainTex_ST]
"!!ARBvp1.0
# 43 ALU
PARAM c[24] = { { 1 },
		state.matrix.mvp,
		program.local[5..23] };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
MUL R1.xyz, vertex.normal, c[22].w;
DP3 R2.w, R1, c[6];
DP3 R0.x, R1, c[5];
DP3 R0.z, R1, c[7];
MOV R0.y, R2.w;
MOV R0.w, c[0].x;
MUL R1, R0.xyzz, R0.yzzx;
DP4 R2.z, R0, c[17];
DP4 R2.y, R0, c[16];
DP4 R2.x, R0, c[15];
MUL R0.w, R2, R2;
MAD R0.w, R0.x, R0.x, -R0;
DP4 R0.z, R1, c[20];
DP4 R0.y, R1, c[19];
DP4 R0.x, R1, c[18];
ADD R0.xyz, R2, R0;
MUL R1.xyz, R0.w, c[21];
ADD result.texcoord[2].xyz, R0, R1;
MOV R1.xyz, c[13];
MOV R1.w, c[0].x;
MOV R0.xyz, vertex.attrib[14];
DP4 R2.z, R1, c[11];
DP4 R2.y, R1, c[10];
DP4 R2.x, R1, c[9];
MAD R2.xyz, R2, c[22].w, -vertex.position;
MUL R1.xyz, vertex.normal.zxyw, R0.yzxw;
MAD R1.xyz, vertex.normal.yzxw, R0.zxyw, -R1;
MOV R0, c[14];
MUL R1.xyz, R1, vertex.attrib[14].w;
DP4 R3.z, R0, c[11];
DP4 R3.y, R0, c[10];
DP4 R3.x, R0, c[9];
DP3 result.texcoord[1].y, R3, R1;
DP3 result.texcoord[3].y, R1, R2;
DP3 result.texcoord[1].z, vertex.normal, R3;
DP3 result.texcoord[1].x, R3, vertex.attrib[14];
DP3 result.texcoord[3].z, vertex.normal, R2;
DP3 result.texcoord[3].x, vertex.attrib[14], R2;
MAD result.texcoord[0].xy, vertex.texcoord[0], c[23], c[23].zwzw;
DP4 result.position.w, vertex.position, c[4];
DP4 result.position.z, vertex.position, c[3];
DP4 result.position.y, vertex.position, c[2];
DP4 result.position.x, vertex.position, c[1];
END
# 43 instructions, 4 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Matrix 0 [glstate_matrix_mvp]
Vector 12 [_WorldSpaceCameraPos]
Vector 13 [_WorldSpaceLightPos0]
Vector 14 [unity_SHAr]
Vector 15 [unity_SHAg]
Vector 16 [unity_SHAb]
Vector 17 [unity_SHBr]
Vector 18 [unity_SHBg]
Vector 19 [unity_SHBb]
Vector 20 [unity_SHC]
Matrix 4 [_Object2World]
Matrix 8 [_World2Object]
Vector 21 [unity_Scale]
Vector 22 [_MainTex_ST]
"vs_2_0
; 46 ALU
def c23, 1.00000000, 0, 0, 0
dcl_position0 v0
dcl_tangent0 v1
dcl_normal0 v2
dcl_texcoord0 v3
mul r1.xyz, v2, c21.w
dp3 r2.w, r1, c5
dp3 r0.x, r1, c4
dp3 r0.z, r1, c6
mov r0.y, r2.w
mov r0.w, c23.x
mul r1, r0.xyzz, r0.yzzx
dp4 r2.z, r0, c16
dp4 r2.y, r0, c15
dp4 r2.x, r0, c14
mul r0.w, r2, r2
mad r0.w, r0.x, r0.x, -r0
dp4 r0.z, r1, c19
dp4 r0.y, r1, c18
dp4 r0.x, r1, c17
mul r1.xyz, r0.w, c20
add r0.xyz, r2, r0
add oT2.xyz, r0, r1
mov r0.w, c23.x
mov r0.xyz, c12
dp4 r1.z, r0, c10
dp4 r1.y, r0, c9
dp4 r1.x, r0, c8
mad r3.xyz, r1, c21.w, -v0
mov r0.xyz, v1
mul r1.xyz, v2.zxyw, r0.yzxw
mov r0.xyz, v1
mad r1.xyz, v2.yzxw, r0.zxyw, -r1
mul r2.xyz, r1, v1.w
mov r0, c10
dp4 r4.z, c13, r0
mov r0, c9
mov r1, c8
dp4 r4.y, c13, r0
dp4 r4.x, c13, r1
dp3 oT1.y, r4, r2
dp3 oT3.y, r2, r3
dp3 oT1.z, v2, r4
dp3 oT1.x, r4, v1
dp3 oT3.z, v2, r3
dp3 oT3.x, v1, r3
mad oT0.xy, v3, c22, c22.zwzw
dp4 oPos.w, v0, c3
dp4 oPos.z, v0, c2
dp4 oPos.y, v0, c1
dp4 oPos.x, v0, c0
"
}

SubProgram "xbox360 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 21 [_MainTex_ST]
Matrix 13 [_Object2World] 3
Matrix 16 [_World2Object] 4
Vector 0 [_WorldSpaceCameraPos]
Vector 1 [_WorldSpaceLightPos0]
Matrix 9 [glstate_matrix_mvp] 4
Vector 4 [unity_SHAb]
Vector 3 [unity_SHAg]
Vector 2 [unity_SHAr]
Vector 7 [unity_SHBb]
Vector 6 [unity_SHBg]
Vector 5 [unity_SHBr]
Vector 8 [unity_SHC]
Vector 20 [unity_Scale]
// Shader Timing Estimate, in Cycles/64 vertex vector:
// ALU: 46.67 (35 instructions), vertex: 32, texture: 0,
//   sequencer: 20,  9 GPRs, 21 threads,
// Performance (if enough threads): ~46 cycles per vector
// * Vertex cycle estimates are assuming 3 vfetch_minis for every vfetch_full,
//     with <= 32 bytes per vfetch_full group.

"vs_360
backbbabaaaaacmiaaaaacbmaaaaaaaaaaaaaaceaaaaaaaaaaaaacgaaaaaaaaa
aaaaaaaaaaaaacdiaaaaaabmaaaaacclpppoadaaaaaaaaaoaaaaaabmaaaaaaaa
aaaaacceaaaaabdeaaacaabfaaabaaaaaaaaabeaaaaaaaaaaaaaabfaaaacaaan
aaadaaaaaaaaabgaaaaaaaaaaaaaabhaaaacaabaaaaeaaaaaaaaabgaaaaaaaaa
aaaaabhoaaacaaaaaaabaaaaaaaaabjeaaaaaaaaaaaaabkeaaacaaabaaabaaaa
aaaaabeaaaaaaaaaaaaaabljaaacaaajaaaeaaaaaaaaabgaaaaaaaaaaaaaabmm
aaacaaaeaaabaaaaaaaaabeaaaaaaaaaaaaaabnhaaacaaadaaabaaaaaaaaabea
aaaaaaaaaaaaabocaaacaaacaaabaaaaaaaaabeaaaaaaaaaaaaaabonaaacaaah
aaabaaaaaaaaabeaaaaaaaaaaaaaabpiaaacaaagaaabaaaaaaaaabeaaaaaaaaa
aaaaacadaaacaaafaaabaaaaaaaaabeaaaaaaaaaaaaaacaoaaacaaaiaaabaaaa
aaaaabeaaaaaaaaaaaaaacbiaaacaabeaaabaaaaaaaaabeaaaaaaaaafpengbgj
gofegfhifpfdfeaaaaabaaadaaabaaaeaaabaaaaaaaaaaaafpepgcgkgfgdhedc
fhgphcgmgeaaklklaaadaaadaaaeaaaeaaabaaaaaaaaaaaafpfhgphcgmgedcep
gcgkgfgdheaafpfhgphcgmgefdhagbgdgfedgbgngfhcgbfagphdaaklaaabaaad
aaabaaadaaabaaaaaaaaaaaafpfhgphcgmgefdhagbgdgfemgjghgihefagphdda
aaghgmhdhegbhegffpgngbhehcgjhifpgnhghaaahfgogjhehjfpfdeiebgcaahf
gogjhehjfpfdeiebghaahfgogjhehjfpfdeiebhcaahfgogjhehjfpfdeiecgcaa
hfgogjhehjfpfdeiecghaahfgogjhehjfpfdeiechcaahfgogjhehjfpfdeiedaa
hfgogjhehjfpfdgdgbgmgfaahghdfpddfpdaaadccodacodcdadddfddcodaaakl
aaaaaaaaaaaaacbmaadbaaaiaaaaaaaaaaaaaaaaaaaacmieaaaaaaabaaaaaaae
aaaaaaaiaaaaacjaaabaaaafaaaagaagaaaadaahaadafaaiaaaadafaaaabhbfb
aaaehcfcaaafhdfdaaaabaccaaaaaabmaaaaaabnaaaababoaaaabaclaaaaaabp
aaaaaacaaaaabacbpaffeaafaaaabcaamcaaaaaaaaaaeaajaaaabcaameaaaaaa
aaaagaangabdbcaabcaaaaaaaaaagabjgabpbcaabcaaaaaaaaaagacfbaclbcaa
ccaaaaaaafpicaaaaaaaagiiaaaaaaaaafpifaaaaaaaagiiaaaaaaaaafpibaaa
aaaaaoiiaaaaaaaaafpiaaaaaaaaapmiaaaaaaaamiapaaadaabliiaakbacamaa
miapaaadaamgiiaaklacaladmiapaaadaalbdejeklacakadmiapiadoaagmaade
klacajadmiahaaadaaleblaacbbdabaamiahaaaeaamamgmaalbcaabdmiahaaai
aalelbleclbbaaaemiahaaaeaalogfaaobabafaamiahaaagaamamgleclbcabad
mialaaadaagfblaakbabbeaamiahaaahaalbleaakbadapaamiahaaagaalelble
clbbabagmiahaaaeabgflomaolabafaemiahaaaiaamagmleclbaaaaimiahaaac
abmablmaklaibeacmiahaaaeaamablaaobaeafaamiahaaagaamagmleclbaabag
miahaaadaagmlemakladaoahmiahaaadaabllemakladanadmiabiaabaaloloaa
paagafaamiaciaabaaloloaapaaeagaamiaeiaabaaloloaapaagabaamiabiaad
aaloloaapaacafaamiaciaadaaloloaapaaeacaamiaeiaadaaloloaapaacabaa
miadiaaaaalalabkilaabfbfceipadaeaalehcgmobadadiamiabaaacaadoanaa
gpacadaamiacaaacaadoanaagpadadaamiaeaaacaadoanaagpaeadaamiabaaaa
aakhkhaakpaeafaaaibcabaaaakhkhgmkpaeagadaiceabaaaakhkhmgkpaeahad
geihaaaaaalologboaacaaabmiahiaacaablmagfklaaaiaaaaaaaaaaaaaaaaaa
aaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Matrix 256 [glstate_matrix_mvp]
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 467 [_WorldSpaceCameraPos]
Vector 466 [_WorldSpaceLightPos0]
Vector 465 [unity_SHAr]
Vector 464 [unity_SHAg]
Vector 463 [unity_SHAb]
Vector 462 [unity_SHBr]
Vector 461 [unity_SHBg]
Vector 460 [unity_SHBb]
Vector 459 [unity_SHC]
Matrix 260 [_Object2World]
Matrix 264 [_World2Object]
Vector 458 [unity_Scale]
Vector 457 [_MainTex_ST]
"sce_vp_rsx // 41 instructions using 5 registers
[Configuration]
8
0000002941050500
[Microcode]
656
00009c6c005d200d8186c0836041fffc00001c6c00400e0c0106c0836041dffc
00021c6c005d300c0186c0836041dffc00019c6c009ca20c013fc0c36041dffc
401f9c6c011c9808010400d740619f9c401f9c6c01d0300d8106c0c360403f80
401f9c6c01d0200d8106c0c360405f80401f9c6c01d0100d8106c0c360409f80
401f9c6c01d0000d8106c0c360411f8000011c6c01d0a00d8286c0c360405ffc
00011c6c01d0900d8286c0c360409ffc00011c6c01d0800d8286c0c360411ffc
00009c6c0150400c068600c360411ffc00009c6c0150600c068600c360405ffc
00001c6c0150500c068600c360403ffc00019c6c0190a00c0886c0c360405ffc
00019c6c0190900c0886c0c360409ffc00019c6c0190800c0886c0c360411ffc
00021c6c00800243011840436041dffc00001c6c01000230812180630221dffc
00021c6c011ca00c06bfc0e30041dffc401f9c6c0140020c0106024360405fa0
401f9c6c01400e0c0486008360411fa000009c6c0080007f80bfc04360403ffc
00009c6c0040007f8086c08360409ffc00019c6c00800e0c00bfc0836041dffc
401f9c6c0140020c0106044360405fa8401f9c6c01400e0c0106044360411fa8
00001c6c019cf00c0286c0c360405ffc00001c6c019d000c0286c0c360409ffc
00001c6c019d100c0286c0c360411ffc00001c6c010000000280017fe0a03ffc
00009c6c0080000d029a01436041fffc401f9c6c0140000c0486034360409fa0
401f9c6c0140000c0686044360409fa800011c6c01dcc00d8286c0c360405ffc
00011c6c01dcd00d8286c0c360409ffc00011c6c01dce00d8286c0c360411ffc
00001c6c00c0000c0086c0830121dffc00009c6c009cb07f808600c36041dffc
401f9c6c00c0000c0286c0830021dfa5
"
}

SubProgram "d3d11 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 96 // 96 used size, 7 vars
Vector 80 [_MainTex_ST] 4
ConstBuffer "UnityPerCamera" 128 // 76 used size, 8 vars
Vector 64 [_WorldSpaceCameraPos] 3
ConstBuffer "UnityLighting" 400 // 400 used size, 16 vars
Vector 0 [_WorldSpaceLightPos0] 4
Vector 288 [unity_SHAr] 4
Vector 304 [unity_SHAg] 4
Vector 320 [unity_SHAb] 4
Vector 336 [unity_SHBr] 4
Vector 352 [unity_SHBg] 4
Vector 368 [unity_SHBb] 4
Vector 384 [unity_SHC] 4
ConstBuffer "UnityPerDraw" 336 // 336 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
Matrix 192 [_Object2World] 4
Matrix 256 [_World2Object] 4
Vector 320 [unity_Scale] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityLighting" 2
BindCB "UnityPerDraw" 3
// 40 instructions, 5 temp regs, 0 temp arrays:
// ALU 23 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0
eefiecedgkjnhkkiogbebdgjkmgpalagjjjlmaifabaaaaaakeahaaaaadaaaaaa
cmaaaaaapeaaaaaajeabaaaaejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaa
abaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaljaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfc
enebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheojiaaaaaaafaaaaaa
aiaaaaaaiaaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaabaaaaaaadamaaaaimaaaaaaabaaaaaaaaaaaaaa
adaaaaaaacaaaaaaahaiaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaa
ahaiaaaaimaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaahaiaaaafdfgfpfa
epfdejfeejepeoaafeeffiedepepfceeaaklklklfdeieefcaiagaaaaeaaaabaa
icabaaaafjaaaaaeegiocaaaaaaaaaaaagaaaaaafjaaaaaeegiocaaaabaaaaaa
afaaaaaafjaaaaaeegiocaaaacaaaaaabjaaaaaafjaaaaaeegiocaaaadaaaaaa
bfaaaaaafpaaaaadpcbabaaaaaaaaaaafpaaaaadpcbabaaaabaaaaaafpaaaaad
hcbabaaaacaaaaaafpaaaaaddcbabaaaadaaaaaaghaaaaaepccabaaaaaaaaaaa
abaaaaaagfaaaaaddccabaaaabaaaaaagfaaaaadhccabaaaacaaaaaagfaaaaad
hccabaaaadaaaaaagfaaaaadhccabaaaaeaaaaaagiaaaaacafaaaaaadiaaaaai
pcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaadaaaaaaabaaaaaadcaaaaak
pcaabaaaaaaaaaaaegiocaaaadaaaaaaaaaaaaaaagbabaaaaaaaaaaaegaobaaa
aaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaacaaaaaakgbkbaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaakpccabaaaaaaaaaaaegiocaaaadaaaaaa
adaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaaldccabaaaabaaaaaa
egbabaaaadaaaaaaegiacaaaaaaaaaaaafaaaaaaogikcaaaaaaaaaaaafaaaaaa
diaaaaahhcaabaaaaaaaaaaajgbebaaaabaaaaaacgbjbaaaacaaaaaadcaaaaak
hcaabaaaaaaaaaaajgbebaaaacaaaaaacgbjbaaaabaaaaaaegacbaiaebaaaaaa
aaaaaaaadiaaaaahhcaabaaaaaaaaaaaegacbaaaaaaaaaaapgbpbaaaabaaaaaa
diaaaaajhcaabaaaabaaaaaafgifcaaaacaaaaaaaaaaaaaaegiccaaaadaaaaaa
bbaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaa
acaaaaaaaaaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaa
adaaaaaabcaaaaaakgikcaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaadcaaaaal
hcaabaaaabaaaaaaegiccaaaadaaaaaabdaaaaaapgipcaaaacaaaaaaaaaaaaaa
egacbaaaabaaaaaabaaaaaahcccabaaaacaaaaaaegacbaaaaaaaaaaaegacbaaa
abaaaaaabaaaaaahbccabaaaacaaaaaaegbcbaaaabaaaaaaegacbaaaabaaaaaa
baaaaaaheccabaaaacaaaaaaegbcbaaaacaaaaaaegacbaaaabaaaaaadiaaaaai
hcaabaaaabaaaaaaegbcbaaaacaaaaaapgipcaaaadaaaaaabeaaaaaadiaaaaai
hcaabaaaacaaaaaafgafbaaaabaaaaaaegiccaaaadaaaaaaanaaaaaadcaaaaak
lcaabaaaabaaaaaaegiicaaaadaaaaaaamaaaaaaagaabaaaabaaaaaaegaibaaa
acaaaaaadcaaaaakhcaabaaaabaaaaaaegiccaaaadaaaaaaaoaaaaaakgakbaaa
abaaaaaaegadbaaaabaaaaaadgaaaaaficaabaaaabaaaaaaabeaaaaaaaaaiadp
bbaaaaaibcaabaaaacaaaaaaegiocaaaacaaaaaabcaaaaaaegaobaaaabaaaaaa
bbaaaaaiccaabaaaacaaaaaaegiocaaaacaaaaaabdaaaaaaegaobaaaabaaaaaa
bbaaaaaiecaabaaaacaaaaaaegiocaaaacaaaaaabeaaaaaaegaobaaaabaaaaaa
diaaaaahpcaabaaaadaaaaaajgacbaaaabaaaaaaegakbaaaabaaaaaabbaaaaai
bcaabaaaaeaaaaaaegiocaaaacaaaaaabfaaaaaaegaobaaaadaaaaaabbaaaaai
ccaabaaaaeaaaaaaegiocaaaacaaaaaabgaaaaaaegaobaaaadaaaaaabbaaaaai
ecaabaaaaeaaaaaaegiocaaaacaaaaaabhaaaaaaegaobaaaadaaaaaaaaaaaaah
hcaabaaaacaaaaaaegacbaaaacaaaaaaegacbaaaaeaaaaaadiaaaaahicaabaaa
aaaaaaaabkaabaaaabaaaaaabkaabaaaabaaaaaadcaaaaakicaabaaaaaaaaaaa
akaabaaaabaaaaaaakaabaaaabaaaaaadkaabaiaebaaaaaaaaaaaaaadcaaaaak
hccabaaaadaaaaaaegiccaaaacaaaaaabiaaaaaapgapbaaaaaaaaaaaegacbaaa
acaaaaaadiaaaaajhcaabaaaabaaaaaafgifcaaaabaaaaaaaeaaaaaaegiccaaa
adaaaaaabbaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabaaaaaaa
agiacaaaabaaaaaaaeaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaa
egiccaaaadaaaaaabcaaaaaakgikcaaaabaaaaaaaeaaaaaaegacbaaaabaaaaaa
aaaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaaegiccaaaadaaaaaabdaaaaaa
dcaaaaalhcaabaaaabaaaaaaegacbaaaabaaaaaapgipcaaaadaaaaaabeaaaaaa
egbcbaiaebaaaaaaaaaaaaaabaaaaaahcccabaaaaeaaaaaaegacbaaaaaaaaaaa
egacbaaaabaaaaaabaaaaaahbccabaaaaeaaaaaaegbcbaaaabaaaaaaegacbaaa
abaaaaaabaaaaaaheccabaaaaeaaaaaaegbcbaaaacaaaaaaegacbaaaabaaaaaa
doaaaaab"
}

SubProgram "gles " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
"!!GLES
#define SHADER_API_GLES 1
#define tex2D texture2D


#ifdef VERTEX
#define gl_ModelViewProjectionMatrix glstate_matrix_mvp
uniform mat4 glstate_matrix_mvp;

varying highp vec3 xlv_TEXCOORD3;
varying lowp vec3 xlv_TEXCOORD2;
varying lowp vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 unity_Scale;
uniform highp mat4 _World2Object;
uniform highp mat4 _Object2World;

uniform highp vec4 unity_SHC;
uniform highp vec4 unity_SHBb;
uniform highp vec4 unity_SHBg;
uniform highp vec4 unity_SHBr;
uniform highp vec4 unity_SHAb;
uniform highp vec4 unity_SHAg;
uniform highp vec4 unity_SHAr;
uniform lowp vec4 _WorldSpaceLightPos0;
uniform highp vec3 _WorldSpaceCameraPos;
attribute vec4 _glesTANGENT;
attribute vec4 _glesMultiTexCoord0;
attribute vec3 _glesNormal;
attribute vec4 _glesVertex;
void main ()
{
  vec4 tmpvar_1;
  tmpvar_1.xyz = normalize(_glesTANGENT.xyz);
  tmpvar_1.w = _glesTANGENT.w;
  vec3 tmpvar_2;
  tmpvar_2 = normalize(_glesNormal);
  highp vec3 shlight_3;
  lowp vec3 tmpvar_4;
  lowp vec3 tmpvar_5;
  mat3 tmpvar_6;
  tmpvar_6[0] = _Object2World[0].xyz;
  tmpvar_6[1] = _Object2World[1].xyz;
  tmpvar_6[2] = _Object2World[2].xyz;
  highp vec3 tmpvar_7;
  highp vec3 tmpvar_8;
  tmpvar_7 = tmpvar_1.xyz;
  tmpvar_8 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_9;
  tmpvar_9[0].x = tmpvar_7.x;
  tmpvar_9[0].y = tmpvar_8.x;
  tmpvar_9[0].z = tmpvar_2.x;
  tmpvar_9[1].x = tmpvar_7.y;
  tmpvar_9[1].y = tmpvar_8.y;
  tmpvar_9[1].z = tmpvar_2.y;
  tmpvar_9[2].x = tmpvar_7.z;
  tmpvar_9[2].y = tmpvar_8.z;
  tmpvar_9[2].z = tmpvar_2.z;
  highp vec3 tmpvar_10;
  tmpvar_10 = (tmpvar_9 * (_World2Object * _WorldSpaceLightPos0).xyz);
  tmpvar_4 = tmpvar_10;
  highp vec4 tmpvar_11;
  tmpvar_11.w = 1.0;
  tmpvar_11.xyz = _WorldSpaceCameraPos;
  highp vec4 tmpvar_12;
  tmpvar_12.w = 1.0;
  tmpvar_12.xyz = (tmpvar_6 * (tmpvar_2 * unity_Scale.w));
  mediump vec3 tmpvar_13;
  mediump vec4 normal_14;
  normal_14 = tmpvar_12;
  highp float vC_15;
  mediump vec3 x3_16;
  mediump vec3 x2_17;
  mediump vec3 x1_18;
  highp float tmpvar_19;
  tmpvar_19 = dot (unity_SHAr, normal_14);
  x1_18.x = tmpvar_19;
  highp float tmpvar_20;
  tmpvar_20 = dot (unity_SHAg, normal_14);
  x1_18.y = tmpvar_20;
  highp float tmpvar_21;
  tmpvar_21 = dot (unity_SHAb, normal_14);
  x1_18.z = tmpvar_21;
  mediump vec4 tmpvar_22;
  tmpvar_22 = (normal_14.xyzz * normal_14.yzzx);
  highp float tmpvar_23;
  tmpvar_23 = dot (unity_SHBr, tmpvar_22);
  x2_17.x = tmpvar_23;
  highp float tmpvar_24;
  tmpvar_24 = dot (unity_SHBg, tmpvar_22);
  x2_17.y = tmpvar_24;
  highp float tmpvar_25;
  tmpvar_25 = dot (unity_SHBb, tmpvar_22);
  x2_17.z = tmpvar_25;
  mediump float tmpvar_26;
  tmpvar_26 = ((normal_14.x * normal_14.x) - (normal_14.y * normal_14.y));
  vC_15 = tmpvar_26;
  highp vec3 tmpvar_27;
  tmpvar_27 = (unity_SHC.xyz * vC_15);
  x3_16 = tmpvar_27;
  tmpvar_13 = ((x1_18 + x2_17) + x3_16);
  shlight_3 = tmpvar_13;
  tmpvar_5 = shlight_3;
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = tmpvar_4;
  xlv_TEXCOORD2 = tmpvar_5;
  xlv_TEXCOORD3 = (tmpvar_9 * (((_World2Object * tmpvar_11).xyz * unity_Scale.w) - _glesVertex.xyz));
}



#endif
#ifdef FRAGMENT

varying highp vec3 xlv_TEXCOORD3;
varying lowp vec3 xlv_TEXCOORD2;
varying lowp vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform mediump float _Gloss;
uniform mediump float _Shininess;
uniform lowp vec4 _Color;
uniform sampler2D _SpecMap;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform lowp vec4 _LightColor0;
void main ()
{
  lowp vec4 c_1;
  mediump vec3 tmpvar_2;
  mediump vec3 tmpvar_3;
  lowp vec4 tmpvar_4;
  tmpvar_4 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_5;
  tmpvar_5 = (tmpvar_4.xyz * _Color.xyz);
  tmpvar_2 = tmpvar_5;
  lowp vec4 tmpvar_6;
  tmpvar_6 = texture2D (_SpecMap, xlv_TEXCOORD0);
  mediump vec3 tmpvar_7;
  tmpvar_7 = ((tmpvar_4.w * tmpvar_6.xyz) * _Gloss);
  lowp vec3 tmpvar_8;
  tmpvar_8 = ((texture2D (_BumpMap, xlv_TEXCOORD0).xyz * 2.0) - 1.0);
  tmpvar_3 = tmpvar_8;
  highp vec3 tmpvar_9;
  tmpvar_9 = normalize(xlv_TEXCOORD3);
  mediump vec3 lightDir_10;
  lightDir_10 = xlv_TEXCOORD1;
  mediump vec3 viewDir_11;
  viewDir_11 = tmpvar_9;
  mediump vec4 c_12;
  mediump vec3 specCol_13;
  highp float nh_14;
  mediump float tmpvar_15;
  tmpvar_15 = max (0.0, dot (tmpvar_3, normalize((lightDir_10 + viewDir_11))));
  nh_14 = tmpvar_15;
  mediump float arg1_16;
  arg1_16 = (32.0 * _Shininess);
  highp vec3 tmpvar_17;
  tmpvar_17 = (pow (nh_14, arg1_16) * tmpvar_7);
  specCol_13 = tmpvar_17;
  c_12.xyz = ((((tmpvar_2 * _LightColor0.xyz) * max (0.0, dot (tmpvar_3, lightDir_10))) + (_LightColor0.xyz * specCol_13)) * 2.0);
  c_12.w = 0.0;
  c_1 = c_12;
  mediump vec3 tmpvar_18;
  tmpvar_18 = (c_1.xyz + (tmpvar_2 * xlv_TEXCOORD2));
  c_1.xyz = tmpvar_18;
  gl_FragData[0] = c_1;
}



#endif"
}

SubProgram "glesdesktop " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
"!!GLES
#define SHADER_API_GLES 1
#define tex2D texture2D


#ifdef VERTEX
#define gl_ModelViewProjectionMatrix glstate_matrix_mvp
uniform mat4 glstate_matrix_mvp;

varying highp vec3 xlv_TEXCOORD3;
varying lowp vec3 xlv_TEXCOORD2;
varying lowp vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 unity_Scale;
uniform highp mat4 _World2Object;
uniform highp mat4 _Object2World;

uniform highp vec4 unity_SHC;
uniform highp vec4 unity_SHBb;
uniform highp vec4 unity_SHBg;
uniform highp vec4 unity_SHBr;
uniform highp vec4 unity_SHAb;
uniform highp vec4 unity_SHAg;
uniform highp vec4 unity_SHAr;
uniform lowp vec4 _WorldSpaceLightPos0;
uniform highp vec3 _WorldSpaceCameraPos;
attribute vec4 _glesTANGENT;
attribute vec4 _glesMultiTexCoord0;
attribute vec3 _glesNormal;
attribute vec4 _glesVertex;
void main ()
{
  vec4 tmpvar_1;
  tmpvar_1.xyz = normalize(_glesTANGENT.xyz);
  tmpvar_1.w = _glesTANGENT.w;
  vec3 tmpvar_2;
  tmpvar_2 = normalize(_glesNormal);
  highp vec3 shlight_3;
  lowp vec3 tmpvar_4;
  lowp vec3 tmpvar_5;
  mat3 tmpvar_6;
  tmpvar_6[0] = _Object2World[0].xyz;
  tmpvar_6[1] = _Object2World[1].xyz;
  tmpvar_6[2] = _Object2World[2].xyz;
  highp vec3 tmpvar_7;
  highp vec3 tmpvar_8;
  tmpvar_7 = tmpvar_1.xyz;
  tmpvar_8 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_9;
  tmpvar_9[0].x = tmpvar_7.x;
  tmpvar_9[0].y = tmpvar_8.x;
  tmpvar_9[0].z = tmpvar_2.x;
  tmpvar_9[1].x = tmpvar_7.y;
  tmpvar_9[1].y = tmpvar_8.y;
  tmpvar_9[1].z = tmpvar_2.y;
  tmpvar_9[2].x = tmpvar_7.z;
  tmpvar_9[2].y = tmpvar_8.z;
  tmpvar_9[2].z = tmpvar_2.z;
  highp vec3 tmpvar_10;
  tmpvar_10 = (tmpvar_9 * (_World2Object * _WorldSpaceLightPos0).xyz);
  tmpvar_4 = tmpvar_10;
  highp vec4 tmpvar_11;
  tmpvar_11.w = 1.0;
  tmpvar_11.xyz = _WorldSpaceCameraPos;
  highp vec4 tmpvar_12;
  tmpvar_12.w = 1.0;
  tmpvar_12.xyz = (tmpvar_6 * (tmpvar_2 * unity_Scale.w));
  mediump vec3 tmpvar_13;
  mediump vec4 normal_14;
  normal_14 = tmpvar_12;
  highp float vC_15;
  mediump vec3 x3_16;
  mediump vec3 x2_17;
  mediump vec3 x1_18;
  highp float tmpvar_19;
  tmpvar_19 = dot (unity_SHAr, normal_14);
  x1_18.x = tmpvar_19;
  highp float tmpvar_20;
  tmpvar_20 = dot (unity_SHAg, normal_14);
  x1_18.y = tmpvar_20;
  highp float tmpvar_21;
  tmpvar_21 = dot (unity_SHAb, normal_14);
  x1_18.z = tmpvar_21;
  mediump vec4 tmpvar_22;
  tmpvar_22 = (normal_14.xyzz * normal_14.yzzx);
  highp float tmpvar_23;
  tmpvar_23 = dot (unity_SHBr, tmpvar_22);
  x2_17.x = tmpvar_23;
  highp float tmpvar_24;
  tmpvar_24 = dot (unity_SHBg, tmpvar_22);
  x2_17.y = tmpvar_24;
  highp float tmpvar_25;
  tmpvar_25 = dot (unity_SHBb, tmpvar_22);
  x2_17.z = tmpvar_25;
  mediump float tmpvar_26;
  tmpvar_26 = ((normal_14.x * normal_14.x) - (normal_14.y * normal_14.y));
  vC_15 = tmpvar_26;
  highp vec3 tmpvar_27;
  tmpvar_27 = (unity_SHC.xyz * vC_15);
  x3_16 = tmpvar_27;
  tmpvar_13 = ((x1_18 + x2_17) + x3_16);
  shlight_3 = tmpvar_13;
  tmpvar_5 = shlight_3;
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = tmpvar_4;
  xlv_TEXCOORD2 = tmpvar_5;
  xlv_TEXCOORD3 = (tmpvar_9 * (((_World2Object * tmpvar_11).xyz * unity_Scale.w) - _glesVertex.xyz));
}



#endif
#ifdef FRAGMENT

varying highp vec3 xlv_TEXCOORD3;
varying lowp vec3 xlv_TEXCOORD2;
varying lowp vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform mediump float _Gloss;
uniform mediump float _Shininess;
uniform lowp vec4 _Color;
uniform sampler2D _SpecMap;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform lowp vec4 _LightColor0;
void main ()
{
  lowp vec4 c_1;
  mediump vec3 tmpvar_2;
  mediump vec3 tmpvar_3;
  lowp vec4 tmpvar_4;
  tmpvar_4 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_5;
  tmpvar_5 = (tmpvar_4.xyz * _Color.xyz);
  tmpvar_2 = tmpvar_5;
  lowp vec4 tmpvar_6;
  tmpvar_6 = texture2D (_SpecMap, xlv_TEXCOORD0);
  mediump vec3 tmpvar_7;
  tmpvar_7 = ((tmpvar_4.w * tmpvar_6.xyz) * _Gloss);
  lowp vec3 normal_8;
  normal_8.xy = ((texture2D (_BumpMap, xlv_TEXCOORD0).wy * 2.0) - 1.0);
  normal_8.z = sqrt(((1.0 - (normal_8.x * normal_8.x)) - (normal_8.y * normal_8.y)));
  tmpvar_3 = normal_8;
  highp vec3 tmpvar_9;
  tmpvar_9 = normalize(xlv_TEXCOORD3);
  mediump vec3 lightDir_10;
  lightDir_10 = xlv_TEXCOORD1;
  mediump vec3 viewDir_11;
  viewDir_11 = tmpvar_9;
  mediump vec4 c_12;
  mediump vec3 specCol_13;
  highp float nh_14;
  mediump float tmpvar_15;
  tmpvar_15 = max (0.0, dot (tmpvar_3, normalize((lightDir_10 + viewDir_11))));
  nh_14 = tmpvar_15;
  mediump float arg1_16;
  arg1_16 = (32.0 * _Shininess);
  highp vec3 tmpvar_17;
  tmpvar_17 = (pow (nh_14, arg1_16) * tmpvar_7);
  specCol_13 = tmpvar_17;
  c_12.xyz = ((((tmpvar_2 * _LightColor0.xyz) * max (0.0, dot (tmpvar_3, lightDir_10))) + (_LightColor0.xyz * specCol_13)) * 2.0);
  c_12.w = 0.0;
  c_1 = c_12;
  mediump vec3 tmpvar_18;
  tmpvar_18 = (c_1.xyz + (tmpvar_2 * xlv_TEXCOORD2));
  c_1.xyz = tmpvar_18;
  gl_FragData[0] = c_1;
}



#endif"
}

SubProgram "flash " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Matrix 0 [glstate_matrix_mvp]
Vector 12 [_WorldSpaceCameraPos]
Vector 13 [_WorldSpaceLightPos0]
Vector 14 [unity_SHAr]
Vector 15 [unity_SHAg]
Vector 16 [unity_SHAb]
Vector 17 [unity_SHBr]
Vector 18 [unity_SHBg]
Vector 19 [unity_SHBb]
Vector 20 [unity_SHC]
Matrix 4 [_Object2World]
Matrix 8 [_World2Object]
Vector 21 [unity_Scale]
Vector 22 [_MainTex_ST]
"agal_vs
c23 1.0 0.0 0.0 0.0
[bc]
adaaaaaaabaaahacabaaaaoeaaaaaaaabfaaaappabaaaaaa mul r1.xyz, a1, c21.w
bcaaaaaaacaaaiacabaaaakeacaaaaaaafaaaaoeabaaaaaa dp3 r2.w, r1.xyzz, c5
bcaaaaaaaaaaabacabaaaakeacaaaaaaaeaaaaoeabaaaaaa dp3 r0.x, r1.xyzz, c4
bcaaaaaaaaaaaeacabaaaakeacaaaaaaagaaaaoeabaaaaaa dp3 r0.z, r1.xyzz, c6
aaaaaaaaaaaaacacacaaaappacaaaaaaaaaaaaaaaaaaaaaa mov r0.y, r2.w
aaaaaaaaaaaaaiacbhaaaaaaabaaaaaaaaaaaaaaaaaaaaaa mov r0.w, c23.x
adaaaaaaabaaapacaaaaaakeacaaaaaaaaaaaacjacaaaaaa mul r1, r0.xyzz, r0.yzzx
bdaaaaaaacaaaeacaaaaaaoeacaaaaaabaaaaaoeabaaaaaa dp4 r2.z, r0, c16
bdaaaaaaacaaacacaaaaaaoeacaaaaaaapaaaaoeabaaaaaa dp4 r2.y, r0, c15
bdaaaaaaacaaabacaaaaaaoeacaaaaaaaoaaaaoeabaaaaaa dp4 r2.x, r0, c14
adaaaaaaaaaaaiacacaaaappacaaaaaaacaaaappacaaaaaa mul r0.w, r2.w, r2.w
adaaaaaaadaaaiacaaaaaaaaacaaaaaaaaaaaaaaacaaaaaa mul r3.w, r0.x, r0.x
acaaaaaaaaaaaiacadaaaappacaaaaaaaaaaaappacaaaaaa sub r0.w, r3.w, r0.w
bdaaaaaaaaaaaeacabaaaaoeacaaaaaabdaaaaoeabaaaaaa dp4 r0.z, r1, c19
bdaaaaaaaaaaacacabaaaaoeacaaaaaabcaaaaoeabaaaaaa dp4 r0.y, r1, c18
bdaaaaaaaaaaabacabaaaaoeacaaaaaabbaaaaoeabaaaaaa dp4 r0.x, r1, c17
adaaaaaaabaaahacaaaaaappacaaaaaabeaaaaoeabaaaaaa mul r1.xyz, r0.w, c20
abaaaaaaaaaaahacacaaaakeacaaaaaaaaaaaakeacaaaaaa add r0.xyz, r2.xyzz, r0.xyzz
abaaaaaaacaaahaeaaaaaakeacaaaaaaabaaaakeacaaaaaa add v2.xyz, r0.xyzz, r1.xyzz
aaaaaaaaaaaaaiacbhaaaaaaabaaaaaaaaaaaaaaaaaaaaaa mov r0.w, c23.x
aaaaaaaaaaaaahacamaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, c12
bdaaaaaaabaaaeacaaaaaaoeacaaaaaaakaaaaoeabaaaaaa dp4 r1.z, r0, c10
bdaaaaaaabaaacacaaaaaaoeacaaaaaaajaaaaoeabaaaaaa dp4 r1.y, r0, c9
bdaaaaaaabaaabacaaaaaaoeacaaaaaaaiaaaaoeabaaaaaa dp4 r1.x, r0, c8
adaaaaaaaeaaahacabaaaakeacaaaaaabfaaaappabaaaaaa mul r4.xyz, r1.xyzz, c21.w
acaaaaaaadaaahacaeaaaakeacaaaaaaaaaaaaoeaaaaaaaa sub r3.xyz, r4.xyzz, a0
aaaaaaaaaaaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, a5
adaaaaaaabaaahacabaaaancaaaaaaaaaaaaaaajacaaaaaa mul r1.xyz, a1.zxyw, r0.yzxx
aaaaaaaaaaaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, a5
adaaaaaaafaaahacabaaaamjaaaaaaaaaaaaaafcacaaaaaa mul r5.xyz, a1.yzxw, r0.zxyy
acaaaaaaabaaahacafaaaakeacaaaaaaabaaaakeacaaaaaa sub r1.xyz, r5.xyzz, r1.xyzz
adaaaaaaacaaahacabaaaakeacaaaaaaafaaaappaaaaaaaa mul r2.xyz, r1.xyzz, a5.w
aaaaaaaaaaaaapacakaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0, c10
bdaaaaaaaeaaaeacanaaaaoeabaaaaaaaaaaaaoeacaaaaaa dp4 r4.z, c13, r0
aaaaaaaaaaaaapacajaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0, c9
aaaaaaaaabaaapacaiaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r1, c8
bdaaaaaaaeaaacacanaaaaoeabaaaaaaaaaaaaoeacaaaaaa dp4 r4.y, c13, r0
bdaaaaaaaeaaabacanaaaaoeabaaaaaaabaaaaoeacaaaaaa dp4 r4.x, c13, r1
bcaaaaaaabaaacaeaeaaaakeacaaaaaaacaaaakeacaaaaaa dp3 v1.y, r4.xyzz, r2.xyzz
bcaaaaaaadaaacaeacaaaakeacaaaaaaadaaaakeacaaaaaa dp3 v3.y, r2.xyzz, r3.xyzz
bcaaaaaaabaaaeaeabaaaaoeaaaaaaaaaeaaaakeacaaaaaa dp3 v1.z, a1, r4.xyzz
bcaaaaaaabaaabaeaeaaaakeacaaaaaaafaaaaoeaaaaaaaa dp3 v1.x, r4.xyzz, a5
bcaaaaaaadaaaeaeabaaaaoeaaaaaaaaadaaaakeacaaaaaa dp3 v3.z, a1, r3.xyzz
bcaaaaaaadaaabaeafaaaaoeaaaaaaaaadaaaakeacaaaaaa dp3 v3.x, a5, r3.xyzz
adaaaaaaafaaadacadaaaaoeaaaaaaaabgaaaaoeabaaaaaa mul r5.xy, a3, c22
abaaaaaaaaaaadaeafaaaafeacaaaaaabgaaaaooabaaaaaa add v0.xy, r5.xyyy, c22.zwzw
bdaaaaaaaaaaaiadaaaaaaoeaaaaaaaaadaaaaoeabaaaaaa dp4 o0.w, a0, c3
bdaaaaaaaaaaaeadaaaaaaoeaaaaaaaaacaaaaoeabaaaaaa dp4 o0.z, a0, c2
bdaaaaaaaaaaacadaaaaaaoeaaaaaaaaabaaaaoeabaaaaaa dp4 o0.y, a0, c1
bdaaaaaaaaaaabadaaaaaaoeaaaaaaaaaaaaaaoeabaaaaaa dp4 o0.x, a0, c0
aaaaaaaaaaaaamaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v0.zw, c0
aaaaaaaaabaaaiaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v1.w, c0
aaaaaaaaacaaaiaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v2.w, c0
aaaaaaaaadaaaiaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v3.w, c0
"
}

SubProgram "d3d11_9x " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 96 // 96 used size, 7 vars
Vector 80 [_MainTex_ST] 4
ConstBuffer "UnityPerCamera" 128 // 76 used size, 8 vars
Vector 64 [_WorldSpaceCameraPos] 3
ConstBuffer "UnityLighting" 400 // 400 used size, 16 vars
Vector 0 [_WorldSpaceLightPos0] 4
Vector 288 [unity_SHAr] 4
Vector 304 [unity_SHAg] 4
Vector 320 [unity_SHAb] 4
Vector 336 [unity_SHBr] 4
Vector 352 [unity_SHBg] 4
Vector 368 [unity_SHBb] 4
Vector 384 [unity_SHC] 4
ConstBuffer "UnityPerDraw" 336 // 336 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
Matrix 192 [_Object2World] 4
Matrix 256 [_World2Object] 4
Vector 320 [unity_Scale] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityLighting" 2
BindCB "UnityPerDraw" 3
// 40 instructions, 5 temp regs, 0 temp arrays:
// ALU 23 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0_level_9_3
eefiecedjbglddjgcgdohicehbojidmcojiobnfeabaaaaaagialaaaaaeaaaaaa
daaaaaaapaadaaaaaaakaaaamiakaaaaebgpgodjliadaaaaliadaaaaaaacpopp
dmadaaaahmaaaaaaahaaceaaaaaahiaaaaaahiaaaaaaceaaabaahiaaaaaaafaa
abaaabaaaaaaaaaaabaaaeaaabaaacaaaaaaaaaaacaaaaaaabaaadaaaaaaaaaa
acaabcaaahaaaeaaaaaaaaaaadaaaaaaaeaaalaaaaaaaaaaadaaamaaadaaapaa
aaaaaaaaadaabaaaafaabcaaaaaaaaaaaaaaaaaaabacpoppfbaaaaafbhaaapka
aaaaiadpaaaaaaaaaaaaaaaaaaaaaaaabpaaaaacafaaaaiaaaaaapjabpaaaaac
afaaabiaabaaapjabpaaaaacafaaaciaacaaapjabpaaaaacafaaadiaadaaapja
aeaaaaaeaaaaadoaadaaoejaabaaoekaabaaookaabaaaaacaaaaapiaadaaoeka
afaaaaadabaaahiaaaaaffiabdaaoekaaeaaaaaeabaaahiabcaaoekaaaaaaaia
abaaoeiaaeaaaaaeaaaaahiabeaaoekaaaaakkiaabaaoeiaaeaaaaaeaaaaahia
bfaaoekaaaaappiaaaaaoeiaaiaaaaadabaaaboaabaaoejaaaaaoeiaabaaaaac
abaaahiaacaaoejaafaaaaadacaaahiaabaanciaabaamjjaaeaaaaaeabaaahia
abaamjiaabaancjaacaaoeibafaaaaadabaaahiaabaaoeiaabaappjaaiaaaaad
abaaacoaabaaoeiaaaaaoeiaaiaaaaadabaaaeoaacaaoejaaaaaoeiaabaaaaac
aaaaahiaacaaoekaafaaaaadacaaahiaaaaaffiabdaaoekaaeaaaaaeaaaaalia
bcaakekaaaaaaaiaacaakeiaaeaaaaaeaaaaahiabeaaoekaaaaakkiaaaaapeia
acaaaaadaaaaahiaaaaaoeiabfaaoekaaeaaaaaeaaaaahiaaaaaoeiabgaappka
aaaaoejbaiaaaaadadaaaboaabaaoejaaaaaoeiaaiaaaaadadaaacoaabaaoeia
aaaaoeiaaiaaaaadadaaaeoaacaaoejaaaaaoeiaafaaaaadaaaaahiaacaaoeja
bgaappkaafaaaaadabaaahiaaaaaffiabaaaoekaaeaaaaaeaaaaaliaapaakeka
aaaaaaiaabaakeiaaeaaaaaeaaaaahiabbaaoekaaaaakkiaaaaapeiaabaaaaac
aaaaaiiabhaaaakaajaaaaadabaaabiaaeaaoekaaaaaoeiaajaaaaadabaaacia
afaaoekaaaaaoeiaajaaaaadabaaaeiaagaaoekaaaaaoeiaafaaaaadacaaapia
aaaacjiaaaaakeiaajaaaaadadaaabiaahaaoekaacaaoeiaajaaaaadadaaacia
aiaaoekaacaaoeiaajaaaaadadaaaeiaajaaoekaacaaoeiaacaaaaadabaaahia
abaaoeiaadaaoeiaafaaaaadaaaaaciaaaaaffiaaaaaffiaaeaaaaaeaaaaabia
aaaaaaiaaaaaaaiaaaaaffibaeaaaaaeacaaahoaakaaoekaaaaaaaiaabaaoeia
afaaaaadaaaaapiaaaaaffjaamaaoekaaeaaaaaeaaaaapiaalaaoekaaaaaaaja
aaaaoeiaaeaaaaaeaaaaapiaanaaoekaaaaakkjaaaaaoeiaaeaaaaaeaaaaapia
aoaaoekaaaaappjaaaaaoeiaaeaaaaaeaaaaadmaaaaappiaaaaaoekaaaaaoeia
abaaaaacaaaaammaaaaaoeiappppaaaafdeieefcaiagaaaaeaaaabaaicabaaaa
fjaaaaaeegiocaaaaaaaaaaaagaaaaaafjaaaaaeegiocaaaabaaaaaaafaaaaaa
fjaaaaaeegiocaaaacaaaaaabjaaaaaafjaaaaaeegiocaaaadaaaaaabfaaaaaa
fpaaaaadpcbabaaaaaaaaaaafpaaaaadpcbabaaaabaaaaaafpaaaaadhcbabaaa
acaaaaaafpaaaaaddcbabaaaadaaaaaaghaaaaaepccabaaaaaaaaaaaabaaaaaa
gfaaaaaddccabaaaabaaaaaagfaaaaadhccabaaaacaaaaaagfaaaaadhccabaaa
adaaaaaagfaaaaadhccabaaaaeaaaaaagiaaaaacafaaaaaadiaaaaaipcaabaaa
aaaaaaaafgbfbaaaaaaaaaaaegiocaaaadaaaaaaabaaaaaadcaaaaakpcaabaaa
aaaaaaaaegiocaaaadaaaaaaaaaaaaaaagbabaaaaaaaaaaaegaobaaaaaaaaaaa
dcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaacaaaaaakgbkbaaaaaaaaaaa
egaobaaaaaaaaaaadcaaaaakpccabaaaaaaaaaaaegiocaaaadaaaaaaadaaaaaa
pgbpbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaaldccabaaaabaaaaaaegbabaaa
adaaaaaaegiacaaaaaaaaaaaafaaaaaaogikcaaaaaaaaaaaafaaaaaadiaaaaah
hcaabaaaaaaaaaaajgbebaaaabaaaaaacgbjbaaaacaaaaaadcaaaaakhcaabaaa
aaaaaaaajgbebaaaacaaaaaacgbjbaaaabaaaaaaegacbaiaebaaaaaaaaaaaaaa
diaaaaahhcaabaaaaaaaaaaaegacbaaaaaaaaaaapgbpbaaaabaaaaaadiaaaaaj
hcaabaaaabaaaaaafgifcaaaacaaaaaaaaaaaaaaegiccaaaadaaaaaabbaaaaaa
dcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaaacaaaaaa
aaaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaa
bcaaaaaakgikcaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaa
abaaaaaaegiccaaaadaaaaaabdaaaaaapgipcaaaacaaaaaaaaaaaaaaegacbaaa
abaaaaaabaaaaaahcccabaaaacaaaaaaegacbaaaaaaaaaaaegacbaaaabaaaaaa
baaaaaahbccabaaaacaaaaaaegbcbaaaabaaaaaaegacbaaaabaaaaaabaaaaaah
eccabaaaacaaaaaaegbcbaaaacaaaaaaegacbaaaabaaaaaadiaaaaaihcaabaaa
abaaaaaaegbcbaaaacaaaaaapgipcaaaadaaaaaabeaaaaaadiaaaaaihcaabaaa
acaaaaaafgafbaaaabaaaaaaegiccaaaadaaaaaaanaaaaaadcaaaaaklcaabaaa
abaaaaaaegiicaaaadaaaaaaamaaaaaaagaabaaaabaaaaaaegaibaaaacaaaaaa
dcaaaaakhcaabaaaabaaaaaaegiccaaaadaaaaaaaoaaaaaakgakbaaaabaaaaaa
egadbaaaabaaaaaadgaaaaaficaabaaaabaaaaaaabeaaaaaaaaaiadpbbaaaaai
bcaabaaaacaaaaaaegiocaaaacaaaaaabcaaaaaaegaobaaaabaaaaaabbaaaaai
ccaabaaaacaaaaaaegiocaaaacaaaaaabdaaaaaaegaobaaaabaaaaaabbaaaaai
ecaabaaaacaaaaaaegiocaaaacaaaaaabeaaaaaaegaobaaaabaaaaaadiaaaaah
pcaabaaaadaaaaaajgacbaaaabaaaaaaegakbaaaabaaaaaabbaaaaaibcaabaaa
aeaaaaaaegiocaaaacaaaaaabfaaaaaaegaobaaaadaaaaaabbaaaaaiccaabaaa
aeaaaaaaegiocaaaacaaaaaabgaaaaaaegaobaaaadaaaaaabbaaaaaiecaabaaa
aeaaaaaaegiocaaaacaaaaaabhaaaaaaegaobaaaadaaaaaaaaaaaaahhcaabaaa
acaaaaaaegacbaaaacaaaaaaegacbaaaaeaaaaaadiaaaaahicaabaaaaaaaaaaa
bkaabaaaabaaaaaabkaabaaaabaaaaaadcaaaaakicaabaaaaaaaaaaaakaabaaa
abaaaaaaakaabaaaabaaaaaadkaabaiaebaaaaaaaaaaaaaadcaaaaakhccabaaa
adaaaaaaegiccaaaacaaaaaabiaaaaaapgapbaaaaaaaaaaaegacbaaaacaaaaaa
diaaaaajhcaabaaaabaaaaaafgifcaaaabaaaaaaaeaaaaaaegiccaaaadaaaaaa
bbaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaa
abaaaaaaaeaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaa
adaaaaaabcaaaaaakgikcaaaabaaaaaaaeaaaaaaegacbaaaabaaaaaaaaaaaaai
hcaabaaaabaaaaaaegacbaaaabaaaaaaegiccaaaadaaaaaabdaaaaaadcaaaaal
hcaabaaaabaaaaaaegacbaaaabaaaaaapgipcaaaadaaaaaabeaaaaaaegbcbaia
ebaaaaaaaaaaaaaabaaaaaahcccabaaaaeaaaaaaegacbaaaaaaaaaaaegacbaaa
abaaaaaabaaaaaahbccabaaaaeaaaaaaegbcbaaaabaaaaaaegacbaaaabaaaaaa
baaaaaaheccabaaaaeaaaaaaegbcbaaaacaaaaaaegacbaaaabaaaaaadoaaaaab
ejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaaaaaaaaaaaaaaaaaaadaaaaaa
aaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaaadaaaaaaabaaaaaaapapaaaa
kjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaaahahaaaalaaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaaabaaaaaaaaaaaaaaadaaaaaa
aeaaaaaaapaaaaaaljaaaaaaaaaaaaaaaaaaaaaaadaaaaaaafaaaaaaapaaaaaa
faepfdejfeejepeoaafeebeoehefeofeaaeoepfcenebemaafeeffiedepepfcee
aaedepemepfcaaklepfdeheojiaaaaaaafaaaaaaaiaaaaaaiaaaaaaaaaaaaaaa
abaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaaaaaaaaaaaaaaaaaaadaaaaaa
abaaaaaaadamaaaaimaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaaahaiaaaa
imaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaahaiaaaaimaaaaaaadaaaaaa
aaaaaaaaadaaaaaaaeaaaaaaahaiaaaafdfgfpfaepfdejfeejepeoaafeeffied
epepfceeaaklklkl"
}

SubProgram "opengl " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Bind "vertex" Vertex
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Vector 14 [unity_LightmapST]
Vector 15 [_MainTex_ST]
"!!ARBvp1.0
# 6 ALU
PARAM c[16] = { program.local[0],
		state.matrix.mvp,
		program.local[5..15] };
MAD result.texcoord[0].xy, vertex.texcoord[0], c[15], c[15].zwzw;
MAD result.texcoord[1].xy, vertex.texcoord[1], c[14], c[14].zwzw;
DP4 result.position.w, vertex.position, c[4];
DP4 result.position.z, vertex.position, c[3];
DP4 result.position.y, vertex.position, c[2];
DP4 result.position.x, vertex.position, c[1];
END
# 6 instructions, 0 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Bind "vertex" Vertex
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Matrix 0 [glstate_matrix_mvp]
Vector 12 [unity_LightmapST]
Vector 13 [_MainTex_ST]
"vs_2_0
; 6 ALU
dcl_position0 v0
dcl_texcoord0 v3
dcl_texcoord1 v4
mad oT0.xy, v3, c13, c13.zwzw
mad oT1.xy, v4, c12, c12.zwzw
dp4 oPos.w, v0, c3
dp4 oPos.z, v0, c2
dp4 oPos.y, v0, c1
dp4 oPos.x, v0, c0
"
}

SubProgram "xbox360 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Bind "vertex" Vertex
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Vector 5 [_MainTex_ST]
Matrix 0 [glstate_matrix_mvp] 4
Vector 4 [unity_LightmapST]
// Shader Timing Estimate, in Cycles/64 vertex vector:
// ALU: 8.00 (6 instructions), vertex: 32, texture: 0,
//   sequencer: 10,  3 GPRs, 31 threads,
// Performance (if enough threads): ~32 cycles per vector
// * Vertex cycle estimates are assuming 3 vfetch_minis for every vfetch_full,
//     with <= 32 bytes per vfetch_full group.

"vs_360
backbbabaaaaabciaaaaaajmaaaaaaaaaaaaaaceaaaaaaaaaaaaaaoeaaaaaaaa
aaaaaaaaaaaaaalmaaaaaabmaaaaaalapppoadaaaaaaaaadaaaaaabmaaaaaaaa
aaaaaakjaaaaaafiaaacaaafaaabaaaaaaaaaageaaaaaaaaaaaaaaheaaacaaaa
aaaeaaaaaaaaaaiiaaaaaaaaaaaaaajiaaacaaaeaaabaaaaaaaaaageaaaaaaaa
fpengbgjgofegfhifpfdfeaaaaabaaadaaabaaaeaaabaaaaaaaaaaaaghgmhdhe
gbhegffpgngbhehcgjhifpgnhghaaaklaaadaaadaaaeaaaeaaabaaaaaaaaaaaa
hfgogjhehjfpemgjghgihegngbhafdfeaahghdfpddfpdaaadccodacodcdadddf
ddcodaaaaaaaaaaaaaaaaajmaabbaaacaaaaaaaaaaaaaaaaaaaabaecaaaaaaab
aaaaaaadaaaaaaacaaaaacjaaabaaaadaaaafaaeaadbfaafaaaadafaaaabdbfb
aaaabaakaaaabaalhabfdaadaaaabcaamcaaaaaaaaaaeaagaaaabcaameaaaaaa
aaaacaakaaaaccaaaaaaaaaaafpicaaaaaaaagiiaaaaaaaaafpiaaaaaaaaacdp
aaaaaaaaafpiaaaaaaaaapmiaaaaaaaamiapaaabaabliiaakbacadaamiapaaab
aamgiiaaklacacabmiapaaabaalbdejeklacababmiapiadoaagmaadeklacaaab
miadiaaaaabklabkilaaafafmiadiaabaalalabkilaaaeaeaaaaaaaaaaaaaaaa
aaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Matrix 256 [glstate_matrix_mvp]
Bind "vertex" Vertex
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Vector 467 [unity_LightmapST]
Vector 466 [_MainTex_ST]
"sce_vp_rsx // 6 instructions using 1 registers
[Configuration]
8
0000000603010100
[Microcode]
96
401f9c6c011d2808010400d740619f9c401f9c6c011d3908010400d740619fa0
401f9c6c01d0300d8106c0c360403f80401f9c6c01d0200d8106c0c360405f80
401f9c6c01d0100d8106c0c360409f80401f9c6c01d0000d8106c0c360411f81
"
}

SubProgram "d3d11 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Bind "vertex" Vertex
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Bind "color" Color
ConstBuffer "$Globals" 112 // 112 used size, 8 vars
Vector 80 [unity_LightmapST] 4
Vector 96 [_MainTex_ST] 4
ConstBuffer "UnityPerDraw" 336 // 64 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
BindCB "$Globals" 0
BindCB "UnityPerDraw" 1
// 7 instructions, 1 temp regs, 0 temp arrays:
// ALU 1 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0
eefiecedalopieiffenlbijlcdmbgdofaimppbfeabaaaaaanmacaaaaadaaaaaa
cmaaaaaapeaaaaaageabaaaaejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapaaaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahaaaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaa
abaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapadaaaaljaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfc
enebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheogiaaaaaaadaaaaaa
aiaaaaaafaaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaafmaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaabaaaaaaadamaaaafmaaaaaaabaaaaaaaaaaaaaa
adaaaaaaabaaaaaaamadaaaafdfgfpfaepfdejfeejepeoaafeeffiedepepfcee
aaklklklfdeieefchaabaaaaeaaaabaafmaaaaaafjaaaaaeegiocaaaaaaaaaaa
ahaaaaaafjaaaaaeegiocaaaabaaaaaaaeaaaaaafpaaaaadpcbabaaaaaaaaaaa
fpaaaaaddcbabaaaadaaaaaafpaaaaaddcbabaaaaeaaaaaaghaaaaaepccabaaa
aaaaaaaaabaaaaaagfaaaaaddccabaaaabaaaaaagfaaaaadmccabaaaabaaaaaa
giaaaaacabaaaaaadiaaaaaipcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaa
abaaaaaaabaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaabaaaaaaaaaaaaaa
agbabaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaa
abaaaaaaacaaaaaakgbkbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpccabaaa
aaaaaaaaegiocaaaabaaaaaaadaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaa
dcaaaaaldccabaaaabaaaaaaegbabaaaadaaaaaaegiacaaaaaaaaaaaagaaaaaa
ogikcaaaaaaaaaaaagaaaaaadcaaaaalmccabaaaabaaaaaaagbebaaaaeaaaaaa
agiecaaaaaaaaaaaafaaaaaakgiocaaaaaaaaaaaafaaaaaadoaaaaab"
}

SubProgram "gles " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
"!!GLES
#define SHADER_API_GLES 1
#define tex2D texture2D


#ifdef VERTEX
#define gl_ModelViewProjectionMatrix glstate_matrix_mvp
uniform mat4 glstate_matrix_mvp;

varying highp vec2 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 unity_LightmapST;

attribute vec4 _glesMultiTexCoord1;
attribute vec4 _glesMultiTexCoord0;
attribute vec4 _glesVertex;
void main ()
{
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = ((_glesMultiTexCoord1.xy * unity_LightmapST.xy) + unity_LightmapST.zw);
}



#endif
#ifdef FRAGMENT

varying highp vec2 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform sampler2D unity_Lightmap;
uniform lowp vec4 _Color;
uniform sampler2D _MainTex;
void main ()
{
  lowp vec4 c_1;
  mediump vec3 tmpvar_2;
  lowp vec3 tmpvar_3;
  tmpvar_3 = (texture2D (_MainTex, xlv_TEXCOORD0).xyz * _Color.xyz);
  tmpvar_2 = tmpvar_3;
  lowp vec3 tmpvar_4;
  tmpvar_4 = (2.0 * texture2D (unity_Lightmap, xlv_TEXCOORD1).xyz);
  mediump vec3 tmpvar_5;
  tmpvar_5 = (tmpvar_2 * tmpvar_4);
  c_1.xyz = tmpvar_5;
  c_1.w = 0.0;
  gl_FragData[0] = c_1;
}



#endif"
}

SubProgram "glesdesktop " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
"!!GLES
#define SHADER_API_GLES 1
#define tex2D texture2D


#ifdef VERTEX
#define gl_ModelViewProjectionMatrix glstate_matrix_mvp
uniform mat4 glstate_matrix_mvp;

varying highp vec2 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 unity_LightmapST;

attribute vec4 _glesMultiTexCoord1;
attribute vec4 _glesMultiTexCoord0;
attribute vec4 _glesVertex;
void main ()
{
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = ((_glesMultiTexCoord1.xy * unity_LightmapST.xy) + unity_LightmapST.zw);
}



#endif
#ifdef FRAGMENT

varying highp vec2 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform sampler2D unity_Lightmap;
uniform lowp vec4 _Color;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
void main ()
{
  lowp vec4 c_1;
  mediump vec3 tmpvar_2;
  lowp vec3 tmpvar_3;
  tmpvar_3 = (texture2D (_MainTex, xlv_TEXCOORD0).xyz * _Color.xyz);
  tmpvar_2 = tmpvar_3;
  lowp vec3 normal_4;
  normal_4.xy = ((texture2D (_BumpMap, xlv_TEXCOORD0).wy * 2.0) - 1.0);
  normal_4.z = sqrt(((1.0 - (normal_4.x * normal_4.x)) - (normal_4.y * normal_4.y)));
  lowp vec4 tmpvar_5;
  tmpvar_5 = texture2D (unity_Lightmap, xlv_TEXCOORD1);
  lowp vec3 tmpvar_6;
  tmpvar_6 = ((8.0 * tmpvar_5.w) * tmpvar_5.xyz);
  mediump vec3 tmpvar_7;
  tmpvar_7 = (tmpvar_2 * tmpvar_6);
  c_1.xyz = tmpvar_7;
  c_1.w = 0.0;
  gl_FragData[0] = c_1;
}



#endif"
}

SubProgram "flash " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Bind "vertex" Vertex
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Matrix 0 [glstate_matrix_mvp]
Vector 12 [unity_LightmapST]
Vector 13 [_MainTex_ST]
"agal_vs
[bc]
adaaaaaaaaaaadacadaaaaoeaaaaaaaaanaaaaoeabaaaaaa mul r0.xy, a3, c13
abaaaaaaaaaaadaeaaaaaafeacaaaaaaanaaaaooabaaaaaa add v0.xy, r0.xyyy, c13.zwzw
adaaaaaaaaaaadacaeaaaaoeaaaaaaaaamaaaaoeabaaaaaa mul r0.xy, a4, c12
abaaaaaaabaaadaeaaaaaafeacaaaaaaamaaaaooabaaaaaa add v1.xy, r0.xyyy, c12.zwzw
bdaaaaaaaaaaaiadaaaaaaoeaaaaaaaaadaaaaoeabaaaaaa dp4 o0.w, a0, c3
bdaaaaaaaaaaaeadaaaaaaoeaaaaaaaaacaaaaoeabaaaaaa dp4 o0.z, a0, c2
bdaaaaaaaaaaacadaaaaaaoeaaaaaaaaabaaaaoeabaaaaaa dp4 o0.y, a0, c1
bdaaaaaaaaaaabadaaaaaaoeaaaaaaaaaaaaaaoeabaaaaaa dp4 o0.x, a0, c0
aaaaaaaaaaaaamaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v0.zw, c0
aaaaaaaaabaaamaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v1.zw, c0
"
}

SubProgram "d3d11_9x " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Bind "vertex" Vertex
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Bind "color" Color
ConstBuffer "$Globals" 112 // 112 used size, 8 vars
Vector 80 [unity_LightmapST] 4
Vector 96 [_MainTex_ST] 4
ConstBuffer "UnityPerDraw" 336 // 64 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
BindCB "$Globals" 0
BindCB "UnityPerDraw" 1
// 7 instructions, 1 temp regs, 0 temp arrays:
// ALU 1 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0_level_9_3
eefiecedkkbcgonaliigeljcnnkllmoijdeeaoenabaaaaaaoiadaaaaaeaaaaaa
daaaaaaadiabaaaalaacaaaahiadaaaaebgpgodjaaabaaaaaaabaaaaaaacpopp
maaaaaaaeaaaaaaaacaaceaaaaaadmaaaaaadmaaaaaaceaaabaadmaaaaaaafaa
acaaabaaaaaaaaaaabaaaaaaaeaaadaaaaaaaaaaaaaaaaaaabacpoppbpaaaaac
afaaaaiaaaaaapjabpaaaaacafaaadiaadaaapjabpaaaaacafaaaeiaaeaaapja
aeaaaaaeaaaaadoaadaaoejaacaaoekaacaaookaaeaaaaaeaaaaamoaaeaabeja
abaabekaabaalekaafaaaaadaaaaapiaaaaaffjaaeaaoekaaeaaaaaeaaaaapia
adaaoekaaaaaaajaaaaaoeiaaeaaaaaeaaaaapiaafaaoekaaaaakkjaaaaaoeia
aeaaaaaeaaaaapiaagaaoekaaaaappjaaaaaoeiaaeaaaaaeaaaaadmaaaaappia
aaaaoekaaaaaoeiaabaaaaacaaaaammaaaaaoeiappppaaaafdeieefchaabaaaa
eaaaabaafmaaaaaafjaaaaaeegiocaaaaaaaaaaaahaaaaaafjaaaaaeegiocaaa
abaaaaaaaeaaaaaafpaaaaadpcbabaaaaaaaaaaafpaaaaaddcbabaaaadaaaaaa
fpaaaaaddcbabaaaaeaaaaaaghaaaaaepccabaaaaaaaaaaaabaaaaaagfaaaaad
dccabaaaabaaaaaagfaaaaadmccabaaaabaaaaaagiaaaaacabaaaaaadiaaaaai
pcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaabaaaaaaabaaaaaadcaaaaak
pcaabaaaaaaaaaaaegiocaaaabaaaaaaaaaaaaaaagbabaaaaaaaaaaaegaobaaa
aaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaabaaaaaaacaaaaaakgbkbaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaakpccabaaaaaaaaaaaegiocaaaabaaaaaa
adaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaaldccabaaaabaaaaaa
egbabaaaadaaaaaaegiacaaaaaaaaaaaagaaaaaaogikcaaaaaaaaaaaagaaaaaa
dcaaaaalmccabaaaabaaaaaaagbebaaaaeaaaaaaagiecaaaaaaaaaaaafaaaaaa
kgiocaaaaaaaaaaaafaaaaaadoaaaaabejfdeheomaaaaaaaagaaaaaaaiaaaaaa
jiaaaaaaaaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaabaaaaaaapaaaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaa
acaaaaaaahaaaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaa
laaaaaaaabaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapadaaaaljaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofe
aaeoepfcenebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheogiaaaaaa
adaaaaaaaiaaaaaafaaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaa
fmaaaaaaaaaaaaaaaaaaaaaaadaaaaaaabaaaaaaadamaaaafmaaaaaaabaaaaaa
aaaaaaaaadaaaaaaabaaaaaaamadaaaafdfgfpfaepfdejfeejepeoaafeeffied
epepfceeaaklklkl"
}

SubProgram "opengl " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Bind "vertex" Vertex
Bind "tangent" ATTR14
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 13 [_WorldSpaceCameraPos]
Vector 14 [_ProjectionParams]
Vector 15 [_WorldSpaceLightPos0]
Vector 16 [unity_SHAr]
Vector 17 [unity_SHAg]
Vector 18 [unity_SHAb]
Vector 19 [unity_SHBr]
Vector 20 [unity_SHBg]
Vector 21 [unity_SHBb]
Vector 22 [unity_SHC]
Matrix 5 [_Object2World]
Matrix 9 [_World2Object]
Vector 23 [unity_Scale]
Vector 24 [_MainTex_ST]
"!!ARBvp1.0
# 48 ALU
PARAM c[25] = { { 1, 0.5 },
		state.matrix.mvp,
		program.local[5..24] };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
MUL R1.xyz, vertex.normal, c[23].w;
DP3 R2.w, R1, c[6];
DP3 R0.x, R1, c[5];
DP3 R0.z, R1, c[7];
MOV R0.y, R2.w;
MOV R0.w, c[0].x;
MUL R1, R0.xyzz, R0.yzzx;
DP4 R2.z, R0, c[18];
DP4 R2.y, R0, c[17];
DP4 R2.x, R0, c[16];
MUL R0.w, R2, R2;
MAD R0.w, R0.x, R0.x, -R0;
DP4 R0.z, R1, c[21];
DP4 R0.y, R1, c[20];
DP4 R0.x, R1, c[19];
ADD R0.xyz, R2, R0;
MUL R1.xyz, R0.w, c[22];
ADD result.texcoord[2].xyz, R0, R1;
MOV R1.xyz, c[13];
MOV R1.w, c[0].x;
MOV R0.xyz, vertex.attrib[14];
DP4 R2.z, R1, c[11];
DP4 R2.y, R1, c[10];
DP4 R2.x, R1, c[9];
MAD R2.xyz, R2, c[23].w, -vertex.position;
MUL R1.xyz, vertex.normal.zxyw, R0.yzxw;
MAD R1.xyz, vertex.normal.yzxw, R0.zxyw, -R1;
MOV R0, c[15];
MUL R1.xyz, R1, vertex.attrib[14].w;
DP4 R3.z, R0, c[11];
DP4 R3.y, R0, c[10];
DP4 R3.x, R0, c[9];
DP4 R0.w, vertex.position, c[4];
DP4 R0.z, vertex.position, c[3];
DP4 R0.x, vertex.position, c[1];
DP4 R0.y, vertex.position, c[2];
DP3 result.texcoord[1].y, R3, R1;
DP3 result.texcoord[3].y, R1, R2;
MUL R1.xyz, R0.xyww, c[0].y;
MUL R1.y, R1, c[14].x;
DP3 result.texcoord[1].z, vertex.normal, R3;
DP3 result.texcoord[1].x, R3, vertex.attrib[14];
DP3 result.texcoord[3].z, vertex.normal, R2;
DP3 result.texcoord[3].x, vertex.attrib[14], R2;
ADD result.texcoord[4].xy, R1, R1.z;
MOV result.position, R0;
MOV result.texcoord[4].zw, R0;
MAD result.texcoord[0].xy, vertex.texcoord[0], c[24], c[24].zwzw;
END
# 48 instructions, 4 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Matrix 0 [glstate_matrix_mvp]
Vector 12 [_WorldSpaceCameraPos]
Vector 13 [_ProjectionParams]
Vector 14 [_ScreenParams]
Vector 15 [_WorldSpaceLightPos0]
Vector 16 [unity_SHAr]
Vector 17 [unity_SHAg]
Vector 18 [unity_SHAb]
Vector 19 [unity_SHBr]
Vector 20 [unity_SHBg]
Vector 21 [unity_SHBb]
Vector 22 [unity_SHC]
Matrix 4 [_Object2World]
Matrix 8 [_World2Object]
Vector 23 [unity_Scale]
Vector 24 [_MainTex_ST]
"vs_2_0
; 51 ALU
def c25, 1.00000000, 0.50000000, 0, 0
dcl_position0 v0
dcl_tangent0 v1
dcl_normal0 v2
dcl_texcoord0 v3
mul r1.xyz, v2, c23.w
dp3 r2.w, r1, c5
dp3 r0.x, r1, c4
dp3 r0.z, r1, c6
mov r0.y, r2.w
mov r0.w, c25.x
mul r1, r0.xyzz, r0.yzzx
dp4 r2.z, r0, c18
dp4 r2.y, r0, c17
dp4 r2.x, r0, c16
mul r0.w, r2, r2
mad r0.w, r0.x, r0.x, -r0
dp4 r0.z, r1, c21
dp4 r0.y, r1, c20
dp4 r0.x, r1, c19
mul r1.xyz, r0.w, c22
add r0.xyz, r2, r0
add oT2.xyz, r0, r1
mov r0.w, c25.x
mov r0.xyz, c12
dp4 r1.z, r0, c10
dp4 r1.y, r0, c9
dp4 r1.x, r0, c8
mad r3.xyz, r1, c23.w, -v0
mov r0.xyz, v1
mul r1.xyz, v2.zxyw, r0.yzxw
mov r0.xyz, v1
mad r1.xyz, v2.yzxw, r0.zxyw, -r1
mul r2.xyz, r1, v1.w
mov r0, c10
dp4 r4.z, c15, r0
mov r0, c9
dp4 r4.y, c15, r0
mov r1, c8
dp4 r4.x, c15, r1
dp4 r0.w, v0, c3
dp4 r0.z, v0, c2
dp4 r0.x, v0, c0
dp4 r0.y, v0, c1
mul r1.xyz, r0.xyww, c25.y
mul r1.y, r1, c13.x
dp3 oT1.y, r4, r2
dp3 oT3.y, r2, r3
dp3 oT1.z, v2, r4
dp3 oT1.x, r4, v1
dp3 oT3.z, v2, r3
dp3 oT3.x, v1, r3
mad oT4.xy, r1.z, c14.zwzw, r1
mov oPos, r0
mov oT4.zw, r0
mad oT0.xy, v3, c24, c24.zwzw
"
}

SubProgram "xbox360 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 23 [_MainTex_ST]
Matrix 15 [_Object2World] 3
Vector 1 [_ProjectionParams]
Vector 2 [_ScreenParams]
Matrix 18 [_World2Object] 4
Vector 0 [_WorldSpaceCameraPos]
Vector 3 [_WorldSpaceLightPos0]
Matrix 11 [glstate_matrix_mvp] 4
Vector 6 [unity_SHAb]
Vector 5 [unity_SHAg]
Vector 4 [unity_SHAr]
Vector 9 [unity_SHBb]
Vector 8 [unity_SHBg]
Vector 7 [unity_SHBr]
Vector 10 [unity_SHC]
Vector 22 [unity_Scale]
// Shader Timing Estimate, in Cycles/64 vertex vector:
// ALU: 52.00 (39 instructions), vertex: 32, texture: 0,
//   sequencer: 20,  10 GPRs, 18 threads,
// Performance (if enough threads): ~52 cycles per vector
// * Vertex cycle estimates are assuming 3 vfetch_minis for every vfetch_full,
//     with <= 32 bytes per vfetch_full group.

"vs_360
backbbabaaaaadeeaaaaacimaaaaaaaaaaaaaaceaaaaackiaaaaacnaaaaaaaaa
aaaaaaaaaaaaaciaaaaaaabmaaaaachdpppoadaaaaaaaabaaaaaaabmaaaaaaaa
aaaaacgmaaaaabfmaaacaabhaaabaaaaaaaaabgiaaaaaaaaaaaaabhiaaacaaap
aaadaaaaaaaaabiiaaaaaaaaaaaaabjiaaacaaabaaabaaaaaaaaabgiaaaaaaaa
aaaaabkkaaacaaacaaabaaaaaaaaabgiaaaaaaaaaaaaabliaaacaabcaaaeaaaa
aaaaabiiaaaaaaaaaaaaabmgaaacaaaaaaabaaaaaaaaabnmaaaaaaaaaaaaabom
aaacaaadaaabaaaaaaaaabgiaaaaaaaaaaaaacabaaacaaalaaaeaaaaaaaaabii
aaaaaaaaaaaaacbeaaacaaagaaabaaaaaaaaabgiaaaaaaaaaaaaacbpaaacaaaf
aaabaaaaaaaaabgiaaaaaaaaaaaaacckaaacaaaeaaabaaaaaaaaabgiaaaaaaaa
aaaaacdfaaacaaajaaabaaaaaaaaabgiaaaaaaaaaaaaaceaaaacaaaiaaabaaaa
aaaaabgiaaaaaaaaaaaaacelaaacaaahaaabaaaaaaaaabgiaaaaaaaaaaaaacfg
aaacaaakaaabaaaaaaaaabgiaaaaaaaaaaaaacgaaaacaabgaaabaaaaaaaaabgi
aaaaaaaafpengbgjgofegfhifpfdfeaaaaabaaadaaabaaaeaaabaaaaaaaaaaaa
fpepgcgkgfgdhedcfhgphcgmgeaaklklaaadaaadaaaeaaaeaaabaaaaaaaaaaaa
fpfahcgpgkgfgdhegjgpgofagbhcgbgnhdaafpfdgdhcgfgfgofagbhcgbgnhdaa
fpfhgphcgmgedcepgcgkgfgdheaafpfhgphcgmgefdhagbgdgfedgbgngfhcgbfa
gphdaaklaaabaaadaaabaaadaaabaaaaaaaaaaaafpfhgphcgmgefdhagbgdgfem
gjghgihefagphddaaaghgmhdhegbhegffpgngbhehcgjhifpgnhghaaahfgogjhe
hjfpfdeiebgcaahfgogjhehjfpfdeiebghaahfgogjhehjfpfdeiebhcaahfgogj
hehjfpfdeiecgcaahfgogjhehjfpfdeiecghaahfgogjhehjfpfdeiechcaahfgo
gjhehjfpfdeiedaahfgogjhehjfpfdgdgbgmgfaahghdfpddfpdaaadccodacodc
dadddfddcodaaaklaaaaaaaaaaaaaaabaaaaaaaaaaaaaaaaaaaaaabeaapmaaba
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaeaaaaaacemaaebaaajaaaaaaaa
aaaaaaaaaaaadmkfaaaaaaabaaaaaaaeaaaaaaakaaaaacjaaabaaaafaaaagaag
aaaadaahaadafaaiaaaadafaaaabhbfbaaaehcfcaaafhdfdaaaipefeaaaabacf
aaaaaabpaaaaaacaaaaabacbaaaabacpaaaaaaccaaaaaacdaaaabaceaaaaaabo
aaaabacnaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaadpaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaapaffeaafaaaabcaamcaaaaaaaaaafaajaaaabcaameaaaaaaaaaagaao
gabebcaabcaaaaaaaaaagabkgacabcaabcaaaaaaaaaagacgeacmbcaaccaaaaaa
afpicaaaaaaaagiiaaaaaaaaafpigaaaaaaaagiiaaaaaaaaafpibaaaaaaaaeeh
aaaaaaaaafpiaaaaaaaaapmiaaaaaaaamiapaaadaabliiaakbacaoaamiapaaad
aamgnapiklacanadmiapaaadaalbdepiklacamadmiapaaaiaagmnajeklacalad
miapiadoaananaaaocaiaiaamiahaaadaaleblaacbbfadaamiahaaaeaamamgma
albeaabfmiahaaajaalelbleclbdaaaemiahaaafaamdgfaaobabagaamiahaaah
aamamgleclbeadadmialaaadaalkblaakbabbgaamiahaaaeaalbleaakbadbbaa
miahaaahaalelbleclbdadahmiahaaafablklomaolabagafmiahaaajaamagmle
clbcaaajmiahaaacabmablmaklajbgacceihaeafaamablgmobafagiamiahaaah
aamagmleclbcadahmiahaaadaagmlemakladbaaemiahaaaeaabllemakladapad
aibhabadaamagmggkbaippaemiamiaaeaanlnlaaocaiaiaamiabiaabaaloloaa
paahagaamiaciaabaaloloaapaafahaamiaeiaabaalomdaapaahabaamiabiaad
aaloloaapaacagaamiaciaadaaloloaapaafacaamiaeiaadaalomdaapaacabaa
miadiaaaaalalabkilaabhbhaicbabacaadoanmbgpaeaeaeaiecabacaadoanlb
gpafaeaeaiieabacaadoanlmgpagaeaemiabaaaaaakhkhaakpabahaamiacaaaa
aakhkhaakpabaiaaaibeabaaaakhkhgmkpabajaeaiciabadaalbgmmgkbadabae
miadiaaeaamgbkbikladacadgeihaaaaaalologboaacaaabmiahiaacaablmagf
klaaakaaaaaaaaaaaaaaaaaaaaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Matrix 256 [glstate_matrix_mvp]
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 467 [_WorldSpaceCameraPos]
Vector 466 [_ProjectionParams]
Vector 465 [_WorldSpaceLightPos0]
Vector 464 [unity_SHAr]
Vector 463 [unity_SHAg]
Vector 462 [unity_SHAb]
Vector 461 [unity_SHBr]
Vector 460 [unity_SHBg]
Vector 459 [unity_SHBb]
Vector 458 [unity_SHC]
Matrix 260 [_Object2World]
Matrix 264 [_World2Object]
Vector 457 [unity_Scale]
Vector 456 [_MainTex_ST]
"sce_vp_rsx // 46 instructions using 7 registers
[Configuration]
8
0000002e41050700
[Defaults]
1
455 1
3f000000
[Microcode]
736
00009c6c005d100d8186c0836041fffc00011c6c00400e0c0106c0836041dffc
00021c6c005d300c0186c0836041dffc00019c6c009c920c013fc0c36041dffc
401f9c6c011c8808010400d740619f9c00001c6c01d0300d8106c0c360403ffc
00001c6c01d0200d8106c0c360405ffc00001c6c01d0100d8106c0c360409ffc
00001c6c01d0000d8106c0c360411ffc00029c6c01d0a00d8286c0c360405ffc
00029c6c01d0900d8286c0c360409ffc00029c6c01d0800d8286c0c360411ffc
00031c6c0150400c068600c360411ffc00031c6c0150600c068600c360405ffc
00009c6c0150500c068600c360403ffc00009c6c0190a00c0886c0c360405ffc
00009c6c0190900c0886c0c360409ffc00009c6c0190800c0886c0c360411ffc
00019c6c00800243011842436041dffc00011c6c010002308121826301a1dffc
401f9c6c0040000d8086c0836041ff80401f9c6c004000558086c08360407fac
00009c6c011c900c02bfc0e30041dffc00001c6c009c700e008000c36041dffc
401f9c6c0140020c0106054360405fa0401f9c6c01400e0c0a86008360411fa0
00001c6c009d202a808000c360409ffc00001c6c0080007f82bfc14360403ffc
00031c6c0040007f8286c08360409ffc401f9c6c00c000080086c09540219fac
00011c6c00800e0c04bfc0836041dffc401f9c6c0140020c0106014360405fa8
401f9c6c01400e0c0106014360411fa800001c6c019ce00c0c86c0c360405ffc
00001c6c019cf00c0c86c0c360409ffc00001c6c019d000c0c86c0c360411ffc
00001c6c010000000c80067fe0203ffc00019c6c0080000d0c9a06436041fffc
401f9c6c0140000c0a86024360409fa0401f9c6c0140000c0486014360409fa8
00009c6c01dcb00d8686c0c360405ffc00009c6c01dcc00d8686c0c360409ffc
00009c6c01dcd00d8686c0c360411ffc00001c6c00c0000c0086c08300a1dffc
00009c6c009ca07f808600c36041dffc401f9c6c00c0000c0286c0830021dfa5
"
}

SubProgram "d3d11 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 160 // 160 used size, 8 vars
Vector 144 [_MainTex_ST] 4
ConstBuffer "UnityPerCamera" 128 // 96 used size, 8 vars
Vector 64 [_WorldSpaceCameraPos] 3
Vector 80 [_ProjectionParams] 4
ConstBuffer "UnityLighting" 400 // 400 used size, 16 vars
Vector 0 [_WorldSpaceLightPos0] 4
Vector 288 [unity_SHAr] 4
Vector 304 [unity_SHAg] 4
Vector 320 [unity_SHAb] 4
Vector 336 [unity_SHBr] 4
Vector 352 [unity_SHBg] 4
Vector 368 [unity_SHBb] 4
Vector 384 [unity_SHC] 4
ConstBuffer "UnityPerDraw" 336 // 336 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
Matrix 192 [_Object2World] 4
Matrix 256 [_World2Object] 4
Vector 320 [unity_Scale] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityLighting" 2
BindCB "UnityPerDraw" 3
// 45 instructions, 6 temp regs, 0 temp arrays:
// ALU 26 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0
eefiecedicgmlbopmnhmcbdcpnaomdhakgaimihnabaaaaaafeaiaaaaadaaaaaa
cmaaaaaapeaaaaaakmabaaaaejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaa
abaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaljaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfc
enebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheolaaaaaaaagaaaaaa
aiaaaaaajiaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaakeaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaabaaaaaaadamaaaakeaaaaaaabaaaaaaaaaaaaaa
adaaaaaaacaaaaaaahaiaaaakeaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaa
ahaiaaaakeaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaahaiaaaakeaaaaaa
aeaaaaaaaaaaaaaaadaaaaaaafaaaaaaapaaaaaafdfgfpfaepfdejfeejepeoaa
feeffiedepepfceeaaklklklfdeieefckaagaaaaeaaaabaakiabaaaafjaaaaae
egiocaaaaaaaaaaaakaaaaaafjaaaaaeegiocaaaabaaaaaaagaaaaaafjaaaaae
egiocaaaacaaaaaabjaaaaaafjaaaaaeegiocaaaadaaaaaabfaaaaaafpaaaaad
pcbabaaaaaaaaaaafpaaaaadpcbabaaaabaaaaaafpaaaaadhcbabaaaacaaaaaa
fpaaaaaddcbabaaaadaaaaaaghaaaaaepccabaaaaaaaaaaaabaaaaaagfaaaaad
dccabaaaabaaaaaagfaaaaadhccabaaaacaaaaaagfaaaaadhccabaaaadaaaaaa
gfaaaaadhccabaaaaeaaaaaagfaaaaadpccabaaaafaaaaaagiaaaaacagaaaaaa
diaaaaaipcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaadaaaaaaabaaaaaa
dcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaaaaaaaaaagbabaaaaaaaaaaa
egaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaacaaaaaa
kgbkbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaa
adaaaaaaadaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadgaaaaafpccabaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaaldccabaaaabaaaaaaegbabaaaadaaaaaa
egiacaaaaaaaaaaaajaaaaaaogikcaaaaaaaaaaaajaaaaaadiaaaaahhcaabaaa
abaaaaaajgbebaaaabaaaaaacgbjbaaaacaaaaaadcaaaaakhcaabaaaabaaaaaa
jgbebaaaacaaaaaacgbjbaaaabaaaaaaegacbaiaebaaaaaaabaaaaaadiaaaaah
hcaabaaaabaaaaaaegacbaaaabaaaaaapgbpbaaaabaaaaaadiaaaaajhcaabaaa
acaaaaaafgifcaaaacaaaaaaaaaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaal
hcaabaaaacaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaaacaaaaaaaaaaaaaa
egacbaaaacaaaaaadcaaaaalhcaabaaaacaaaaaaegiccaaaadaaaaaabcaaaaaa
kgikcaaaacaaaaaaaaaaaaaaegacbaaaacaaaaaadcaaaaalhcaabaaaacaaaaaa
egiccaaaadaaaaaabdaaaaaapgipcaaaacaaaaaaaaaaaaaaegacbaaaacaaaaaa
baaaaaahcccabaaaacaaaaaaegacbaaaabaaaaaaegacbaaaacaaaaaabaaaaaah
bccabaaaacaaaaaaegbcbaaaabaaaaaaegacbaaaacaaaaaabaaaaaaheccabaaa
acaaaaaaegbcbaaaacaaaaaaegacbaaaacaaaaaadiaaaaaihcaabaaaacaaaaaa
egbcbaaaacaaaaaapgipcaaaadaaaaaabeaaaaaadiaaaaaihcaabaaaadaaaaaa
fgafbaaaacaaaaaaegiccaaaadaaaaaaanaaaaaadcaaaaaklcaabaaaacaaaaaa
egiicaaaadaaaaaaamaaaaaaagaabaaaacaaaaaaegaibaaaadaaaaaadcaaaaak
hcaabaaaacaaaaaaegiccaaaadaaaaaaaoaaaaaakgakbaaaacaaaaaaegadbaaa
acaaaaaadgaaaaaficaabaaaacaaaaaaabeaaaaaaaaaiadpbbaaaaaibcaabaaa
adaaaaaaegiocaaaacaaaaaabcaaaaaaegaobaaaacaaaaaabbaaaaaiccaabaaa
adaaaaaaegiocaaaacaaaaaabdaaaaaaegaobaaaacaaaaaabbaaaaaiecaabaaa
adaaaaaaegiocaaaacaaaaaabeaaaaaaegaobaaaacaaaaaadiaaaaahpcaabaaa
aeaaaaaajgacbaaaacaaaaaaegakbaaaacaaaaaabbaaaaaibcaabaaaafaaaaaa
egiocaaaacaaaaaabfaaaaaaegaobaaaaeaaaaaabbaaaaaiccaabaaaafaaaaaa
egiocaaaacaaaaaabgaaaaaaegaobaaaaeaaaaaabbaaaaaiecaabaaaafaaaaaa
egiocaaaacaaaaaabhaaaaaaegaobaaaaeaaaaaaaaaaaaahhcaabaaaadaaaaaa
egacbaaaadaaaaaaegacbaaaafaaaaaadiaaaaahicaabaaaabaaaaaabkaabaaa
acaaaaaabkaabaaaacaaaaaadcaaaaakicaabaaaabaaaaaaakaabaaaacaaaaaa
akaabaaaacaaaaaadkaabaiaebaaaaaaabaaaaaadcaaaaakhccabaaaadaaaaaa
egiccaaaacaaaaaabiaaaaaapgapbaaaabaaaaaaegacbaaaadaaaaaadiaaaaaj
hcaabaaaacaaaaaafgifcaaaabaaaaaaaeaaaaaaegiccaaaadaaaaaabbaaaaaa
dcaaaaalhcaabaaaacaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaaabaaaaaa
aeaaaaaaegacbaaaacaaaaaadcaaaaalhcaabaaaacaaaaaaegiccaaaadaaaaaa
bcaaaaaakgikcaaaabaaaaaaaeaaaaaaegacbaaaacaaaaaaaaaaaaaihcaabaaa
acaaaaaaegacbaaaacaaaaaaegiccaaaadaaaaaabdaaaaaadcaaaaalhcaabaaa
acaaaaaaegacbaaaacaaaaaapgipcaaaadaaaaaabeaaaaaaegbcbaiaebaaaaaa
aaaaaaaabaaaaaahcccabaaaaeaaaaaaegacbaaaabaaaaaaegacbaaaacaaaaaa
baaaaaahbccabaaaaeaaaaaaegbcbaaaabaaaaaaegacbaaaacaaaaaabaaaaaah
eccabaaaaeaaaaaaegbcbaaaacaaaaaaegacbaaaacaaaaaadiaaaaaiccaabaaa
aaaaaaaabkaabaaaaaaaaaaaakiacaaaabaaaaaaafaaaaaadiaaaaakncaabaaa
abaaaaaaagahbaaaaaaaaaaaaceaaaaaaaaaaadpaaaaaaaaaaaaaadpaaaaaadp
dgaaaaafmccabaaaafaaaaaakgaobaaaaaaaaaaaaaaaaaahdccabaaaafaaaaaa
kgakbaaaabaaaaaamgaabaaaabaaaaaadoaaaaab"
}

SubProgram "gles " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
"!!GLES
#define SHADER_API_GLES 1
#define tex2D texture2D


#ifdef VERTEX
#define gl_ModelViewProjectionMatrix glstate_matrix_mvp
uniform mat4 glstate_matrix_mvp;

varying highp vec4 xlv_TEXCOORD4;
varying highp vec3 xlv_TEXCOORD3;
varying lowp vec3 xlv_TEXCOORD2;
varying lowp vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 unity_Scale;
uniform highp mat4 _World2Object;
uniform highp mat4 _Object2World;

uniform highp mat4 unity_World2Shadow[4];
uniform highp vec4 unity_SHC;
uniform highp vec4 unity_SHBb;
uniform highp vec4 unity_SHBg;
uniform highp vec4 unity_SHBr;
uniform highp vec4 unity_SHAb;
uniform highp vec4 unity_SHAg;
uniform highp vec4 unity_SHAr;
uniform lowp vec4 _WorldSpaceLightPos0;
uniform highp vec3 _WorldSpaceCameraPos;
attribute vec4 _glesTANGENT;
attribute vec4 _glesMultiTexCoord0;
attribute vec3 _glesNormal;
attribute vec4 _glesVertex;
void main ()
{
  vec4 tmpvar_1;
  tmpvar_1.xyz = normalize(_glesTANGENT.xyz);
  tmpvar_1.w = _glesTANGENT.w;
  vec3 tmpvar_2;
  tmpvar_2 = normalize(_glesNormal);
  highp vec3 shlight_3;
  lowp vec3 tmpvar_4;
  lowp vec3 tmpvar_5;
  mat3 tmpvar_6;
  tmpvar_6[0] = _Object2World[0].xyz;
  tmpvar_6[1] = _Object2World[1].xyz;
  tmpvar_6[2] = _Object2World[2].xyz;
  highp vec3 tmpvar_7;
  highp vec3 tmpvar_8;
  tmpvar_7 = tmpvar_1.xyz;
  tmpvar_8 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_9;
  tmpvar_9[0].x = tmpvar_7.x;
  tmpvar_9[0].y = tmpvar_8.x;
  tmpvar_9[0].z = tmpvar_2.x;
  tmpvar_9[1].x = tmpvar_7.y;
  tmpvar_9[1].y = tmpvar_8.y;
  tmpvar_9[1].z = tmpvar_2.y;
  tmpvar_9[2].x = tmpvar_7.z;
  tmpvar_9[2].y = tmpvar_8.z;
  tmpvar_9[2].z = tmpvar_2.z;
  highp vec3 tmpvar_10;
  tmpvar_10 = (tmpvar_9 * (_World2Object * _WorldSpaceLightPos0).xyz);
  tmpvar_4 = tmpvar_10;
  highp vec4 tmpvar_11;
  tmpvar_11.w = 1.0;
  tmpvar_11.xyz = _WorldSpaceCameraPos;
  highp vec4 tmpvar_12;
  tmpvar_12.w = 1.0;
  tmpvar_12.xyz = (tmpvar_6 * (tmpvar_2 * unity_Scale.w));
  mediump vec3 tmpvar_13;
  mediump vec4 normal_14;
  normal_14 = tmpvar_12;
  highp float vC_15;
  mediump vec3 x3_16;
  mediump vec3 x2_17;
  mediump vec3 x1_18;
  highp float tmpvar_19;
  tmpvar_19 = dot (unity_SHAr, normal_14);
  x1_18.x = tmpvar_19;
  highp float tmpvar_20;
  tmpvar_20 = dot (unity_SHAg, normal_14);
  x1_18.y = tmpvar_20;
  highp float tmpvar_21;
  tmpvar_21 = dot (unity_SHAb, normal_14);
  x1_18.z = tmpvar_21;
  mediump vec4 tmpvar_22;
  tmpvar_22 = (normal_14.xyzz * normal_14.yzzx);
  highp float tmpvar_23;
  tmpvar_23 = dot (unity_SHBr, tmpvar_22);
  x2_17.x = tmpvar_23;
  highp float tmpvar_24;
  tmpvar_24 = dot (unity_SHBg, tmpvar_22);
  x2_17.y = tmpvar_24;
  highp float tmpvar_25;
  tmpvar_25 = dot (unity_SHBb, tmpvar_22);
  x2_17.z = tmpvar_25;
  mediump float tmpvar_26;
  tmpvar_26 = ((normal_14.x * normal_14.x) - (normal_14.y * normal_14.y));
  vC_15 = tmpvar_26;
  highp vec3 tmpvar_27;
  tmpvar_27 = (unity_SHC.xyz * vC_15);
  x3_16 = tmpvar_27;
  tmpvar_13 = ((x1_18 + x2_17) + x3_16);
  shlight_3 = tmpvar_13;
  tmpvar_5 = shlight_3;
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = tmpvar_4;
  xlv_TEXCOORD2 = tmpvar_5;
  xlv_TEXCOORD3 = (tmpvar_9 * (((_World2Object * tmpvar_11).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD4 = (unity_World2Shadow[0] * (_Object2World * _glesVertex));
}



#endif
#ifdef FRAGMENT

varying highp vec4 xlv_TEXCOORD4;
varying highp vec3 xlv_TEXCOORD3;
varying lowp vec3 xlv_TEXCOORD2;
varying lowp vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform mediump float _Gloss;
uniform mediump float _Shininess;
uniform lowp vec4 _Color;
uniform sampler2D _SpecMap;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform sampler2D _ShadowMapTexture;
uniform lowp vec4 _LightColor0;
uniform highp vec4 _LightShadowData;
void main ()
{
  lowp vec4 c_1;
  mediump vec3 tmpvar_2;
  mediump vec3 tmpvar_3;
  lowp vec4 tmpvar_4;
  tmpvar_4 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_5;
  tmpvar_5 = (tmpvar_4.xyz * _Color.xyz);
  tmpvar_2 = tmpvar_5;
  lowp vec4 tmpvar_6;
  tmpvar_6 = texture2D (_SpecMap, xlv_TEXCOORD0);
  mediump vec3 tmpvar_7;
  tmpvar_7 = ((tmpvar_4.w * tmpvar_6.xyz) * _Gloss);
  lowp vec3 tmpvar_8;
  tmpvar_8 = ((texture2D (_BumpMap, xlv_TEXCOORD0).xyz * 2.0) - 1.0);
  tmpvar_3 = tmpvar_8;
  lowp float tmpvar_9;
  mediump float lightShadowDataX_10;
  highp float dist_11;
  lowp float tmpvar_12;
  tmpvar_12 = texture2DProj (_ShadowMapTexture, xlv_TEXCOORD4).x;
  dist_11 = tmpvar_12;
  highp float tmpvar_13;
  tmpvar_13 = _LightShadowData.x;
  lightShadowDataX_10 = tmpvar_13;
  highp float tmpvar_14;
  tmpvar_14 = max (float((dist_11 > (xlv_TEXCOORD4.z / xlv_TEXCOORD4.w))), lightShadowDataX_10);
  tmpvar_9 = tmpvar_14;
  highp vec3 tmpvar_15;
  tmpvar_15 = normalize(xlv_TEXCOORD3);
  mediump vec3 lightDir_16;
  lightDir_16 = xlv_TEXCOORD1;
  mediump vec3 viewDir_17;
  viewDir_17 = tmpvar_15;
  mediump float atten_18;
  atten_18 = tmpvar_9;
  mediump vec4 c_19;
  mediump vec3 specCol_20;
  highp float nh_21;
  mediump float tmpvar_22;
  tmpvar_22 = max (0.0, dot (tmpvar_3, normalize((lightDir_16 + viewDir_17))));
  nh_21 = tmpvar_22;
  mediump float arg1_23;
  arg1_23 = (32.0 * _Shininess);
  highp vec3 tmpvar_24;
  tmpvar_24 = (pow (nh_21, arg1_23) * tmpvar_7);
  specCol_20 = tmpvar_24;
  c_19.xyz = ((((tmpvar_2 * _LightColor0.xyz) * max (0.0, dot (tmpvar_3, lightDir_16))) + (_LightColor0.xyz * specCol_20)) * (atten_18 * 2.0));
  c_19.w = 0.0;
  c_1 = c_19;
  mediump vec3 tmpvar_25;
  tmpvar_25 = (c_1.xyz + (tmpvar_2 * xlv_TEXCOORD2));
  c_1.xyz = tmpvar_25;
  gl_FragData[0] = c_1;
}



#endif"
}

SubProgram "glesdesktop " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
"!!GLES
#define SHADER_API_GLES 1
#define tex2D texture2D


#ifdef VERTEX
#define gl_ModelViewProjectionMatrix glstate_matrix_mvp
uniform mat4 glstate_matrix_mvp;

varying highp vec4 xlv_TEXCOORD4;
varying highp vec3 xlv_TEXCOORD3;
varying lowp vec3 xlv_TEXCOORD2;
varying lowp vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 unity_Scale;
uniform highp mat4 _World2Object;
uniform highp mat4 _Object2World;

uniform highp vec4 unity_SHC;
uniform highp vec4 unity_SHBb;
uniform highp vec4 unity_SHBg;
uniform highp vec4 unity_SHBr;
uniform highp vec4 unity_SHAb;
uniform highp vec4 unity_SHAg;
uniform highp vec4 unity_SHAr;
uniform lowp vec4 _WorldSpaceLightPos0;
uniform highp vec4 _ProjectionParams;
uniform highp vec3 _WorldSpaceCameraPos;
attribute vec4 _glesTANGENT;
attribute vec4 _glesMultiTexCoord0;
attribute vec3 _glesNormal;
attribute vec4 _glesVertex;
void main ()
{
  vec4 tmpvar_1;
  tmpvar_1.xyz = normalize(_glesTANGENT.xyz);
  tmpvar_1.w = _glesTANGENT.w;
  vec3 tmpvar_2;
  tmpvar_2 = normalize(_glesNormal);
  highp vec3 shlight_3;
  lowp vec3 tmpvar_4;
  lowp vec3 tmpvar_5;
  highp vec4 tmpvar_6;
  tmpvar_6 = (gl_ModelViewProjectionMatrix * _glesVertex);
  mat3 tmpvar_7;
  tmpvar_7[0] = _Object2World[0].xyz;
  tmpvar_7[1] = _Object2World[1].xyz;
  tmpvar_7[2] = _Object2World[2].xyz;
  highp vec3 tmpvar_8;
  highp vec3 tmpvar_9;
  tmpvar_8 = tmpvar_1.xyz;
  tmpvar_9 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_10;
  tmpvar_10[0].x = tmpvar_8.x;
  tmpvar_10[0].y = tmpvar_9.x;
  tmpvar_10[0].z = tmpvar_2.x;
  tmpvar_10[1].x = tmpvar_8.y;
  tmpvar_10[1].y = tmpvar_9.y;
  tmpvar_10[1].z = tmpvar_2.y;
  tmpvar_10[2].x = tmpvar_8.z;
  tmpvar_10[2].y = tmpvar_9.z;
  tmpvar_10[2].z = tmpvar_2.z;
  highp vec3 tmpvar_11;
  tmpvar_11 = (tmpvar_10 * (_World2Object * _WorldSpaceLightPos0).xyz);
  tmpvar_4 = tmpvar_11;
  highp vec4 tmpvar_12;
  tmpvar_12.w = 1.0;
  tmpvar_12.xyz = _WorldSpaceCameraPos;
  highp vec4 tmpvar_13;
  tmpvar_13.w = 1.0;
  tmpvar_13.xyz = (tmpvar_7 * (tmpvar_2 * unity_Scale.w));
  mediump vec3 tmpvar_14;
  mediump vec4 normal_15;
  normal_15 = tmpvar_13;
  highp float vC_16;
  mediump vec3 x3_17;
  mediump vec3 x2_18;
  mediump vec3 x1_19;
  highp float tmpvar_20;
  tmpvar_20 = dot (unity_SHAr, normal_15);
  x1_19.x = tmpvar_20;
  highp float tmpvar_21;
  tmpvar_21 = dot (unity_SHAg, normal_15);
  x1_19.y = tmpvar_21;
  highp float tmpvar_22;
  tmpvar_22 = dot (unity_SHAb, normal_15);
  x1_19.z = tmpvar_22;
  mediump vec4 tmpvar_23;
  tmpvar_23 = (normal_15.xyzz * normal_15.yzzx);
  highp float tmpvar_24;
  tmpvar_24 = dot (unity_SHBr, tmpvar_23);
  x2_18.x = tmpvar_24;
  highp float tmpvar_25;
  tmpvar_25 = dot (unity_SHBg, tmpvar_23);
  x2_18.y = tmpvar_25;
  highp float tmpvar_26;
  tmpvar_26 = dot (unity_SHBb, tmpvar_23);
  x2_18.z = tmpvar_26;
  mediump float tmpvar_27;
  tmpvar_27 = ((normal_15.x * normal_15.x) - (normal_15.y * normal_15.y));
  vC_16 = tmpvar_27;
  highp vec3 tmpvar_28;
  tmpvar_28 = (unity_SHC.xyz * vC_16);
  x3_17 = tmpvar_28;
  tmpvar_14 = ((x1_19 + x2_18) + x3_17);
  shlight_3 = tmpvar_14;
  tmpvar_5 = shlight_3;
  highp vec4 o_29;
  highp vec4 tmpvar_30;
  tmpvar_30 = (tmpvar_6 * 0.5);
  highp vec2 tmpvar_31;
  tmpvar_31.x = tmpvar_30.x;
  tmpvar_31.y = (tmpvar_30.y * _ProjectionParams.x);
  o_29.xy = (tmpvar_31 + tmpvar_30.w);
  o_29.zw = tmpvar_6.zw;
  gl_Position = tmpvar_6;
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = tmpvar_4;
  xlv_TEXCOORD2 = tmpvar_5;
  xlv_TEXCOORD3 = (tmpvar_10 * (((_World2Object * tmpvar_12).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD4 = o_29;
}



#endif
#ifdef FRAGMENT

varying highp vec4 xlv_TEXCOORD4;
varying highp vec3 xlv_TEXCOORD3;
varying lowp vec3 xlv_TEXCOORD2;
varying lowp vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform mediump float _Gloss;
uniform mediump float _Shininess;
uniform lowp vec4 _Color;
uniform sampler2D _SpecMap;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform sampler2D _ShadowMapTexture;
uniform lowp vec4 _LightColor0;
void main ()
{
  lowp vec4 c_1;
  mediump vec3 tmpvar_2;
  mediump vec3 tmpvar_3;
  lowp vec4 tmpvar_4;
  tmpvar_4 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_5;
  tmpvar_5 = (tmpvar_4.xyz * _Color.xyz);
  tmpvar_2 = tmpvar_5;
  lowp vec4 tmpvar_6;
  tmpvar_6 = texture2D (_SpecMap, xlv_TEXCOORD0);
  mediump vec3 tmpvar_7;
  tmpvar_7 = ((tmpvar_4.w * tmpvar_6.xyz) * _Gloss);
  lowp vec3 normal_8;
  normal_8.xy = ((texture2D (_BumpMap, xlv_TEXCOORD0).wy * 2.0) - 1.0);
  normal_8.z = sqrt(((1.0 - (normal_8.x * normal_8.x)) - (normal_8.y * normal_8.y)));
  tmpvar_3 = normal_8;
  lowp float tmpvar_9;
  tmpvar_9 = texture2DProj (_ShadowMapTexture, xlv_TEXCOORD4).x;
  highp vec3 tmpvar_10;
  tmpvar_10 = normalize(xlv_TEXCOORD3);
  mediump vec3 lightDir_11;
  lightDir_11 = xlv_TEXCOORD1;
  mediump vec3 viewDir_12;
  viewDir_12 = tmpvar_10;
  mediump float atten_13;
  atten_13 = tmpvar_9;
  mediump vec4 c_14;
  mediump vec3 specCol_15;
  highp float nh_16;
  mediump float tmpvar_17;
  tmpvar_17 = max (0.0, dot (tmpvar_3, normalize((lightDir_11 + viewDir_12))));
  nh_16 = tmpvar_17;
  mediump float arg1_18;
  arg1_18 = (32.0 * _Shininess);
  highp vec3 tmpvar_19;
  tmpvar_19 = (pow (nh_16, arg1_18) * tmpvar_7);
  specCol_15 = tmpvar_19;
  c_14.xyz = ((((tmpvar_2 * _LightColor0.xyz) * max (0.0, dot (tmpvar_3, lightDir_11))) + (_LightColor0.xyz * specCol_15)) * (atten_13 * 2.0));
  c_14.w = 0.0;
  c_1 = c_14;
  mediump vec3 tmpvar_20;
  tmpvar_20 = (c_1.xyz + (tmpvar_2 * xlv_TEXCOORD2));
  c_1.xyz = tmpvar_20;
  gl_FragData[0] = c_1;
}



#endif"
}

SubProgram "flash " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Matrix 0 [glstate_matrix_mvp]
Vector 12 [_WorldSpaceCameraPos]
Vector 13 [_ProjectionParams]
Vector 14 [_WorldSpaceLightPos0]
Vector 15 [unity_SHAr]
Vector 16 [unity_SHAg]
Vector 17 [unity_SHAb]
Vector 18 [unity_SHBr]
Vector 19 [unity_SHBg]
Vector 20 [unity_SHBb]
Vector 21 [unity_SHC]
Matrix 4 [_Object2World]
Matrix 8 [_World2Object]
Vector 22 [unity_Scale]
Vector 23 [unity_NPOTScale]
Vector 24 [_MainTex_ST]
"agal_vs
c25 1.0 0.5 0.0 0.0
[bc]
adaaaaaaabaaahacabaaaaoeaaaaaaaabgaaaappabaaaaaa mul r1.xyz, a1, c22.w
bcaaaaaaacaaaiacabaaaakeacaaaaaaafaaaaoeabaaaaaa dp3 r2.w, r1.xyzz, c5
bcaaaaaaaaaaabacabaaaakeacaaaaaaaeaaaaoeabaaaaaa dp3 r0.x, r1.xyzz, c4
bcaaaaaaaaaaaeacabaaaakeacaaaaaaagaaaaoeabaaaaaa dp3 r0.z, r1.xyzz, c6
aaaaaaaaaaaaacacacaaaappacaaaaaaaaaaaaaaaaaaaaaa mov r0.y, r2.w
aaaaaaaaaaaaaiacbjaaaaaaabaaaaaaaaaaaaaaaaaaaaaa mov r0.w, c25.x
adaaaaaaabaaapacaaaaaakeacaaaaaaaaaaaacjacaaaaaa mul r1, r0.xyzz, r0.yzzx
bdaaaaaaacaaaeacaaaaaaoeacaaaaaabbaaaaoeabaaaaaa dp4 r2.z, r0, c17
bdaaaaaaacaaacacaaaaaaoeacaaaaaabaaaaaoeabaaaaaa dp4 r2.y, r0, c16
bdaaaaaaacaaabacaaaaaaoeacaaaaaaapaaaaoeabaaaaaa dp4 r2.x, r0, c15
adaaaaaaaaaaaiacacaaaappacaaaaaaacaaaappacaaaaaa mul r0.w, r2.w, r2.w
adaaaaaaadaaaiacaaaaaaaaacaaaaaaaaaaaaaaacaaaaaa mul r3.w, r0.x, r0.x
acaaaaaaaaaaaiacadaaaappacaaaaaaaaaaaappacaaaaaa sub r0.w, r3.w, r0.w
bdaaaaaaaaaaaeacabaaaaoeacaaaaaabeaaaaoeabaaaaaa dp4 r0.z, r1, c20
bdaaaaaaaaaaacacabaaaaoeacaaaaaabdaaaaoeabaaaaaa dp4 r0.y, r1, c19
bdaaaaaaaaaaabacabaaaaoeacaaaaaabcaaaaoeabaaaaaa dp4 r0.x, r1, c18
adaaaaaaabaaahacaaaaaappacaaaaaabfaaaaoeabaaaaaa mul r1.xyz, r0.w, c21
abaaaaaaaaaaahacacaaaakeacaaaaaaaaaaaakeacaaaaaa add r0.xyz, r2.xyzz, r0.xyzz
abaaaaaaacaaahaeaaaaaakeacaaaaaaabaaaakeacaaaaaa add v2.xyz, r0.xyzz, r1.xyzz
aaaaaaaaaaaaaiacbjaaaaaaabaaaaaaaaaaaaaaaaaaaaaa mov r0.w, c25.x
aaaaaaaaaaaaahacamaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, c12
bdaaaaaaabaaaeacaaaaaaoeacaaaaaaakaaaaoeabaaaaaa dp4 r1.z, r0, c10
bdaaaaaaabaaacacaaaaaaoeacaaaaaaajaaaaoeabaaaaaa dp4 r1.y, r0, c9
bdaaaaaaabaaabacaaaaaaoeacaaaaaaaiaaaaoeabaaaaaa dp4 r1.x, r0, c8
adaaaaaaaeaaahacabaaaakeacaaaaaabgaaaappabaaaaaa mul r4.xyz, r1.xyzz, c22.w
acaaaaaaadaaahacaeaaaakeacaaaaaaaaaaaaoeaaaaaaaa sub r3.xyz, r4.xyzz, a0
aaaaaaaaaaaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, a5
adaaaaaaabaaahacabaaaancaaaaaaaaaaaaaaajacaaaaaa mul r1.xyz, a1.zxyw, r0.yzxx
aaaaaaaaaaaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, a5
adaaaaaaafaaahacabaaaamjaaaaaaaaaaaaaafcacaaaaaa mul r5.xyz, a1.yzxw, r0.zxyy
acaaaaaaabaaahacafaaaakeacaaaaaaabaaaakeacaaaaaa sub r1.xyz, r5.xyzz, r1.xyzz
adaaaaaaacaaahacabaaaakeacaaaaaaafaaaappaaaaaaaa mul r2.xyz, r1.xyzz, a5.w
aaaaaaaaaaaaapacakaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0, c10
bdaaaaaaaeaaaeacaoaaaaoeabaaaaaaaaaaaaoeacaaaaaa dp4 r4.z, c14, r0
aaaaaaaaaaaaapacajaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0, c9
bdaaaaaaaeaaacacaoaaaaoeabaaaaaaaaaaaaoeacaaaaaa dp4 r4.y, c14, r0
aaaaaaaaabaaapacaiaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r1, c8
bdaaaaaaaeaaabacaoaaaaoeabaaaaaaabaaaaoeacaaaaaa dp4 r4.x, c14, r1
bdaaaaaaaaaaaiacaaaaaaoeaaaaaaaaadaaaaoeabaaaaaa dp4 r0.w, a0, c3
bdaaaaaaaaaaaeacaaaaaaoeaaaaaaaaacaaaaoeabaaaaaa dp4 r0.z, a0, c2
bdaaaaaaaaaaabacaaaaaaoeaaaaaaaaaaaaaaoeabaaaaaa dp4 r0.x, a0, c0
bdaaaaaaaaaaacacaaaaaaoeaaaaaaaaabaaaaoeabaaaaaa dp4 r0.y, a0, c1
adaaaaaaabaaahacaaaaaapeacaaaaaabjaaaaffabaaaaaa mul r1.xyz, r0.xyww, c25.y
adaaaaaaabaaacacabaaaaffacaaaaaaanaaaaaaabaaaaaa mul r1.y, r1.y, c13.x
abaaaaaaabaaadacabaaaafeacaaaaaaabaaaakkacaaaaaa add r1.xy, r1.xyyy, r1.z
bcaaaaaaabaaacaeaeaaaakeacaaaaaaacaaaakeacaaaaaa dp3 v1.y, r4.xyzz, r2.xyzz
bcaaaaaaadaaacaeacaaaakeacaaaaaaadaaaakeacaaaaaa dp3 v3.y, r2.xyzz, r3.xyzz
bcaaaaaaabaaaeaeabaaaaoeaaaaaaaaaeaaaakeacaaaaaa dp3 v1.z, a1, r4.xyzz
bcaaaaaaabaaabaeaeaaaakeacaaaaaaafaaaaoeaaaaaaaa dp3 v1.x, r4.xyzz, a5
bcaaaaaaadaaaeaeabaaaaoeaaaaaaaaadaaaakeacaaaaaa dp3 v3.z, a1, r3.xyzz
bcaaaaaaadaaabaeafaaaaoeaaaaaaaaadaaaakeacaaaaaa dp3 v3.x, a5, r3.xyzz
adaaaaaaaeaaadaeabaaaafeacaaaaaabhaaaaoeabaaaaaa mul v4.xy, r1.xyyy, c23
aaaaaaaaaaaaapadaaaaaaoeacaaaaaaaaaaaaaaaaaaaaaa mov o0, r0
aaaaaaaaaeaaamaeaaaaaaopacaaaaaaaaaaaaaaaaaaaaaa mov v4.zw, r0.wwzw
adaaaaaaafaaadacadaaaaoeaaaaaaaabiaaaaoeabaaaaaa mul r5.xy, a3, c24
abaaaaaaaaaaadaeafaaaafeacaaaaaabiaaaaooabaaaaaa add v0.xy, r5.xyyy, c24.zwzw
aaaaaaaaaaaaamaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v0.zw, c0
aaaaaaaaabaaaiaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v1.w, c0
aaaaaaaaacaaaiaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v2.w, c0
aaaaaaaaadaaaiaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v3.w, c0
"
}

SubProgram "opengl " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Bind "vertex" Vertex
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Vector 13 [_ProjectionParams]
Vector 15 [unity_LightmapST]
Vector 16 [_MainTex_ST]
"!!ARBvp1.0
# 11 ALU
PARAM c[17] = { { 0.5 },
		state.matrix.mvp,
		program.local[5..16] };
TEMP R0;
TEMP R1;
DP4 R0.w, vertex.position, c[4];
DP4 R0.z, vertex.position, c[3];
DP4 R0.x, vertex.position, c[1];
DP4 R0.y, vertex.position, c[2];
MUL R1.xyz, R0.xyww, c[0].x;
MUL R1.y, R1, c[13].x;
ADD result.texcoord[2].xy, R1, R1.z;
MOV result.position, R0;
MOV result.texcoord[2].zw, R0;
MAD result.texcoord[0].xy, vertex.texcoord[0], c[16], c[16].zwzw;
MAD result.texcoord[1].xy, vertex.texcoord[1], c[15], c[15].zwzw;
END
# 11 instructions, 2 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Bind "vertex" Vertex
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Matrix 0 [glstate_matrix_mvp]
Vector 12 [_ProjectionParams]
Vector 13 [_ScreenParams]
Vector 14 [unity_LightmapST]
Vector 15 [_MainTex_ST]
"vs_2_0
; 11 ALU
def c16, 0.50000000, 0, 0, 0
dcl_position0 v0
dcl_texcoord0 v3
dcl_texcoord1 v4
dp4 r0.w, v0, c3
dp4 r0.z, v0, c2
dp4 r0.x, v0, c0
dp4 r0.y, v0, c1
mul r1.xyz, r0.xyww, c16.x
mul r1.y, r1, c12.x
mad oT2.xy, r1.z, c13.zwzw, r1
mov oPos, r0
mov oT2.zw, r0
mad oT0.xy, v3, c15, c15.zwzw
mad oT1.xy, v4, c14, c14.zwzw
"
}

SubProgram "xbox360 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Bind "vertex" Vertex
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Vector 7 [_MainTex_ST]
Vector 0 [_ProjectionParams]
Vector 1 [_ScreenParams]
Matrix 2 [glstate_matrix_mvp] 4
Vector 6 [unity_LightmapST]
// Shader Timing Estimate, in Cycles/64 vertex vector:
// ALU: 14.67 (11 instructions), vertex: 32, texture: 0,
//   sequencer: 10,  3 GPRs, 31 threads,
// Performance (if enough threads): ~32 cycles per vector
// * Vertex cycle estimates are assuming 3 vfetch_minis for every vfetch_full,
//     with <= 32 bytes per vfetch_full group.

"vs_360
backbbabaaaaabkeaaaaabbiaaaaaaaaaaaaaaceaaaaabcmaaaaabfeaaaaaaaa
aaaaaaaaaaaaabaeaaaaaabmaaaaaapipppoadaaaaaaaaafaaaaaabmaaaaaaaa
aaaaaapbaaaaaaiaaaacaaahaaabaaaaaaaaaaimaaaaaaaaaaaaaajmaaacaaaa
aaabaaaaaaaaaaimaaaaaaaaaaaaaakoaaacaaabaaabaaaaaaaaaaimaaaaaaaa
aaaaaalmaaacaaacaaaeaaaaaaaaaanaaaaaaaaaaaaaaaoaaaacaaagaaabaaaa
aaaaaaimaaaaaaaafpengbgjgofegfhifpfdfeaaaaabaaadaaabaaaeaaabaaaa
aaaaaaaafpfahcgpgkgfgdhegjgpgofagbhcgbgnhdaafpfdgdhcgfgfgofagbhc
gbgnhdaaghgmhdhegbhegffpgngbhehcgjhifpgnhghaaaklaaadaaadaaaeaaae
aaabaaaaaaaaaaaahfgogjhehjfpemgjghgihegngbhafdfeaahghdfpddfpdaaa
dccodacodcdadddfddcodaaaaaaaaaaaaaaaaaabaaaaaaaaaaaaaaaaaaaaaabe
aapmaabaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaeaaaaaaaniaacbaaac
aaaaaaaaaaaaaaaaaaaacagdaaaaaaabaaaaaaadaaaaaaaeaaaaacjaaabaaaad
aaaafaaeaacbfaafaaaadafaaaabdbfbaaacpcfcaaaabaanaaaabaaoaaaaaaam
aaaababaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaadpaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaahabfdaadaaaabcaamcaaaaaaaaaafaagaaaabcaameaaaaaaaaaagaal
aaaaccaaaaaaaaaaafpicaaaaaaaagiiaaaaaaaaafpibaaaaaaaacdpaaaaaaaa
afpibaaaaaaaapmiaaaaaaaamiapaaaaaabliiaakbacafaamiapaaaaaamgnapi
klacaeaamiapaaaaaalbdepiklacadaamiapaaacaagmnajeklacacaamiapiado
aananaaaocacacaamiahaaaaaamagmaakbacppaamiamiaacaanlnlaaocacacaa
miadiaaaaabklabkilabahahmiadiaabaalalabkilabagagkiiaaaaaaaaaaaeb
mcaaaaaamiadiaacaamgbkbiklaaabaaaaaaaaaaaaaaaaaaaaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Matrix 256 [glstate_matrix_mvp]
Bind "vertex" Vertex
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Vector 467 [_ProjectionParams]
Vector 466 [unity_LightmapST]
Vector 465 [_MainTex_ST]
"sce_vp_rsx // 11 instructions using 1 registers
[Configuration]
8
0000000b03010100
[Defaults]
1
464 1
3f000000
[Microcode]
176
401f9c6c011d1808010400d740619f9c00001c6c01d0300d8106c0c360403ffc
00001c6c01d0200d8106c0c360405ffc00001c6c01d0100d8106c0c360409ffc
00001c6c01d0000d8106c0c360411ffc401f9c6c011d2908010400d740619fa0
401f9c6c0040000d8086c0836041ff80401f9c6c004000558086c08360407fa4
00001c6c009d000e008000c36041dffc00001c6c009d302a808000c360409ffc
401f9c6c00c000080086c09540219fa5
"
}

SubProgram "d3d11 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Bind "vertex" Vertex
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Bind "color" Color
ConstBuffer "$Globals" 176 // 176 used size, 9 vars
Vector 144 [unity_LightmapST] 4
Vector 160 [_MainTex_ST] 4
ConstBuffer "UnityPerCamera" 128 // 96 used size, 8 vars
Vector 80 [_ProjectionParams] 4
ConstBuffer "UnityPerDraw" 336 // 64 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityPerDraw" 2
// 12 instructions, 2 temp regs, 0 temp arrays:
// ALU 4 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0
eefiecedolhhgmhmflchhflaclfjpdbpaageddcgabaaaaaajmadaaaaadaaaaaa
cmaaaaaapeaaaaaahmabaaaaejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapaaaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahaaaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaa
abaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapadaaaaljaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfc
enebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheoiaaaaaaaaeaaaaaa
aiaaaaaagiaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaheaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaabaaaaaaadamaaaaheaaaaaaabaaaaaaaaaaaaaa
adaaaaaaabaaaaaaamadaaaaheaaaaaaacaaaaaaaaaaaaaaadaaaaaaacaaaaaa
apaaaaaafdfgfpfaepfdejfeejepeoaafeeffiedepepfceeaaklklklfdeieefc
biacaaaaeaaaabaaigaaaaaafjaaaaaeegiocaaaaaaaaaaaalaaaaaafjaaaaae
egiocaaaabaaaaaaagaaaaaafjaaaaaeegiocaaaacaaaaaaaeaaaaaafpaaaaad
pcbabaaaaaaaaaaafpaaaaaddcbabaaaadaaaaaafpaaaaaddcbabaaaaeaaaaaa
ghaaaaaepccabaaaaaaaaaaaabaaaaaagfaaaaaddccabaaaabaaaaaagfaaaaad
mccabaaaabaaaaaagfaaaaadpccabaaaacaaaaaagiaaaaacacaaaaaadiaaaaai
pcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaacaaaaaaabaaaaaadcaaaaak
pcaabaaaaaaaaaaaegiocaaaacaaaaaaaaaaaaaaagbabaaaaaaaaaaaegaobaaa
aaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaacaaaaaaacaaaaaakgbkbaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaacaaaaaa
adaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadgaaaaafpccabaaaaaaaaaaa
egaobaaaaaaaaaaadcaaaaaldccabaaaabaaaaaaegbabaaaadaaaaaaegiacaaa
aaaaaaaaakaaaaaaogikcaaaaaaaaaaaakaaaaaadcaaaaalmccabaaaabaaaaaa
agbebaaaaeaaaaaaagiecaaaaaaaaaaaajaaaaaakgiocaaaaaaaaaaaajaaaaaa
diaaaaaiccaabaaaaaaaaaaabkaabaaaaaaaaaaaakiacaaaabaaaaaaafaaaaaa
diaaaaakncaabaaaabaaaaaaagahbaaaaaaaaaaaaceaaaaaaaaaaadpaaaaaaaa
aaaaaadpaaaaaadpdgaaaaafmccabaaaacaaaaaakgaobaaaaaaaaaaaaaaaaaah
dccabaaaacaaaaaakgakbaaaabaaaaaamgaabaaaabaaaaaadoaaaaab"
}

SubProgram "gles " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
"!!GLES
#define SHADER_API_GLES 1
#define tex2D texture2D


#ifdef VERTEX
#define gl_ModelViewProjectionMatrix glstate_matrix_mvp
uniform mat4 glstate_matrix_mvp;

varying highp vec4 xlv_TEXCOORD2;
varying highp vec2 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 unity_LightmapST;
uniform highp mat4 _Object2World;

uniform highp mat4 unity_World2Shadow[4];
attribute vec4 _glesMultiTexCoord1;
attribute vec4 _glesMultiTexCoord0;
attribute vec4 _glesVertex;
void main ()
{
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = ((_glesMultiTexCoord1.xy * unity_LightmapST.xy) + unity_LightmapST.zw);
  xlv_TEXCOORD2 = (unity_World2Shadow[0] * (_Object2World * _glesVertex));
}



#endif
#ifdef FRAGMENT

varying highp vec4 xlv_TEXCOORD2;
varying highp vec2 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform sampler2D unity_Lightmap;
uniform lowp vec4 _Color;
uniform sampler2D _MainTex;
uniform sampler2D _ShadowMapTexture;
uniform highp vec4 _LightShadowData;
void main ()
{
  lowp vec4 c_1;
  mediump vec3 tmpvar_2;
  lowp vec3 tmpvar_3;
  tmpvar_3 = (texture2D (_MainTex, xlv_TEXCOORD0).xyz * _Color.xyz);
  tmpvar_2 = tmpvar_3;
  lowp float tmpvar_4;
  mediump float lightShadowDataX_5;
  highp float dist_6;
  lowp float tmpvar_7;
  tmpvar_7 = texture2DProj (_ShadowMapTexture, xlv_TEXCOORD2).x;
  dist_6 = tmpvar_7;
  highp float tmpvar_8;
  tmpvar_8 = _LightShadowData.x;
  lightShadowDataX_5 = tmpvar_8;
  highp float tmpvar_9;
  tmpvar_9 = max (float((dist_6 > (xlv_TEXCOORD2.z / xlv_TEXCOORD2.w))), lightShadowDataX_5);
  tmpvar_4 = tmpvar_9;
  lowp vec3 tmpvar_10;
  tmpvar_10 = min ((2.0 * texture2D (unity_Lightmap, xlv_TEXCOORD1).xyz), vec3((tmpvar_4 * 2.0)));
  mediump vec3 tmpvar_11;
  tmpvar_11 = (tmpvar_2 * tmpvar_10);
  c_1.xyz = tmpvar_11;
  c_1.w = 0.0;
  gl_FragData[0] = c_1;
}



#endif"
}

SubProgram "glesdesktop " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
"!!GLES
#define SHADER_API_GLES 1
#define tex2D texture2D


#ifdef VERTEX
#define gl_ModelViewProjectionMatrix glstate_matrix_mvp
uniform mat4 glstate_matrix_mvp;

varying highp vec4 xlv_TEXCOORD2;
varying highp vec2 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 unity_LightmapST;

uniform highp vec4 _ProjectionParams;
attribute vec4 _glesMultiTexCoord1;
attribute vec4 _glesMultiTexCoord0;
attribute vec4 _glesVertex;
void main ()
{
  highp vec4 tmpvar_1;
  tmpvar_1 = (gl_ModelViewProjectionMatrix * _glesVertex);
  highp vec4 o_2;
  highp vec4 tmpvar_3;
  tmpvar_3 = (tmpvar_1 * 0.5);
  highp vec2 tmpvar_4;
  tmpvar_4.x = tmpvar_3.x;
  tmpvar_4.y = (tmpvar_3.y * _ProjectionParams.x);
  o_2.xy = (tmpvar_4 + tmpvar_3.w);
  o_2.zw = tmpvar_1.zw;
  gl_Position = tmpvar_1;
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = ((_glesMultiTexCoord1.xy * unity_LightmapST.xy) + unity_LightmapST.zw);
  xlv_TEXCOORD2 = o_2;
}



#endif
#ifdef FRAGMENT

varying highp vec4 xlv_TEXCOORD2;
varying highp vec2 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform sampler2D unity_Lightmap;
uniform lowp vec4 _Color;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform sampler2D _ShadowMapTexture;
void main ()
{
  lowp vec4 c_1;
  mediump vec3 tmpvar_2;
  lowp vec3 tmpvar_3;
  tmpvar_3 = (texture2D (_MainTex, xlv_TEXCOORD0).xyz * _Color.xyz);
  tmpvar_2 = tmpvar_3;
  lowp vec3 normal_4;
  normal_4.xy = ((texture2D (_BumpMap, xlv_TEXCOORD0).wy * 2.0) - 1.0);
  normal_4.z = sqrt(((1.0 - (normal_4.x * normal_4.x)) - (normal_4.y * normal_4.y)));
  lowp vec4 tmpvar_5;
  tmpvar_5 = texture2DProj (_ShadowMapTexture, xlv_TEXCOORD2);
  lowp vec4 tmpvar_6;
  tmpvar_6 = texture2D (unity_Lightmap, xlv_TEXCOORD1);
  lowp vec3 tmpvar_7;
  tmpvar_7 = ((8.0 * tmpvar_6.w) * tmpvar_6.xyz);
  lowp vec3 tmpvar_8;
  tmpvar_8 = max (min (tmpvar_7, ((tmpvar_5.x * 2.0) * tmpvar_6.xyz)), (tmpvar_7 * tmpvar_5.x));
  mediump vec3 tmpvar_9;
  tmpvar_9 = (tmpvar_2 * tmpvar_8);
  c_1.xyz = tmpvar_9;
  c_1.w = 0.0;
  gl_FragData[0] = c_1;
}



#endif"
}

SubProgram "flash " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Bind "vertex" Vertex
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Matrix 0 [glstate_matrix_mvp]
Vector 12 [_ProjectionParams]
Vector 13 [unity_NPOTScale]
Vector 14 [unity_LightmapST]
Vector 15 [_MainTex_ST]
"agal_vs
c16 0.5 0.0 0.0 0.0
[bc]
bdaaaaaaaaaaaiacaaaaaaoeaaaaaaaaadaaaaoeabaaaaaa dp4 r0.w, a0, c3
bdaaaaaaaaaaaeacaaaaaaoeaaaaaaaaacaaaaoeabaaaaaa dp4 r0.z, a0, c2
bdaaaaaaaaaaabacaaaaaaoeaaaaaaaaaaaaaaoeabaaaaaa dp4 r0.x, a0, c0
bdaaaaaaaaaaacacaaaaaaoeaaaaaaaaabaaaaoeabaaaaaa dp4 r0.y, a0, c1
adaaaaaaabaaahacaaaaaapeacaaaaaabaaaaaaaabaaaaaa mul r1.xyz, r0.xyww, c16.x
adaaaaaaabaaacacabaaaaffacaaaaaaamaaaaaaabaaaaaa mul r1.y, r1.y, c12.x
abaaaaaaabaaadacabaaaafeacaaaaaaabaaaakkacaaaaaa add r1.xy, r1.xyyy, r1.z
adaaaaaaacaaadaeabaaaafeacaaaaaaanaaaaoeabaaaaaa mul v2.xy, r1.xyyy, c13
aaaaaaaaaaaaapadaaaaaaoeacaaaaaaaaaaaaaaaaaaaaaa mov o0, r0
aaaaaaaaacaaamaeaaaaaaopacaaaaaaaaaaaaaaaaaaaaaa mov v2.zw, r0.wwzw
adaaaaaaaaaaadacadaaaaoeaaaaaaaaapaaaaoeabaaaaaa mul r0.xy, a3, c15
abaaaaaaaaaaadaeaaaaaafeacaaaaaaapaaaaooabaaaaaa add v0.xy, r0.xyyy, c15.zwzw
adaaaaaaaaaaadacaeaaaaoeaaaaaaaaaoaaaaoeabaaaaaa mul r0.xy, a4, c14
abaaaaaaabaaadaeaaaaaafeacaaaaaaaoaaaaooabaaaaaa add v1.xy, r0.xyyy, c14.zwzw
aaaaaaaaaaaaamaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v0.zw, c0
aaaaaaaaabaaamaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v1.zw, c0
"
}

SubProgram "opengl " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" "VERTEXLIGHT_ON" }
Bind "vertex" Vertex
Bind "tangent" ATTR14
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 13 [_WorldSpaceCameraPos]
Vector 14 [_WorldSpaceLightPos0]
Vector 15 [unity_4LightPosX0]
Vector 16 [unity_4LightPosY0]
Vector 17 [unity_4LightPosZ0]
Vector 18 [unity_4LightAtten0]
Vector 19 [unity_LightColor0]
Vector 20 [unity_LightColor1]
Vector 21 [unity_LightColor2]
Vector 22 [unity_LightColor3]
Vector 23 [unity_SHAr]
Vector 24 [unity_SHAg]
Vector 25 [unity_SHAb]
Vector 26 [unity_SHBr]
Vector 27 [unity_SHBg]
Vector 28 [unity_SHBb]
Vector 29 [unity_SHC]
Matrix 5 [_Object2World]
Matrix 9 [_World2Object]
Vector 30 [unity_Scale]
Vector 31 [_MainTex_ST]
"!!ARBvp1.0
# 74 ALU
PARAM c[32] = { { 1, 0 },
		state.matrix.mvp,
		program.local[5..31] };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
TEMP R4;
MUL R3.xyz, vertex.normal, c[30].w;
DP4 R0.x, vertex.position, c[6];
ADD R1, -R0.x, c[16];
DP3 R3.w, R3, c[6];
DP3 R4.x, R3, c[5];
DP3 R3.x, R3, c[7];
MUL R2, R3.w, R1;
DP4 R0.x, vertex.position, c[5];
ADD R0, -R0.x, c[15];
MUL R1, R1, R1;
MOV R4.z, R3.x;
MAD R2, R4.x, R0, R2;
MOV R4.w, c[0].x;
DP4 R4.y, vertex.position, c[7];
MAD R1, R0, R0, R1;
ADD R0, -R4.y, c[17];
MAD R1, R0, R0, R1;
MAD R0, R3.x, R0, R2;
MUL R2, R1, c[18];
MOV R4.y, R3.w;
RSQ R1.x, R1.x;
RSQ R1.y, R1.y;
RSQ R1.w, R1.w;
RSQ R1.z, R1.z;
MUL R0, R0, R1;
ADD R1, R2, c[0].x;
RCP R1.x, R1.x;
RCP R1.y, R1.y;
RCP R1.w, R1.w;
RCP R1.z, R1.z;
MAX R0, R0, c[0].y;
MUL R0, R0, R1;
MUL R1.xyz, R0.y, c[20];
MAD R1.xyz, R0.x, c[19], R1;
MAD R0.xyz, R0.z, c[21], R1;
MAD R1.xyz, R0.w, c[22], R0;
MUL R0, R4.xyzz, R4.yzzx;
MUL R1.w, R3, R3;
DP4 R3.z, R0, c[28];
DP4 R3.y, R0, c[27];
DP4 R3.x, R0, c[26];
MAD R1.w, R4.x, R4.x, -R1;
MUL R0.xyz, R1.w, c[29];
MOV R1.w, c[0].x;
DP4 R2.z, R4, c[25];
DP4 R2.y, R4, c[24];
DP4 R2.x, R4, c[23];
ADD R2.xyz, R2, R3;
ADD R0.xyz, R2, R0;
ADD result.texcoord[2].xyz, R0, R1;
MOV R1.xyz, c[13];
DP4 R2.z, R1, c[11];
DP4 R2.y, R1, c[10];
DP4 R2.x, R1, c[9];
MAD R2.xyz, R2, c[30].w, -vertex.position;
MOV R0.xyz, vertex.attrib[14];
MUL R1.xyz, vertex.normal.zxyw, R0.yzxw;
MAD R0.xyz, vertex.normal.yzxw, R0.zxyw, -R1;
MOV R1, c[14];
MUL R0.xyz, R0, vertex.attrib[14].w;
DP4 R3.z, R1, c[11];
DP4 R3.y, R1, c[10];
DP4 R3.x, R1, c[9];
DP3 result.texcoord[1].y, R3, R0;
DP3 result.texcoord[3].y, R0, R2;
DP3 result.texcoord[1].z, vertex.normal, R3;
DP3 result.texcoord[1].x, R3, vertex.attrib[14];
DP3 result.texcoord[3].z, vertex.normal, R2;
DP3 result.texcoord[3].x, vertex.attrib[14], R2;
MAD result.texcoord[0].xy, vertex.texcoord[0], c[31], c[31].zwzw;
DP4 result.position.w, vertex.position, c[4];
DP4 result.position.z, vertex.position, c[3];
DP4 result.position.y, vertex.position, c[2];
DP4 result.position.x, vertex.position, c[1];
END
# 74 instructions, 5 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" "VERTEXLIGHT_ON" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Matrix 0 [glstate_matrix_mvp]
Vector 12 [_WorldSpaceCameraPos]
Vector 13 [_WorldSpaceLightPos0]
Vector 14 [unity_4LightPosX0]
Vector 15 [unity_4LightPosY0]
Vector 16 [unity_4LightPosZ0]
Vector 17 [unity_4LightAtten0]
Vector 18 [unity_LightColor0]
Vector 19 [unity_LightColor1]
Vector 20 [unity_LightColor2]
Vector 21 [unity_LightColor3]
Vector 22 [unity_SHAr]
Vector 23 [unity_SHAg]
Vector 24 [unity_SHAb]
Vector 25 [unity_SHBr]
Vector 26 [unity_SHBg]
Vector 27 [unity_SHBb]
Vector 28 [unity_SHC]
Matrix 4 [_Object2World]
Matrix 8 [_World2Object]
Vector 29 [unity_Scale]
Vector 30 [_MainTex_ST]
"vs_2_0
; 77 ALU
def c31, 1.00000000, 0.00000000, 0, 0
dcl_position0 v0
dcl_tangent0 v1
dcl_normal0 v2
dcl_texcoord0 v3
mul r3.xyz, v2, c29.w
dp4 r0.x, v0, c5
add r1, -r0.x, c15
dp3 r3.w, r3, c5
dp3 r4.x, r3, c4
dp3 r3.x, r3, c6
mul r2, r3.w, r1
dp4 r0.x, v0, c4
add r0, -r0.x, c14
mul r1, r1, r1
mov r4.z, r3.x
mad r2, r4.x, r0, r2
mov r4.w, c31.x
dp4 r4.y, v0, c6
mad r1, r0, r0, r1
add r0, -r4.y, c16
mad r1, r0, r0, r1
mad r0, r3.x, r0, r2
mul r2, r1, c17
mov r4.y, r3.w
rsq r1.x, r1.x
rsq r1.y, r1.y
rsq r1.w, r1.w
rsq r1.z, r1.z
mul r0, r0, r1
add r1, r2, c31.x
dp4 r2.z, r4, c24
dp4 r2.y, r4, c23
dp4 r2.x, r4, c22
rcp r1.x, r1.x
rcp r1.y, r1.y
rcp r1.w, r1.w
rcp r1.z, r1.z
max r0, r0, c31.y
mul r0, r0, r1
mul r1.xyz, r0.y, c19
mad r1.xyz, r0.x, c18, r1
mad r0.xyz, r0.z, c20, r1
mad r1.xyz, r0.w, c21, r0
mul r0, r4.xyzz, r4.yzzx
mul r1.w, r3, r3
dp4 r3.z, r0, c27
dp4 r3.y, r0, c26
dp4 r3.x, r0, c25
mad r1.w, r4.x, r4.x, -r1
mul r0.xyz, r1.w, c28
add r2.xyz, r2, r3
add r0.xyz, r2, r0
add oT2.xyz, r0, r1
mov r1.w, c31.x
mov r1.xyz, c12
dp4 r0.z, r1, c10
dp4 r0.y, r1, c9
dp4 r0.x, r1, c8
mad r3.xyz, r0, c29.w, -v0
mov r1.xyz, v1
mov r0.xyz, v1
mul r1.xyz, v2.zxyw, r1.yzxw
mad r1.xyz, v2.yzxw, r0.zxyw, -r1
mul r2.xyz, r1, v1.w
mov r0, c10
dp4 r4.z, c13, r0
mov r1, c9
mov r0, c8
dp4 r4.y, c13, r1
dp4 r4.x, c13, r0
dp3 oT1.y, r4, r2
dp3 oT3.y, r2, r3
dp3 oT1.z, v2, r4
dp3 oT1.x, r4, v1
dp3 oT3.z, v2, r3
dp3 oT3.x, v1, r3
mad oT0.xy, v3, c30, c30.zwzw
dp4 oPos.w, v0, c3
dp4 oPos.z, v0, c2
dp4 oPos.y, v0, c1
dp4 oPos.x, v0, c0
"
}

SubProgram "xbox360 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" "VERTEXLIGHT_ON" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 30 [_MainTex_ST]
Matrix 21 [_Object2World] 4
Matrix 25 [_World2Object] 4
Vector 0 [_WorldSpaceCameraPos]
Vector 1 [_WorldSpaceLightPos0]
Matrix 17 [glstate_matrix_mvp] 4
Vector 5 [unity_4LightAtten0]
Vector 2 [unity_4LightPosX0]
Vector 3 [unity_4LightPosY0]
Vector 4 [unity_4LightPosZ0]
Vector 6 [unity_LightColor0]
Vector 7 [unity_LightColor1]
Vector 8 [unity_LightColor2]
Vector 9 [unity_LightColor3]
Vector 12 [unity_SHAb]
Vector 11 [unity_SHAg]
Vector 10 [unity_SHAr]
Vector 15 [unity_SHBb]
Vector 14 [unity_SHBg]
Vector 13 [unity_SHBr]
Vector 16 [unity_SHC]
Vector 29 [unity_Scale]
// Shader Timing Estimate, in Cycles/64 vertex vector:
// ALU: 89.33 (67 instructions), vertex: 32, texture: 0,
//   sequencer: 30,  10 GPRs, 18 threads,
// Performance (if enough threads): ~89 cycles per vector
// * Vertex cycle estimates are assuming 3 vfetch_minis for every vfetch_full,
//     with <= 32 bytes per vfetch_full group.

"vs_360
backbbabaaaaadmaaaaaaeaaaaaaaaaaaaaaaaceaaaaaddaaaaaadfiaaaaaaaa
aaaaaaaaaaaaadaiaaaaaabmaaaaacplpppoadaaaaaaaabdaaaaaabmaaaaaaaa
aaaaacpeaaaaabjiaaacaaboaaabaaaaaaaaabkeaaaaaaaaaaaaableaaacaabf
aaaeaaaaaaaaabmeaaaaaaaaaaaaabneaaacaabjaaaeaaaaaaaaabmeaaaaaaaa
aaaaabocaaacaaaaaaabaaaaaaaaabpiaaaaaaaaaaaaacaiaaacaaabaaabaaaa
aaaaabkeaaaaaaaaaaaaacbnaaacaabbaaaeaaaaaaaaabmeaaaaaaaaaaaaacda
aaacaaafaaabaaaaaaaaabkeaaaaaaaaaaaaacedaaacaaacaaabaaaaaaaaabke
aaaaaaaaaaaaacffaaacaaadaaabaaaaaaaaabkeaaaaaaaaaaaaacghaaacaaae
aaabaaaaaaaaabkeaaaaaaaaaaaaachjaaacaaagaaaeaaaaaaaaacimaaaaaaaa
aaaaacjmaaacaaamaaabaaaaaaaaabkeaaaaaaaaaaaaackhaaacaaalaaabaaaa
aaaaabkeaaaaaaaaaaaaaclcaaacaaakaaabaaaaaaaaabkeaaaaaaaaaaaaacln
aaacaaapaaabaaaaaaaaabkeaaaaaaaaaaaaacmiaaacaaaoaaabaaaaaaaaabke
aaaaaaaaaaaaacndaaacaaanaaabaaaaaaaaabkeaaaaaaaaaaaaacnoaaacaaba
aaabaaaaaaaaabkeaaaaaaaaaaaaacoiaaacaabnaaabaaaaaaaaabkeaaaaaaaa
fpengbgjgofegfhifpfdfeaaaaabaaadaaabaaaeaaabaaaaaaaaaaaafpepgcgk
gfgdhedcfhgphcgmgeaaklklaaadaaadaaaeaaaeaaabaaaaaaaaaaaafpfhgphc
gmgedcepgcgkgfgdheaafpfhgphcgmgefdhagbgdgfedgbgngfhcgbfagphdaakl
aaabaaadaaabaaadaaabaaaaaaaaaaaafpfhgphcgmgefdhagbgdgfemgjghgihe
fagphddaaaghgmhdhegbhegffpgngbhehcgjhifpgnhghaaahfgogjhehjfpdeem
gjghgiheebhehegfgodaaahfgogjhehjfpdeemgjghgihefagphdfidaaahfgogj
hehjfpdeemgjghgihefagphdfjdaaahfgogjhehjfpdeemgjghgihefagphdfkda
aahfgogjhehjfpemgjghgiheedgpgmgphcaaklklaaabaaadaaabaaaeaaaeaaaa
aaaaaaaahfgogjhehjfpfdeiebgcaahfgogjhehjfpfdeiebghaahfgogjhehjfp
fdeiebhcaahfgogjhehjfpfdeiecgcaahfgogjhehjfpfdeiecghaahfgogjhehj
fpfdeiechcaahfgogjhehjfpfdeiedaahfgogjhehjfpfdgdgbgmgfaahghdfpdd
fpdaaadccodacodcdadddfddcodaaaklaaaaaaaaaaaaaaabaaaaaaaaaaaaaaaa
aaaaaabeaapmaabaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaeaaaaaadma
aadbaaajaaaaaaaaaaaaaaaaaaaacmieaaaaaaabaaaaaaaeaaaaaaaiaaaaacja
aabaaaaiaaaagaajaaaadaakaadafaalaaaadafaaaabhbfbaaaehcfcaaafhdfd
aaaabacjaaaaaacdaaaaaaceaaaabacfaaaabaeoaaaaaacgaaaaaachaaaabaci
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaadpiaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
paffeaaiaaaabcaamcaaaaaaaaaaeaamaaaabcaameaaaaaaaaaagabagabgbcaa
bcaaaaaaaaaagabmgaccbcaabcaaaaaaaaaagacigacobcaabcaaaaaaaaaagade
gadkbcaabcaaaaaaaaaagaeagaegbcaabcaaaaaaaaaadaemaaaaccaaaaaaaaaa
afpiiaaaaaaaaanbaaaaaaaaafpifaaaaaaaagiiaaaaaaaaafpicaaaaaaaaoii
aaaaaaaaafpiaaaaaaaaapmiaaaaaaaamiapaaabaamgiiaakbaibeaamiapaaab
aalbiiaaklaibdabmiapaaabaagmdejeklaibcabmiapiadoaablaadeklaibbab
miahaaabaaleblaacbbmabaamiahaaadaamamgmaalblaabmmiahaaadaalelble
clbkaaadmiahaaaeaalogfaaobacafaamiahaaahaamamgleclblababmiahaaab
aagfblaakbacbnaamiahaaagaamgleaakbaibiaamiahaaajaalbmaleklaibhag
miahaaagaalbleaakbabbhaamiahaaahaalelbleclbkabahmiahaaaeabgfloma
olacafaemiahaaadaamagmleclbjaaadmiahaaadabmabllpkladbnaimiahaaae
aamablaaobaeafaamiahaaahaamagmleclbjabahmiahaaagaagmlemaklabbgag
miahaaaiaagmleleklaibgajmialaaabaabllemaklaibfaimiahaaagaamglema
klabbfagmiabiaabaaloloaapaahafaamiaciaabaaloloaapaaeahaamiaeiaab
aaloloaapaahacaamiabiaadaaloloaapaadafaamiaciaadaaloloaapaaeadaa
miaeiaadaaloloaapaadacaamiadiaaaaalalabkilaaboboceipagaaaalehcgm
obagagiaaibpadafaegmaagmkaabacagaicpadacaelbaamgkaabaeagbeabaaae
abdoanblgpakagabaebcahaeaadoangmepalagadbeaeaaaeabdoanblgpamagab
aecbahabaakhkhlbipaaanadbeacaaababkhkhblkpaaaoabaeeeahabaakhkhmg
ipaaapadbeapaaaaabpipiblobacacabaeipahacaapilbblmbacagadmiapaaaa
aajejepiolahahaamiapaaacaajemgpiolahagacmiapaaacaajegmaaolafagac
miapaaaaaaaaaapiolafafaageihababaalologboaaeabadmiahaaabaabllemn
klabbaabmiapaaaeaapipigmilaaafppfibaaaaaaaaaaagmocaaaaiaficaaaaa
aaaaaalbocaaaaiafieaaaaaaaaaaamgocaaaaiafiiaaaaaaaaaaablocaaaaia
miapaaaaaapiaaaaobacaaaaemipaaadaapilbmgkcaappaeemecacaaaamgblgm
obadaaaeemciacacaagmmgblobadacaeembbaaacaabllblbobadacaemiaeaaaa
aalbgmaaobadaaaakibhacaeaalmmaecibacaiajkiciacaeaamgblicmbaeadaj
kieoacafaabgpmmaibacagajbeahaaaaaabbmalbkbaaahafambiafaaaamgmggm
obaaadadbeahaaaaaabebamgoaafaaacamihacaaaamabalboaaaaeadmiahaaaa
aamabaaaoaaaacaamiahiaacaalemaaaoaabaaaaaaaaaaaaaaaaaaaaaaaaaaaa
"
}

SubProgram "ps3 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" "VERTEXLIGHT_ON" }
Matrix 256 [glstate_matrix_mvp]
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 467 [_WorldSpaceCameraPos]
Vector 466 [_WorldSpaceLightPos0]
Vector 465 [unity_4LightPosX0]
Vector 464 [unity_4LightPosY0]
Vector 463 [unity_4LightPosZ0]
Vector 462 [unity_4LightAtten0]
Vector 461 [unity_LightColor0]
Vector 460 [unity_LightColor1]
Vector 459 [unity_LightColor2]
Vector 458 [unity_LightColor3]
Vector 457 [unity_SHAr]
Vector 456 [unity_SHAg]
Vector 455 [unity_SHAb]
Vector 454 [unity_SHBr]
Vector 453 [unity_SHBg]
Vector 452 [unity_SHBb]
Vector 451 [unity_SHC]
Matrix 260 [_Object2World]
Matrix 264 [_World2Object]
Vector 450 [unity_Scale]
Vector 449 [_MainTex_ST]
"sce_vp_rsx // 63 instructions using 9 registers
[Configuration]
8
0000003f41050900
[Defaults]
1
448 2
000000003f800000
[Microcode]
1008
00011c6c005d200d8186c0836041fffc00031c6c00400e0c0106c0836041dffc
00001c6c005d300c0186c0836041dffc00009c6c009c220c013fc0c36041dffc
401f9c6c011c1808010400d740619f9c401f9c6c01d0300d8106c0c360403f80
401f9c6c01d0200d8106c0c360405f80401f9c6c01d0100d8106c0c360409f80
401f9c6c01d0000d8106c0c360411f8000019c6c01d0500d8106c0c360411ffc
00009c6c01d0400d8106c0c360403ffc00001c6c01d0600d8106c0c360403ffc
00029c6c01d0a00d8486c0c360405ffc00029c6c01d0900d8486c0c360409ffc
00029c6c01d0800d8486c0c360411ffc00021c6c0150400c028600c360411ffc
00021c6c0150600c028600c360403ffc00021c6c0150500c028600c360409ffc
00011c6c0190a00c0086c0c360405ffc00011c6c0190900c0086c0c360409ffc
00011c6c0190800c0086c0c360411ffc00001c6c00dcf00d8186c0bfe021fffc
00009c6c00dd100d8186c0bfe0a1fffc00019c6c00dd000d8186c0a001a1fffc
00039c6c00800243011846436041dffc00031c6c010002308121866303a1dffc
00039c6c011c200c04bfc0e30041dffc401f9c6c0140020c0106054360405fa0
00011c6c0080002a8886c3436041fffc00019c6c0080000d8686c3436041fffc
00029c6c0080002a8895444360403ffc00021c6c0040007f8886c08360405ffc
00011c6c010000000886c1436121fffc00009c6c0100000d8286c14361a1fffc
00041c6c019c700c0886c0c360405ffc00041c6c019c800c0886c0c360409ffc
00041c6c019c900c0886c0c360411ffc00029c6c010000000880047fe2a03ffc
00019c6c0080000d089a04436041fffc00011c6c0100007f8886c0436121fffc
00001c6c0100000d8086c04360a1fffc00009c6c01dc400d8686c0c360405ffc
00009c6c01dc500d8686c0c360409ffc00009c6c01dc600d8686c0c360411ffc
00009c6c00c0000c1086c08300a1dffc00019c6c009c307f8a8600c36041dffc
00019c6c00c0000c0686c08300a1dffc401f9c6c21400e0c0a860080003100a0
00031c6c20800e0c0cbfc08aa029c0fc00021c6c209ce00d8086c0d54025e0fc
00021c6c00dc002a8186c0836221fffc401f9c6c2140020c0106075fe02240a8
401f9c6c11400e0c0106074002310028401f9c6c1140000c0a86064aa2288020
401f9c6c1140000c0c8607554224802800009c6c1080000d8486c15fe223e07c
00009c6c029c000d828000c36041fffc00001c6c0080000d8286c0436041fffc
00009c6c009cc02a808600c36041dffc00009c6c011cd000008600c300a1dffc
00001c6c011cb055008600c300a1dffc00001c6c011ca07f808600c30021dffc
401f9c6c00c0000c0686c0830021dfa5
"
}

SubProgram "d3d11 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" "VERTEXLIGHT_ON" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 96 // 96 used size, 7 vars
Vector 80 [_MainTex_ST] 4
ConstBuffer "UnityPerCamera" 128 // 76 used size, 8 vars
Vector 64 [_WorldSpaceCameraPos] 3
ConstBuffer "UnityLighting" 400 // 400 used size, 16 vars
Vector 0 [_WorldSpaceLightPos0] 4
Vector 32 [unity_4LightPosX0] 4
Vector 48 [unity_4LightPosY0] 4
Vector 64 [unity_4LightPosZ0] 4
Vector 80 [unity_4LightAtten0] 4
Vector 96 [unity_LightColor0] 4
Vector 112 [unity_LightColor1] 4
Vector 128 [unity_LightColor2] 4
Vector 144 [unity_LightColor3] 4
Vector 288 [unity_SHAr] 4
Vector 304 [unity_SHAg] 4
Vector 320 [unity_SHAb] 4
Vector 336 [unity_SHBr] 4
Vector 352 [unity_SHBg] 4
Vector 368 [unity_SHBb] 4
Vector 384 [unity_SHC] 4
ConstBuffer "UnityPerDraw" 336 // 336 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
Matrix 192 [_Object2World] 4
Matrix 256 [_World2Object] 4
Vector 320 [unity_Scale] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityLighting" 2
BindCB "UnityPerDraw" 3
// 64 instructions, 7 temp regs, 0 temp arrays:
// ALU 36 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0
eefiecedollpaohiekicibgpbkpnjhdjpabjjhpiabaaaaaapeakaaaaadaaaaaa
cmaaaaaapeaaaaaajeabaaaaejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaa
abaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaljaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfc
enebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheojiaaaaaaafaaaaaa
aiaaaaaaiaaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaabaaaaaaadamaaaaimaaaaaaabaaaaaaaaaaaaaa
adaaaaaaacaaaaaaahaiaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaa
ahaiaaaaimaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaahaiaaaafdfgfpfa
epfdejfeejepeoaafeeffiedepepfceeaaklklklfdeieefcfiajaaaaeaaaabaa
fgacaaaafjaaaaaeegiocaaaaaaaaaaaagaaaaaafjaaaaaeegiocaaaabaaaaaa
afaaaaaafjaaaaaeegiocaaaacaaaaaabjaaaaaafjaaaaaeegiocaaaadaaaaaa
bfaaaaaafpaaaaadpcbabaaaaaaaaaaafpaaaaadpcbabaaaabaaaaaafpaaaaad
hcbabaaaacaaaaaafpaaaaaddcbabaaaadaaaaaaghaaaaaepccabaaaaaaaaaaa
abaaaaaagfaaaaaddccabaaaabaaaaaagfaaaaadhccabaaaacaaaaaagfaaaaad
hccabaaaadaaaaaagfaaaaadhccabaaaaeaaaaaagiaaaaacahaaaaaadiaaaaai
pcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaadaaaaaaabaaaaaadcaaaaak
pcaabaaaaaaaaaaaegiocaaaadaaaaaaaaaaaaaaagbabaaaaaaaaaaaegaobaaa
aaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaacaaaaaakgbkbaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaakpccabaaaaaaaaaaaegiocaaaadaaaaaa
adaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaaldccabaaaabaaaaaa
egbabaaaadaaaaaaegiacaaaaaaaaaaaafaaaaaaogikcaaaaaaaaaaaafaaaaaa
diaaaaahhcaabaaaaaaaaaaajgbebaaaabaaaaaacgbjbaaaacaaaaaadcaaaaak
hcaabaaaaaaaaaaajgbebaaaacaaaaaacgbjbaaaabaaaaaaegacbaiaebaaaaaa
aaaaaaaadiaaaaahhcaabaaaaaaaaaaaegacbaaaaaaaaaaapgbpbaaaabaaaaaa
diaaaaajhcaabaaaabaaaaaafgifcaaaacaaaaaaaaaaaaaaegiccaaaadaaaaaa
bbaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaa
acaaaaaaaaaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaa
adaaaaaabcaaaaaakgikcaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaadcaaaaal
hcaabaaaabaaaaaaegiccaaaadaaaaaabdaaaaaapgipcaaaacaaaaaaaaaaaaaa
egacbaaaabaaaaaabaaaaaahcccabaaaacaaaaaaegacbaaaaaaaaaaaegacbaaa
abaaaaaabaaaaaahbccabaaaacaaaaaaegbcbaaaabaaaaaaegacbaaaabaaaaaa
baaaaaaheccabaaaacaaaaaaegbcbaaaacaaaaaaegacbaaaabaaaaaadgaaaaaf
icaabaaaabaaaaaaabeaaaaaaaaaiadpdiaaaaaihcaabaaaacaaaaaaegbcbaaa
acaaaaaapgipcaaaadaaaaaabeaaaaaadiaaaaaihcaabaaaadaaaaaafgafbaaa
acaaaaaaegiccaaaadaaaaaaanaaaaaadcaaaaaklcaabaaaacaaaaaaegiicaaa
adaaaaaaamaaaaaaagaabaaaacaaaaaaegaibaaaadaaaaaadcaaaaakhcaabaaa
abaaaaaaegiccaaaadaaaaaaaoaaaaaakgakbaaaacaaaaaaegadbaaaacaaaaaa
bbaaaaaibcaabaaaacaaaaaaegiocaaaacaaaaaabcaaaaaaegaobaaaabaaaaaa
bbaaaaaiccaabaaaacaaaaaaegiocaaaacaaaaaabdaaaaaaegaobaaaabaaaaaa
bbaaaaaiecaabaaaacaaaaaaegiocaaaacaaaaaabeaaaaaaegaobaaaabaaaaaa
diaaaaahpcaabaaaadaaaaaajgacbaaaabaaaaaaegakbaaaabaaaaaabbaaaaai
bcaabaaaaeaaaaaaegiocaaaacaaaaaabfaaaaaaegaobaaaadaaaaaabbaaaaai
ccaabaaaaeaaaaaaegiocaaaacaaaaaabgaaaaaaegaobaaaadaaaaaabbaaaaai
ecaabaaaaeaaaaaaegiocaaaacaaaaaabhaaaaaaegaobaaaadaaaaaaaaaaaaah
hcaabaaaacaaaaaaegacbaaaacaaaaaaegacbaaaaeaaaaaadiaaaaahicaabaaa
aaaaaaaabkaabaaaabaaaaaabkaabaaaabaaaaaadcaaaaakicaabaaaaaaaaaaa
akaabaaaabaaaaaaakaabaaaabaaaaaadkaabaiaebaaaaaaaaaaaaaadcaaaaak
hcaabaaaacaaaaaaegiccaaaacaaaaaabiaaaaaapgapbaaaaaaaaaaaegacbaaa
acaaaaaadiaaaaaihcaabaaaadaaaaaafgbfbaaaaaaaaaaaegiccaaaadaaaaaa
anaaaaaadcaaaaakhcaabaaaadaaaaaaegiccaaaadaaaaaaamaaaaaaagbabaaa
aaaaaaaaegacbaaaadaaaaaadcaaaaakhcaabaaaadaaaaaaegiccaaaadaaaaaa
aoaaaaaakgbkbaaaaaaaaaaaegacbaaaadaaaaaadcaaaaakhcaabaaaadaaaaaa
egiccaaaadaaaaaaapaaaaaapgbpbaaaaaaaaaaaegacbaaaadaaaaaaaaaaaaaj
pcaabaaaaeaaaaaafgafbaiaebaaaaaaadaaaaaaegiocaaaacaaaaaaadaaaaaa
diaaaaahpcaabaaaafaaaaaafgafbaaaabaaaaaaegaobaaaaeaaaaaadiaaaaah
pcaabaaaaeaaaaaaegaobaaaaeaaaaaaegaobaaaaeaaaaaaaaaaaaajpcaabaaa
agaaaaaaagaabaiaebaaaaaaadaaaaaaegiocaaaacaaaaaaacaaaaaaaaaaaaaj
pcaabaaaadaaaaaakgakbaiaebaaaaaaadaaaaaaegiocaaaacaaaaaaaeaaaaaa
dcaaaaajpcaabaaaafaaaaaaegaobaaaagaaaaaaagaabaaaabaaaaaaegaobaaa
afaaaaaadcaaaaajpcaabaaaabaaaaaaegaobaaaadaaaaaakgakbaaaabaaaaaa
egaobaaaafaaaaaadcaaaaajpcaabaaaaeaaaaaaegaobaaaagaaaaaaegaobaaa
agaaaaaaegaobaaaaeaaaaaadcaaaaajpcaabaaaadaaaaaaegaobaaaadaaaaaa
egaobaaaadaaaaaaegaobaaaaeaaaaaaeeaaaaafpcaabaaaaeaaaaaaegaobaaa
adaaaaaadcaaaaanpcaabaaaadaaaaaaegaobaaaadaaaaaaegiocaaaacaaaaaa
afaaaaaaaceaaaaaaaaaiadpaaaaiadpaaaaiadpaaaaiadpaoaaaaakpcaabaaa
adaaaaaaaceaaaaaaaaaiadpaaaaiadpaaaaiadpaaaaiadpegaobaaaadaaaaaa
diaaaaahpcaabaaaabaaaaaaegaobaaaabaaaaaaegaobaaaaeaaaaaadeaaaaak
pcaabaaaabaaaaaaegaobaaaabaaaaaaaceaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaadiaaaaahpcaabaaaabaaaaaaegaobaaaadaaaaaaegaobaaaabaaaaaa
diaaaaaihcaabaaaadaaaaaafgafbaaaabaaaaaaegiccaaaacaaaaaaahaaaaaa
dcaaaaakhcaabaaaadaaaaaaegiccaaaacaaaaaaagaaaaaaagaabaaaabaaaaaa
egacbaaaadaaaaaadcaaaaakhcaabaaaabaaaaaaegiccaaaacaaaaaaaiaaaaaa
kgakbaaaabaaaaaaegacbaaaadaaaaaadcaaaaakhcaabaaaabaaaaaaegiccaaa
acaaaaaaajaaaaaapgapbaaaabaaaaaaegacbaaaabaaaaaaaaaaaaahhccabaaa
adaaaaaaegacbaaaabaaaaaaegacbaaaacaaaaaadiaaaaajhcaabaaaabaaaaaa
fgifcaaaabaaaaaaaeaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaa
abaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaaabaaaaaaaeaaaaaaegacbaaa
abaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaa
abaaaaaaaeaaaaaaegacbaaaabaaaaaaaaaaaaaihcaabaaaabaaaaaaegacbaaa
abaaaaaaegiccaaaadaaaaaabdaaaaaadcaaaaalhcaabaaaabaaaaaaegacbaaa
abaaaaaapgipcaaaadaaaaaabeaaaaaaegbcbaiaebaaaaaaaaaaaaaabaaaaaah
cccabaaaaeaaaaaaegacbaaaaaaaaaaaegacbaaaabaaaaaabaaaaaahbccabaaa
aeaaaaaaegbcbaaaabaaaaaaegacbaaaabaaaaaabaaaaaaheccabaaaaeaaaaaa
egbcbaaaacaaaaaaegacbaaaabaaaaaadoaaaaab"
}

SubProgram "gles " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" "VERTEXLIGHT_ON" }
"!!GLES
#define SHADER_API_GLES 1
#define tex2D texture2D


#ifdef VERTEX
#define gl_ModelViewProjectionMatrix glstate_matrix_mvp
uniform mat4 glstate_matrix_mvp;

varying highp vec3 xlv_TEXCOORD3;
varying lowp vec3 xlv_TEXCOORD2;
varying lowp vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 unity_Scale;
uniform highp mat4 _World2Object;
uniform highp mat4 _Object2World;

uniform highp vec4 unity_SHC;
uniform highp vec4 unity_SHBb;
uniform highp vec4 unity_SHBg;
uniform highp vec4 unity_SHBr;
uniform highp vec4 unity_SHAb;
uniform highp vec4 unity_SHAg;
uniform highp vec4 unity_SHAr;
uniform highp vec4 unity_LightColor[4];
uniform highp vec4 unity_4LightAtten0;
uniform highp vec4 unity_4LightPosZ0;
uniform highp vec4 unity_4LightPosY0;
uniform highp vec4 unity_4LightPosX0;
uniform lowp vec4 _WorldSpaceLightPos0;
uniform highp vec3 _WorldSpaceCameraPos;
attribute vec4 _glesTANGENT;
attribute vec4 _glesMultiTexCoord0;
attribute vec3 _glesNormal;
attribute vec4 _glesVertex;
void main ()
{
  vec4 tmpvar_1;
  tmpvar_1.xyz = normalize(_glesTANGENT.xyz);
  tmpvar_1.w = _glesTANGENT.w;
  vec3 tmpvar_2;
  tmpvar_2 = normalize(_glesNormal);
  highp vec3 shlight_3;
  lowp vec3 tmpvar_4;
  lowp vec3 tmpvar_5;
  mat3 tmpvar_6;
  tmpvar_6[0] = _Object2World[0].xyz;
  tmpvar_6[1] = _Object2World[1].xyz;
  tmpvar_6[2] = _Object2World[2].xyz;
  highp vec3 tmpvar_7;
  tmpvar_7 = (tmpvar_6 * (tmpvar_2 * unity_Scale.w));
  highp vec3 tmpvar_8;
  highp vec3 tmpvar_9;
  tmpvar_8 = tmpvar_1.xyz;
  tmpvar_9 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_10;
  tmpvar_10[0].x = tmpvar_8.x;
  tmpvar_10[0].y = tmpvar_9.x;
  tmpvar_10[0].z = tmpvar_2.x;
  tmpvar_10[1].x = tmpvar_8.y;
  tmpvar_10[1].y = tmpvar_9.y;
  tmpvar_10[1].z = tmpvar_2.y;
  tmpvar_10[2].x = tmpvar_8.z;
  tmpvar_10[2].y = tmpvar_9.z;
  tmpvar_10[2].z = tmpvar_2.z;
  highp vec3 tmpvar_11;
  tmpvar_11 = (tmpvar_10 * (_World2Object * _WorldSpaceLightPos0).xyz);
  tmpvar_4 = tmpvar_11;
  highp vec4 tmpvar_12;
  tmpvar_12.w = 1.0;
  tmpvar_12.xyz = _WorldSpaceCameraPos;
  highp vec4 tmpvar_13;
  tmpvar_13.w = 1.0;
  tmpvar_13.xyz = tmpvar_7;
  mediump vec3 tmpvar_14;
  mediump vec4 normal_15;
  normal_15 = tmpvar_13;
  highp float vC_16;
  mediump vec3 x3_17;
  mediump vec3 x2_18;
  mediump vec3 x1_19;
  highp float tmpvar_20;
  tmpvar_20 = dot (unity_SHAr, normal_15);
  x1_19.x = tmpvar_20;
  highp float tmpvar_21;
  tmpvar_21 = dot (unity_SHAg, normal_15);
  x1_19.y = tmpvar_21;
  highp float tmpvar_22;
  tmpvar_22 = dot (unity_SHAb, normal_15);
  x1_19.z = tmpvar_22;
  mediump vec4 tmpvar_23;
  tmpvar_23 = (normal_15.xyzz * normal_15.yzzx);
  highp float tmpvar_24;
  tmpvar_24 = dot (unity_SHBr, tmpvar_23);
  x2_18.x = tmpvar_24;
  highp float tmpvar_25;
  tmpvar_25 = dot (unity_SHBg, tmpvar_23);
  x2_18.y = tmpvar_25;
  highp float tmpvar_26;
  tmpvar_26 = dot (unity_SHBb, tmpvar_23);
  x2_18.z = tmpvar_26;
  mediump float tmpvar_27;
  tmpvar_27 = ((normal_15.x * normal_15.x) - (normal_15.y * normal_15.y));
  vC_16 = tmpvar_27;
  highp vec3 tmpvar_28;
  tmpvar_28 = (unity_SHC.xyz * vC_16);
  x3_17 = tmpvar_28;
  tmpvar_14 = ((x1_19 + x2_18) + x3_17);
  shlight_3 = tmpvar_14;
  tmpvar_5 = shlight_3;
  highp vec3 tmpvar_29;
  tmpvar_29 = (_Object2World * _glesVertex).xyz;
  highp vec4 tmpvar_30;
  tmpvar_30 = (unity_4LightPosX0 - tmpvar_29.x);
  highp vec4 tmpvar_31;
  tmpvar_31 = (unity_4LightPosY0 - tmpvar_29.y);
  highp vec4 tmpvar_32;
  tmpvar_32 = (unity_4LightPosZ0 - tmpvar_29.z);
  highp vec4 tmpvar_33;
  tmpvar_33 = (((tmpvar_30 * tmpvar_30) + (tmpvar_31 * tmpvar_31)) + (tmpvar_32 * tmpvar_32));
  highp vec4 tmpvar_34;
  tmpvar_34 = (max (vec4(0.0, 0.0, 0.0, 0.0), ((((tmpvar_30 * tmpvar_7.x) + (tmpvar_31 * tmpvar_7.y)) + (tmpvar_32 * tmpvar_7.z)) * inversesqrt(tmpvar_33))) * (1.0/((1.0 + (tmpvar_33 * unity_4LightAtten0)))));
  highp vec3 tmpvar_35;
  tmpvar_35 = (tmpvar_5 + ((((unity_LightColor[0].xyz * tmpvar_34.x) + (unity_LightColor[1].xyz * tmpvar_34.y)) + (unity_LightColor[2].xyz * tmpvar_34.z)) + (unity_LightColor[3].xyz * tmpvar_34.w)));
  tmpvar_5 = tmpvar_35;
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = tmpvar_4;
  xlv_TEXCOORD2 = tmpvar_5;
  xlv_TEXCOORD3 = (tmpvar_10 * (((_World2Object * tmpvar_12).xyz * unity_Scale.w) - _glesVertex.xyz));
}



#endif
#ifdef FRAGMENT

varying highp vec3 xlv_TEXCOORD3;
varying lowp vec3 xlv_TEXCOORD2;
varying lowp vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform mediump float _Gloss;
uniform mediump float _Shininess;
uniform lowp vec4 _Color;
uniform sampler2D _SpecMap;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform lowp vec4 _LightColor0;
void main ()
{
  lowp vec4 c_1;
  mediump vec3 tmpvar_2;
  mediump vec3 tmpvar_3;
  lowp vec4 tmpvar_4;
  tmpvar_4 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_5;
  tmpvar_5 = (tmpvar_4.xyz * _Color.xyz);
  tmpvar_2 = tmpvar_5;
  lowp vec4 tmpvar_6;
  tmpvar_6 = texture2D (_SpecMap, xlv_TEXCOORD0);
  mediump vec3 tmpvar_7;
  tmpvar_7 = ((tmpvar_4.w * tmpvar_6.xyz) * _Gloss);
  lowp vec3 tmpvar_8;
  tmpvar_8 = ((texture2D (_BumpMap, xlv_TEXCOORD0).xyz * 2.0) - 1.0);
  tmpvar_3 = tmpvar_8;
  highp vec3 tmpvar_9;
  tmpvar_9 = normalize(xlv_TEXCOORD3);
  mediump vec3 lightDir_10;
  lightDir_10 = xlv_TEXCOORD1;
  mediump vec3 viewDir_11;
  viewDir_11 = tmpvar_9;
  mediump vec4 c_12;
  mediump vec3 specCol_13;
  highp float nh_14;
  mediump float tmpvar_15;
  tmpvar_15 = max (0.0, dot (tmpvar_3, normalize((lightDir_10 + viewDir_11))));
  nh_14 = tmpvar_15;
  mediump float arg1_16;
  arg1_16 = (32.0 * _Shininess);
  highp vec3 tmpvar_17;
  tmpvar_17 = (pow (nh_14, arg1_16) * tmpvar_7);
  specCol_13 = tmpvar_17;
  c_12.xyz = ((((tmpvar_2 * _LightColor0.xyz) * max (0.0, dot (tmpvar_3, lightDir_10))) + (_LightColor0.xyz * specCol_13)) * 2.0);
  c_12.w = 0.0;
  c_1 = c_12;
  mediump vec3 tmpvar_18;
  tmpvar_18 = (c_1.xyz + (tmpvar_2 * xlv_TEXCOORD2));
  c_1.xyz = tmpvar_18;
  gl_FragData[0] = c_1;
}



#endif"
}

SubProgram "glesdesktop " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" "VERTEXLIGHT_ON" }
"!!GLES
#define SHADER_API_GLES 1
#define tex2D texture2D


#ifdef VERTEX
#define gl_ModelViewProjectionMatrix glstate_matrix_mvp
uniform mat4 glstate_matrix_mvp;

varying highp vec3 xlv_TEXCOORD3;
varying lowp vec3 xlv_TEXCOORD2;
varying lowp vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 unity_Scale;
uniform highp mat4 _World2Object;
uniform highp mat4 _Object2World;

uniform highp vec4 unity_SHC;
uniform highp vec4 unity_SHBb;
uniform highp vec4 unity_SHBg;
uniform highp vec4 unity_SHBr;
uniform highp vec4 unity_SHAb;
uniform highp vec4 unity_SHAg;
uniform highp vec4 unity_SHAr;
uniform highp vec4 unity_LightColor[4];
uniform highp vec4 unity_4LightAtten0;
uniform highp vec4 unity_4LightPosZ0;
uniform highp vec4 unity_4LightPosY0;
uniform highp vec4 unity_4LightPosX0;
uniform lowp vec4 _WorldSpaceLightPos0;
uniform highp vec3 _WorldSpaceCameraPos;
attribute vec4 _glesTANGENT;
attribute vec4 _glesMultiTexCoord0;
attribute vec3 _glesNormal;
attribute vec4 _glesVertex;
void main ()
{
  vec4 tmpvar_1;
  tmpvar_1.xyz = normalize(_glesTANGENT.xyz);
  tmpvar_1.w = _glesTANGENT.w;
  vec3 tmpvar_2;
  tmpvar_2 = normalize(_glesNormal);
  highp vec3 shlight_3;
  lowp vec3 tmpvar_4;
  lowp vec3 tmpvar_5;
  mat3 tmpvar_6;
  tmpvar_6[0] = _Object2World[0].xyz;
  tmpvar_6[1] = _Object2World[1].xyz;
  tmpvar_6[2] = _Object2World[2].xyz;
  highp vec3 tmpvar_7;
  tmpvar_7 = (tmpvar_6 * (tmpvar_2 * unity_Scale.w));
  highp vec3 tmpvar_8;
  highp vec3 tmpvar_9;
  tmpvar_8 = tmpvar_1.xyz;
  tmpvar_9 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_10;
  tmpvar_10[0].x = tmpvar_8.x;
  tmpvar_10[0].y = tmpvar_9.x;
  tmpvar_10[0].z = tmpvar_2.x;
  tmpvar_10[1].x = tmpvar_8.y;
  tmpvar_10[1].y = tmpvar_9.y;
  tmpvar_10[1].z = tmpvar_2.y;
  tmpvar_10[2].x = tmpvar_8.z;
  tmpvar_10[2].y = tmpvar_9.z;
  tmpvar_10[2].z = tmpvar_2.z;
  highp vec3 tmpvar_11;
  tmpvar_11 = (tmpvar_10 * (_World2Object * _WorldSpaceLightPos0).xyz);
  tmpvar_4 = tmpvar_11;
  highp vec4 tmpvar_12;
  tmpvar_12.w = 1.0;
  tmpvar_12.xyz = _WorldSpaceCameraPos;
  highp vec4 tmpvar_13;
  tmpvar_13.w = 1.0;
  tmpvar_13.xyz = tmpvar_7;
  mediump vec3 tmpvar_14;
  mediump vec4 normal_15;
  normal_15 = tmpvar_13;
  highp float vC_16;
  mediump vec3 x3_17;
  mediump vec3 x2_18;
  mediump vec3 x1_19;
  highp float tmpvar_20;
  tmpvar_20 = dot (unity_SHAr, normal_15);
  x1_19.x = tmpvar_20;
  highp float tmpvar_21;
  tmpvar_21 = dot (unity_SHAg, normal_15);
  x1_19.y = tmpvar_21;
  highp float tmpvar_22;
  tmpvar_22 = dot (unity_SHAb, normal_15);
  x1_19.z = tmpvar_22;
  mediump vec4 tmpvar_23;
  tmpvar_23 = (normal_15.xyzz * normal_15.yzzx);
  highp float tmpvar_24;
  tmpvar_24 = dot (unity_SHBr, tmpvar_23);
  x2_18.x = tmpvar_24;
  highp float tmpvar_25;
  tmpvar_25 = dot (unity_SHBg, tmpvar_23);
  x2_18.y = tmpvar_25;
  highp float tmpvar_26;
  tmpvar_26 = dot (unity_SHBb, tmpvar_23);
  x2_18.z = tmpvar_26;
  mediump float tmpvar_27;
  tmpvar_27 = ((normal_15.x * normal_15.x) - (normal_15.y * normal_15.y));
  vC_16 = tmpvar_27;
  highp vec3 tmpvar_28;
  tmpvar_28 = (unity_SHC.xyz * vC_16);
  x3_17 = tmpvar_28;
  tmpvar_14 = ((x1_19 + x2_18) + x3_17);
  shlight_3 = tmpvar_14;
  tmpvar_5 = shlight_3;
  highp vec3 tmpvar_29;
  tmpvar_29 = (_Object2World * _glesVertex).xyz;
  highp vec4 tmpvar_30;
  tmpvar_30 = (unity_4LightPosX0 - tmpvar_29.x);
  highp vec4 tmpvar_31;
  tmpvar_31 = (unity_4LightPosY0 - tmpvar_29.y);
  highp vec4 tmpvar_32;
  tmpvar_32 = (unity_4LightPosZ0 - tmpvar_29.z);
  highp vec4 tmpvar_33;
  tmpvar_33 = (((tmpvar_30 * tmpvar_30) + (tmpvar_31 * tmpvar_31)) + (tmpvar_32 * tmpvar_32));
  highp vec4 tmpvar_34;
  tmpvar_34 = (max (vec4(0.0, 0.0, 0.0, 0.0), ((((tmpvar_30 * tmpvar_7.x) + (tmpvar_31 * tmpvar_7.y)) + (tmpvar_32 * tmpvar_7.z)) * inversesqrt(tmpvar_33))) * (1.0/((1.0 + (tmpvar_33 * unity_4LightAtten0)))));
  highp vec3 tmpvar_35;
  tmpvar_35 = (tmpvar_5 + ((((unity_LightColor[0].xyz * tmpvar_34.x) + (unity_LightColor[1].xyz * tmpvar_34.y)) + (unity_LightColor[2].xyz * tmpvar_34.z)) + (unity_LightColor[3].xyz * tmpvar_34.w)));
  tmpvar_5 = tmpvar_35;
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = tmpvar_4;
  xlv_TEXCOORD2 = tmpvar_5;
  xlv_TEXCOORD3 = (tmpvar_10 * (((_World2Object * tmpvar_12).xyz * unity_Scale.w) - _glesVertex.xyz));
}



#endif
#ifdef FRAGMENT

varying highp vec3 xlv_TEXCOORD3;
varying lowp vec3 xlv_TEXCOORD2;
varying lowp vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform mediump float _Gloss;
uniform mediump float _Shininess;
uniform lowp vec4 _Color;
uniform sampler2D _SpecMap;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform lowp vec4 _LightColor0;
void main ()
{
  lowp vec4 c_1;
  mediump vec3 tmpvar_2;
  mediump vec3 tmpvar_3;
  lowp vec4 tmpvar_4;
  tmpvar_4 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_5;
  tmpvar_5 = (tmpvar_4.xyz * _Color.xyz);
  tmpvar_2 = tmpvar_5;
  lowp vec4 tmpvar_6;
  tmpvar_6 = texture2D (_SpecMap, xlv_TEXCOORD0);
  mediump vec3 tmpvar_7;
  tmpvar_7 = ((tmpvar_4.w * tmpvar_6.xyz) * _Gloss);
  lowp vec3 normal_8;
  normal_8.xy = ((texture2D (_BumpMap, xlv_TEXCOORD0).wy * 2.0) - 1.0);
  normal_8.z = sqrt(((1.0 - (normal_8.x * normal_8.x)) - (normal_8.y * normal_8.y)));
  tmpvar_3 = normal_8;
  highp vec3 tmpvar_9;
  tmpvar_9 = normalize(xlv_TEXCOORD3);
  mediump vec3 lightDir_10;
  lightDir_10 = xlv_TEXCOORD1;
  mediump vec3 viewDir_11;
  viewDir_11 = tmpvar_9;
  mediump vec4 c_12;
  mediump vec3 specCol_13;
  highp float nh_14;
  mediump float tmpvar_15;
  tmpvar_15 = max (0.0, dot (tmpvar_3, normalize((lightDir_10 + viewDir_11))));
  nh_14 = tmpvar_15;
  mediump float arg1_16;
  arg1_16 = (32.0 * _Shininess);
  highp vec3 tmpvar_17;
  tmpvar_17 = (pow (nh_14, arg1_16) * tmpvar_7);
  specCol_13 = tmpvar_17;
  c_12.xyz = ((((tmpvar_2 * _LightColor0.xyz) * max (0.0, dot (tmpvar_3, lightDir_10))) + (_LightColor0.xyz * specCol_13)) * 2.0);
  c_12.w = 0.0;
  c_1 = c_12;
  mediump vec3 tmpvar_18;
  tmpvar_18 = (c_1.xyz + (tmpvar_2 * xlv_TEXCOORD2));
  c_1.xyz = tmpvar_18;
  gl_FragData[0] = c_1;
}



#endif"
}

SubProgram "flash " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" "VERTEXLIGHT_ON" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Matrix 0 [glstate_matrix_mvp]
Vector 12 [_WorldSpaceCameraPos]
Vector 13 [_WorldSpaceLightPos0]
Vector 14 [unity_4LightPosX0]
Vector 15 [unity_4LightPosY0]
Vector 16 [unity_4LightPosZ0]
Vector 17 [unity_4LightAtten0]
Vector 18 [unity_LightColor0]
Vector 19 [unity_LightColor1]
Vector 20 [unity_LightColor2]
Vector 21 [unity_LightColor3]
Vector 22 [unity_SHAr]
Vector 23 [unity_SHAg]
Vector 24 [unity_SHAb]
Vector 25 [unity_SHBr]
Vector 26 [unity_SHBg]
Vector 27 [unity_SHBb]
Vector 28 [unity_SHC]
Matrix 4 [_Object2World]
Matrix 8 [_World2Object]
Vector 29 [unity_Scale]
Vector 30 [_MainTex_ST]
"agal_vs
c31 1.0 0.0 0.0 0.0
[bc]
adaaaaaaadaaahacabaaaaoeaaaaaaaabnaaaappabaaaaaa mul r3.xyz, a1, c29.w
bdaaaaaaaaaaabacaaaaaaoeaaaaaaaaafaaaaoeabaaaaaa dp4 r0.x, a0, c5
bfaaaaaaabaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa neg r1.x, r0.x
abaaaaaaabaaapacabaaaaaaacaaaaaaapaaaaoeabaaaaaa add r1, r1.x, c15
bcaaaaaaadaaaiacadaaaakeacaaaaaaafaaaaoeabaaaaaa dp3 r3.w, r3.xyzz, c5
bcaaaaaaaeaaabacadaaaakeacaaaaaaaeaaaaoeabaaaaaa dp3 r4.x, r3.xyzz, c4
bcaaaaaaadaaabacadaaaakeacaaaaaaagaaaaoeabaaaaaa dp3 r3.x, r3.xyzz, c6
adaaaaaaacaaapacadaaaappacaaaaaaabaaaaoeacaaaaaa mul r2, r3.w, r1
bdaaaaaaaaaaabacaaaaaaoeaaaaaaaaaeaaaaoeabaaaaaa dp4 r0.x, a0, c4
bfaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa neg r0.x, r0.x
abaaaaaaaaaaapacaaaaaaaaacaaaaaaaoaaaaoeabaaaaaa add r0, r0.x, c14
adaaaaaaabaaapacabaaaaoeacaaaaaaabaaaaoeacaaaaaa mul r1, r1, r1
aaaaaaaaaeaaaeacadaaaaaaacaaaaaaaaaaaaaaaaaaaaaa mov r4.z, r3.x
adaaaaaaafaaapacaeaaaaaaacaaaaaaaaaaaaoeacaaaaaa mul r5, r4.x, r0
abaaaaaaacaaapacafaaaaoeacaaaaaaacaaaaoeacaaaaaa add r2, r5, r2
aaaaaaaaaeaaaiacbpaaaaaaabaaaaaaaaaaaaaaaaaaaaaa mov r4.w, c31.x
bdaaaaaaaeaaacacaaaaaaoeaaaaaaaaagaaaaoeabaaaaaa dp4 r4.y, a0, c6
adaaaaaaafaaapacaaaaaaoeacaaaaaaaaaaaaoeacaaaaaa mul r5, r0, r0
abaaaaaaabaaapacafaaaaoeacaaaaaaabaaaaoeacaaaaaa add r1, r5, r1
bfaaaaaaaaaaacacaeaaaaffacaaaaaaaaaaaaaaaaaaaaaa neg r0.y, r4.y
abaaaaaaaaaaapacaaaaaaffacaaaaaabaaaaaoeabaaaaaa add r0, r0.y, c16
adaaaaaaafaaapacaaaaaaoeacaaaaaaaaaaaaoeacaaaaaa mul r5, r0, r0
abaaaaaaabaaapacafaaaaoeacaaaaaaabaaaaoeacaaaaaa add r1, r5, r1
adaaaaaaaaaaapacadaaaaaaacaaaaaaaaaaaaoeacaaaaaa mul r0, r3.x, r0
abaaaaaaaaaaapacaaaaaaoeacaaaaaaacaaaaoeacaaaaaa add r0, r0, r2
adaaaaaaacaaapacabaaaaoeacaaaaaabbaaaaoeabaaaaaa mul r2, r1, c17
aaaaaaaaaeaaacacadaaaappacaaaaaaaaaaaaaaaaaaaaaa mov r4.y, r3.w
akaaaaaaabaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r1.x, r1.x
akaaaaaaabaaacacabaaaaffacaaaaaaaaaaaaaaaaaaaaaa rsq r1.y, r1.y
akaaaaaaabaaaiacabaaaappacaaaaaaaaaaaaaaaaaaaaaa rsq r1.w, r1.w
akaaaaaaabaaaeacabaaaakkacaaaaaaaaaaaaaaaaaaaaaa rsq r1.z, r1.z
adaaaaaaaaaaapacaaaaaaoeacaaaaaaabaaaaoeacaaaaaa mul r0, r0, r1
abaaaaaaabaaapacacaaaaoeacaaaaaabpaaaaaaabaaaaaa add r1, r2, c31.x
bdaaaaaaacaaaeacaeaaaaoeacaaaaaabiaaaaoeabaaaaaa dp4 r2.z, r4, c24
bdaaaaaaacaaacacaeaaaaoeacaaaaaabhaaaaoeabaaaaaa dp4 r2.y, r4, c23
bdaaaaaaacaaabacaeaaaaoeacaaaaaabgaaaaoeabaaaaaa dp4 r2.x, r4, c22
afaaaaaaabaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rcp r1.x, r1.x
afaaaaaaabaaacacabaaaaffacaaaaaaaaaaaaaaaaaaaaaa rcp r1.y, r1.y
afaaaaaaabaaaiacabaaaappacaaaaaaaaaaaaaaaaaaaaaa rcp r1.w, r1.w
afaaaaaaabaaaeacabaaaakkacaaaaaaaaaaaaaaaaaaaaaa rcp r1.z, r1.z
ahaaaaaaaaaaapacaaaaaaoeacaaaaaabpaaaaffabaaaaaa max r0, r0, c31.y
adaaaaaaaaaaapacaaaaaaoeacaaaaaaabaaaaoeacaaaaaa mul r0, r0, r1
adaaaaaaabaaahacaaaaaaffacaaaaaabdaaaaoeabaaaaaa mul r1.xyz, r0.y, c19
adaaaaaaafaaahacaaaaaaaaacaaaaaabcaaaaoeabaaaaaa mul r5.xyz, r0.x, c18
abaaaaaaabaaahacafaaaakeacaaaaaaabaaaakeacaaaaaa add r1.xyz, r5.xyzz, r1.xyzz
adaaaaaaaaaaahacaaaaaakkacaaaaaabeaaaaoeabaaaaaa mul r0.xyz, r0.z, c20
abaaaaaaaaaaahacaaaaaakeacaaaaaaabaaaakeacaaaaaa add r0.xyz, r0.xyzz, r1.xyzz
adaaaaaaabaaahacaaaaaappacaaaaaabfaaaaoeabaaaaaa mul r1.xyz, r0.w, c21
abaaaaaaabaaahacabaaaakeacaaaaaaaaaaaakeacaaaaaa add r1.xyz, r1.xyzz, r0.xyzz
adaaaaaaaaaaapacaeaaaakeacaaaaaaaeaaaacjacaaaaaa mul r0, r4.xyzz, r4.yzzx
adaaaaaaabaaaiacadaaaappacaaaaaaadaaaappacaaaaaa mul r1.w, r3.w, r3.w
bdaaaaaaadaaaeacaaaaaaoeacaaaaaablaaaaoeabaaaaaa dp4 r3.z, r0, c27
bdaaaaaaadaaacacaaaaaaoeacaaaaaabkaaaaoeabaaaaaa dp4 r3.y, r0, c26
bdaaaaaaadaaabacaaaaaaoeacaaaaaabjaaaaoeabaaaaaa dp4 r3.x, r0, c25
adaaaaaaafaaaiacaeaaaaaaacaaaaaaaeaaaaaaacaaaaaa mul r5.w, r4.x, r4.x
acaaaaaaabaaaiacafaaaappacaaaaaaabaaaappacaaaaaa sub r1.w, r5.w, r1.w
adaaaaaaaaaaahacabaaaappacaaaaaabmaaaaoeabaaaaaa mul r0.xyz, r1.w, c28
abaaaaaaacaaahacacaaaakeacaaaaaaadaaaakeacaaaaaa add r2.xyz, r2.xyzz, r3.xyzz
abaaaaaaaaaaahacacaaaakeacaaaaaaaaaaaakeacaaaaaa add r0.xyz, r2.xyzz, r0.xyzz
abaaaaaaacaaahaeaaaaaakeacaaaaaaabaaaakeacaaaaaa add v2.xyz, r0.xyzz, r1.xyzz
aaaaaaaaabaaaiacbpaaaaaaabaaaaaaaaaaaaaaaaaaaaaa mov r1.w, c31.x
aaaaaaaaabaaahacamaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r1.xyz, c12
bdaaaaaaaaaaaeacabaaaaoeacaaaaaaakaaaaoeabaaaaaa dp4 r0.z, r1, c10
bdaaaaaaaaaaacacabaaaaoeacaaaaaaajaaaaoeabaaaaaa dp4 r0.y, r1, c9
bdaaaaaaaaaaabacabaaaaoeacaaaaaaaiaaaaoeabaaaaaa dp4 r0.x, r1, c8
adaaaaaaafaaahacaaaaaakeacaaaaaabnaaaappabaaaaaa mul r5.xyz, r0.xyzz, c29.w
acaaaaaaadaaahacafaaaakeacaaaaaaaaaaaaoeaaaaaaaa sub r3.xyz, r5.xyzz, a0
aaaaaaaaabaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r1.xyz, a5
aaaaaaaaaaaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, a5
adaaaaaaabaaahacabaaaancaaaaaaaaabaaaaajacaaaaaa mul r1.xyz, a1.zxyw, r1.yzxx
adaaaaaaafaaahacabaaaamjaaaaaaaaaaaaaafcacaaaaaa mul r5.xyz, a1.yzxw, r0.zxyy
acaaaaaaabaaahacafaaaakeacaaaaaaabaaaakeacaaaaaa sub r1.xyz, r5.xyzz, r1.xyzz
adaaaaaaacaaahacabaaaakeacaaaaaaafaaaappaaaaaaaa mul r2.xyz, r1.xyzz, a5.w
aaaaaaaaaaaaapacakaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0, c10
bdaaaaaaaeaaaeacanaaaaoeabaaaaaaaaaaaaoeacaaaaaa dp4 r4.z, c13, r0
aaaaaaaaabaaapacajaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r1, c9
aaaaaaaaaaaaapacaiaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0, c8
bdaaaaaaaeaaacacanaaaaoeabaaaaaaabaaaaoeacaaaaaa dp4 r4.y, c13, r1
bdaaaaaaaeaaabacanaaaaoeabaaaaaaaaaaaaoeacaaaaaa dp4 r4.x, c13, r0
bcaaaaaaabaaacaeaeaaaakeacaaaaaaacaaaakeacaaaaaa dp3 v1.y, r4.xyzz, r2.xyzz
bcaaaaaaadaaacaeacaaaakeacaaaaaaadaaaakeacaaaaaa dp3 v3.y, r2.xyzz, r3.xyzz
bcaaaaaaabaaaeaeabaaaaoeaaaaaaaaaeaaaakeacaaaaaa dp3 v1.z, a1, r4.xyzz
bcaaaaaaabaaabaeaeaaaakeacaaaaaaafaaaaoeaaaaaaaa dp3 v1.x, r4.xyzz, a5
bcaaaaaaadaaaeaeabaaaaoeaaaaaaaaadaaaakeacaaaaaa dp3 v3.z, a1, r3.xyzz
bcaaaaaaadaaabaeafaaaaoeaaaaaaaaadaaaakeacaaaaaa dp3 v3.x, a5, r3.xyzz
adaaaaaaafaaadacadaaaaoeaaaaaaaaboaaaaoeabaaaaaa mul r5.xy, a3, c30
abaaaaaaaaaaadaeafaaaafeacaaaaaaboaaaaooabaaaaaa add v0.xy, r5.xyyy, c30.zwzw
bdaaaaaaaaaaaiadaaaaaaoeaaaaaaaaadaaaaoeabaaaaaa dp4 o0.w, a0, c3
bdaaaaaaaaaaaeadaaaaaaoeaaaaaaaaacaaaaoeabaaaaaa dp4 o0.z, a0, c2
bdaaaaaaaaaaacadaaaaaaoeaaaaaaaaabaaaaoeabaaaaaa dp4 o0.y, a0, c1
bdaaaaaaaaaaabadaaaaaaoeaaaaaaaaaaaaaaoeabaaaaaa dp4 o0.x, a0, c0
aaaaaaaaaaaaamaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v0.zw, c0
aaaaaaaaabaaaiaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v1.w, c0
aaaaaaaaacaaaiaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v2.w, c0
aaaaaaaaadaaaiaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v3.w, c0
"
}

SubProgram "d3d11_9x " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" "VERTEXLIGHT_ON" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 96 // 96 used size, 7 vars
Vector 80 [_MainTex_ST] 4
ConstBuffer "UnityPerCamera" 128 // 76 used size, 8 vars
Vector 64 [_WorldSpaceCameraPos] 3
ConstBuffer "UnityLighting" 400 // 400 used size, 16 vars
Vector 0 [_WorldSpaceLightPos0] 4
Vector 32 [unity_4LightPosX0] 4
Vector 48 [unity_4LightPosY0] 4
Vector 64 [unity_4LightPosZ0] 4
Vector 80 [unity_4LightAtten0] 4
Vector 96 [unity_LightColor0] 4
Vector 112 [unity_LightColor1] 4
Vector 128 [unity_LightColor2] 4
Vector 144 [unity_LightColor3] 4
Vector 288 [unity_SHAr] 4
Vector 304 [unity_SHAg] 4
Vector 320 [unity_SHAb] 4
Vector 336 [unity_SHBr] 4
Vector 352 [unity_SHBg] 4
Vector 368 [unity_SHBb] 4
Vector 384 [unity_SHC] 4
ConstBuffer "UnityPerDraw" 336 // 336 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
Matrix 192 [_Object2World] 4
Matrix 256 [_World2Object] 4
Vector 320 [unity_Scale] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityLighting" 2
BindCB "UnityPerDraw" 3
// 64 instructions, 7 temp regs, 0 temp arrays:
// ALU 36 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0_level_9_3
eefiecedckkoanfalphmiedmghjodpkipedmlmpcabaaaaaalabaaaaaaeaaaaaa
daaaaaaaoiafaaaaeiapaaaababaaaaaebgpgodjlaafaaaalaafaaaaaaacpopp
deafaaaahmaaaaaaahaaceaaaaaahiaaaaaahiaaaaaaceaaabaahiaaaaaaafaa
abaaabaaaaaaaaaaabaaaeaaabaaacaaaaaaaaaaacaaaaaaabaaadaaaaaaaaaa
acaaacaaaiaaaeaaaaaaaaaaacaabcaaahaaamaaaaaaaaaaadaaaaaaaeaabdaa
aaaaaaaaadaaamaaajaabhaaaaaaaaaaaaaaaaaaabacpoppfbaaaaafcaaaapka
aaaaiadpaaaaaaaaaaaaaaaaaaaaaaaabpaaaaacafaaaaiaaaaaapjabpaaaaac
afaaabiaabaaapjabpaaaaacafaaaciaacaaapjabpaaaaacafaaadiaadaaapja
aeaaaaaeaaaaadoaadaaoejaabaaoekaabaaookaabaaaaacaaaaapiaadaaoeka
afaaaaadabaaahiaaaaaffiabmaaoekaaeaaaaaeabaaahiablaaoekaaaaaaaia
abaaoeiaaeaaaaaeaaaaahiabnaaoekaaaaakkiaabaaoeiaaeaaaaaeaaaaahia
boaaoekaaaaappiaaaaaoeiaaiaaaaadabaaaboaabaaoejaaaaaoeiaabaaaaac
abaaahiaacaaoejaafaaaaadacaaahiaabaanciaabaamjjaaeaaaaaeabaaahia
abaamjiaabaancjaacaaoeibafaaaaadabaaahiaabaaoeiaabaappjaaiaaaaad
abaaacoaabaaoeiaaaaaoeiaaiaaaaadabaaaeoaacaaoejaaaaaoeiaabaaaaac
aaaaahiaacaaoekaafaaaaadacaaahiaaaaaffiabmaaoekaaeaaaaaeaaaaalia
blaakekaaaaaaaiaacaakeiaaeaaaaaeaaaaahiabnaaoekaaaaakkiaaaaapeia
acaaaaadaaaaahiaaaaaoeiaboaaoekaaeaaaaaeaaaaahiaaaaaoeiabpaappka
aaaaoejbaiaaaaadadaaaboaabaaoejaaaaaoeiaaiaaaaadadaaacoaabaaoeia
aaaaoeiaaiaaaaadadaaaeoaacaaoejaaaaaoeiaafaaaaadaaaaahiaaaaaffja
biaaoekaaeaaaaaeaaaaahiabhaaoekaaaaaaajaaaaaoeiaaeaaaaaeaaaaahia
bjaaoekaaaaakkjaaaaaoeiaaeaaaaaeaaaaahiabkaaoekaaaaappjaaaaaoeia
acaaaaadabaaapiaaaaakkibagaaoekaacaaaaadacaaapiaaaaaaaibaeaaoeka
acaaaaadaaaaapiaaaaaffibafaaoekaafaaaaadadaaahiaacaaoejabpaappka
afaaaaadaeaaahiaadaaffiabiaaoekaaeaaaaaeadaaaliabhaakekaadaaaaia
aeaakeiaaeaaaaaeadaaahiabjaaoekaadaakkiaadaapeiaafaaaaadaeaaapia
aaaaoeiaadaaffiaafaaaaadaaaaapiaaaaaoeiaaaaaoeiaaeaaaaaeaaaaapia
acaaoeiaacaaoeiaaaaaoeiaaeaaaaaeacaaapiaacaaoeiaadaaaaiaaeaaoeia
aeaaaaaeacaaapiaabaaoeiaadaakkiaacaaoeiaaeaaaaaeaaaaapiaabaaoeia
abaaoeiaaaaaoeiaahaaaaacabaaabiaaaaaaaiaahaaaaacabaaaciaaaaaffia
ahaaaaacabaaaeiaaaaakkiaahaaaaacabaaaiiaaaaappiaabaaaaacaeaaabia
caaaaakaaeaaaaaeaaaaapiaaaaaoeiaahaaoekaaeaaaaiaafaaaaadabaaapia
abaaoeiaacaaoeiaalaaaaadabaaapiaabaaoeiacaaaffkaagaaaaacacaaabia
aaaaaaiaagaaaaacacaaaciaaaaaffiaagaaaaacacaaaeiaaaaakkiaagaaaaac
acaaaiiaaaaappiaafaaaaadaaaaapiaabaaoeiaacaaoeiaafaaaaadabaaahia
aaaaffiaajaaoekaaeaaaaaeabaaahiaaiaaoekaaaaaaaiaabaaoeiaaeaaaaae
aaaaahiaakaaoekaaaaakkiaabaaoeiaaeaaaaaeaaaaahiaalaaoekaaaaappia
aaaaoeiaabaaaaacadaaaiiacaaaaakaajaaaaadabaaabiaamaaoekaadaaoeia
ajaaaaadabaaaciaanaaoekaadaaoeiaajaaaaadabaaaeiaaoaaoekaadaaoeia
afaaaaadacaaapiaadaacjiaadaakeiaajaaaaadaeaaabiaapaaoekaacaaoeia
ajaaaaadaeaaaciabaaaoekaacaaoeiaajaaaaadaeaaaeiabbaaoekaacaaoeia
acaaaaadabaaahiaabaaoeiaaeaaoeiaafaaaaadaaaaaiiaadaaffiaadaaffia
aeaaaaaeaaaaaiiaadaaaaiaadaaaaiaaaaappibaeaaaaaeabaaahiabcaaoeka
aaaappiaabaaoeiaacaaaaadacaaahoaaaaaoeiaabaaoeiaafaaaaadaaaaapia
aaaaffjabeaaoekaaeaaaaaeaaaaapiabdaaoekaaaaaaajaaaaaoeiaaeaaaaae
aaaaapiabfaaoekaaaaakkjaaaaaoeiaaeaaaaaeaaaaapiabgaaoekaaaaappja
aaaaoeiaaeaaaaaeaaaaadmaaaaappiaaaaaoekaaaaaoeiaabaaaaacaaaaamma
aaaaoeiappppaaaafdeieefcfiajaaaaeaaaabaafgacaaaafjaaaaaeegiocaaa
aaaaaaaaagaaaaaafjaaaaaeegiocaaaabaaaaaaafaaaaaafjaaaaaeegiocaaa
acaaaaaabjaaaaaafjaaaaaeegiocaaaadaaaaaabfaaaaaafpaaaaadpcbabaaa
aaaaaaaafpaaaaadpcbabaaaabaaaaaafpaaaaadhcbabaaaacaaaaaafpaaaaad
dcbabaaaadaaaaaaghaaaaaepccabaaaaaaaaaaaabaaaaaagfaaaaaddccabaaa
abaaaaaagfaaaaadhccabaaaacaaaaaagfaaaaadhccabaaaadaaaaaagfaaaaad
hccabaaaaeaaaaaagiaaaaacahaaaaaadiaaaaaipcaabaaaaaaaaaaafgbfbaaa
aaaaaaaaegiocaaaadaaaaaaabaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaa
adaaaaaaaaaaaaaaagbabaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaa
aaaaaaaaegiocaaaadaaaaaaacaaaaaakgbkbaaaaaaaaaaaegaobaaaaaaaaaaa
dcaaaaakpccabaaaaaaaaaaaegiocaaaadaaaaaaadaaaaaapgbpbaaaaaaaaaaa
egaobaaaaaaaaaaadcaaaaaldccabaaaabaaaaaaegbabaaaadaaaaaaegiacaaa
aaaaaaaaafaaaaaaogikcaaaaaaaaaaaafaaaaaadiaaaaahhcaabaaaaaaaaaaa
jgbebaaaabaaaaaacgbjbaaaacaaaaaadcaaaaakhcaabaaaaaaaaaaajgbebaaa
acaaaaaacgbjbaaaabaaaaaaegacbaiaebaaaaaaaaaaaaaadiaaaaahhcaabaaa
aaaaaaaaegacbaaaaaaaaaaapgbpbaaaabaaaaaadiaaaaajhcaabaaaabaaaaaa
fgifcaaaacaaaaaaaaaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaa
abaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaaacaaaaaaaaaaaaaaegacbaaa
abaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaa
acaaaaaaaaaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaa
adaaaaaabdaaaaaapgipcaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaabaaaaaah
cccabaaaacaaaaaaegacbaaaaaaaaaaaegacbaaaabaaaaaabaaaaaahbccabaaa
acaaaaaaegbcbaaaabaaaaaaegacbaaaabaaaaaabaaaaaaheccabaaaacaaaaaa
egbcbaaaacaaaaaaegacbaaaabaaaaaadgaaaaaficaabaaaabaaaaaaabeaaaaa
aaaaiadpdiaaaaaihcaabaaaacaaaaaaegbcbaaaacaaaaaapgipcaaaadaaaaaa
beaaaaaadiaaaaaihcaabaaaadaaaaaafgafbaaaacaaaaaaegiccaaaadaaaaaa
anaaaaaadcaaaaaklcaabaaaacaaaaaaegiicaaaadaaaaaaamaaaaaaagaabaaa
acaaaaaaegaibaaaadaaaaaadcaaaaakhcaabaaaabaaaaaaegiccaaaadaaaaaa
aoaaaaaakgakbaaaacaaaaaaegadbaaaacaaaaaabbaaaaaibcaabaaaacaaaaaa
egiocaaaacaaaaaabcaaaaaaegaobaaaabaaaaaabbaaaaaiccaabaaaacaaaaaa
egiocaaaacaaaaaabdaaaaaaegaobaaaabaaaaaabbaaaaaiecaabaaaacaaaaaa
egiocaaaacaaaaaabeaaaaaaegaobaaaabaaaaaadiaaaaahpcaabaaaadaaaaaa
jgacbaaaabaaaaaaegakbaaaabaaaaaabbaaaaaibcaabaaaaeaaaaaaegiocaaa
acaaaaaabfaaaaaaegaobaaaadaaaaaabbaaaaaiccaabaaaaeaaaaaaegiocaaa
acaaaaaabgaaaaaaegaobaaaadaaaaaabbaaaaaiecaabaaaaeaaaaaaegiocaaa
acaaaaaabhaaaaaaegaobaaaadaaaaaaaaaaaaahhcaabaaaacaaaaaaegacbaaa
acaaaaaaegacbaaaaeaaaaaadiaaaaahicaabaaaaaaaaaaabkaabaaaabaaaaaa
bkaabaaaabaaaaaadcaaaaakicaabaaaaaaaaaaaakaabaaaabaaaaaaakaabaaa
abaaaaaadkaabaiaebaaaaaaaaaaaaaadcaaaaakhcaabaaaacaaaaaaegiccaaa
acaaaaaabiaaaaaapgapbaaaaaaaaaaaegacbaaaacaaaaaadiaaaaaihcaabaaa
adaaaaaafgbfbaaaaaaaaaaaegiccaaaadaaaaaaanaaaaaadcaaaaakhcaabaaa
adaaaaaaegiccaaaadaaaaaaamaaaaaaagbabaaaaaaaaaaaegacbaaaadaaaaaa
dcaaaaakhcaabaaaadaaaaaaegiccaaaadaaaaaaaoaaaaaakgbkbaaaaaaaaaaa
egacbaaaadaaaaaadcaaaaakhcaabaaaadaaaaaaegiccaaaadaaaaaaapaaaaaa
pgbpbaaaaaaaaaaaegacbaaaadaaaaaaaaaaaaajpcaabaaaaeaaaaaafgafbaia
ebaaaaaaadaaaaaaegiocaaaacaaaaaaadaaaaaadiaaaaahpcaabaaaafaaaaaa
fgafbaaaabaaaaaaegaobaaaaeaaaaaadiaaaaahpcaabaaaaeaaaaaaegaobaaa
aeaaaaaaegaobaaaaeaaaaaaaaaaaaajpcaabaaaagaaaaaaagaabaiaebaaaaaa
adaaaaaaegiocaaaacaaaaaaacaaaaaaaaaaaaajpcaabaaaadaaaaaakgakbaia
ebaaaaaaadaaaaaaegiocaaaacaaaaaaaeaaaaaadcaaaaajpcaabaaaafaaaaaa
egaobaaaagaaaaaaagaabaaaabaaaaaaegaobaaaafaaaaaadcaaaaajpcaabaaa
abaaaaaaegaobaaaadaaaaaakgakbaaaabaaaaaaegaobaaaafaaaaaadcaaaaaj
pcaabaaaaeaaaaaaegaobaaaagaaaaaaegaobaaaagaaaaaaegaobaaaaeaaaaaa
dcaaaaajpcaabaaaadaaaaaaegaobaaaadaaaaaaegaobaaaadaaaaaaegaobaaa
aeaaaaaaeeaaaaafpcaabaaaaeaaaaaaegaobaaaadaaaaaadcaaaaanpcaabaaa
adaaaaaaegaobaaaadaaaaaaegiocaaaacaaaaaaafaaaaaaaceaaaaaaaaaiadp
aaaaiadpaaaaiadpaaaaiadpaoaaaaakpcaabaaaadaaaaaaaceaaaaaaaaaiadp
aaaaiadpaaaaiadpaaaaiadpegaobaaaadaaaaaadiaaaaahpcaabaaaabaaaaaa
egaobaaaabaaaaaaegaobaaaaeaaaaaadeaaaaakpcaabaaaabaaaaaaegaobaaa
abaaaaaaaceaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaadiaaaaahpcaabaaa
abaaaaaaegaobaaaadaaaaaaegaobaaaabaaaaaadiaaaaaihcaabaaaadaaaaaa
fgafbaaaabaaaaaaegiccaaaacaaaaaaahaaaaaadcaaaaakhcaabaaaadaaaaaa
egiccaaaacaaaaaaagaaaaaaagaabaaaabaaaaaaegacbaaaadaaaaaadcaaaaak
hcaabaaaabaaaaaaegiccaaaacaaaaaaaiaaaaaakgakbaaaabaaaaaaegacbaaa
adaaaaaadcaaaaakhcaabaaaabaaaaaaegiccaaaacaaaaaaajaaaaaapgapbaaa
abaaaaaaegacbaaaabaaaaaaaaaaaaahhccabaaaadaaaaaaegacbaaaabaaaaaa
egacbaaaacaaaaaadiaaaaajhcaabaaaabaaaaaafgifcaaaabaaaaaaaeaaaaaa
egiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaa
baaaaaaaagiacaaaabaaaaaaaeaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaa
abaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaaabaaaaaaaeaaaaaaegacbaaa
abaaaaaaaaaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaaegiccaaaadaaaaaa
bdaaaaaadcaaaaalhcaabaaaabaaaaaaegacbaaaabaaaaaapgipcaaaadaaaaaa
beaaaaaaegbcbaiaebaaaaaaaaaaaaaabaaaaaahcccabaaaaeaaaaaaegacbaaa
aaaaaaaaegacbaaaabaaaaaabaaaaaahbccabaaaaeaaaaaaegbcbaaaabaaaaaa
egacbaaaabaaaaaabaaaaaaheccabaaaaeaaaaaaegbcbaaaacaaaaaaegacbaaa
abaaaaaadoaaaaabejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaaadaaaaaa
abaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaaahahaaaa
laaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaaabaaaaaa
aaaaaaaaadaaaaaaaeaaaaaaapaaaaaaljaaaaaaaaaaaaaaaaaaaaaaadaaaaaa
afaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfcenebemaa
feeffiedepepfceeaaedepemepfcaaklepfdeheojiaaaaaaafaaaaaaaiaaaaaa
iaaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaabaaaaaaadamaaaaimaaaaaaabaaaaaaaaaaaaaaadaaaaaa
acaaaaaaahaiaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaahaiaaaa
imaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaahaiaaaafdfgfpfaepfdejfe
ejepeoaafeeffiedepepfceeaaklklkl"
}

SubProgram "opengl " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" "VERTEXLIGHT_ON" }
Bind "vertex" Vertex
Bind "tangent" ATTR14
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 13 [_WorldSpaceCameraPos]
Vector 14 [_ProjectionParams]
Vector 15 [_WorldSpaceLightPos0]
Vector 16 [unity_4LightPosX0]
Vector 17 [unity_4LightPosY0]
Vector 18 [unity_4LightPosZ0]
Vector 19 [unity_4LightAtten0]
Vector 20 [unity_LightColor0]
Vector 21 [unity_LightColor1]
Vector 22 [unity_LightColor2]
Vector 23 [unity_LightColor3]
Vector 24 [unity_SHAr]
Vector 25 [unity_SHAg]
Vector 26 [unity_SHAb]
Vector 27 [unity_SHBr]
Vector 28 [unity_SHBg]
Vector 29 [unity_SHBb]
Vector 30 [unity_SHC]
Matrix 5 [_Object2World]
Matrix 9 [_World2Object]
Vector 31 [unity_Scale]
Vector 32 [_MainTex_ST]
"!!ARBvp1.0
# 79 ALU
PARAM c[33] = { { 1, 0, 0.5 },
		state.matrix.mvp,
		program.local[5..32] };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
TEMP R4;
MUL R3.xyz, vertex.normal, c[31].w;
DP4 R0.x, vertex.position, c[6];
ADD R1, -R0.x, c[17];
DP3 R3.w, R3, c[6];
DP3 R4.x, R3, c[5];
DP3 R3.x, R3, c[7];
MUL R2, R3.w, R1;
DP4 R0.x, vertex.position, c[5];
ADD R0, -R0.x, c[16];
MUL R1, R1, R1;
MOV R4.z, R3.x;
MAD R2, R4.x, R0, R2;
MOV R4.w, c[0].x;
DP4 R4.y, vertex.position, c[7];
MAD R1, R0, R0, R1;
ADD R0, -R4.y, c[18];
MAD R1, R0, R0, R1;
MAD R0, R3.x, R0, R2;
MUL R2, R1, c[19];
MOV R4.y, R3.w;
RSQ R1.x, R1.x;
RSQ R1.y, R1.y;
RSQ R1.w, R1.w;
RSQ R1.z, R1.z;
MUL R0, R0, R1;
ADD R1, R2, c[0].x;
RCP R1.x, R1.x;
RCP R1.y, R1.y;
RCP R1.w, R1.w;
RCP R1.z, R1.z;
MAX R0, R0, c[0].y;
MUL R0, R0, R1;
MUL R1.xyz, R0.y, c[21];
MAD R1.xyz, R0.x, c[20], R1;
MAD R0.xyz, R0.z, c[22], R1;
MAD R1.xyz, R0.w, c[23], R0;
MUL R0, R4.xyzz, R4.yzzx;
MUL R1.w, R3, R3;
DP4 R3.z, R0, c[29];
DP4 R3.y, R0, c[28];
DP4 R3.x, R0, c[27];
MAD R1.w, R4.x, R4.x, -R1;
MUL R0.xyz, R1.w, c[30];
MOV R1.w, c[0].x;
DP4 R0.w, vertex.position, c[4];
DP4 R2.z, R4, c[26];
DP4 R2.y, R4, c[25];
DP4 R2.x, R4, c[24];
ADD R2.xyz, R2, R3;
ADD R0.xyz, R2, R0;
ADD result.texcoord[2].xyz, R0, R1;
MOV R1.xyz, c[13];
DP4 R2.z, R1, c[11];
DP4 R2.y, R1, c[10];
DP4 R2.x, R1, c[9];
MAD R2.xyz, R2, c[31].w, -vertex.position;
MOV R0.xyz, vertex.attrib[14];
MUL R1.xyz, vertex.normal.zxyw, R0.yzxw;
MAD R0.xyz, vertex.normal.yzxw, R0.zxyw, -R1;
MOV R1, c[15];
MUL R0.xyz, R0, vertex.attrib[14].w;
DP4 R3.z, R1, c[11];
DP4 R3.y, R1, c[10];
DP4 R3.x, R1, c[9];
DP3 result.texcoord[1].y, R3, R0;
DP3 result.texcoord[3].y, R0, R2;
DP4 R0.z, vertex.position, c[3];
DP4 R0.x, vertex.position, c[1];
DP4 R0.y, vertex.position, c[2];
MUL R1.xyz, R0.xyww, c[0].z;
MUL R1.y, R1, c[14].x;
DP3 result.texcoord[1].z, vertex.normal, R3;
DP3 result.texcoord[1].x, R3, vertex.attrib[14];
DP3 result.texcoord[3].z, vertex.normal, R2;
DP3 result.texcoord[3].x, vertex.attrib[14], R2;
ADD result.texcoord[4].xy, R1, R1.z;
MOV result.position, R0;
MOV result.texcoord[4].zw, R0;
MAD result.texcoord[0].xy, vertex.texcoord[0], c[32], c[32].zwzw;
END
# 79 instructions, 5 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" "VERTEXLIGHT_ON" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Matrix 0 [glstate_matrix_mvp]
Vector 12 [_WorldSpaceCameraPos]
Vector 13 [_ProjectionParams]
Vector 14 [_ScreenParams]
Vector 15 [_WorldSpaceLightPos0]
Vector 16 [unity_4LightPosX0]
Vector 17 [unity_4LightPosY0]
Vector 18 [unity_4LightPosZ0]
Vector 19 [unity_4LightAtten0]
Vector 20 [unity_LightColor0]
Vector 21 [unity_LightColor1]
Vector 22 [unity_LightColor2]
Vector 23 [unity_LightColor3]
Vector 24 [unity_SHAr]
Vector 25 [unity_SHAg]
Vector 26 [unity_SHAb]
Vector 27 [unity_SHBr]
Vector 28 [unity_SHBg]
Vector 29 [unity_SHBb]
Vector 30 [unity_SHC]
Matrix 4 [_Object2World]
Matrix 8 [_World2Object]
Vector 31 [unity_Scale]
Vector 32 [_MainTex_ST]
"vs_2_0
; 82 ALU
def c33, 1.00000000, 0.00000000, 0.50000000, 0
dcl_position0 v0
dcl_tangent0 v1
dcl_normal0 v2
dcl_texcoord0 v3
mul r3.xyz, v2, c31.w
dp4 r0.x, v0, c5
add r1, -r0.x, c17
dp3 r3.w, r3, c5
dp3 r4.x, r3, c4
dp3 r3.x, r3, c6
mul r2, r3.w, r1
dp4 r0.x, v0, c4
add r0, -r0.x, c16
mul r1, r1, r1
mov r4.z, r3.x
mad r2, r4.x, r0, r2
mov r4.w, c33.x
dp4 r4.y, v0, c6
mad r1, r0, r0, r1
add r0, -r4.y, c18
mad r1, r0, r0, r1
mad r0, r3.x, r0, r2
mul r2, r1, c19
mov r4.y, r3.w
rsq r1.x, r1.x
rsq r1.y, r1.y
rsq r1.w, r1.w
rsq r1.z, r1.z
mul r0, r0, r1
add r1, r2, c33.x
dp4 r2.z, r4, c26
dp4 r2.y, r4, c25
dp4 r2.x, r4, c24
rcp r1.x, r1.x
rcp r1.y, r1.y
rcp r1.w, r1.w
rcp r1.z, r1.z
max r0, r0, c33.y
mul r0, r0, r1
mul r1.xyz, r0.y, c21
mad r1.xyz, r0.x, c20, r1
mad r0.xyz, r0.z, c22, r1
mad r1.xyz, r0.w, c23, r0
mul r0, r4.xyzz, r4.yzzx
mul r1.w, r3, r3
dp4 r3.z, r0, c29
dp4 r3.y, r0, c28
dp4 r3.x, r0, c27
mad r1.w, r4.x, r4.x, -r1
mul r0.xyz, r1.w, c30
add r2.xyz, r2, r3
add r0.xyz, r2, r0
add oT2.xyz, r0, r1
mov r1.w, c33.x
mov r1.xyz, c12
dp4 r0.z, r1, c10
dp4 r0.y, r1, c9
dp4 r0.x, r1, c8
mad r3.xyz, r0, c31.w, -v0
mov r1.xyz, v1
mov r0.xyz, v1
mul r1.xyz, v2.zxyw, r1.yzxw
mad r1.xyz, v2.yzxw, r0.zxyw, -r1
mul r2.xyz, r1, v1.w
mov r0, c10
dp4 r4.z, c15, r0
mov r0, c8
dp4 r4.x, c15, r0
mov r1, c9
dp4 r4.y, c15, r1
dp4 r0.w, v0, c3
dp4 r0.z, v0, c2
dp4 r0.x, v0, c0
dp4 r0.y, v0, c1
mul r1.xyz, r0.xyww, c33.z
mul r1.y, r1, c13.x
dp3 oT1.y, r4, r2
dp3 oT3.y, r2, r3
dp3 oT1.z, v2, r4
dp3 oT1.x, r4, v1
dp3 oT3.z, v2, r3
dp3 oT3.x, v1, r3
mad oT4.xy, r1.z, c14.zwzw, r1
mov oPos, r0
mov oT4.zw, r0
mad oT0.xy, v3, c32, c32.zwzw
"
}

SubProgram "xbox360 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" "VERTEXLIGHT_ON" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 32 [_MainTex_ST]
Matrix 23 [_Object2World] 4
Vector 1 [_ProjectionParams]
Vector 2 [_ScreenParams]
Matrix 27 [_World2Object] 4
Vector 0 [_WorldSpaceCameraPos]
Vector 3 [_WorldSpaceLightPos0]
Matrix 19 [glstate_matrix_mvp] 4
Vector 7 [unity_4LightAtten0]
Vector 4 [unity_4LightPosX0]
Vector 5 [unity_4LightPosY0]
Vector 6 [unity_4LightPosZ0]
Vector 8 [unity_LightColor0]
Vector 9 [unity_LightColor1]
Vector 10 [unity_LightColor2]
Vector 11 [unity_LightColor3]
Vector 14 [unity_SHAb]
Vector 13 [unity_SHAg]
Vector 12 [unity_SHAr]
Vector 17 [unity_SHBb]
Vector 16 [unity_SHBg]
Vector 15 [unity_SHBr]
Vector 18 [unity_SHC]
Vector 31 [unity_Scale]
// Shader Timing Estimate, in Cycles/64 vertex vector:
// ALU: 94.67 (71 instructions), vertex: 32, texture: 0,
//   sequencer: 30,  13 GPRs, 12 threads,
// Performance (if enough threads): ~94 cycles per vector
// * Vertex cycle estimates are assuming 3 vfetch_minis for every vfetch_full,
//     with <= 32 bytes per vfetch_full group.

"vs_360
backbbabaaaaaebeaaaaaedaaaaaaaaaaaaaaaceaaaaadhiaaaaadkaaaaaaaaa
aaaaaaaaaaaaadfaaaaaaabmaaaaadedpppoadaaaaaaaabfaaaaaabmaaaaaaaa
aaaaaddmaaaaabmaaaacaacaaaabaaaaaaaaabmmaaaaaaaaaaaaabnmaaacaabh
aaaeaaaaaaaaabomaaaaaaaaaaaaabpmaaacaaabaaabaaaaaaaaabmmaaaaaaaa
aaaaacaoaaacaaacaaabaaaaaaaaabmmaaaaaaaaaaaaacbmaaacaablaaaeaaaa
aaaaabomaaaaaaaaaaaaacckaaacaaaaaaabaaaaaaaaaceaaaaaaaaaaaaaacfa
aaacaaadaaabaaaaaaaaabmmaaaaaaaaaaaaacgfaaacaabdaaaeaaaaaaaaabom
aaaaaaaaaaaaachiaaacaaahaaabaaaaaaaaabmmaaaaaaaaaaaaacilaaacaaae
aaabaaaaaaaaabmmaaaaaaaaaaaaacjnaaacaaafaaabaaaaaaaaabmmaaaaaaaa
aaaaackpaaacaaagaaabaaaaaaaaabmmaaaaaaaaaaaaacmbaaacaaaiaaaeaaaa
aaaaacneaaaaaaaaaaaaacoeaaacaaaoaaabaaaaaaaaabmmaaaaaaaaaaaaacop
aaacaaanaaabaaaaaaaaabmmaaaaaaaaaaaaacpkaaacaaamaaabaaaaaaaaabmm
aaaaaaaaaaaaadafaaacaabbaaabaaaaaaaaabmmaaaaaaaaaaaaadbaaaacaaba
aaabaaaaaaaaabmmaaaaaaaaaaaaadblaaacaaapaaabaaaaaaaaabmmaaaaaaaa
aaaaadcgaaacaabcaaabaaaaaaaaabmmaaaaaaaaaaaaaddaaaacaabpaaabaaaa
aaaaabmmaaaaaaaafpengbgjgofegfhifpfdfeaaaaabaaadaaabaaaeaaabaaaa
aaaaaaaafpepgcgkgfgdhedcfhgphcgmgeaaklklaaadaaadaaaeaaaeaaabaaaa
aaaaaaaafpfahcgpgkgfgdhegjgpgofagbhcgbgnhdaafpfdgdhcgfgfgofagbhc
gbgnhdaafpfhgphcgmgedcepgcgkgfgdheaafpfhgphcgmgefdhagbgdgfedgbgn
gfhcgbfagphdaaklaaabaaadaaabaaadaaabaaaaaaaaaaaafpfhgphcgmgefdha
gbgdgfemgjghgihefagphddaaaghgmhdhegbhegffpgngbhehcgjhifpgnhghaaa
hfgogjhehjfpdeemgjghgiheebhehegfgodaaahfgogjhehjfpdeemgjghgihefa
gphdfidaaahfgogjhehjfpdeemgjghgihefagphdfjdaaahfgogjhehjfpdeemgj
ghgihefagphdfkdaaahfgogjhehjfpemgjghgiheedgpgmgphcaaklklaaabaaad
aaabaaaeaaaeaaaaaaaaaaaahfgogjhehjfpfdeiebgcaahfgogjhehjfpfdeieb
ghaahfgogjhehjfpfdeiebhcaahfgogjhehjfpfdeiecgcaahfgogjhehjfpfdei
ecghaahfgogjhehjfpfdeiechcaahfgogjhehjfpfdeiedaahfgogjhehjfpfdgd
gbgmgfaahghdfpddfpdaaadccodacodcdadddfddcodaaaklaaaaaaaaaaaaaaab
aaaaaaaaaaaaaaaaaaaaaabeaapmaabaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaeaaaaaadpaaaebaaamaaaaaaaaaaaaaaaaaaaadmkfaaaaaaabaaaaaaae
aaaaaaakaaaaacjaaabaaaaiaaaagaajaaaadaakaacafaalaaaadafaaaabhbfb
aaaehcfcaaafhdfdaaaipefeaaaabacpaaaaaacjaaaaaackaaaabaclaaaabafc
aaaaaacmaaaaaacnaaaabacoaaaaaaciaaaabadkaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaadpaaaaaaaaaaaaaadpiaaaaaaaaaaaaapaffeaaiaaaabcaamcaaaaaa
aaaafaamaaaabcaameaaaaaaaaaagabbgabhbcaabcaaaaaaaaaagabngacdbcaa
bcaaaaaaaaaagacjgacpbcaabcaaaaaaaaaagadfgadlbcaabcaaaaaaaaaagaeb
gaehbcaabcaaaaaaaaaagaenaaaaccaaaaaaaaaaafpibaaaaaaaaanbaaaaaaaa
afpikaaaaaaaagiiaaaaaaaaafpieaaaaaaaaoiiaaaaaaaaafpidaaaaaaaacdp
aaaaaaaamiapaaaaaamgiiaakbabbgaamiapaaaaaalbnapiklabbfaamiapaaaa
aagmdepiklabbeaamiapaaamaablnajeklabbdaamiapiadoaananaaaocamamaa
miahaaaaaaleblaacbboadaamiahaaacaamamgmaalbnaabomiahaaaiaalelble
clbmaaacmiahaaahaalogfaaobaeakaamiahaaagaamamgleclbnadaamialaaaa
aagfblaakbaebpaamiahaaacaamgleaakbabbkaamiahaaacaalbmaleklabbjac
miahaaafaalbleaakbaabjaamiahaaagaalelbleclbmadagmiahaaahabgfloma
olaeakahmiahaaaiaamagmleclblaaaimiahaaaiabmabllpklaibpabmiahaaaj
aamablaaobahakaamiahaaalaamagmleclbladagmiahaaaaaagmlemaklaabiaf
miahaaabaagmleleklabbiacmialaaabaabllemaklabbhabmiahaaahaabllema
klaabhaaceihahaaaamagmgmkbamppiaaibpadafaalehcgmobahahahaicpadag
aegmaamgkaabaeahbeapaaacaflbaablkaabagabmiamiaaeaanlnlaaocamamaa
miabiaabaaloloaapaalakaamiaciaabaaloloaapaajalaamiaeiaabaaloloaa
paalaeaamiabiaadaaloloaapaaiakaamiaciaadaaloloaapaajaiaamiaeiaad
aaloloaapaaiaeaamiadiaaaaabklabkiladcacaaebbaiaeaadoangmepamahaf
beacaaaeabdoanblgpanahabaeceaiaeaadoanlbepaoahafbeabaaababkhkhbl
kpafapabaeecaiabaakhkhmgipafbaafbeaeaaababkhkhblkpafbbabaeipaiaf
aapipiblmbacacafkiipaaacaapilbebmbacahabmiapaaafaajejepiolaiaiaf
miapaaacaajemgpiolaiahacmiadiaaeaamgbkbiklaaacaamiapaaacaajegmaa
olagahacmiapaaaaaaaaaapiolagagafgeihababaalologboaaeabadmiahaaab
aabllemnklabbcabmiapaaaeaapipimgilaaahppfibaaaaaaaaaaagmocaaaaia
ficaaaaaaaaaaalbocaaaaiafieaaaaaaaaaaamgocaaaaiafiiaaaaaaaaaaabl
ocaaaaiamiapaaaaaapiaaaaobacaaaaemipaaadaapilbmgkcaappaeemecacaa
aamgblgmobadaaaeemciacacaagmmgblobadacaeembbaaacaabllblbobadacae
miaeaaaaaalbgmaaobadaaaakibhacaeaalmmaecibacakalkiciacaeaamgblic
mbaeadalkieoacafaabgpmmaibacaialbeahaaaaaabbmalbkbaaajafambiafaa
aamgmggmobaaadadbeahaaaaaabebamgoaafaaacamihacaaaamabalboaaaaead
miahaaaaaamabaaaoaaaacaamiahiaacaalemaaaoaabaaaaaaaaaaaaaaaaaaaa
aaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" "VERTEXLIGHT_ON" }
Matrix 256 [glstate_matrix_mvp]
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 467 [_WorldSpaceCameraPos]
Vector 466 [_ProjectionParams]
Vector 465 [_WorldSpaceLightPos0]
Vector 464 [unity_4LightPosX0]
Vector 463 [unity_4LightPosY0]
Vector 462 [unity_4LightPosZ0]
Vector 461 [unity_4LightAtten0]
Vector 460 [unity_LightColor0]
Vector 459 [unity_LightColor1]
Vector 458 [unity_LightColor2]
Vector 457 [unity_LightColor3]
Vector 456 [unity_SHAr]
Vector 455 [unity_SHAg]
Vector 454 [unity_SHAb]
Vector 453 [unity_SHBr]
Vector 452 [unity_SHBg]
Vector 451 [unity_SHBb]
Vector 450 [unity_SHC]
Matrix 260 [_Object2World]
Matrix 264 [_World2Object]
Vector 449 [unity_Scale]
Vector 448 [_MainTex_ST]
"sce_vp_rsx // 68 instructions using 9 registers
[Configuration]
8
0000004441050900
[Defaults]
1
447 3
000000003f8000003f000000
[Microcode]
1088
00009c6c005d100d8186c0836041fffc00031c6c00400e0c0106c0836041dffc
00001c6c005d300c0186c0836041dffc00019c6c009c120c013fc0c36041dffc
401f9c6c011c0808010400d740619f9c00011c6c01d0300d8106c0c360403ffc
00011c6c01d0200d8106c0c360405ffc00011c6c01d0100d8106c0c360409ffc
00011c6c01d0000d8106c0c360411ffc00019c6c01d0500d8106c0c360403ffc
00021c6c01d0400d8106c0c360405ffc00001c6c01d0600d8106c0c360403ffc
00029c6c01d0a00d8286c0c360405ffc00029c6c01d0900d8286c0c360409ffc
00029c6c01d0800d8286c0c360411ffc00021c6c0150400c068600c360411ffc
00021c6c0150600c068600c360403ffc00021c6c0150500c068600c360409ffc
00039c6c0190a00c0086c0c360405ffc00039c6c0190900c0086c0c360409ffc
00039c6c0190800c0086c0c360411ffc00001c6c00dce00d8186c0bfe021fffc
00009c6c00dd000d8186c0b54221fffc00019c6c00dcf00d8186c0bfe1a1fffc
00041c6c00800243011846436041dffc00031c6c01000230812186630421dffc
401f9c6c0040000d8486c0836041ff80401f9c6c004000558486c08360407fac
00039c6c011c100c0ebfc0e30041dffc00041c6c009bf00e04aa80c36041dffc
401f9c6c0140020c0106054360405fa000011c6c0080002a8886c3436041fffc
00041c6c009d202a908000c360409ffc00019c6c0080000d8686c3436041fffc
00029c6c0080002a8895444360403ffc00021c6c0040007f8886c08360405ffc
00011c6c010000000886c1436121fffc00009c6c0100000d8286c14361a1fffc
401f9c6c00c000081086c09544219fac00041c6c019c600c0886c0c360405ffc
00041c6c019c700c0886c0c360409ffc00041c6c019c800c0886c0c360411ffc
00029c6c010000000880047fe2a03ffc00019c6c0080000d089a04436041fffc
00011c6c0100007f8886c0436121fffc00001c6c0100000d8086c04360a1fffc
00009c6c01dc300d8686c0c360405ffc00009c6c01dc400d8686c0c360409ffc
00009c6c01dc500d8686c0c360411ffc00009c6c00c0000c1086c08300a1dffc
00019c6c009c207f8a8600c36041dffc00019c6c00c0000c0686c08300a1dffc
401f9c6c21400e0c0a860080003100a000031c6c20800e0c0cbfc08aa029c0fc
00021c6c209cd00d8086c0d54025e0fc00021c6c00dbf02a8186c0836221fffc
401f9c6c2140020c0106075fe02240a8401f9c6c11400e0c0106074002310028
401f9c6c1140000c0a86064aa2288020401f9c6c1140000c0c86075542248028
00009c6c1080000d8486c15fe223e07c00009c6c029bf00d828000c36041fffc
00001c6c0080000d8286c0436041fffc00009c6c009cb02a808600c36041dffc
00009c6c011cc000008600c300a1dffc00001c6c011ca055008600c300a1dffc
00001c6c011c907f808600c30021dffc401f9c6c00c0000c0686c0830021dfa5
"
}

SubProgram "d3d11 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" "VERTEXLIGHT_ON" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 160 // 160 used size, 8 vars
Vector 144 [_MainTex_ST] 4
ConstBuffer "UnityPerCamera" 128 // 96 used size, 8 vars
Vector 64 [_WorldSpaceCameraPos] 3
Vector 80 [_ProjectionParams] 4
ConstBuffer "UnityLighting" 400 // 400 used size, 16 vars
Vector 0 [_WorldSpaceLightPos0] 4
Vector 32 [unity_4LightPosX0] 4
Vector 48 [unity_4LightPosY0] 4
Vector 64 [unity_4LightPosZ0] 4
Vector 80 [unity_4LightAtten0] 4
Vector 96 [unity_LightColor0] 4
Vector 112 [unity_LightColor1] 4
Vector 128 [unity_LightColor2] 4
Vector 144 [unity_LightColor3] 4
Vector 288 [unity_SHAr] 4
Vector 304 [unity_SHAg] 4
Vector 320 [unity_SHAb] 4
Vector 336 [unity_SHBr] 4
Vector 352 [unity_SHBg] 4
Vector 368 [unity_SHBb] 4
Vector 384 [unity_SHC] 4
ConstBuffer "UnityPerDraw" 336 // 336 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
Matrix 192 [_Object2World] 4
Matrix 256 [_World2Object] 4
Vector 320 [unity_Scale] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityLighting" 2
BindCB "UnityPerDraw" 3
// 69 instructions, 8 temp regs, 0 temp arrays:
// ALU 39 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0
eefiecedidakmggmmehbjlpebbikkkmfglkmepghabaaaaaakealaaaaadaaaaaa
cmaaaaaapeaaaaaakmabaaaaejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaa
abaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaljaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfc
enebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheolaaaaaaaagaaaaaa
aiaaaaaajiaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaakeaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaabaaaaaaadamaaaakeaaaaaaabaaaaaaaaaaaaaa
adaaaaaaacaaaaaaahaiaaaakeaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaa
ahaiaaaakeaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaahaiaaaakeaaaaaa
aeaaaaaaaaaaaaaaadaaaaaaafaaaaaaapaaaaaafdfgfpfaepfdejfeejepeoaa
feeffiedepepfceeaaklklklfdeieefcpaajaaaaeaaaabaahmacaaaafjaaaaae
egiocaaaaaaaaaaaakaaaaaafjaaaaaeegiocaaaabaaaaaaagaaaaaafjaaaaae
egiocaaaacaaaaaabjaaaaaafjaaaaaeegiocaaaadaaaaaabfaaaaaafpaaaaad
pcbabaaaaaaaaaaafpaaaaadpcbabaaaabaaaaaafpaaaaadhcbabaaaacaaaaaa
fpaaaaaddcbabaaaadaaaaaaghaaaaaepccabaaaaaaaaaaaabaaaaaagfaaaaad
dccabaaaabaaaaaagfaaaaadhccabaaaacaaaaaagfaaaaadhccabaaaadaaaaaa
gfaaaaadhccabaaaaeaaaaaagfaaaaadpccabaaaafaaaaaagiaaaaacaiaaaaaa
diaaaaaipcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaadaaaaaaabaaaaaa
dcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaaaaaaaaaagbabaaaaaaaaaaa
egaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaacaaaaaa
kgbkbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaa
adaaaaaaadaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadgaaaaafpccabaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaaldccabaaaabaaaaaaegbabaaaadaaaaaa
egiacaaaaaaaaaaaajaaaaaaogikcaaaaaaaaaaaajaaaaaadiaaaaahhcaabaaa
abaaaaaajgbebaaaabaaaaaacgbjbaaaacaaaaaadcaaaaakhcaabaaaabaaaaaa
jgbebaaaacaaaaaacgbjbaaaabaaaaaaegacbaiaebaaaaaaabaaaaaadiaaaaah
hcaabaaaabaaaaaaegacbaaaabaaaaaapgbpbaaaabaaaaaadiaaaaajhcaabaaa
acaaaaaafgifcaaaacaaaaaaaaaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaal
hcaabaaaacaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaaacaaaaaaaaaaaaaa
egacbaaaacaaaaaadcaaaaalhcaabaaaacaaaaaaegiccaaaadaaaaaabcaaaaaa
kgikcaaaacaaaaaaaaaaaaaaegacbaaaacaaaaaadcaaaaalhcaabaaaacaaaaaa
egiccaaaadaaaaaabdaaaaaapgipcaaaacaaaaaaaaaaaaaaegacbaaaacaaaaaa
baaaaaahcccabaaaacaaaaaaegacbaaaabaaaaaaegacbaaaacaaaaaabaaaaaah
bccabaaaacaaaaaaegbcbaaaabaaaaaaegacbaaaacaaaaaabaaaaaaheccabaaa
acaaaaaaegbcbaaaacaaaaaaegacbaaaacaaaaaadgaaaaaficaabaaaacaaaaaa
abeaaaaaaaaaiadpdiaaaaaihcaabaaaadaaaaaaegbcbaaaacaaaaaapgipcaaa
adaaaaaabeaaaaaadiaaaaaihcaabaaaaeaaaaaafgafbaaaadaaaaaaegiccaaa
adaaaaaaanaaaaaadcaaaaaklcaabaaaadaaaaaaegiicaaaadaaaaaaamaaaaaa
agaabaaaadaaaaaaegaibaaaaeaaaaaadcaaaaakhcaabaaaacaaaaaaegiccaaa
adaaaaaaaoaaaaaakgakbaaaadaaaaaaegadbaaaadaaaaaabbaaaaaibcaabaaa
adaaaaaaegiocaaaacaaaaaabcaaaaaaegaobaaaacaaaaaabbaaaaaiccaabaaa
adaaaaaaegiocaaaacaaaaaabdaaaaaaegaobaaaacaaaaaabbaaaaaiecaabaaa
adaaaaaaegiocaaaacaaaaaabeaaaaaaegaobaaaacaaaaaadiaaaaahpcaabaaa
aeaaaaaajgacbaaaacaaaaaaegakbaaaacaaaaaabbaaaaaibcaabaaaafaaaaaa
egiocaaaacaaaaaabfaaaaaaegaobaaaaeaaaaaabbaaaaaiccaabaaaafaaaaaa
egiocaaaacaaaaaabgaaaaaaegaobaaaaeaaaaaabbaaaaaiecaabaaaafaaaaaa
egiocaaaacaaaaaabhaaaaaaegaobaaaaeaaaaaaaaaaaaahhcaabaaaadaaaaaa
egacbaaaadaaaaaaegacbaaaafaaaaaadiaaaaahicaabaaaabaaaaaabkaabaaa
acaaaaaabkaabaaaacaaaaaadcaaaaakicaabaaaabaaaaaaakaabaaaacaaaaaa
akaabaaaacaaaaaadkaabaiaebaaaaaaabaaaaaadcaaaaakhcaabaaaadaaaaaa
egiccaaaacaaaaaabiaaaaaapgapbaaaabaaaaaaegacbaaaadaaaaaadiaaaaai
hcaabaaaaeaaaaaafgbfbaaaaaaaaaaaegiccaaaadaaaaaaanaaaaaadcaaaaak
hcaabaaaaeaaaaaaegiccaaaadaaaaaaamaaaaaaagbabaaaaaaaaaaaegacbaaa
aeaaaaaadcaaaaakhcaabaaaaeaaaaaaegiccaaaadaaaaaaaoaaaaaakgbkbaaa
aaaaaaaaegacbaaaaeaaaaaadcaaaaakhcaabaaaaeaaaaaaegiccaaaadaaaaaa
apaaaaaapgbpbaaaaaaaaaaaegacbaaaaeaaaaaaaaaaaaajpcaabaaaafaaaaaa
fgafbaiaebaaaaaaaeaaaaaaegiocaaaacaaaaaaadaaaaaadiaaaaahpcaabaaa
agaaaaaafgafbaaaacaaaaaaegaobaaaafaaaaaadiaaaaahpcaabaaaafaaaaaa
egaobaaaafaaaaaaegaobaaaafaaaaaaaaaaaaajpcaabaaaahaaaaaaagaabaia
ebaaaaaaaeaaaaaaegiocaaaacaaaaaaacaaaaaaaaaaaaajpcaabaaaaeaaaaaa
kgakbaiaebaaaaaaaeaaaaaaegiocaaaacaaaaaaaeaaaaaadcaaaaajpcaabaaa
agaaaaaaegaobaaaahaaaaaaagaabaaaacaaaaaaegaobaaaagaaaaaadcaaaaaj
pcaabaaaacaaaaaaegaobaaaaeaaaaaakgakbaaaacaaaaaaegaobaaaagaaaaaa
dcaaaaajpcaabaaaafaaaaaaegaobaaaahaaaaaaegaobaaaahaaaaaaegaobaaa
afaaaaaadcaaaaajpcaabaaaaeaaaaaaegaobaaaaeaaaaaaegaobaaaaeaaaaaa
egaobaaaafaaaaaaeeaaaaafpcaabaaaafaaaaaaegaobaaaaeaaaaaadcaaaaan
pcaabaaaaeaaaaaaegaobaaaaeaaaaaaegiocaaaacaaaaaaafaaaaaaaceaaaaa
aaaaiadpaaaaiadpaaaaiadpaaaaiadpaoaaaaakpcaabaaaaeaaaaaaaceaaaaa
aaaaiadpaaaaiadpaaaaiadpaaaaiadpegaobaaaaeaaaaaadiaaaaahpcaabaaa
acaaaaaaegaobaaaacaaaaaaegaobaaaafaaaaaadeaaaaakpcaabaaaacaaaaaa
egaobaaaacaaaaaaaceaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaadiaaaaah
pcaabaaaacaaaaaaegaobaaaaeaaaaaaegaobaaaacaaaaaadiaaaaaihcaabaaa
aeaaaaaafgafbaaaacaaaaaaegiccaaaacaaaaaaahaaaaaadcaaaaakhcaabaaa
aeaaaaaaegiccaaaacaaaaaaagaaaaaaagaabaaaacaaaaaaegacbaaaaeaaaaaa
dcaaaaakhcaabaaaacaaaaaaegiccaaaacaaaaaaaiaaaaaakgakbaaaacaaaaaa
egacbaaaaeaaaaaadcaaaaakhcaabaaaacaaaaaaegiccaaaacaaaaaaajaaaaaa
pgapbaaaacaaaaaaegacbaaaacaaaaaaaaaaaaahhccabaaaadaaaaaaegacbaaa
acaaaaaaegacbaaaadaaaaaadiaaaaajhcaabaaaacaaaaaafgifcaaaabaaaaaa
aeaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaaacaaaaaaegiccaaa
adaaaaaabaaaaaaaagiacaaaabaaaaaaaeaaaaaaegacbaaaacaaaaaadcaaaaal
hcaabaaaacaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaaabaaaaaaaeaaaaaa
egacbaaaacaaaaaaaaaaaaaihcaabaaaacaaaaaaegacbaaaacaaaaaaegiccaaa
adaaaaaabdaaaaaadcaaaaalhcaabaaaacaaaaaaegacbaaaacaaaaaapgipcaaa
adaaaaaabeaaaaaaegbcbaiaebaaaaaaaaaaaaaabaaaaaahcccabaaaaeaaaaaa
egacbaaaabaaaaaaegacbaaaacaaaaaabaaaaaahbccabaaaaeaaaaaaegbcbaaa
abaaaaaaegacbaaaacaaaaaabaaaaaaheccabaaaaeaaaaaaegbcbaaaacaaaaaa
egacbaaaacaaaaaadiaaaaaiccaabaaaaaaaaaaabkaabaaaaaaaaaaaakiacaaa
abaaaaaaafaaaaaadiaaaaakncaabaaaabaaaaaaagahbaaaaaaaaaaaaceaaaaa
aaaaaadpaaaaaaaaaaaaaadpaaaaaadpdgaaaaafmccabaaaafaaaaaakgaobaaa
aaaaaaaaaaaaaaahdccabaaaafaaaaaakgakbaaaabaaaaaamgaabaaaabaaaaaa
doaaaaab"
}

SubProgram "gles " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" "VERTEXLIGHT_ON" }
"!!GLES
#define SHADER_API_GLES 1
#define tex2D texture2D


#ifdef VERTEX
#define gl_ModelViewProjectionMatrix glstate_matrix_mvp
uniform mat4 glstate_matrix_mvp;

varying highp vec4 xlv_TEXCOORD4;
varying highp vec3 xlv_TEXCOORD3;
varying lowp vec3 xlv_TEXCOORD2;
varying lowp vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 unity_Scale;
uniform highp mat4 _World2Object;
uniform highp mat4 _Object2World;

uniform highp mat4 unity_World2Shadow[4];
uniform highp vec4 unity_SHC;
uniform highp vec4 unity_SHBb;
uniform highp vec4 unity_SHBg;
uniform highp vec4 unity_SHBr;
uniform highp vec4 unity_SHAb;
uniform highp vec4 unity_SHAg;
uniform highp vec4 unity_SHAr;
uniform highp vec4 unity_LightColor[4];
uniform highp vec4 unity_4LightAtten0;
uniform highp vec4 unity_4LightPosZ0;
uniform highp vec4 unity_4LightPosY0;
uniform highp vec4 unity_4LightPosX0;
uniform lowp vec4 _WorldSpaceLightPos0;
uniform highp vec3 _WorldSpaceCameraPos;
attribute vec4 _glesTANGENT;
attribute vec4 _glesMultiTexCoord0;
attribute vec3 _glesNormal;
attribute vec4 _glesVertex;
void main ()
{
  vec4 tmpvar_1;
  tmpvar_1.xyz = normalize(_glesTANGENT.xyz);
  tmpvar_1.w = _glesTANGENT.w;
  vec3 tmpvar_2;
  tmpvar_2 = normalize(_glesNormal);
  highp vec3 shlight_3;
  lowp vec3 tmpvar_4;
  lowp vec3 tmpvar_5;
  mat3 tmpvar_6;
  tmpvar_6[0] = _Object2World[0].xyz;
  tmpvar_6[1] = _Object2World[1].xyz;
  tmpvar_6[2] = _Object2World[2].xyz;
  highp vec3 tmpvar_7;
  tmpvar_7 = (tmpvar_6 * (tmpvar_2 * unity_Scale.w));
  highp vec3 tmpvar_8;
  highp vec3 tmpvar_9;
  tmpvar_8 = tmpvar_1.xyz;
  tmpvar_9 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_10;
  tmpvar_10[0].x = tmpvar_8.x;
  tmpvar_10[0].y = tmpvar_9.x;
  tmpvar_10[0].z = tmpvar_2.x;
  tmpvar_10[1].x = tmpvar_8.y;
  tmpvar_10[1].y = tmpvar_9.y;
  tmpvar_10[1].z = tmpvar_2.y;
  tmpvar_10[2].x = tmpvar_8.z;
  tmpvar_10[2].y = tmpvar_9.z;
  tmpvar_10[2].z = tmpvar_2.z;
  highp vec3 tmpvar_11;
  tmpvar_11 = (tmpvar_10 * (_World2Object * _WorldSpaceLightPos0).xyz);
  tmpvar_4 = tmpvar_11;
  highp vec4 tmpvar_12;
  tmpvar_12.w = 1.0;
  tmpvar_12.xyz = _WorldSpaceCameraPos;
  highp vec4 tmpvar_13;
  tmpvar_13.w = 1.0;
  tmpvar_13.xyz = tmpvar_7;
  mediump vec3 tmpvar_14;
  mediump vec4 normal_15;
  normal_15 = tmpvar_13;
  highp float vC_16;
  mediump vec3 x3_17;
  mediump vec3 x2_18;
  mediump vec3 x1_19;
  highp float tmpvar_20;
  tmpvar_20 = dot (unity_SHAr, normal_15);
  x1_19.x = tmpvar_20;
  highp float tmpvar_21;
  tmpvar_21 = dot (unity_SHAg, normal_15);
  x1_19.y = tmpvar_21;
  highp float tmpvar_22;
  tmpvar_22 = dot (unity_SHAb, normal_15);
  x1_19.z = tmpvar_22;
  mediump vec4 tmpvar_23;
  tmpvar_23 = (normal_15.xyzz * normal_15.yzzx);
  highp float tmpvar_24;
  tmpvar_24 = dot (unity_SHBr, tmpvar_23);
  x2_18.x = tmpvar_24;
  highp float tmpvar_25;
  tmpvar_25 = dot (unity_SHBg, tmpvar_23);
  x2_18.y = tmpvar_25;
  highp float tmpvar_26;
  tmpvar_26 = dot (unity_SHBb, tmpvar_23);
  x2_18.z = tmpvar_26;
  mediump float tmpvar_27;
  tmpvar_27 = ((normal_15.x * normal_15.x) - (normal_15.y * normal_15.y));
  vC_16 = tmpvar_27;
  highp vec3 tmpvar_28;
  tmpvar_28 = (unity_SHC.xyz * vC_16);
  x3_17 = tmpvar_28;
  tmpvar_14 = ((x1_19 + x2_18) + x3_17);
  shlight_3 = tmpvar_14;
  tmpvar_5 = shlight_3;
  highp vec3 tmpvar_29;
  tmpvar_29 = (_Object2World * _glesVertex).xyz;
  highp vec4 tmpvar_30;
  tmpvar_30 = (unity_4LightPosX0 - tmpvar_29.x);
  highp vec4 tmpvar_31;
  tmpvar_31 = (unity_4LightPosY0 - tmpvar_29.y);
  highp vec4 tmpvar_32;
  tmpvar_32 = (unity_4LightPosZ0 - tmpvar_29.z);
  highp vec4 tmpvar_33;
  tmpvar_33 = (((tmpvar_30 * tmpvar_30) + (tmpvar_31 * tmpvar_31)) + (tmpvar_32 * tmpvar_32));
  highp vec4 tmpvar_34;
  tmpvar_34 = (max (vec4(0.0, 0.0, 0.0, 0.0), ((((tmpvar_30 * tmpvar_7.x) + (tmpvar_31 * tmpvar_7.y)) + (tmpvar_32 * tmpvar_7.z)) * inversesqrt(tmpvar_33))) * (1.0/((1.0 + (tmpvar_33 * unity_4LightAtten0)))));
  highp vec3 tmpvar_35;
  tmpvar_35 = (tmpvar_5 + ((((unity_LightColor[0].xyz * tmpvar_34.x) + (unity_LightColor[1].xyz * tmpvar_34.y)) + (unity_LightColor[2].xyz * tmpvar_34.z)) + (unity_LightColor[3].xyz * tmpvar_34.w)));
  tmpvar_5 = tmpvar_35;
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = tmpvar_4;
  xlv_TEXCOORD2 = tmpvar_5;
  xlv_TEXCOORD3 = (tmpvar_10 * (((_World2Object * tmpvar_12).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD4 = (unity_World2Shadow[0] * (_Object2World * _glesVertex));
}



#endif
#ifdef FRAGMENT

varying highp vec4 xlv_TEXCOORD4;
varying highp vec3 xlv_TEXCOORD3;
varying lowp vec3 xlv_TEXCOORD2;
varying lowp vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform mediump float _Gloss;
uniform mediump float _Shininess;
uniform lowp vec4 _Color;
uniform sampler2D _SpecMap;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform sampler2D _ShadowMapTexture;
uniform lowp vec4 _LightColor0;
uniform highp vec4 _LightShadowData;
void main ()
{
  lowp vec4 c_1;
  mediump vec3 tmpvar_2;
  mediump vec3 tmpvar_3;
  lowp vec4 tmpvar_4;
  tmpvar_4 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_5;
  tmpvar_5 = (tmpvar_4.xyz * _Color.xyz);
  tmpvar_2 = tmpvar_5;
  lowp vec4 tmpvar_6;
  tmpvar_6 = texture2D (_SpecMap, xlv_TEXCOORD0);
  mediump vec3 tmpvar_7;
  tmpvar_7 = ((tmpvar_4.w * tmpvar_6.xyz) * _Gloss);
  lowp vec3 tmpvar_8;
  tmpvar_8 = ((texture2D (_BumpMap, xlv_TEXCOORD0).xyz * 2.0) - 1.0);
  tmpvar_3 = tmpvar_8;
  lowp float tmpvar_9;
  mediump float lightShadowDataX_10;
  highp float dist_11;
  lowp float tmpvar_12;
  tmpvar_12 = texture2DProj (_ShadowMapTexture, xlv_TEXCOORD4).x;
  dist_11 = tmpvar_12;
  highp float tmpvar_13;
  tmpvar_13 = _LightShadowData.x;
  lightShadowDataX_10 = tmpvar_13;
  highp float tmpvar_14;
  tmpvar_14 = max (float((dist_11 > (xlv_TEXCOORD4.z / xlv_TEXCOORD4.w))), lightShadowDataX_10);
  tmpvar_9 = tmpvar_14;
  highp vec3 tmpvar_15;
  tmpvar_15 = normalize(xlv_TEXCOORD3);
  mediump vec3 lightDir_16;
  lightDir_16 = xlv_TEXCOORD1;
  mediump vec3 viewDir_17;
  viewDir_17 = tmpvar_15;
  mediump float atten_18;
  atten_18 = tmpvar_9;
  mediump vec4 c_19;
  mediump vec3 specCol_20;
  highp float nh_21;
  mediump float tmpvar_22;
  tmpvar_22 = max (0.0, dot (tmpvar_3, normalize((lightDir_16 + viewDir_17))));
  nh_21 = tmpvar_22;
  mediump float arg1_23;
  arg1_23 = (32.0 * _Shininess);
  highp vec3 tmpvar_24;
  tmpvar_24 = (pow (nh_21, arg1_23) * tmpvar_7);
  specCol_20 = tmpvar_24;
  c_19.xyz = ((((tmpvar_2 * _LightColor0.xyz) * max (0.0, dot (tmpvar_3, lightDir_16))) + (_LightColor0.xyz * specCol_20)) * (atten_18 * 2.0));
  c_19.w = 0.0;
  c_1 = c_19;
  mediump vec3 tmpvar_25;
  tmpvar_25 = (c_1.xyz + (tmpvar_2 * xlv_TEXCOORD2));
  c_1.xyz = tmpvar_25;
  gl_FragData[0] = c_1;
}



#endif"
}

SubProgram "glesdesktop " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" "VERTEXLIGHT_ON" }
"!!GLES
#define SHADER_API_GLES 1
#define tex2D texture2D


#ifdef VERTEX
#define gl_ModelViewProjectionMatrix glstate_matrix_mvp
uniform mat4 glstate_matrix_mvp;

varying highp vec4 xlv_TEXCOORD4;
varying highp vec3 xlv_TEXCOORD3;
varying lowp vec3 xlv_TEXCOORD2;
varying lowp vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 unity_Scale;
uniform highp mat4 _World2Object;
uniform highp mat4 _Object2World;

uniform highp vec4 unity_SHC;
uniform highp vec4 unity_SHBb;
uniform highp vec4 unity_SHBg;
uniform highp vec4 unity_SHBr;
uniform highp vec4 unity_SHAb;
uniform highp vec4 unity_SHAg;
uniform highp vec4 unity_SHAr;
uniform highp vec4 unity_LightColor[4];
uniform highp vec4 unity_4LightAtten0;
uniform highp vec4 unity_4LightPosZ0;
uniform highp vec4 unity_4LightPosY0;
uniform highp vec4 unity_4LightPosX0;
uniform lowp vec4 _WorldSpaceLightPos0;
uniform highp vec4 _ProjectionParams;
uniform highp vec3 _WorldSpaceCameraPos;
attribute vec4 _glesTANGENT;
attribute vec4 _glesMultiTexCoord0;
attribute vec3 _glesNormal;
attribute vec4 _glesVertex;
void main ()
{
  vec4 tmpvar_1;
  tmpvar_1.xyz = normalize(_glesTANGENT.xyz);
  tmpvar_1.w = _glesTANGENT.w;
  vec3 tmpvar_2;
  tmpvar_2 = normalize(_glesNormal);
  highp vec3 shlight_3;
  lowp vec3 tmpvar_4;
  lowp vec3 tmpvar_5;
  highp vec4 tmpvar_6;
  tmpvar_6 = (gl_ModelViewProjectionMatrix * _glesVertex);
  mat3 tmpvar_7;
  tmpvar_7[0] = _Object2World[0].xyz;
  tmpvar_7[1] = _Object2World[1].xyz;
  tmpvar_7[2] = _Object2World[2].xyz;
  highp vec3 tmpvar_8;
  tmpvar_8 = (tmpvar_7 * (tmpvar_2 * unity_Scale.w));
  highp vec3 tmpvar_9;
  highp vec3 tmpvar_10;
  tmpvar_9 = tmpvar_1.xyz;
  tmpvar_10 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_11;
  tmpvar_11[0].x = tmpvar_9.x;
  tmpvar_11[0].y = tmpvar_10.x;
  tmpvar_11[0].z = tmpvar_2.x;
  tmpvar_11[1].x = tmpvar_9.y;
  tmpvar_11[1].y = tmpvar_10.y;
  tmpvar_11[1].z = tmpvar_2.y;
  tmpvar_11[2].x = tmpvar_9.z;
  tmpvar_11[2].y = tmpvar_10.z;
  tmpvar_11[2].z = tmpvar_2.z;
  highp vec3 tmpvar_12;
  tmpvar_12 = (tmpvar_11 * (_World2Object * _WorldSpaceLightPos0).xyz);
  tmpvar_4 = tmpvar_12;
  highp vec4 tmpvar_13;
  tmpvar_13.w = 1.0;
  tmpvar_13.xyz = _WorldSpaceCameraPos;
  highp vec4 tmpvar_14;
  tmpvar_14.w = 1.0;
  tmpvar_14.xyz = tmpvar_8;
  mediump vec3 tmpvar_15;
  mediump vec4 normal_16;
  normal_16 = tmpvar_14;
  highp float vC_17;
  mediump vec3 x3_18;
  mediump vec3 x2_19;
  mediump vec3 x1_20;
  highp float tmpvar_21;
  tmpvar_21 = dot (unity_SHAr, normal_16);
  x1_20.x = tmpvar_21;
  highp float tmpvar_22;
  tmpvar_22 = dot (unity_SHAg, normal_16);
  x1_20.y = tmpvar_22;
  highp float tmpvar_23;
  tmpvar_23 = dot (unity_SHAb, normal_16);
  x1_20.z = tmpvar_23;
  mediump vec4 tmpvar_24;
  tmpvar_24 = (normal_16.xyzz * normal_16.yzzx);
  highp float tmpvar_25;
  tmpvar_25 = dot (unity_SHBr, tmpvar_24);
  x2_19.x = tmpvar_25;
  highp float tmpvar_26;
  tmpvar_26 = dot (unity_SHBg, tmpvar_24);
  x2_19.y = tmpvar_26;
  highp float tmpvar_27;
  tmpvar_27 = dot (unity_SHBb, tmpvar_24);
  x2_19.z = tmpvar_27;
  mediump float tmpvar_28;
  tmpvar_28 = ((normal_16.x * normal_16.x) - (normal_16.y * normal_16.y));
  vC_17 = tmpvar_28;
  highp vec3 tmpvar_29;
  tmpvar_29 = (unity_SHC.xyz * vC_17);
  x3_18 = tmpvar_29;
  tmpvar_15 = ((x1_20 + x2_19) + x3_18);
  shlight_3 = tmpvar_15;
  tmpvar_5 = shlight_3;
  highp vec3 tmpvar_30;
  tmpvar_30 = (_Object2World * _glesVertex).xyz;
  highp vec4 tmpvar_31;
  tmpvar_31 = (unity_4LightPosX0 - tmpvar_30.x);
  highp vec4 tmpvar_32;
  tmpvar_32 = (unity_4LightPosY0 - tmpvar_30.y);
  highp vec4 tmpvar_33;
  tmpvar_33 = (unity_4LightPosZ0 - tmpvar_30.z);
  highp vec4 tmpvar_34;
  tmpvar_34 = (((tmpvar_31 * tmpvar_31) + (tmpvar_32 * tmpvar_32)) + (tmpvar_33 * tmpvar_33));
  highp vec4 tmpvar_35;
  tmpvar_35 = (max (vec4(0.0, 0.0, 0.0, 0.0), ((((tmpvar_31 * tmpvar_8.x) + (tmpvar_32 * tmpvar_8.y)) + (tmpvar_33 * tmpvar_8.z)) * inversesqrt(tmpvar_34))) * (1.0/((1.0 + (tmpvar_34 * unity_4LightAtten0)))));
  highp vec3 tmpvar_36;
  tmpvar_36 = (tmpvar_5 + ((((unity_LightColor[0].xyz * tmpvar_35.x) + (unity_LightColor[1].xyz * tmpvar_35.y)) + (unity_LightColor[2].xyz * tmpvar_35.z)) + (unity_LightColor[3].xyz * tmpvar_35.w)));
  tmpvar_5 = tmpvar_36;
  highp vec4 o_37;
  highp vec4 tmpvar_38;
  tmpvar_38 = (tmpvar_6 * 0.5);
  highp vec2 tmpvar_39;
  tmpvar_39.x = tmpvar_38.x;
  tmpvar_39.y = (tmpvar_38.y * _ProjectionParams.x);
  o_37.xy = (tmpvar_39 + tmpvar_38.w);
  o_37.zw = tmpvar_6.zw;
  gl_Position = tmpvar_6;
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = tmpvar_4;
  xlv_TEXCOORD2 = tmpvar_5;
  xlv_TEXCOORD3 = (tmpvar_11 * (((_World2Object * tmpvar_13).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD4 = o_37;
}



#endif
#ifdef FRAGMENT

varying highp vec4 xlv_TEXCOORD4;
varying highp vec3 xlv_TEXCOORD3;
varying lowp vec3 xlv_TEXCOORD2;
varying lowp vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform mediump float _Gloss;
uniform mediump float _Shininess;
uniform lowp vec4 _Color;
uniform sampler2D _SpecMap;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform sampler2D _ShadowMapTexture;
uniform lowp vec4 _LightColor0;
void main ()
{
  lowp vec4 c_1;
  mediump vec3 tmpvar_2;
  mediump vec3 tmpvar_3;
  lowp vec4 tmpvar_4;
  tmpvar_4 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_5;
  tmpvar_5 = (tmpvar_4.xyz * _Color.xyz);
  tmpvar_2 = tmpvar_5;
  lowp vec4 tmpvar_6;
  tmpvar_6 = texture2D (_SpecMap, xlv_TEXCOORD0);
  mediump vec3 tmpvar_7;
  tmpvar_7 = ((tmpvar_4.w * tmpvar_6.xyz) * _Gloss);
  lowp vec3 normal_8;
  normal_8.xy = ((texture2D (_BumpMap, xlv_TEXCOORD0).wy * 2.0) - 1.0);
  normal_8.z = sqrt(((1.0 - (normal_8.x * normal_8.x)) - (normal_8.y * normal_8.y)));
  tmpvar_3 = normal_8;
  lowp float tmpvar_9;
  tmpvar_9 = texture2DProj (_ShadowMapTexture, xlv_TEXCOORD4).x;
  highp vec3 tmpvar_10;
  tmpvar_10 = normalize(xlv_TEXCOORD3);
  mediump vec3 lightDir_11;
  lightDir_11 = xlv_TEXCOORD1;
  mediump vec3 viewDir_12;
  viewDir_12 = tmpvar_10;
  mediump float atten_13;
  atten_13 = tmpvar_9;
  mediump vec4 c_14;
  mediump vec3 specCol_15;
  highp float nh_16;
  mediump float tmpvar_17;
  tmpvar_17 = max (0.0, dot (tmpvar_3, normalize((lightDir_11 + viewDir_12))));
  nh_16 = tmpvar_17;
  mediump float arg1_18;
  arg1_18 = (32.0 * _Shininess);
  highp vec3 tmpvar_19;
  tmpvar_19 = (pow (nh_16, arg1_18) * tmpvar_7);
  specCol_15 = tmpvar_19;
  c_14.xyz = ((((tmpvar_2 * _LightColor0.xyz) * max (0.0, dot (tmpvar_3, lightDir_11))) + (_LightColor0.xyz * specCol_15)) * (atten_13 * 2.0));
  c_14.w = 0.0;
  c_1 = c_14;
  mediump vec3 tmpvar_20;
  tmpvar_20 = (c_1.xyz + (tmpvar_2 * xlv_TEXCOORD2));
  c_1.xyz = tmpvar_20;
  gl_FragData[0] = c_1;
}



#endif"
}

SubProgram "flash " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" "VERTEXLIGHT_ON" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Matrix 0 [glstate_matrix_mvp]
Vector 12 [_WorldSpaceCameraPos]
Vector 13 [_ProjectionParams]
Vector 14 [_WorldSpaceLightPos0]
Vector 15 [unity_4LightPosX0]
Vector 16 [unity_4LightPosY0]
Vector 17 [unity_4LightPosZ0]
Vector 18 [unity_4LightAtten0]
Vector 19 [unity_LightColor0]
Vector 20 [unity_LightColor1]
Vector 21 [unity_LightColor2]
Vector 22 [unity_LightColor3]
Vector 23 [unity_SHAr]
Vector 24 [unity_SHAg]
Vector 25 [unity_SHAb]
Vector 26 [unity_SHBr]
Vector 27 [unity_SHBg]
Vector 28 [unity_SHBb]
Vector 29 [unity_SHC]
Matrix 4 [_Object2World]
Matrix 8 [_World2Object]
Vector 30 [unity_Scale]
Vector 31 [unity_NPOTScale]
Vector 32 [_MainTex_ST]
"agal_vs
c33 1.0 0.0 0.5 0.0
[bc]
adaaaaaaadaaahacabaaaaoeaaaaaaaaboaaaappabaaaaaa mul r3.xyz, a1, c30.w
bdaaaaaaaaaaabacaaaaaaoeaaaaaaaaafaaaaoeabaaaaaa dp4 r0.x, a0, c5
bfaaaaaaabaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa neg r1.x, r0.x
abaaaaaaabaaapacabaaaaaaacaaaaaabaaaaaoeabaaaaaa add r1, r1.x, c16
bcaaaaaaadaaaiacadaaaakeacaaaaaaafaaaaoeabaaaaaa dp3 r3.w, r3.xyzz, c5
bcaaaaaaaeaaabacadaaaakeacaaaaaaaeaaaaoeabaaaaaa dp3 r4.x, r3.xyzz, c4
bcaaaaaaadaaabacadaaaakeacaaaaaaagaaaaoeabaaaaaa dp3 r3.x, r3.xyzz, c6
adaaaaaaacaaapacadaaaappacaaaaaaabaaaaoeacaaaaaa mul r2, r3.w, r1
bdaaaaaaaaaaabacaaaaaaoeaaaaaaaaaeaaaaoeabaaaaaa dp4 r0.x, a0, c4
bfaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa neg r0.x, r0.x
abaaaaaaaaaaapacaaaaaaaaacaaaaaaapaaaaoeabaaaaaa add r0, r0.x, c15
adaaaaaaabaaapacabaaaaoeacaaaaaaabaaaaoeacaaaaaa mul r1, r1, r1
aaaaaaaaaeaaaeacadaaaaaaacaaaaaaaaaaaaaaaaaaaaaa mov r4.z, r3.x
adaaaaaaafaaapacaeaaaaaaacaaaaaaaaaaaaoeacaaaaaa mul r5, r4.x, r0
abaaaaaaacaaapacafaaaaoeacaaaaaaacaaaaoeacaaaaaa add r2, r5, r2
aaaaaaaaaeaaaiaccbaaaaaaabaaaaaaaaaaaaaaaaaaaaaa mov r4.w, c33.x
bdaaaaaaaeaaacacaaaaaaoeaaaaaaaaagaaaaoeabaaaaaa dp4 r4.y, a0, c6
adaaaaaaafaaapacaaaaaaoeacaaaaaaaaaaaaoeacaaaaaa mul r5, r0, r0
abaaaaaaabaaapacafaaaaoeacaaaaaaabaaaaoeacaaaaaa add r1, r5, r1
bfaaaaaaaaaaacacaeaaaaffacaaaaaaaaaaaaaaaaaaaaaa neg r0.y, r4.y
abaaaaaaaaaaapacaaaaaaffacaaaaaabbaaaaoeabaaaaaa add r0, r0.y, c17
adaaaaaaafaaapacaaaaaaoeacaaaaaaaaaaaaoeacaaaaaa mul r5, r0, r0
abaaaaaaabaaapacafaaaaoeacaaaaaaabaaaaoeacaaaaaa add r1, r5, r1
adaaaaaaaaaaapacadaaaaaaacaaaaaaaaaaaaoeacaaaaaa mul r0, r3.x, r0
abaaaaaaaaaaapacaaaaaaoeacaaaaaaacaaaaoeacaaaaaa add r0, r0, r2
adaaaaaaacaaapacabaaaaoeacaaaaaabcaaaaoeabaaaaaa mul r2, r1, c18
aaaaaaaaaeaaacacadaaaappacaaaaaaaaaaaaaaaaaaaaaa mov r4.y, r3.w
akaaaaaaabaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r1.x, r1.x
akaaaaaaabaaacacabaaaaffacaaaaaaaaaaaaaaaaaaaaaa rsq r1.y, r1.y
akaaaaaaabaaaiacabaaaappacaaaaaaaaaaaaaaaaaaaaaa rsq r1.w, r1.w
akaaaaaaabaaaeacabaaaakkacaaaaaaaaaaaaaaaaaaaaaa rsq r1.z, r1.z
adaaaaaaaaaaapacaaaaaaoeacaaaaaaabaaaaoeacaaaaaa mul r0, r0, r1
abaaaaaaabaaapacacaaaaoeacaaaaaacbaaaaaaabaaaaaa add r1, r2, c33.x
bdaaaaaaacaaaeacaeaaaaoeacaaaaaabjaaaaoeabaaaaaa dp4 r2.z, r4, c25
bdaaaaaaacaaacacaeaaaaoeacaaaaaabiaaaaoeabaaaaaa dp4 r2.y, r4, c24
bdaaaaaaacaaabacaeaaaaoeacaaaaaabhaaaaoeabaaaaaa dp4 r2.x, r4, c23
afaaaaaaabaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rcp r1.x, r1.x
afaaaaaaabaaacacabaaaaffacaaaaaaaaaaaaaaaaaaaaaa rcp r1.y, r1.y
afaaaaaaabaaaiacabaaaappacaaaaaaaaaaaaaaaaaaaaaa rcp r1.w, r1.w
afaaaaaaabaaaeacabaaaakkacaaaaaaaaaaaaaaaaaaaaaa rcp r1.z, r1.z
ahaaaaaaaaaaapacaaaaaaoeacaaaaaacbaaaaffabaaaaaa max r0, r0, c33.y
adaaaaaaaaaaapacaaaaaaoeacaaaaaaabaaaaoeacaaaaaa mul r0, r0, r1
adaaaaaaabaaahacaaaaaaffacaaaaaabeaaaaoeabaaaaaa mul r1.xyz, r0.y, c20
adaaaaaaafaaahacaaaaaaaaacaaaaaabdaaaaoeabaaaaaa mul r5.xyz, r0.x, c19
abaaaaaaabaaahacafaaaakeacaaaaaaabaaaakeacaaaaaa add r1.xyz, r5.xyzz, r1.xyzz
adaaaaaaaaaaahacaaaaaakkacaaaaaabfaaaaoeabaaaaaa mul r0.xyz, r0.z, c21
abaaaaaaaaaaahacaaaaaakeacaaaaaaabaaaakeacaaaaaa add r0.xyz, r0.xyzz, r1.xyzz
adaaaaaaabaaahacaaaaaappacaaaaaabgaaaaoeabaaaaaa mul r1.xyz, r0.w, c22
abaaaaaaabaaahacabaaaakeacaaaaaaaaaaaakeacaaaaaa add r1.xyz, r1.xyzz, r0.xyzz
adaaaaaaaaaaapacaeaaaakeacaaaaaaaeaaaacjacaaaaaa mul r0, r4.xyzz, r4.yzzx
adaaaaaaabaaaiacadaaaappacaaaaaaadaaaappacaaaaaa mul r1.w, r3.w, r3.w
bdaaaaaaadaaaeacaaaaaaoeacaaaaaabmaaaaoeabaaaaaa dp4 r3.z, r0, c28
bdaaaaaaadaaacacaaaaaaoeacaaaaaablaaaaoeabaaaaaa dp4 r3.y, r0, c27
bdaaaaaaadaaabacaaaaaaoeacaaaaaabkaaaaoeabaaaaaa dp4 r3.x, r0, c26
adaaaaaaafaaaiacaeaaaaaaacaaaaaaaeaaaaaaacaaaaaa mul r5.w, r4.x, r4.x
acaaaaaaabaaaiacafaaaappacaaaaaaabaaaappacaaaaaa sub r1.w, r5.w, r1.w
adaaaaaaaaaaahacabaaaappacaaaaaabnaaaaoeabaaaaaa mul r0.xyz, r1.w, c29
abaaaaaaacaaahacacaaaakeacaaaaaaadaaaakeacaaaaaa add r2.xyz, r2.xyzz, r3.xyzz
abaaaaaaaaaaahacacaaaakeacaaaaaaaaaaaakeacaaaaaa add r0.xyz, r2.xyzz, r0.xyzz
abaaaaaaacaaahaeaaaaaakeacaaaaaaabaaaakeacaaaaaa add v2.xyz, r0.xyzz, r1.xyzz
aaaaaaaaabaaaiaccbaaaaaaabaaaaaaaaaaaaaaaaaaaaaa mov r1.w, c33.x
aaaaaaaaabaaahacamaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r1.xyz, c12
bdaaaaaaaaaaaeacabaaaaoeacaaaaaaakaaaaoeabaaaaaa dp4 r0.z, r1, c10
bdaaaaaaaaaaacacabaaaaoeacaaaaaaajaaaaoeabaaaaaa dp4 r0.y, r1, c9
bdaaaaaaaaaaabacabaaaaoeacaaaaaaaiaaaaoeabaaaaaa dp4 r0.x, r1, c8
adaaaaaaafaaahacaaaaaakeacaaaaaaboaaaappabaaaaaa mul r5.xyz, r0.xyzz, c30.w
acaaaaaaadaaahacafaaaakeacaaaaaaaaaaaaoeaaaaaaaa sub r3.xyz, r5.xyzz, a0
aaaaaaaaabaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r1.xyz, a5
aaaaaaaaaaaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, a5
adaaaaaaabaaahacabaaaancaaaaaaaaabaaaaajacaaaaaa mul r1.xyz, a1.zxyw, r1.yzxx
adaaaaaaafaaahacabaaaamjaaaaaaaaaaaaaafcacaaaaaa mul r5.xyz, a1.yzxw, r0.zxyy
acaaaaaaabaaahacafaaaakeacaaaaaaabaaaakeacaaaaaa sub r1.xyz, r5.xyzz, r1.xyzz
adaaaaaaacaaahacabaaaakeacaaaaaaafaaaappaaaaaaaa mul r2.xyz, r1.xyzz, a5.w
aaaaaaaaaaaaapacakaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0, c10
bdaaaaaaaeaaaeacaoaaaaoeabaaaaaaaaaaaaoeacaaaaaa dp4 r4.z, c14, r0
aaaaaaaaaaaaapacaiaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0, c8
bdaaaaaaaeaaabacaoaaaaoeabaaaaaaaaaaaaoeacaaaaaa dp4 r4.x, c14, r0
aaaaaaaaabaaapacajaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r1, c9
bdaaaaaaaeaaacacaoaaaaoeabaaaaaaabaaaaoeacaaaaaa dp4 r4.y, c14, r1
bdaaaaaaaaaaaiacaaaaaaoeaaaaaaaaadaaaaoeabaaaaaa dp4 r0.w, a0, c3
bdaaaaaaaaaaaeacaaaaaaoeaaaaaaaaacaaaaoeabaaaaaa dp4 r0.z, a0, c2
bdaaaaaaaaaaabacaaaaaaoeaaaaaaaaaaaaaaoeabaaaaaa dp4 r0.x, a0, c0
bdaaaaaaaaaaacacaaaaaaoeaaaaaaaaabaaaaoeabaaaaaa dp4 r0.y, a0, c1
adaaaaaaabaaahacaaaaaapeacaaaaaacbaaaakkabaaaaaa mul r1.xyz, r0.xyww, c33.z
adaaaaaaabaaacacabaaaaffacaaaaaaanaaaaaaabaaaaaa mul r1.y, r1.y, c13.x
abaaaaaaabaaadacabaaaafeacaaaaaaabaaaakkacaaaaaa add r1.xy, r1.xyyy, r1.z
bcaaaaaaabaaacaeaeaaaakeacaaaaaaacaaaakeacaaaaaa dp3 v1.y, r4.xyzz, r2.xyzz
bcaaaaaaadaaacaeacaaaakeacaaaaaaadaaaakeacaaaaaa dp3 v3.y, r2.xyzz, r3.xyzz
bcaaaaaaabaaaeaeabaaaaoeaaaaaaaaaeaaaakeacaaaaaa dp3 v1.z, a1, r4.xyzz
bcaaaaaaabaaabaeaeaaaakeacaaaaaaafaaaaoeaaaaaaaa dp3 v1.x, r4.xyzz, a5
bcaaaaaaadaaaeaeabaaaaoeaaaaaaaaadaaaakeacaaaaaa dp3 v3.z, a1, r3.xyzz
bcaaaaaaadaaabaeafaaaaoeaaaaaaaaadaaaakeacaaaaaa dp3 v3.x, a5, r3.xyzz
adaaaaaaaeaaadaeabaaaafeacaaaaaabpaaaaoeabaaaaaa mul v4.xy, r1.xyyy, c31
aaaaaaaaaaaaapadaaaaaaoeacaaaaaaaaaaaaaaaaaaaaaa mov o0, r0
aaaaaaaaaeaaamaeaaaaaaopacaaaaaaaaaaaaaaaaaaaaaa mov v4.zw, r0.wwzw
adaaaaaaafaaadacadaaaaoeaaaaaaaacaaaaaoeabaaaaaa mul r5.xy, a3, c32
abaaaaaaaaaaadaeafaaaafeacaaaaaacaaaaaooabaaaaaa add v0.xy, r5.xyyy, c32.zwzw
aaaaaaaaaaaaamaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v0.zw, c0
aaaaaaaaabaaaiaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v1.w, c0
aaaaaaaaacaaaiaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v2.w, c0
aaaaaaaaadaaaiaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v3.w, c0
"
}

SubProgram "gles " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" "SHADOWS_NATIVE" }
"!!GLES
#define SHADER_API_GLES 1
#define tex2D texture2D


#ifdef VERTEX

#extension GL_EXT_shadow_samplers : enable
#define gl_ModelViewProjectionMatrix glstate_matrix_mvp
uniform mat4 glstate_matrix_mvp;
varying highp vec4 xlv_TEXCOORD4;
varying highp vec3 xlv_TEXCOORD3;
varying lowp vec3 xlv_TEXCOORD2;
varying lowp vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 unity_Scale;
uniform highp mat4 _World2Object;
uniform highp mat4 _Object2World;

uniform highp mat4 unity_World2Shadow[4];
uniform highp vec4 unity_SHC;
uniform highp vec4 unity_SHBb;
uniform highp vec4 unity_SHBg;
uniform highp vec4 unity_SHBr;
uniform highp vec4 unity_SHAb;
uniform highp vec4 unity_SHAg;
uniform highp vec4 unity_SHAr;
uniform lowp vec4 _WorldSpaceLightPos0;
uniform highp vec3 _WorldSpaceCameraPos;
attribute vec4 _glesTANGENT;
attribute vec4 _glesMultiTexCoord0;
attribute vec3 _glesNormal;
attribute vec4 _glesVertex;
void main ()
{
  vec4 tmpvar_1;
  tmpvar_1.xyz = normalize(_glesTANGENT.xyz);
  tmpvar_1.w = _glesTANGENT.w;
  vec3 tmpvar_2;
  tmpvar_2 = normalize(_glesNormal);
  highp vec3 shlight_3;
  lowp vec3 tmpvar_4;
  lowp vec3 tmpvar_5;
  mat3 tmpvar_6;
  tmpvar_6[0] = _Object2World[0].xyz;
  tmpvar_6[1] = _Object2World[1].xyz;
  tmpvar_6[2] = _Object2World[2].xyz;
  highp vec3 tmpvar_7;
  highp vec3 tmpvar_8;
  tmpvar_7 = tmpvar_1.xyz;
  tmpvar_8 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_9;
  tmpvar_9[0].x = tmpvar_7.x;
  tmpvar_9[0].y = tmpvar_8.x;
  tmpvar_9[0].z = tmpvar_2.x;
  tmpvar_9[1].x = tmpvar_7.y;
  tmpvar_9[1].y = tmpvar_8.y;
  tmpvar_9[1].z = tmpvar_2.y;
  tmpvar_9[2].x = tmpvar_7.z;
  tmpvar_9[2].y = tmpvar_8.z;
  tmpvar_9[2].z = tmpvar_2.z;
  highp vec3 tmpvar_10;
  tmpvar_10 = (tmpvar_9 * (_World2Object * _WorldSpaceLightPos0).xyz);
  tmpvar_4 = tmpvar_10;
  highp vec4 tmpvar_11;
  tmpvar_11.w = 1.0;
  tmpvar_11.xyz = _WorldSpaceCameraPos;
  highp vec4 tmpvar_12;
  tmpvar_12.w = 1.0;
  tmpvar_12.xyz = (tmpvar_6 * (tmpvar_2 * unity_Scale.w));
  mediump vec3 tmpvar_13;
  mediump vec4 normal_14;
  normal_14 = tmpvar_12;
  highp float vC_15;
  mediump vec3 x3_16;
  mediump vec3 x2_17;
  mediump vec3 x1_18;
  highp float tmpvar_19;
  tmpvar_19 = dot (unity_SHAr, normal_14);
  x1_18.x = tmpvar_19;
  highp float tmpvar_20;
  tmpvar_20 = dot (unity_SHAg, normal_14);
  x1_18.y = tmpvar_20;
  highp float tmpvar_21;
  tmpvar_21 = dot (unity_SHAb, normal_14);
  x1_18.z = tmpvar_21;
  mediump vec4 tmpvar_22;
  tmpvar_22 = (normal_14.xyzz * normal_14.yzzx);
  highp float tmpvar_23;
  tmpvar_23 = dot (unity_SHBr, tmpvar_22);
  x2_17.x = tmpvar_23;
  highp float tmpvar_24;
  tmpvar_24 = dot (unity_SHBg, tmpvar_22);
  x2_17.y = tmpvar_24;
  highp float tmpvar_25;
  tmpvar_25 = dot (unity_SHBb, tmpvar_22);
  x2_17.z = tmpvar_25;
  mediump float tmpvar_26;
  tmpvar_26 = ((normal_14.x * normal_14.x) - (normal_14.y * normal_14.y));
  vC_15 = tmpvar_26;
  highp vec3 tmpvar_27;
  tmpvar_27 = (unity_SHC.xyz * vC_15);
  x3_16 = tmpvar_27;
  tmpvar_13 = ((x1_18 + x2_17) + x3_16);
  shlight_3 = tmpvar_13;
  tmpvar_5 = shlight_3;
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = tmpvar_4;
  xlv_TEXCOORD2 = tmpvar_5;
  xlv_TEXCOORD3 = (tmpvar_9 * (((_World2Object * tmpvar_11).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD4 = (unity_World2Shadow[0] * (_Object2World * _glesVertex));
}



#endif
#ifdef FRAGMENT

#extension GL_EXT_shadow_samplers : enable
varying highp vec4 xlv_TEXCOORD4;
varying highp vec3 xlv_TEXCOORD3;
varying lowp vec3 xlv_TEXCOORD2;
varying lowp vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform mediump float _Gloss;
uniform mediump float _Shininess;
uniform lowp vec4 _Color;
uniform sampler2D _SpecMap;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform sampler2DShadow _ShadowMapTexture;
uniform lowp vec4 _LightColor0;
uniform highp vec4 _LightShadowData;
void main ()
{
  lowp vec4 c_1;
  mediump vec3 tmpvar_2;
  mediump vec3 tmpvar_3;
  lowp vec4 tmpvar_4;
  tmpvar_4 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_5;
  tmpvar_5 = (tmpvar_4.xyz * _Color.xyz);
  tmpvar_2 = tmpvar_5;
  lowp vec4 tmpvar_6;
  tmpvar_6 = texture2D (_SpecMap, xlv_TEXCOORD0);
  mediump vec3 tmpvar_7;
  tmpvar_7 = ((tmpvar_4.w * tmpvar_6.xyz) * _Gloss);
  lowp vec3 tmpvar_8;
  tmpvar_8 = ((texture2D (_BumpMap, xlv_TEXCOORD0).xyz * 2.0) - 1.0);
  tmpvar_3 = tmpvar_8;
  lowp float shadow_9;
  lowp float tmpvar_10;
  tmpvar_10 = shadow2DEXT (_ShadowMapTexture, xlv_TEXCOORD4.xyz);
  highp float tmpvar_11;
  tmpvar_11 = (_LightShadowData.x + (tmpvar_10 * (1.0 - _LightShadowData.x)));
  shadow_9 = tmpvar_11;
  highp vec3 tmpvar_12;
  tmpvar_12 = normalize(xlv_TEXCOORD3);
  mediump vec3 lightDir_13;
  lightDir_13 = xlv_TEXCOORD1;
  mediump vec3 viewDir_14;
  viewDir_14 = tmpvar_12;
  mediump float atten_15;
  atten_15 = shadow_9;
  mediump vec4 c_16;
  mediump vec3 specCol_17;
  highp float nh_18;
  mediump float tmpvar_19;
  tmpvar_19 = max (0.0, dot (tmpvar_3, normalize((lightDir_13 + viewDir_14))));
  nh_18 = tmpvar_19;
  mediump float arg1_20;
  arg1_20 = (32.0 * _Shininess);
  highp vec3 tmpvar_21;
  tmpvar_21 = (pow (nh_18, arg1_20) * tmpvar_7);
  specCol_17 = tmpvar_21;
  c_16.xyz = ((((tmpvar_2 * _LightColor0.xyz) * max (0.0, dot (tmpvar_3, lightDir_13))) + (_LightColor0.xyz * specCol_17)) * (atten_15 * 2.0));
  c_16.w = 0.0;
  c_1 = c_16;
  mediump vec3 tmpvar_22;
  tmpvar_22 = (c_1.xyz + (tmpvar_2 * xlv_TEXCOORD2));
  c_1.xyz = tmpvar_22;
  gl_FragData[0] = c_1;
}



#endif"
}

SubProgram "gles " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" "SHADOWS_NATIVE" }
"!!GLES
#define SHADER_API_GLES 1
#define tex2D texture2D


#ifdef VERTEX

#extension GL_EXT_shadow_samplers : enable
#define gl_ModelViewProjectionMatrix glstate_matrix_mvp
uniform mat4 glstate_matrix_mvp;
varying highp vec4 xlv_TEXCOORD2;
varying highp vec2 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 unity_LightmapST;
uniform highp mat4 _Object2World;

uniform highp mat4 unity_World2Shadow[4];
attribute vec4 _glesMultiTexCoord1;
attribute vec4 _glesMultiTexCoord0;
attribute vec4 _glesVertex;
void main ()
{
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = ((_glesMultiTexCoord1.xy * unity_LightmapST.xy) + unity_LightmapST.zw);
  xlv_TEXCOORD2 = (unity_World2Shadow[0] * (_Object2World * _glesVertex));
}



#endif
#ifdef FRAGMENT

#extension GL_EXT_shadow_samplers : enable
varying highp vec4 xlv_TEXCOORD2;
varying highp vec2 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform sampler2D unity_Lightmap;
uniform lowp vec4 _Color;
uniform sampler2D _MainTex;
uniform sampler2DShadow _ShadowMapTexture;
uniform highp vec4 _LightShadowData;
void main ()
{
  lowp vec4 c_1;
  mediump vec3 tmpvar_2;
  lowp vec3 tmpvar_3;
  tmpvar_3 = (texture2D (_MainTex, xlv_TEXCOORD0).xyz * _Color.xyz);
  tmpvar_2 = tmpvar_3;
  lowp float shadow_4;
  lowp float tmpvar_5;
  tmpvar_5 = shadow2DEXT (_ShadowMapTexture, xlv_TEXCOORD2.xyz);
  highp float tmpvar_6;
  tmpvar_6 = (_LightShadowData.x + (tmpvar_5 * (1.0 - _LightShadowData.x)));
  shadow_4 = tmpvar_6;
  lowp vec3 tmpvar_7;
  tmpvar_7 = min ((2.0 * texture2D (unity_Lightmap, xlv_TEXCOORD1).xyz), vec3((shadow_4 * 2.0)));
  mediump vec3 tmpvar_8;
  tmpvar_8 = (tmpvar_2 * tmpvar_7);
  c_1.xyz = tmpvar_8;
  c_1.w = 0.0;
  gl_FragData[0] = c_1;
}



#endif"
}

SubProgram "gles " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" "SHADOWS_NATIVE" "VERTEXLIGHT_ON" }
"!!GLES
#define SHADER_API_GLES 1
#define tex2D texture2D


#ifdef VERTEX

#extension GL_EXT_shadow_samplers : enable
#define gl_ModelViewProjectionMatrix glstate_matrix_mvp
uniform mat4 glstate_matrix_mvp;
varying highp vec4 xlv_TEXCOORD4;
varying highp vec3 xlv_TEXCOORD3;
varying lowp vec3 xlv_TEXCOORD2;
varying lowp vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 unity_Scale;
uniform highp mat4 _World2Object;
uniform highp mat4 _Object2World;

uniform highp mat4 unity_World2Shadow[4];
uniform highp vec4 unity_SHC;
uniform highp vec4 unity_SHBb;
uniform highp vec4 unity_SHBg;
uniform highp vec4 unity_SHBr;
uniform highp vec4 unity_SHAb;
uniform highp vec4 unity_SHAg;
uniform highp vec4 unity_SHAr;
uniform highp vec4 unity_LightColor[4];
uniform highp vec4 unity_4LightAtten0;
uniform highp vec4 unity_4LightPosZ0;
uniform highp vec4 unity_4LightPosY0;
uniform highp vec4 unity_4LightPosX0;
uniform lowp vec4 _WorldSpaceLightPos0;
uniform highp vec3 _WorldSpaceCameraPos;
attribute vec4 _glesTANGENT;
attribute vec4 _glesMultiTexCoord0;
attribute vec3 _glesNormal;
attribute vec4 _glesVertex;
void main ()
{
  vec4 tmpvar_1;
  tmpvar_1.xyz = normalize(_glesTANGENT.xyz);
  tmpvar_1.w = _glesTANGENT.w;
  vec3 tmpvar_2;
  tmpvar_2 = normalize(_glesNormal);
  highp vec3 shlight_3;
  lowp vec3 tmpvar_4;
  lowp vec3 tmpvar_5;
  mat3 tmpvar_6;
  tmpvar_6[0] = _Object2World[0].xyz;
  tmpvar_6[1] = _Object2World[1].xyz;
  tmpvar_6[2] = _Object2World[2].xyz;
  highp vec3 tmpvar_7;
  tmpvar_7 = (tmpvar_6 * (tmpvar_2 * unity_Scale.w));
  highp vec3 tmpvar_8;
  highp vec3 tmpvar_9;
  tmpvar_8 = tmpvar_1.xyz;
  tmpvar_9 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_10;
  tmpvar_10[0].x = tmpvar_8.x;
  tmpvar_10[0].y = tmpvar_9.x;
  tmpvar_10[0].z = tmpvar_2.x;
  tmpvar_10[1].x = tmpvar_8.y;
  tmpvar_10[1].y = tmpvar_9.y;
  tmpvar_10[1].z = tmpvar_2.y;
  tmpvar_10[2].x = tmpvar_8.z;
  tmpvar_10[2].y = tmpvar_9.z;
  tmpvar_10[2].z = tmpvar_2.z;
  highp vec3 tmpvar_11;
  tmpvar_11 = (tmpvar_10 * (_World2Object * _WorldSpaceLightPos0).xyz);
  tmpvar_4 = tmpvar_11;
  highp vec4 tmpvar_12;
  tmpvar_12.w = 1.0;
  tmpvar_12.xyz = _WorldSpaceCameraPos;
  highp vec4 tmpvar_13;
  tmpvar_13.w = 1.0;
  tmpvar_13.xyz = tmpvar_7;
  mediump vec3 tmpvar_14;
  mediump vec4 normal_15;
  normal_15 = tmpvar_13;
  highp float vC_16;
  mediump vec3 x3_17;
  mediump vec3 x2_18;
  mediump vec3 x1_19;
  highp float tmpvar_20;
  tmpvar_20 = dot (unity_SHAr, normal_15);
  x1_19.x = tmpvar_20;
  highp float tmpvar_21;
  tmpvar_21 = dot (unity_SHAg, normal_15);
  x1_19.y = tmpvar_21;
  highp float tmpvar_22;
  tmpvar_22 = dot (unity_SHAb, normal_15);
  x1_19.z = tmpvar_22;
  mediump vec4 tmpvar_23;
  tmpvar_23 = (normal_15.xyzz * normal_15.yzzx);
  highp float tmpvar_24;
  tmpvar_24 = dot (unity_SHBr, tmpvar_23);
  x2_18.x = tmpvar_24;
  highp float tmpvar_25;
  tmpvar_25 = dot (unity_SHBg, tmpvar_23);
  x2_18.y = tmpvar_25;
  highp float tmpvar_26;
  tmpvar_26 = dot (unity_SHBb, tmpvar_23);
  x2_18.z = tmpvar_26;
  mediump float tmpvar_27;
  tmpvar_27 = ((normal_15.x * normal_15.x) - (normal_15.y * normal_15.y));
  vC_16 = tmpvar_27;
  highp vec3 tmpvar_28;
  tmpvar_28 = (unity_SHC.xyz * vC_16);
  x3_17 = tmpvar_28;
  tmpvar_14 = ((x1_19 + x2_18) + x3_17);
  shlight_3 = tmpvar_14;
  tmpvar_5 = shlight_3;
  highp vec3 tmpvar_29;
  tmpvar_29 = (_Object2World * _glesVertex).xyz;
  highp vec4 tmpvar_30;
  tmpvar_30 = (unity_4LightPosX0 - tmpvar_29.x);
  highp vec4 tmpvar_31;
  tmpvar_31 = (unity_4LightPosY0 - tmpvar_29.y);
  highp vec4 tmpvar_32;
  tmpvar_32 = (unity_4LightPosZ0 - tmpvar_29.z);
  highp vec4 tmpvar_33;
  tmpvar_33 = (((tmpvar_30 * tmpvar_30) + (tmpvar_31 * tmpvar_31)) + (tmpvar_32 * tmpvar_32));
  highp vec4 tmpvar_34;
  tmpvar_34 = (max (vec4(0.0, 0.0, 0.0, 0.0), ((((tmpvar_30 * tmpvar_7.x) + (tmpvar_31 * tmpvar_7.y)) + (tmpvar_32 * tmpvar_7.z)) * inversesqrt(tmpvar_33))) * (1.0/((1.0 + (tmpvar_33 * unity_4LightAtten0)))));
  highp vec3 tmpvar_35;
  tmpvar_35 = (tmpvar_5 + ((((unity_LightColor[0].xyz * tmpvar_34.x) + (unity_LightColor[1].xyz * tmpvar_34.y)) + (unity_LightColor[2].xyz * tmpvar_34.z)) + (unity_LightColor[3].xyz * tmpvar_34.w)));
  tmpvar_5 = tmpvar_35;
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = tmpvar_4;
  xlv_TEXCOORD2 = tmpvar_5;
  xlv_TEXCOORD3 = (tmpvar_10 * (((_World2Object * tmpvar_12).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD4 = (unity_World2Shadow[0] * (_Object2World * _glesVertex));
}



#endif
#ifdef FRAGMENT

#extension GL_EXT_shadow_samplers : enable
varying highp vec4 xlv_TEXCOORD4;
varying highp vec3 xlv_TEXCOORD3;
varying lowp vec3 xlv_TEXCOORD2;
varying lowp vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform mediump float _Gloss;
uniform mediump float _Shininess;
uniform lowp vec4 _Color;
uniform sampler2D _SpecMap;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform sampler2DShadow _ShadowMapTexture;
uniform lowp vec4 _LightColor0;
uniform highp vec4 _LightShadowData;
void main ()
{
  lowp vec4 c_1;
  mediump vec3 tmpvar_2;
  mediump vec3 tmpvar_3;
  lowp vec4 tmpvar_4;
  tmpvar_4 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_5;
  tmpvar_5 = (tmpvar_4.xyz * _Color.xyz);
  tmpvar_2 = tmpvar_5;
  lowp vec4 tmpvar_6;
  tmpvar_6 = texture2D (_SpecMap, xlv_TEXCOORD0);
  mediump vec3 tmpvar_7;
  tmpvar_7 = ((tmpvar_4.w * tmpvar_6.xyz) * _Gloss);
  lowp vec3 tmpvar_8;
  tmpvar_8 = ((texture2D (_BumpMap, xlv_TEXCOORD0).xyz * 2.0) - 1.0);
  tmpvar_3 = tmpvar_8;
  lowp float shadow_9;
  lowp float tmpvar_10;
  tmpvar_10 = shadow2DEXT (_ShadowMapTexture, xlv_TEXCOORD4.xyz);
  highp float tmpvar_11;
  tmpvar_11 = (_LightShadowData.x + (tmpvar_10 * (1.0 - _LightShadowData.x)));
  shadow_9 = tmpvar_11;
  highp vec3 tmpvar_12;
  tmpvar_12 = normalize(xlv_TEXCOORD3);
  mediump vec3 lightDir_13;
  lightDir_13 = xlv_TEXCOORD1;
  mediump vec3 viewDir_14;
  viewDir_14 = tmpvar_12;
  mediump float atten_15;
  atten_15 = shadow_9;
  mediump vec4 c_16;
  mediump vec3 specCol_17;
  highp float nh_18;
  mediump float tmpvar_19;
  tmpvar_19 = max (0.0, dot (tmpvar_3, normalize((lightDir_13 + viewDir_14))));
  nh_18 = tmpvar_19;
  mediump float arg1_20;
  arg1_20 = (32.0 * _Shininess);
  highp vec3 tmpvar_21;
  tmpvar_21 = (pow (nh_18, arg1_20) * tmpvar_7);
  specCol_17 = tmpvar_21;
  c_16.xyz = ((((tmpvar_2 * _LightColor0.xyz) * max (0.0, dot (tmpvar_3, lightDir_13))) + (_LightColor0.xyz * specCol_17)) * (atten_15 * 2.0));
  c_16.w = 0.0;
  c_1 = c_16;
  mediump vec3 tmpvar_22;
  tmpvar_22 = (c_1.xyz + (tmpvar_2 * xlv_TEXCOORD2));
  c_1.xyz = tmpvar_22;
  gl_FragData[0] = c_1;
}



#endif"
}

}
Program "fp" {
// Fragment combos: 4
//   opengl - ALU: 7 to 35, TEX: 2 to 4
//   d3d9 - ALU: 6 to 36, TEX: 2 to 4
//   d3d11 - ALU: 4 to 22, TEX: 2 to 4, FLOW: 1 to 1
//   d3d11_9x - ALU: 4 to 20, TEX: 2 to 3, FLOW: 1 to 1
SubProgram "opengl " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
"!!ARBfp1.0
OPTION ARB_precision_hint_fastest;
# 33 ALU, 3 TEX
PARAM c[5] = { program.local[0..3],
		{ 2, 1, 0, 32 } };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
TEX R0, fragment.texcoord[0], texture[0], 2D;
TEX R2.yw, fragment.texcoord[0], texture[2], 2D;
TEX R1.xyz, fragment.texcoord[0], texture[1], 2D;
MAD R2.xy, R2.wyzw, c[4].x, -c[4].y;
MUL R1.xyz, R0.w, R1;
DP3 R2.z, fragment.texcoord[3], fragment.texcoord[3];
MUL R1.w, R2.y, R2.y;
MAD R1.w, -R2.x, R2.x, -R1;
ADD R1.w, R1, c[4].y;
MOV R2.w, c[4];
RSQ R2.z, R2.z;
MOV R3.xyz, fragment.texcoord[1];
MAD R3.xyz, R2.z, fragment.texcoord[3], R3;
DP3 R2.z, R3, R3;
RSQ R2.z, R2.z;
MUL R3.xyz, R2.z, R3;
RSQ R1.w, R1.w;
RCP R2.z, R1.w;
DP3 R1.w, R2, R3;
MUL R0.xyz, R0, c[1];
MUL R3.xyz, R0, fragment.texcoord[2];
MAX R1.w, R1, c[4].z;
MUL R0.w, R2, c[2].x;
POW R0.w, R1.w, R0.w;
MUL R1.xyz, R1, c[3].x;
MUL R1.xyz, R0.w, R1;
DP3 R0.w, R2, fragment.texcoord[1];
MUL R1.xyz, R1, c[0];
MUL R0.xyz, R0, c[0];
MAX R0.w, R0, c[4].z;
MAD R0.xyz, R0, R0.w, R1;
MAD result.color.xyz, R0, c[4].x, R3;
MOV result.color.w, c[4].z;
END
# 33 instructions, 4 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
"ps_2_0
; 35 ALU, 3 TEX
dcl_2d s0
dcl_2d s1
dcl_2d s2
def c4, 2.00000000, -1.00000000, 1.00000000, 0.00000000
def c5, 32.00000000, 0, 0, 0
dcl t0.xy
dcl t1.xyz
dcl t2.xyz
dcl t3.xyz
texld r0, t0, s2
texld r2, t0, s0
texld r3, t0, s1
mov r0.x, r0.w
mad_pp r5.xy, r0, c4.x, c4.y
mul_pp r0.x, r5.y, r5.y
mad_pp r0.x, -r5, r5, -r0
dp3_pp r1.x, t3, t3
add_pp r0.x, r0, c4.z
rsq_pp r0.x, r0.x
rcp_pp r5.z, r0.x
mov_pp r0.x, c2
rsq_pp r1.x, r1.x
mov_pp r4.xyz, t1
mad_pp r4.xyz, r1.x, t3, r4
dp3_pp r1.x, r4, r4
rsq_pp r1.x, r1.x
mul_pp r1.xyz, r1.x, r4
dp3_pp r1.x, r5, r1
mul_pp r0.x, c5, r0
max_pp r1.x, r1, c4.w
pow r4.x, r1.x, r0.x
mul_pp r0.xyz, r2.w, r3
mul_pp r2.xyz, r2, c1
mul_pp r1.xyz, r0, c3.x
mov r0.x, r4.x
mul r0.xyz, r0.x, r1
mul_pp r1.xyz, r0, c0
dp3_pp r0.x, r5, t1
max_pp r0.x, r0, c4.w
mul_pp r3.xyz, r2, c0
mad_pp r0.xyz, r3, r0.x, r1
mul_pp r1.xyz, r2, t2
mov_pp r0.w, c4
mad_pp r0.xyz, r0, c4.x, r1
mov_pp oC0, r0
"
}

SubProgram "xbox360 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Vector 1 [_Color]
Float 3 [_Gloss]
Vector 0 [_LightColor0]
Float 2 [_Shininess]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_BumpMap] 2D
SetTexture 2 [_SpecMap] 2D
// Shader Timing Estimate, in Cycles/64 pixel vector:
// ALU: 24.00 (18 instructions), vertex: 0, texture: 12,
//   sequencer: 10, interpolator: 16;    7 GPRs, 27 threads,
// Performance (if enough threads): ~24 cycles per vector
// * Texture cycle estimates are assuming an 8bit/component texture with no
//     aniso or trilinear filtering.

"ps_360
backbbaaaaaaableaaaaabgmaaaaaaaaaaaaaaceaaaaabfmaaaaabieaaaaaaaa
aaaaaaaaaaaaabdeaaaaaabmaaaaabcfppppadaaaaaaaaahaaaaaabmaaaaaaaa
aaaaabboaaaaaakiaaadaaabaaabaaaaaaaaaaleaaaaaaaaaaaaaameaaacaaab
aaabaaaaaaaaaammaaaaaaaaaaaaaanmaaacaaadaaabaaaaaaaaaaoeaaaaaaaa
aaaaaapeaaacaaaaaaabaaaaaaaaaammaaaaaaaaaaaaababaaadaaaaaaabaaaa
aaaaaaleaaaaaaaaaaaaabakaaacaaacaaabaaaaaaaaaaoeaaaaaaaaaaaaabbf
aaadaaacaaabaaaaaaaaaaleaaaaaaaafpechfgnhaengbhaaaklklklaaaeaaam
aaabaaabaaabaaaaaaaaaaaafpedgpgmgphcaaklaaabaaadaaabaaaeaaabaaaa
aaaaaaaafpehgmgphdhdaaklaaaaaaadaaabaaabaaabaaaaaaaaaaaafpemgjgh
giheedgpgmgphcdaaafpengbgjgofegfhiaafpfdgigjgogjgogfhdhdaafpfdha
gfgdengbhaaahahdfpddfpdaaadccodacodcdadddfddcodaaaklklklaaaaaaaa
aaaaaaabaaaaaaaaaaaaaaaaaaaaaabeabpmaabaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaeaaaaaabcmbaaaagaaaaaaaaaeaaaaaaaaaaaacmieaaapaaap
aaaaaaabaaaadafaaaaahbfbaaaahcfcaaaahdfdaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaeaaaaaaaaaaaaaaalpiaaaaa
dpiaaaaaecaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaabfdaadaaaabcaameaaaaaa
aaaagaaggaambcaabcaaaaaaaaaagabcaaaaccaaaaaaaaaabacigaabbpbppoii
aaaaeaaababiaaabbpbppghpaaaaeaaabaaieaabbpbppgiiaaaaeaaamiaiaaaf
aagmgmaacbacppaamiabaaaaaaloloaapaadadaamiahaaaeaamamaaakbaeabaa
miadaaafaamhgmmgilaapopofiboaaaaaablpmgmobaeagiamiahaaadaagmmama
olaaadabmiaiaaabaegngnblnbafafpokaebafaaaaloloblpaadadibfibcaaab
aalologmpaafabiamianaaabaapagmaaobadaaaakibbadabaamploebnaabafad
kicdadabaalalbecicabpoadeaehabafaalelegmkbaeaaibkiepadabaadepbed
mbafabaddiihaaaaaamamablobaeacabmiahaaacaaleblaaobadaaaamiahaaab
aamaleleklacaaabmiahmaaaaalegmmaklabpoaaaaaaaaaaaaaaaaaaaaaaaaaa
"
}

SubProgram "ps3 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
"sce_fp_rsx // 42 instructions using 4 registers
[Configuration]
24
ffffffff0003c020000ffff1000000000000840004000000
[Offsets]
4
_LightColor0 2 0
0000028000000220
_Color 1 0
000000c0
_Shininess 1 0
00000140
_Gloss 1 0
00000110
[Microcode]
672
94001704c8011c9dc8000001c8003fe1068e0440ce001c9d00020000aa020000
000040000000bf8000000000000000008e021702c8011c9dc8000001c8003fe1
ee803940c8011c9dc8000029c800bfe110800240ab1c1c9cab1c0000c8000001
108a0440011c1c9e011c0000c9000003ae8c0140c8011c9dc8000001c8003fe1
0e040340c9181c9dc9000001c80000019e001700c8011c9dc8000001c8003fe1
0e883940c8081c9dc8000029c80000010e800240c8001c9dc8020001c8000001
0000000000000000000000000000000010800340c9141c9dc8020001c8000001
00000000000000000000000000003f80088e3b40ff003c9dff000001c8000001
02820240fe001c9d00020000c800000100000000000000000000000000000000
0e82024001041c9cc8040001c80000011082014000021c9cc8000001c8000001
0000000000000000000000000000000010800540c91c1c9dc9180001c8000001
02880540c91c1c9dc9100001c800000110800900c9001c9d00020000c8000001
0000000000000000000000000000000002020900c9101c9d00020000c8000001
0000000000000000000000000000000002021d00c8041c9dc8000001c8000001
02860240ff041c9d00020000c800000100004200000000000000000000000000
1004020000041c9c010c0000c8000001ce860140c8011c9dc8000001c8003fe1
02041c00fe081c9dc8000001c80000010e840240c9001c9dc8020001c8000001
000000000000000000000000000000000e82020000081c9cc9040001c8000001
0e840240c9081c9dff000001c80000011080014000021c9cc8000001c8000001
000000000000000000000000000000000e820440c9041c9dc8021001c9080001
000000000000000000000000000000000e810440c9001c9dc90c0001c9040001
"
}

SubProgram "d3d11 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
ConstBuffer "$Globals" 96 // 72 used size, 7 vars
Vector 16 [_LightColor0] 4
Vector 48 [_Color] 4
Float 64 [_Shininess]
Float 68 [_Gloss]
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 0
SetTexture 1 [_SpecMap] 2D 2
SetTexture 2 [_BumpMap] 2D 1
// 31 instructions, 3 temp regs, 0 temp arrays:
// ALU 20 float, 0 int, 0 uint
// TEX 3 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0
eefiecedbddnehkdjccdhjhnclafipjelnkcflbkabaaaaaafmafaaaaadaaaaaa
cmaaaaaammaaaaaaaaabaaaaejfdeheojiaaaaaaafaaaaaaaiaaaaaaiaaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaadadaaaaimaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaahahaaaaimaaaaaa
adaaaaaaaaaaaaaaadaaaaaaaeaaaaaaahahaaaafdfgfpfaepfdejfeejepeoaa
feeffiedepepfceeaaklklklepfdeheocmaaaaaaabaaaaaaaiaaaaaacaaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaafdfgfpfegbhcghgfheaaklkl
fdeieefcfeaeaaaaeaaaaaaabfabaaaafjaaaaaeegiocaaaaaaaaaaaafaaaaaa
fkaaaaadaagabaaaaaaaaaaafkaaaaadaagabaaaabaaaaaafkaaaaadaagabaaa
acaaaaaafibiaaaeaahabaaaaaaaaaaaffffaaaafibiaaaeaahabaaaabaaaaaa
ffffaaaafibiaaaeaahabaaaacaaaaaaffffaaaagcbaaaaddcbabaaaabaaaaaa
gcbaaaadhcbabaaaacaaaaaagcbaaaadhcbabaaaadaaaaaagcbaaaadhcbabaaa
aeaaaaaagfaaaaadpccabaaaaaaaaaaagiaaaaacadaaaaaabaaaaaahbcaabaaa
aaaaaaaaegbcbaaaaeaaaaaaegbcbaaaaeaaaaaaeeaaaaafbcaabaaaaaaaaaaa
akaabaaaaaaaaaaadcaaaaajhcaabaaaaaaaaaaaegbcbaaaaeaaaaaaagaabaaa
aaaaaaaaegbcbaaaacaaaaaabaaaaaahicaabaaaaaaaaaaaegacbaaaaaaaaaaa
egacbaaaaaaaaaaaeeaaaaaficaabaaaaaaaaaaadkaabaaaaaaaaaaadiaaaaah
hcaabaaaaaaaaaaapgapbaaaaaaaaaaaegacbaaaaaaaaaaaefaaaaajpcaabaaa
abaaaaaaegbabaaaabaaaaaaeghobaaaacaaaaaaaagabaaaabaaaaaadcaaaaap
dcaabaaaabaaaaaahgapbaaaabaaaaaaaceaaaaaaaaaaaeaaaaaaaeaaaaaaaaa
aaaaaaaaaceaaaaaaaaaialpaaaaialpaaaaaaaaaaaaaaaadcaaaaakicaabaaa
aaaaaaaaakaabaiaebaaaaaaabaaaaaaakaabaaaabaaaaaaabeaaaaaaaaaiadp
dcaaaaakicaabaaaaaaaaaaabkaabaiaebaaaaaaabaaaaaabkaabaaaabaaaaaa
dkaabaaaaaaaaaaaelaaaaafecaabaaaabaaaaaadkaabaaaaaaaaaaabaaaaaah
bcaabaaaaaaaaaaaegacbaaaabaaaaaaegacbaaaaaaaaaaabaaaaaahccaabaaa
aaaaaaaaegacbaaaabaaaaaaegbcbaaaacaaaaaadeaaaaakdcaabaaaaaaaaaaa
egaabaaaaaaaaaaaaceaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaacpaaaaaf
bcaabaaaaaaaaaaaakaabaaaaaaaaaaadiaaaaaiecaabaaaaaaaaaaaakiacaaa
aaaaaaaaaeaaaaaaabeaaaaaaaaaaaecdiaaaaahbcaabaaaaaaaaaaaakaabaaa
aaaaaaaackaabaaaaaaaaaaabjaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaa
efaaaaajpcaabaaaabaaaaaaegbabaaaabaaaaaaeghobaaaabaaaaaaaagabaaa
acaaaaaaefaaaaajpcaabaaaacaaaaaaegbabaaaabaaaaaaeghobaaaaaaaaaaa
aagabaaaaaaaaaaadiaaaaahhcaabaaaabaaaaaaegacbaaaabaaaaaapgapbaaa
acaaaaaadiaaaaaihcaabaaaacaaaaaaegacbaaaacaaaaaaegiccaaaaaaaaaaa
adaaaaaadiaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaafgifcaaaaaaaaaaa
aeaaaaaadiaaaaahncaabaaaaaaaaaaaagaabaaaaaaaaaaaagajbaaaabaaaaaa
diaaaaaincaabaaaaaaaaaaaagaobaaaaaaaaaaaagijcaaaaaaaaaaaabaaaaaa
diaaaaaihcaabaaaabaaaaaaegacbaaaacaaaaaaegiccaaaaaaaaaaaabaaaaaa
dcaaaaajhcaabaaaaaaaaaaaegacbaaaabaaaaaafgafbaaaaaaaaaaaigadbaaa
aaaaaaaaaaaaaaahhcaabaaaaaaaaaaaegacbaaaaaaaaaaaegacbaaaaaaaaaaa
dcaaaaajhccabaaaaaaaaaaaegacbaaaacaaaaaaegbcbaaaadaaaaaaegacbaaa
aaaaaaaadgaaaaaficcabaaaaaaaaaaaabeaaaaaaaaaaaaadoaaaaab"
}

SubProgram "gles " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
"!!GLES"
}

SubProgram "glesdesktop " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
"!!GLES"
}

SubProgram "flash " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
"agal_ps
c4 2.0 -1.0 1.0 0.0
c5 32.0 0.0 0.0 0.0
[bc]
ciaaaaaaaaaaapacaaaaaaoeaeaaaaaaacaaaaaaafaababb tex r0, v0, s2 <2d wrap linear point>
ciaaaaaaacaaapacaaaaaaoeaeaaaaaaaaaaaaaaafaababb tex r2, v0, s0 <2d wrap linear point>
ciaaaaaaadaaapacaaaaaaoeaeaaaaaaabaaaaaaafaababb tex r3, v0, s1 <2d wrap linear point>
aaaaaaaaaaaaabacaaaaaappacaaaaaaaaaaaaaaaaaaaaaa mov r0.x, r0.w
adaaaaaaafaaadacaaaaaafeacaaaaaaaeaaaaaaabaaaaaa mul r5.xy, r0.xyyy, c4.x
abaaaaaaafaaadacafaaaafeacaaaaaaaeaaaaffabaaaaaa add r5.xy, r5.xyyy, c4.y
adaaaaaaaaaaabacafaaaaffacaaaaaaafaaaaffacaaaaaa mul r0.x, r5.y, r5.y
bfaaaaaaabaaabacafaaaaaaacaaaaaaaaaaaaaaaaaaaaaa neg r1.x, r5.x
adaaaaaaabaaabacabaaaaaaacaaaaaaafaaaaaaacaaaaaa mul r1.x, r1.x, r5.x
acaaaaaaaaaaabacabaaaaaaacaaaaaaaaaaaaaaacaaaaaa sub r0.x, r1.x, r0.x
bcaaaaaaabaaabacadaaaaoeaeaaaaaaadaaaaoeaeaaaaaa dp3 r1.x, v3, v3
abaaaaaaaaaaabacaaaaaaaaacaaaaaaaeaaaakkabaaaaaa add r0.x, r0.x, c4.z
akaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r0.x, r0.x
afaaaaaaafaaaeacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rcp r5.z, r0.x
aaaaaaaaaaaaabacacaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0.x, c2
akaaaaaaabaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r1.x, r1.x
aaaaaaaaaeaaahacabaaaaoeaeaaaaaaaaaaaaaaaaaaaaaa mov r4.xyz, v1
adaaaaaaagaaahacabaaaaaaacaaaaaaadaaaaoeaeaaaaaa mul r6.xyz, r1.x, v3
abaaaaaaaeaaahacagaaaakeacaaaaaaaeaaaakeacaaaaaa add r4.xyz, r6.xyzz, r4.xyzz
bcaaaaaaabaaabacaeaaaakeacaaaaaaaeaaaakeacaaaaaa dp3 r1.x, r4.xyzz, r4.xyzz
akaaaaaaabaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r1.x, r1.x
adaaaaaaabaaahacabaaaaaaacaaaaaaaeaaaakeacaaaaaa mul r1.xyz, r1.x, r4.xyzz
bcaaaaaaabaaabacafaaaakeacaaaaaaabaaaakeacaaaaaa dp3 r1.x, r5.xyzz, r1.xyzz
adaaaaaaaaaaabacafaaaaoeabaaaaaaaaaaaaaaacaaaaaa mul r0.x, c5, r0.x
ahaaaaaaabaaabacabaaaaaaacaaaaaaaeaaaappabaaaaaa max r1.x, r1.x, c4.w
alaaaaaaaeaaapacabaaaaaaacaaaaaaaaaaaaaaacaaaaaa pow r4, r1.x, r0.x
adaaaaaaaaaaahacacaaaappacaaaaaaadaaaakeacaaaaaa mul r0.xyz, r2.w, r3.xyzz
adaaaaaaacaaahacacaaaakeacaaaaaaabaaaaoeabaaaaaa mul r2.xyz, r2.xyzz, c1
adaaaaaaabaaahacaaaaaakeacaaaaaaadaaaaaaabaaaaaa mul r1.xyz, r0.xyzz, c3.x
aaaaaaaaaaaaabacaeaaaaaaacaaaaaaaaaaaaaaaaaaaaaa mov r0.x, r4.x
adaaaaaaaaaaahacaaaaaaaaacaaaaaaabaaaakeacaaaaaa mul r0.xyz, r0.x, r1.xyzz
adaaaaaaabaaahacaaaaaakeacaaaaaaaaaaaaoeabaaaaaa mul r1.xyz, r0.xyzz, c0
bcaaaaaaaaaaabacafaaaakeacaaaaaaabaaaaoeaeaaaaaa dp3 r0.x, r5.xyzz, v1
ahaaaaaaaaaaabacaaaaaaaaacaaaaaaaeaaaappabaaaaaa max r0.x, r0.x, c4.w
adaaaaaaadaaahacacaaaakeacaaaaaaaaaaaaoeabaaaaaa mul r3.xyz, r2.xyzz, c0
adaaaaaaaaaaahacadaaaakeacaaaaaaaaaaaaaaacaaaaaa mul r0.xyz, r3.xyzz, r0.x
abaaaaaaaaaaahacaaaaaakeacaaaaaaabaaaakeacaaaaaa add r0.xyz, r0.xyzz, r1.xyzz
adaaaaaaabaaahacacaaaakeacaaaaaaacaaaaoeaeaaaaaa mul r1.xyz, r2.xyzz, v2
aaaaaaaaaaaaaiacaeaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0.w, c4
adaaaaaaaaaaahacaaaaaakeacaaaaaaaeaaaaaaabaaaaaa mul r0.xyz, r0.xyzz, c4.x
abaaaaaaaaaaahacaaaaaakeacaaaaaaabaaaakeacaaaaaa add r0.xyz, r0.xyzz, r1.xyzz
aaaaaaaaaaaaapadaaaaaaoeacaaaaaaaaaaaaaaaaaaaaaa mov o0, r0
"
}

SubProgram "d3d11_9x " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
ConstBuffer "$Globals" 96 // 72 used size, 7 vars
Vector 16 [_LightColor0] 4
Vector 48 [_Color] 4
Float 64 [_Shininess]
Float 68 [_Gloss]
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 0
SetTexture 1 [_SpecMap] 2D 2
SetTexture 2 [_BumpMap] 2D 1
// 31 instructions, 3 temp regs, 0 temp arrays:
// ALU 20 float, 0 int, 0 uint
// TEX 3 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0_level_9_3
eefiecedidigkfogpbgkmjhaecdmmijcfagppcipabaaaaaaceaiaaaaaeaaaaaa
daaaaaaapeacaaaafaahaaaapaahaaaaebgpgodjlmacaaaalmacaaaaaaacpppp
heacaaaaeiaaaaaaacaadaaaaaaaeiaaaaaaeiaaadaaceaaaaaaeiaaaaaaaaaa
acababaaabacacaaaaaaabaaabaaaaaaaaaaaaaaaaaaadaaacaaabaaaaaaaaaa
abacppppfbaaaaafadaaapkaaaaaaaeaaaaaialpaaaaiadpaaaaaaaafbaaaaaf
aeaaapkaaaaaaaecaaaaaaaaaaaaaaaaaaaaaaaabpaaaaacaaaaaaiaaaaaadla
bpaaaaacaaaaaaiaabaachlabpaaaaacaaaaaaiaacaachlabpaaaaacaaaaaaia
adaachlabpaaaaacaaaaaajaaaaiapkabpaaaaacaaaaaajaabaiapkabpaaaaac
aaaaaajaacaiapkaaiaaaaadaaaaciiaadaaoelaadaaoelaahaaaaacaaaacbia
aaaappiaabaaaaacabaaahiaadaaoelaaeaaaaaeaaaachiaabaaoeiaaaaaaaia
abaaoelaceaaaaacabaachiaaaaaoeiaecaaaaadaaaacpiaaaaaoelaabaioeka
aeaaaaaeaaaacdiaaaaaohiaadaaaakaadaaffkaaeaaaaaeaaaaciiaaaaaaaia
aaaaaaibadaakkkaaeaaaaaeaaaaciiaaaaaffiaaaaaffibaaaappiaahaaaaac
aaaaciiaaaaappiaagaaaaacaaaaceiaaaaappiaaiaaaaadaaaaciiaaaaaoeia
abaaoeiaaiaaaaadaaaacbiaaaaaoeiaabaaoelaalaaaaadabaacbiaaaaaaaia
adaappkaalaaaaadabaaaciaaaaappiaadaappkaabaaaaacaaaaabiaacaaaaka
afaaaaadaaaaabiaaaaaaaiaaeaaaakacaaaaaadacaaaiiaabaaffiaaaaaaaia
ecaaaaadaaaacpiaaaaaoelaaaaioekaecaaaaadadaacpiaaaaaoelaacaioeka
afaaaaadabaacoiaaaaappiaadaajaiaafaaaaadaaaachiaaaaaoeiaabaaoeka
afaaaaadabaacoiaabaaoeiaacaaffkaafaaaaadabaacoiaabaaoeiaacaappia
afaaaaadabaacoiaabaaoeiaaaaajakaafaaaaadacaachiaaaaaoeiaaaaaoeka
aeaaaaaeabaachiaacaaoeiaabaaaaiaabaapjiaacaaaaadabaachiaabaaoeia
abaaoeiaaeaaaaaeaaaachiaaaaaoeiaacaaoelaabaaoeiaabaaaaacaaaaciia
adaappkaabaaaaacaaaicpiaaaaaoeiappppaaaafdeieefcfeaeaaaaeaaaaaaa
bfabaaaafjaaaaaeegiocaaaaaaaaaaaafaaaaaafkaaaaadaagabaaaaaaaaaaa
fkaaaaadaagabaaaabaaaaaafkaaaaadaagabaaaacaaaaaafibiaaaeaahabaaa
aaaaaaaaffffaaaafibiaaaeaahabaaaabaaaaaaffffaaaafibiaaaeaahabaaa
acaaaaaaffffaaaagcbaaaaddcbabaaaabaaaaaagcbaaaadhcbabaaaacaaaaaa
gcbaaaadhcbabaaaadaaaaaagcbaaaadhcbabaaaaeaaaaaagfaaaaadpccabaaa
aaaaaaaagiaaaaacadaaaaaabaaaaaahbcaabaaaaaaaaaaaegbcbaaaaeaaaaaa
egbcbaaaaeaaaaaaeeaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaadcaaaaaj
hcaabaaaaaaaaaaaegbcbaaaaeaaaaaaagaabaaaaaaaaaaaegbcbaaaacaaaaaa
baaaaaahicaabaaaaaaaaaaaegacbaaaaaaaaaaaegacbaaaaaaaaaaaeeaaaaaf
icaabaaaaaaaaaaadkaabaaaaaaaaaaadiaaaaahhcaabaaaaaaaaaaapgapbaaa
aaaaaaaaegacbaaaaaaaaaaaefaaaaajpcaabaaaabaaaaaaegbabaaaabaaaaaa
eghobaaaacaaaaaaaagabaaaabaaaaaadcaaaaapdcaabaaaabaaaaaahgapbaaa
abaaaaaaaceaaaaaaaaaaaeaaaaaaaeaaaaaaaaaaaaaaaaaaceaaaaaaaaaialp
aaaaialpaaaaaaaaaaaaaaaadcaaaaakicaabaaaaaaaaaaaakaabaiaebaaaaaa
abaaaaaaakaabaaaabaaaaaaabeaaaaaaaaaiadpdcaaaaakicaabaaaaaaaaaaa
bkaabaiaebaaaaaaabaaaaaabkaabaaaabaaaaaadkaabaaaaaaaaaaaelaaaaaf
ecaabaaaabaaaaaadkaabaaaaaaaaaaabaaaaaahbcaabaaaaaaaaaaaegacbaaa
abaaaaaaegacbaaaaaaaaaaabaaaaaahccaabaaaaaaaaaaaegacbaaaabaaaaaa
egbcbaaaacaaaaaadeaaaaakdcaabaaaaaaaaaaaegaabaaaaaaaaaaaaceaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaacpaaaaafbcaabaaaaaaaaaaaakaabaaa
aaaaaaaadiaaaaaiecaabaaaaaaaaaaaakiacaaaaaaaaaaaaeaaaaaaabeaaaaa
aaaaaaecdiaaaaahbcaabaaaaaaaaaaaakaabaaaaaaaaaaackaabaaaaaaaaaaa
bjaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaaefaaaaajpcaabaaaabaaaaaa
egbabaaaabaaaaaaeghobaaaabaaaaaaaagabaaaacaaaaaaefaaaaajpcaabaaa
acaaaaaaegbabaaaabaaaaaaeghobaaaaaaaaaaaaagabaaaaaaaaaaadiaaaaah
hcaabaaaabaaaaaaegacbaaaabaaaaaapgapbaaaacaaaaaadiaaaaaihcaabaaa
acaaaaaaegacbaaaacaaaaaaegiccaaaaaaaaaaaadaaaaaadiaaaaaihcaabaaa
abaaaaaaegacbaaaabaaaaaafgifcaaaaaaaaaaaaeaaaaaadiaaaaahncaabaaa
aaaaaaaaagaabaaaaaaaaaaaagajbaaaabaaaaaadiaaaaaincaabaaaaaaaaaaa
agaobaaaaaaaaaaaagijcaaaaaaaaaaaabaaaaaadiaaaaaihcaabaaaabaaaaaa
egacbaaaacaaaaaaegiccaaaaaaaaaaaabaaaaaadcaaaaajhcaabaaaaaaaaaaa
egacbaaaabaaaaaafgafbaaaaaaaaaaaigadbaaaaaaaaaaaaaaaaaahhcaabaaa
aaaaaaaaegacbaaaaaaaaaaaegacbaaaaaaaaaaadcaaaaajhccabaaaaaaaaaaa
egacbaaaacaaaaaaegbcbaaaadaaaaaaegacbaaaaaaaaaaadgaaaaaficcabaaa
aaaaaaaaabeaaaaaaaaaaaaadoaaaaabejfdeheojiaaaaaaafaaaaaaaiaaaaaa
iaaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaabaaaaaaadadaaaaimaaaaaaabaaaaaaaaaaaaaaadaaaaaa
acaaaaaaahahaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaahahaaaa
imaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaahahaaaafdfgfpfaepfdejfe
ejepeoaafeeffiedepepfceeaaklklklepfdeheocmaaaaaaabaaaaaaaiaaaaaa
caaaaaaaaaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaafdfgfpfegbhcghgf
heaaklkl"
}

SubProgram "opengl " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Vector 0 [_Color]
SetTexture 0 [_MainTex] 2D
SetTexture 2 [unity_Lightmap] 2D
"!!ARBfp1.0
OPTION ARB_precision_hint_fastest;
# 7 ALU, 2 TEX
PARAM c[2] = { program.local[0],
		{ 0, 8 } };
TEMP R0;
TEMP R1;
TEX R1.xyz, fragment.texcoord[0], texture[0], 2D;
TEX R0, fragment.texcoord[1], texture[2], 2D;
MUL R1.xyz, R1, c[0];
MUL R0.xyz, R0.w, R0;
MUL R0.xyz, R0, R1;
MUL result.color.xyz, R0, c[1].y;
MOV result.color.w, c[1].x;
END
# 7 instructions, 2 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Vector 0 [_Color]
SetTexture 0 [_MainTex] 2D
SetTexture 2 [unity_Lightmap] 2D
"ps_2_0
; 6 ALU, 2 TEX
dcl_2d s0
dcl_2d s2
def c1, 8.00000000, 0.00000000, 0, 0
dcl t0.xy
dcl t1.xy
texld r1, t0, s0
texld r0, t1, s2
mul_pp r0.xyz, r0.w, r0
mul_pp r1.xyz, r1, c0
mul_pp r0.xyz, r0, r1
mul_pp r0.xyz, r0, c1.x
mov_pp r0.w, c1.y
mov_pp oC0, r0
"
}

SubProgram "xbox360 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Vector 0 [_Color]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [unity_Lightmap] 2D
// Shader Timing Estimate, in Cycles/64 pixel vector:
// ALU: 4.00 (3 instructions), vertex: 0, texture: 8,
//   sequencer: 6, interpolator: 8;    3 GPRs, 63 threads,
// Performance (if enough threads): ~8 cycles per vector
// * Texture cycle estimates are assuming an 8bit/component texture with no
//     aniso or trilinear filtering.

"ps_360
backbbaaaaaaabciaaaaaakaaaaaaaaaaaaaaaceaaaaaaniaaaaabaaaaaaaaaa
aaaaaaaaaaaaaalaaaaaaabmaaaaaakcppppadaaaaaaaaadaaaaaabmaaaaaaaa
aaaaaajlaaaaaafiaaacaaaaaaabaaaaaaaaaagaaaaaaaaaaaaaaahaaaadaaaa
aaabaaaaaaaaaahmaaaaaaaaaaaaaaimaaadaaabaaabaaaaaaaaaahmaaaaaaaa
fpedgpgmgphcaaklaaabaaadaaabaaaeaaabaaaaaaaaaaaafpengbgjgofegfhi
aaklklklaaaeaaamaaabaaabaaabaaaaaaaaaaaahfgogjhehjfpemgjghgihegn
gbhaaahahdfpddfpdaaadccodacodcdadddfddcodaaaklklaaaaaaaaaaaaaaab
aaaaaaaaaaaaaaaaaaaaaabeabpmaabaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaeaaaaaaagabaaaacaaaaaaaaaeaaaaaaaaaaaabaecaaadaaadaaaaaaab
aaaadafaaaaadbfbaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaebaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaafcaacaaaabcaameaaaaaaaaaadaaeaaaaccaaaaaaaaaa
baaicaabbpbppoiiaaaaeaaababiaacbbpbppgiiaaaaeaaakiihababaamamaed
ibacaappmiahaaaaaablmaaaobabaaaamiahmaaaaamamaaaobabaaaaaaaaaaaa
aaaaaaaaaaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Vector 0 [_Color]
SetTexture 0 [_MainTex] 2D
SetTexture 2 [unity_Lightmap] 2D
"sce_fp_rsx // 8 instructions using 2 registers
[Configuration]
24
ffffffff0000c0200003ffff000000000000840002000000
[Offsets]
1
_Color 1 0
00000020
[Microcode]
128
8e001700c8011c9dc8000001c8003fe10e800240c8001c9dc8020001c8000001
0000000000000000000000000000000010800140c8021c9dc8000001c8000001
00000000000000000000000000000000be021704c8011c9dc8000001c8003fe1
0e800240fe041c9dc9000001c80000010e810240c9001c9dc8043001c8000001
"
}

SubProgram "d3d11 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
ConstBuffer "$Globals" 112 // 64 used size, 8 vars
Vector 48 [_Color] 4
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 0
SetTexture 1 [unity_Lightmap] 2D 1
// 8 instructions, 2 temp regs, 0 temp arrays:
// ALU 4 float, 0 int, 0 uint
// TEX 2 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0
eefiecedeidfnnakeahgnoeplpplhdofbaokpjbmabaaaaaaciacaaaaadaaaaaa
cmaaaaaajmaaaaaanaaaaaaaejfdeheogiaaaaaaadaaaaaaaiaaaaaafaaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaafmaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaadadaaaafmaaaaaaabaaaaaaaaaaaaaaadaaaaaaabaaaaaa
amamaaaafdfgfpfaepfdejfeejepeoaafeeffiedepepfceeaaklklklepfdeheo
cmaaaaaaabaaaaaaaiaaaaaacaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaaaaaaaaa
apaaaaaafdfgfpfegbhcghgfheaaklklfdeieefcfaabaaaaeaaaaaaafeaaaaaa
fjaaaaaeegiocaaaaaaaaaaaaeaaaaaafkaaaaadaagabaaaaaaaaaaafkaaaaad
aagabaaaabaaaaaafibiaaaeaahabaaaaaaaaaaaffffaaaafibiaaaeaahabaaa
abaaaaaaffffaaaagcbaaaaddcbabaaaabaaaaaagcbaaaadmcbabaaaabaaaaaa
gfaaaaadpccabaaaaaaaaaaagiaaaaacacaaaaaaefaaaaajpcaabaaaaaaaaaaa
ogbkbaaaabaaaaaaeghobaaaabaaaaaaaagabaaaabaaaaaadiaaaaahicaabaaa
aaaaaaaadkaabaaaaaaaaaaaabeaaaaaaaaaaaebdiaaaaahhcaabaaaaaaaaaaa
egacbaaaaaaaaaaapgapbaaaaaaaaaaaefaaaaajpcaabaaaabaaaaaaegbabaaa
abaaaaaaeghobaaaaaaaaaaaaagabaaaaaaaaaaadiaaaaaihcaabaaaabaaaaaa
egacbaaaabaaaaaaegiccaaaaaaaaaaaadaaaaaadiaaaaahhccabaaaaaaaaaaa
egacbaaaaaaaaaaaegacbaaaabaaaaaadgaaaaaficcabaaaaaaaaaaaabeaaaaa
aaaaaaaadoaaaaab"
}

SubProgram "gles " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
"!!GLES"
}

SubProgram "glesdesktop " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
"!!GLES"
}

SubProgram "flash " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Vector 0 [_Color]
SetTexture 0 [_MainTex] 2D
SetTexture 2 [unity_Lightmap] 2D
"agal_ps
c1 8.0 0.0 0.0 0.0
[bc]
ciaaaaaaabaaapacaaaaaaoeaeaaaaaaaaaaaaaaafaababb tex r1, v0, s0 <2d wrap linear point>
ciaaaaaaaaaaapacabaaaaoeaeaaaaaaacaaaaaaafaababb tex r0, v1, s2 <2d wrap linear point>
adaaaaaaaaaaahacaaaaaappacaaaaaaaaaaaakeacaaaaaa mul r0.xyz, r0.w, r0.xyzz
adaaaaaaabaaahacabaaaakeacaaaaaaaaaaaaoeabaaaaaa mul r1.xyz, r1.xyzz, c0
adaaaaaaaaaaahacaaaaaakeacaaaaaaabaaaakeacaaaaaa mul r0.xyz, r0.xyzz, r1.xyzz
adaaaaaaaaaaahacaaaaaakeacaaaaaaabaaaaaaabaaaaaa mul r0.xyz, r0.xyzz, c1.x
aaaaaaaaaaaaaiacabaaaaffabaaaaaaaaaaaaaaaaaaaaaa mov r0.w, c1.y
aaaaaaaaaaaaapadaaaaaaoeacaaaaaaaaaaaaaaaaaaaaaa mov o0, r0
"
}

SubProgram "d3d11_9x " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
ConstBuffer "$Globals" 112 // 64 used size, 8 vars
Vector 48 [_Color] 4
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 0
SetTexture 1 [unity_Lightmap] 2D 1
// 8 instructions, 2 temp regs, 0 temp arrays:
// ALU 4 float, 0 int, 0 uint
// TEX 2 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0_level_9_3
eefiecedhldnpfmbfoebmlbaikfhlmmkddfcalidabaaaaaadeadaaaaaeaaaaaa
daaaaaaadiabaaaajaacaaaaaaadaaaaebgpgodjaaabaaaaaaabaaaaaaacpppp
miaaaaaadiaaaaaaabaacmaaaaaadiaaaaaadiaaacaaceaaaaaadiaaaaaaaaaa
abababaaaaaaadaaabaaaaaaaaaaaaaaabacppppfbaaaaafabaaapkaaaaaaaeb
aaaaaaaaaaaaaaaaaaaaaaaabpaaaaacaaaaaaiaaaaaaplabpaaaaacaaaaaaja
aaaiapkabpaaaaacaaaaaajaabaiapkaabaaaaacaaaaadiaaaaaollaecaaaaad
abaacpiaaaaaoelaaaaioekaecaaaaadaaaacpiaaaaaoeiaabaioekaafaaaaad
aaaaciiaaaaappiaabaaaakaafaaaaadaaaachiaaaaaoeiaaaaappiaafaaaaad
abaachiaabaaoeiaaaaaoekaafaaaaadaaaachiaaaaaoeiaabaaoeiaabaaaaac
aaaaciiaabaaffkaabaaaaacaaaicpiaaaaaoeiappppaaaafdeieefcfaabaaaa
eaaaaaaafeaaaaaafjaaaaaeegiocaaaaaaaaaaaaeaaaaaafkaaaaadaagabaaa
aaaaaaaafkaaaaadaagabaaaabaaaaaafibiaaaeaahabaaaaaaaaaaaffffaaaa
fibiaaaeaahabaaaabaaaaaaffffaaaagcbaaaaddcbabaaaabaaaaaagcbaaaad
mcbabaaaabaaaaaagfaaaaadpccabaaaaaaaaaaagiaaaaacacaaaaaaefaaaaaj
pcaabaaaaaaaaaaaogbkbaaaabaaaaaaeghobaaaabaaaaaaaagabaaaabaaaaaa
diaaaaahicaabaaaaaaaaaaadkaabaaaaaaaaaaaabeaaaaaaaaaaaebdiaaaaah
hcaabaaaaaaaaaaaegacbaaaaaaaaaaapgapbaaaaaaaaaaaefaaaaajpcaabaaa
abaaaaaaegbabaaaabaaaaaaeghobaaaaaaaaaaaaagabaaaaaaaaaaadiaaaaai
hcaabaaaabaaaaaaegacbaaaabaaaaaaegiccaaaaaaaaaaaadaaaaaadiaaaaah
hccabaaaaaaaaaaaegacbaaaaaaaaaaaegacbaaaabaaaaaadgaaaaaficcabaaa
aaaaaaaaabeaaaaaaaaaaaaadoaaaaabejfdeheogiaaaaaaadaaaaaaaiaaaaaa
faaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaafmaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaabaaaaaaadadaaaafmaaaaaaabaaaaaaaaaaaaaaadaaaaaa
abaaaaaaamamaaaafdfgfpfaepfdejfeejepeoaafeeffiedepepfceeaaklklkl
epfdeheocmaaaaaaabaaaaaaaiaaaaaacaaaaaaaaaaaaaaaaaaaaaaaadaaaaaa
aaaaaaaaapaaaaaafdfgfpfegbhcghgfheaaklkl"
}

SubProgram "opengl " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_ShadowMapTexture] 2D
"!!ARBfp1.0
OPTION ARB_precision_hint_fastest;
# 35 ALU, 4 TEX
PARAM c[5] = { program.local[0..3],
		{ 2, 1, 0, 32 } };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
TEX R0, fragment.texcoord[0], texture[0], 2D;
TEX R1.yw, fragment.texcoord[0], texture[2], 2D;
TEX R2.xyz, fragment.texcoord[0], texture[1], 2D;
TXP R3.x, fragment.texcoord[4], texture[3], 2D;
MAD R1.xy, R1.wyzw, c[4].x, -c[4].y;
MUL R2.xyz, R0.w, R2;
DP3 R1.w, fragment.texcoord[3], fragment.texcoord[3];
MUL R1.z, R1.y, R1.y;
RSQ R1.w, R1.w;
MOV R3.yzw, fragment.texcoord[1].xxyz;
MAD R3.yzw, R1.w, fragment.texcoord[3].xxyz, R3;
MAD R1.w, -R1.x, R1.x, -R1.z;
DP3 R1.z, R3.yzww, R3.yzww;
ADD R2.w, R1, c[4].y;
RSQ R1.w, R1.z;
RSQ R1.z, R2.w;
MOV R2.w, c[4];
RCP R1.z, R1.z;
MUL R3.yzw, R1.w, R3;
DP3 R1.w, R1, R3.yzww;
MUL R0.xyz, R0, c[1];
MAX R1.w, R1, c[4].z;
MUL R0.w, R2, c[2].x;
POW R0.w, R1.w, R0.w;
MUL R2.xyz, R2, c[3].x;
MUL R2.xyz, R0.w, R2;
DP3 R0.w, R1, fragment.texcoord[1];
MUL R1.xyz, R0, c[0];
MUL R2.xyz, R2, c[0];
MAX R0.w, R0, c[4].z;
MAD R1.xyz, R1, R0.w, R2;
MUL R0.xyz, R0, fragment.texcoord[2];
MUL R1.xyz, R3.x, R1;
MAD result.color.xyz, R1, c[4].x, R0;
MOV result.color.w, c[4].z;
END
# 35 instructions, 4 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_ShadowMapTexture] 2D
"ps_2_0
; 36 ALU, 4 TEX
dcl_2d s0
dcl_2d s1
dcl_2d s2
dcl_2d s3
def c4, 2.00000000, -1.00000000, 1.00000000, 0.00000000
def c5, 32.00000000, 0, 0, 0
dcl t0.xy
dcl t1.xyz
dcl t2.xyz
dcl t3.xyz
dcl t4
texld r0, t0, s2
texldp r6, t4, s3
texld r2, t0, s0
texld r3, t0, s1
mov r0.x, r0.w
mad_pp r4.xy, r0, c4.x, c4.y
mul_pp r0.x, r4.y, r4.y
mad_pp r0.x, -r4, r4, -r0
dp3_pp r1.x, t3, t3
add_pp r0.x, r0, c4.z
rsq_pp r0.x, r0.x
rcp_pp r4.z, r0.x
mov_pp r0.x, c2
rsq_pp r1.x, r1.x
mov_pp r5.xyz, t1
mad_pp r5.xyz, r1.x, t3, r5
dp3_pp r1.x, r5, r5
rsq_pp r1.x, r1.x
mul_pp r1.xyz, r1.x, r5
dp3_pp r1.x, r4, r1
mul_pp r0.x, c5, r0
max_pp r1.x, r1, c4.w
pow r5.x, r1.x, r0.x
mul_pp r0.xyz, r2.w, r3
mul_pp r2.xyz, r2, c1
mul_pp r1.xyz, r0, c3.x
mov r0.x, r5.x
mul r0.xyz, r0.x, r1
mul_pp r1.xyz, r0, c0
dp3_pp r0.x, r4, t1
max_pp r0.x, r0, c4.w
mul_pp r3.xyz, r2, c0
mad_pp r0.xyz, r3, r0.x, r1
mul_pp r0.xyz, r6.x, r0
mul_pp r1.xyz, r2, t2
mov_pp r0.w, c4
mad_pp r0.xyz, r0, c4.x, r1
mov_pp oC0, r0
"
}

SubProgram "xbox360 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Vector 1 [_Color]
Float 3 [_Gloss]
Vector 0 [_LightColor0]
Float 2 [_Shininess]
SetTexture 0 [_ShadowMapTexture] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_SpecMap] 2D
// Shader Timing Estimate, in Cycles/64 pixel vector:
// ALU: 25.33 (19 instructions), vertex: 0, texture: 16,
//   sequencer: 10, interpolator: 20;    8 GPRs, 24 threads,
// Performance (if enough threads): ~25 cycles per vector
// * Texture cycle estimates are assuming an 8bit/component texture with no
//     aniso or trilinear filtering.

"ps_360
backbbaaaaaaabnmaaaaabieaaaaaaaaaaaaaaceaaaaabiaaaaaabkiaaaaaaaa
aaaaaaaaaaaaabfiaaaaaabmaaaaabelppppadaaaaaaaaaiaaaaaabmaaaaaaaa
aaaaabeeaaaaaalmaaadaaacaaabaaaaaaaaaamiaaaaaaaaaaaaaaniaaacaaab
aaabaaaaaaaaaaoaaaaaaaaaaaaaaapaaaacaaadaaabaaaaaaaaaapiaaaaaaaa
aaaaabaiaaacaaaaaaabaaaaaaaaaaoaaaaaaaaaaaaaabbfaaadaaabaaabaaaa
aaaaaamiaaaaaaaaaaaaabboaaadaaaaaaabaaaaaaaaaamiaaaaaaaaaaaaabda
aaacaaacaaabaaaaaaaaaapiaaaaaaaaaaaaabdlaaadaaadaaabaaaaaaaaaami
aaaaaaaafpechfgnhaengbhaaaklklklaaaeaaamaaabaaabaaabaaaaaaaaaaaa
fpedgpgmgphcaaklaaabaaadaaabaaaeaaabaaaaaaaaaaaafpehgmgphdhdaakl
aaaaaaadaaabaaabaaabaaaaaaaaaaaafpemgjghgiheedgpgmgphcdaaafpengb
gjgofegfhiaafpfdgigbgegphhengbhafegfhihehfhcgfaafpfdgigjgogjgogf
hdhdaafpfdhagfgdengbhaaahahdfpddfpdaaadccodacodcdadddfddcodaaakl
aaaaaaaaaaaaaaabaaaaaaaaaaaaaaaaaaaaaabeabpmaabaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaeaaaaaabeebaaaahaaaaaaaaaeaaaaaaaaaaaadmkf
aabpaabpaaaaaaabaaaadafaaaaahbfbaaaahcfcaaaahdfdaaaapefeaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaalpiaaaaaaaaaaaaadpiaaaaaecaaaaaaaffagaad
aaaabcaameaaaaaaaaaagaajgaapbcaabcaaaaaaaaaafabfaaaaccaaaaaaaaaa
emebaaahaagmblblcbacppaemiamaaaaaamgkmaaobaaaeaaliaifaabbpbppppi
aaaaeaaabacifaabbpbppompaaaaeaaabadigaabbpbppoiiaaaaeaaababieaab
bpbppgbbaaaaeaaabeibabaaaalolomgpaadadaekmboagaaaablpmedmbaeagab
fibhaaafaalelegmoaafafiamiaoaaahaagmpmpmolaaadabkicbagaaaamdmdie
naahahabfiifabadaamfgmgmkaafppiamiabaaaaaegogomgnbadadppkaioadah
aaabblgmobahabiakiecagadaamdmpmfnaahadabkibeadadaamploebnaadabad
kicgadabaamblbecicadppadeaboabahaapmpmlbkbagaaibkiepadabaaaameed
mbahabaddiihaaaaaamamagmobagacabmiahaaacaamablaaobadaaaamiahaaab
aamamabfklacaaabmiahmaaaaamagmmaolabafaaaaaaaaaaaaaaaaaaaaaaaaaa
"
}

SubProgram "ps3 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_ShadowMapTexture] 2D
"sce_fp_rsx // 43 instructions using 4 registers
[Configuration]
24
ffffffff0007c020001fffe1000000000000840004000000
[Offsets]
4
_LightColor0 2 0
00000290000001a0
_Color 1 0
00000020
_Shininess 1 0
00000160
_Gloss 1 0
00000080
[Microcode]
688
9e001700c8011c9dc8000001c8003fe10e880240c8001c9dc8020001c8000001
0000000000000000000000000000000094021704c8011c9dc8000001c8003fe1
ee803940c8011c9dc8000029c800bfe106840440ce041c9daa02000054020001
00000000000040000000bf800000000008820240fe001c9d00020000c8000001
0000000000000000000000000000000010800240ab081c9cab080000c8000001
ae860140c8011c9dc8000001c8003fe11082044001081c9e01080000c9000003
0e000340c90c1c9dc9000001c80000018e061702c8011c9dc8000001c8003fe1
0e803940c8001c9dc8000029c800000110800340c9041c9d00020000c8000001
00003f800000000000000000000000000e82024055041c9dc80c0001c8000001
08843b40ff003c9dff000001c800000110880540c9081c9dc90c0001c8000001
10820540c9081c9dc9000001c80000011080014000021c9cc8000001c8000001
0000000000000000000000000000000010800240c9001c9dc8020001c8000001
000000000000000000000000000042000e800240c9101c9dc8020001c8000001
0000000000000000000000000000000010020900c9041c9d00020000c8000001
0000000000000000000000000000000002840900ff101c9d00020000c8000001
000000000000000000000000000000000e840240c9001c9d01080000c8000001
02001d00fe041c9dc8000001c80000011002020000001c9cc9000001c8000001
ce800240c9101c9dc8015001c8003fe108021c00fe041c9dc8000001c8000001
02041807c8011c9dc8000001c8003fe10e82020054041c9dc9040001c8000001
1080014000021c9cc8000001c800000100000000000000000000000000000000
0e820440c9041c9dc8020001c908000100000000000000000000000000000000
0e81044000081c9cc9041001c9000001
"
}

SubProgram "d3d11 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
ConstBuffer "$Globals" 160 // 136 used size, 8 vars
Vector 16 [_LightColor0] 4
Vector 112 [_Color] 4
Float 128 [_Shininess]
Float 132 [_Gloss]
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 1
SetTexture 1 [_SpecMap] 2D 3
SetTexture 2 [_BumpMap] 2D 2
SetTexture 3 [_ShadowMapTexture] 2D 0
// 34 instructions, 3 temp regs, 0 temp arrays:
// ALU 22 float, 0 int, 0 uint
// TEX 4 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0
eefiecedfphfhcjogahpibcjmeijlbjejgknbjaaabaaaaaapiafaaaaadaaaaaa
cmaaaaaaoeaaaaaabiabaaaaejfdeheolaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaakeaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaadadaaaakeaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaakeaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaahahaaaakeaaaaaa
adaaaaaaaaaaaaaaadaaaaaaaeaaaaaaahahaaaakeaaaaaaaeaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapalaaaafdfgfpfaepfdejfeejepeoaafeeffiedepepfcee
aaklklklepfdeheocmaaaaaaabaaaaaaaiaaaaaacaaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaaaaaaaaaapaaaaaafdfgfpfegbhcghgfheaaklklfdeieefcniaeaaaa
eaaaaaaadgabaaaafjaaaaaeegiocaaaaaaaaaaaajaaaaaafkaaaaadaagabaaa
aaaaaaaafkaaaaadaagabaaaabaaaaaafkaaaaadaagabaaaacaaaaaafkaaaaad
aagabaaaadaaaaaafibiaaaeaahabaaaaaaaaaaaffffaaaafibiaaaeaahabaaa
abaaaaaaffffaaaafibiaaaeaahabaaaacaaaaaaffffaaaafibiaaaeaahabaaa
adaaaaaaffffaaaagcbaaaaddcbabaaaabaaaaaagcbaaaadhcbabaaaacaaaaaa
gcbaaaadhcbabaaaadaaaaaagcbaaaadhcbabaaaaeaaaaaagcbaaaadlcbabaaa
afaaaaaagfaaaaadpccabaaaaaaaaaaagiaaaaacadaaaaaabaaaaaahbcaabaaa
aaaaaaaaegbcbaaaaeaaaaaaegbcbaaaaeaaaaaaeeaaaaafbcaabaaaaaaaaaaa
akaabaaaaaaaaaaadcaaaaajhcaabaaaaaaaaaaaegbcbaaaaeaaaaaaagaabaaa
aaaaaaaaegbcbaaaacaaaaaabaaaaaahicaabaaaaaaaaaaaegacbaaaaaaaaaaa
egacbaaaaaaaaaaaeeaaaaaficaabaaaaaaaaaaadkaabaaaaaaaaaaadiaaaaah
hcaabaaaaaaaaaaapgapbaaaaaaaaaaaegacbaaaaaaaaaaaefaaaaajpcaabaaa
abaaaaaaegbabaaaabaaaaaaeghobaaaacaaaaaaaagabaaaacaaaaaadcaaaaap
dcaabaaaabaaaaaahgapbaaaabaaaaaaaceaaaaaaaaaaaeaaaaaaaeaaaaaaaaa
aaaaaaaaaceaaaaaaaaaialpaaaaialpaaaaaaaaaaaaaaaadcaaaaakicaabaaa
aaaaaaaaakaabaiaebaaaaaaabaaaaaaakaabaaaabaaaaaaabeaaaaaaaaaiadp
dcaaaaakicaabaaaaaaaaaaabkaabaiaebaaaaaaabaaaaaabkaabaaaabaaaaaa
dkaabaaaaaaaaaaaelaaaaafecaabaaaabaaaaaadkaabaaaaaaaaaaabaaaaaah
bcaabaaaaaaaaaaaegacbaaaabaaaaaaegacbaaaaaaaaaaabaaaaaahccaabaaa
aaaaaaaaegacbaaaabaaaaaaegbcbaaaacaaaaaadeaaaaakdcaabaaaaaaaaaaa
egaabaaaaaaaaaaaaceaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaacpaaaaaf
bcaabaaaaaaaaaaaakaabaaaaaaaaaaadiaaaaaiecaabaaaaaaaaaaaakiacaaa
aaaaaaaaaiaaaaaaabeaaaaaaaaaaaecdiaaaaahbcaabaaaaaaaaaaaakaabaaa
aaaaaaaackaabaaaaaaaaaaabjaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaa
efaaaaajpcaabaaaabaaaaaaegbabaaaabaaaaaaeghobaaaabaaaaaaaagabaaa
adaaaaaaefaaaaajpcaabaaaacaaaaaaegbabaaaabaaaaaaeghobaaaaaaaaaaa
aagabaaaabaaaaaadiaaaaahhcaabaaaabaaaaaaegacbaaaabaaaaaapgapbaaa
acaaaaaadiaaaaaihcaabaaaacaaaaaaegacbaaaacaaaaaaegiccaaaaaaaaaaa
ahaaaaaadiaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaafgifcaaaaaaaaaaa
aiaaaaaadiaaaaahncaabaaaaaaaaaaaagaabaaaaaaaaaaaagajbaaaabaaaaaa
diaaaaaincaabaaaaaaaaaaaagaobaaaaaaaaaaaagijcaaaaaaaaaaaabaaaaaa
diaaaaaihcaabaaaabaaaaaaegacbaaaacaaaaaaegiccaaaaaaaaaaaabaaaaaa
diaaaaahhcaabaaaacaaaaaaegacbaaaacaaaaaaegbcbaaaadaaaaaadcaaaaaj
hcaabaaaaaaaaaaaegacbaaaabaaaaaafgafbaaaaaaaaaaaigadbaaaaaaaaaaa
aoaaaaahdcaabaaaabaaaaaaegbabaaaafaaaaaapgbpbaaaafaaaaaaefaaaaaj
pcaabaaaabaaaaaaegaabaaaabaaaaaaeghobaaaadaaaaaaaagabaaaaaaaaaaa
aaaaaaahicaabaaaaaaaaaaaakaabaaaabaaaaaaakaabaaaabaaaaaadcaaaaaj
hccabaaaaaaaaaaaegacbaaaaaaaaaaapgapbaaaaaaaaaaaegacbaaaacaaaaaa
dgaaaaaficcabaaaaaaaaaaaabeaaaaaaaaaaaaadoaaaaab"
}

SubProgram "gles " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
"!!GLES"
}

SubProgram "glesdesktop " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
"!!GLES"
}

SubProgram "flash " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_ShadowMapTexture] 2D
"agal_ps
c4 2.0 -1.0 1.0 0.0
c5 32.0 0.0 0.0 0.0
[bc]
ciaaaaaaaaaaapacaaaaaaoeaeaaaaaaacaaaaaaafaababb tex r0, v0, s2 <2d wrap linear point>
aeaaaaaaabaaapacaeaaaaoeaeaaaaaaaeaaaappaeaaaaaa div r1, v4, v4.w
ciaaaaaaagaaapacabaaaafeacaaaaaaadaaaaaaafaababb tex r6, r1.xyyy, s3 <2d wrap linear point>
ciaaaaaaacaaapacaaaaaaoeaeaaaaaaaaaaaaaaafaababb tex r2, v0, s0 <2d wrap linear point>
ciaaaaaaadaaapacaaaaaaoeaeaaaaaaabaaaaaaafaababb tex r3, v0, s1 <2d wrap linear point>
aaaaaaaaaaaaabacaaaaaappacaaaaaaaaaaaaaaaaaaaaaa mov r0.x, r0.w
adaaaaaaaeaaadacaaaaaafeacaaaaaaaeaaaaaaabaaaaaa mul r4.xy, r0.xyyy, c4.x
abaaaaaaaeaaadacaeaaaafeacaaaaaaaeaaaaffabaaaaaa add r4.xy, r4.xyyy, c4.y
adaaaaaaaaaaabacaeaaaaffacaaaaaaaeaaaaffacaaaaaa mul r0.x, r4.y, r4.y
bfaaaaaaabaaaiacaeaaaaaaacaaaaaaaaaaaaaaaaaaaaaa neg r1.w, r4.x
adaaaaaaabaaaiacabaaaappacaaaaaaaeaaaaaaacaaaaaa mul r1.w, r1.w, r4.x
acaaaaaaaaaaabacabaaaappacaaaaaaaaaaaaaaacaaaaaa sub r0.x, r1.w, r0.x
bcaaaaaaabaaabacadaaaaoeaeaaaaaaadaaaaoeaeaaaaaa dp3 r1.x, v3, v3
abaaaaaaaaaaabacaaaaaaaaacaaaaaaaeaaaakkabaaaaaa add r0.x, r0.x, c4.z
akaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r0.x, r0.x
afaaaaaaaeaaaeacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rcp r4.z, r0.x
aaaaaaaaaaaaabacacaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0.x, c2
akaaaaaaabaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r1.x, r1.x
aaaaaaaaafaaahacabaaaaoeaeaaaaaaaaaaaaaaaaaaaaaa mov r5.xyz, v1
adaaaaaaagaaaoacabaaaaaaacaaaaaaadaaaaoeaeaaaaaa mul r6.yzw, r1.x, v3
abaaaaaaafaaahacagaaaapjacaaaaaaafaaaakeacaaaaaa add r5.xyz, r6.yzww, r5.xyzz
bcaaaaaaabaaabacafaaaakeacaaaaaaafaaaakeacaaaaaa dp3 r1.x, r5.xyzz, r5.xyzz
akaaaaaaabaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r1.x, r1.x
adaaaaaaabaaahacabaaaaaaacaaaaaaafaaaakeacaaaaaa mul r1.xyz, r1.x, r5.xyzz
bcaaaaaaabaaabacaeaaaakeacaaaaaaabaaaakeacaaaaaa dp3 r1.x, r4.xyzz, r1.xyzz
adaaaaaaaaaaabacafaaaaoeabaaaaaaaaaaaaaaacaaaaaa mul r0.x, c5, r0.x
ahaaaaaaabaaabacabaaaaaaacaaaaaaaeaaaappabaaaaaa max r1.x, r1.x, c4.w
alaaaaaaafaaapacabaaaaaaacaaaaaaaaaaaaaaacaaaaaa pow r5, r1.x, r0.x
adaaaaaaaaaaahacacaaaappacaaaaaaadaaaakeacaaaaaa mul r0.xyz, r2.w, r3.xyzz
adaaaaaaacaaahacacaaaakeacaaaaaaabaaaaoeabaaaaaa mul r2.xyz, r2.xyzz, c1
adaaaaaaabaaahacaaaaaakeacaaaaaaadaaaaaaabaaaaaa mul r1.xyz, r0.xyzz, c3.x
aaaaaaaaaaaaabacafaaaaaaacaaaaaaaaaaaaaaaaaaaaaa mov r0.x, r5.x
adaaaaaaaaaaahacaaaaaaaaacaaaaaaabaaaakeacaaaaaa mul r0.xyz, r0.x, r1.xyzz
adaaaaaaabaaahacaaaaaakeacaaaaaaaaaaaaoeabaaaaaa mul r1.xyz, r0.xyzz, c0
bcaaaaaaaaaaabacaeaaaakeacaaaaaaabaaaaoeaeaaaaaa dp3 r0.x, r4.xyzz, v1
ahaaaaaaaaaaabacaaaaaaaaacaaaaaaaeaaaappabaaaaaa max r0.x, r0.x, c4.w
adaaaaaaadaaahacacaaaakeacaaaaaaaaaaaaoeabaaaaaa mul r3.xyz, r2.xyzz, c0
adaaaaaaaaaaahacadaaaakeacaaaaaaaaaaaaaaacaaaaaa mul r0.xyz, r3.xyzz, r0.x
abaaaaaaaaaaahacaaaaaakeacaaaaaaabaaaakeacaaaaaa add r0.xyz, r0.xyzz, r1.xyzz
adaaaaaaaaaaahacagaaaaaaacaaaaaaaaaaaakeacaaaaaa mul r0.xyz, r6.x, r0.xyzz
adaaaaaaabaaahacacaaaakeacaaaaaaacaaaaoeaeaaaaaa mul r1.xyz, r2.xyzz, v2
aaaaaaaaaaaaaiacaeaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0.w, c4
adaaaaaaaaaaahacaaaaaakeacaaaaaaaeaaaaaaabaaaaaa mul r0.xyz, r0.xyzz, c4.x
abaaaaaaaaaaahacaaaaaakeacaaaaaaabaaaakeacaaaaaa add r0.xyz, r0.xyzz, r1.xyzz
aaaaaaaaaaaaapadaaaaaaoeacaaaaaaaaaaaaaaaaaaaaaa mov o0, r0
"
}

SubProgram "opengl " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Vector 0 [_Color]
SetTexture 0 [_MainTex] 2D
SetTexture 2 [_ShadowMapTexture] 2D
SetTexture 3 [unity_Lightmap] 2D
"!!ARBfp1.0
OPTION ARB_precision_hint_fastest;
# 13 ALU, 3 TEX
PARAM c[2] = { program.local[0],
		{ 0, 8, 2 } };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
TEX R2, fragment.texcoord[1], texture[3], 2D;
TEX R0.xyz, fragment.texcoord[0], texture[0], 2D;
TXP R3.x, fragment.texcoord[2], texture[2], 2D;
MUL R1.xyz, R2.w, R2;
MUL R2.xyz, R2, R3.x;
MUL R1.xyz, R1, c[1].y;
MUL R3.xyz, R1, R3.x;
MUL R2.xyz, R2, c[1].z;
MIN R1.xyz, R1, R2;
MAX R1.xyz, R1, R3;
MUL R0.xyz, R0, c[0];
MUL result.color.xyz, R0, R1;
MOV result.color.w, c[1].x;
END
# 13 instructions, 4 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Vector 0 [_Color]
SetTexture 0 [_MainTex] 2D
SetTexture 2 [_ShadowMapTexture] 2D
SetTexture 3 [unity_Lightmap] 2D
"ps_2_0
; 11 ALU, 3 TEX
dcl_2d s0
dcl_2d s2
dcl_2d s3
def c1, 8.00000000, 2.00000000, 0.00000000, 0
dcl t0.xy
dcl t1.xy
dcl t2
texld r1, t0, s0
texldp r3, t2, s2
texld r0, t1, s3
mul_pp r2.xyz, r0, r3.x
mul_pp r0.xyz, r0.w, r0
mul_pp r0.xyz, r0, c1.x
mul_pp r2.xyz, r2, c1.y
min_pp r2.xyz, r0, r2
mul_pp r0.xyz, r0, r3.x
max_pp r0.xyz, r2, r0
mul_pp r1.xyz, r1, c0
mul_pp r0.xyz, r1, r0
mov_pp r0.w, c1.z
mov_pp oC0, r0
"
}

SubProgram "xbox360 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Vector 0 [_Color]
SetTexture 0 [_ShadowMapTexture] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 2 [unity_Lightmap] 2D
// Shader Timing Estimate, in Cycles/64 pixel vector:
// ALU: 12.00 (9 instructions), vertex: 0, texture: 12,
//   sequencer: 8, interpolator: 12;    4 GPRs, 48 threads,
// Performance (if enough threads): ~12 cycles per vector
// * Texture cycle estimates are assuming an 8bit/component texture with no
//     aniso or trilinear filtering.

"ps_360
backbbaaaaaaabfaaaaaaapeaaaaaaaaaaaaaaceaaaaaapmaaaaabceaaaaaaaa
aaaaaaaaaaaaaaneaaaaaabmaaaaaamippppadaaaaaaaaaeaaaaaabmaaaaaaaa
aaaaaambaaaaaagmaaacaaaaaaabaaaaaaaaaaheaaaaaaaaaaaaaaieaaadaaab
aaabaaaaaaaaaajaaaaaaaaaaaaaaakaaaadaaaaaaabaaaaaaaaaajaaaaaaaaa
aaaaaalcaaadaaacaaabaaaaaaaaaajaaaaaaaaafpedgpgmgphcaaklaaabaaad
aaabaaaeaaabaaaaaaaaaaaafpengbgjgofegfhiaaklklklaaaeaaamaaabaaab
aaabaaaaaaaaaaaafpfdgigbgegphhengbhafegfhihehfhcgfaahfgogjhehjfp
emgjghgihegngbhaaahahdfpddfpdaaadccodacodcdadddfddcodaaaaaaaaaaa
aaaaaaabaaaaaaaaaaaaaaaaaaaaaabeabpmaabaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaeaaaaaaalebaaaadaaaaaaaaaeaaaaaaaaaaaacagdaaahaaah
aaaaaaabaaaadafaaaaadbfbaaaapcfcaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
ebaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaabfafaacaaaabcaameaaaaaaaaaagaah
baanbcaaccaaaaaaemeaaaaaaaaaaablocaaaaacmiamaaabaamgkmaaobaaacaa
babiaaabbpbppoecaaaaeaaabacicacbbpbppgiiaaaaeaaaliaiaacbbpbppbpp
aaaaeaaaaabiababaablgmblkbacppaamiahaaabaagmmaaaobabacaamiaoaaac
aablpmaaobabacaakibhacadaabfblebmbacaaaakichacabaabfmaicmdacabaa
kiehacabaamamamamcadabaamiahmaaaaamamaaaobacabaaaaaaaaaaaaaaaaaa
aaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Vector 0 [_Color]
SetTexture 0 [_MainTex] 2D
SetTexture 2 [_ShadowMapTexture] 2D
SetTexture 3 [unity_Lightmap] 2D
"sce_fp_rsx // 13 instructions using 3 registers
[Configuration]
24
ffffffff0001c0200007fffb000000000000840003000000
[Offsets]
1
_Color 1 0
00000020
[Microcode]
208
8e001700c8011c9dc8000001c8003fe10e840240c8001c9dc8020001c8000001
00000000000000000000000000000000be001706c8011c9dc8000001c8003fe1
0e860240fe001c9dc8003001c8000001c2041804c8011c9dc8000001c8003fe1
0e800240c8001c9d00081000c80000011080014000021c9cc8000001c8000001
000000000000000000000000000000000e8a0840c90c1c9dc9000001c8000001
0e860240c90c1c9d00080000c80000010e800940c9141c9dc90c0001c8000001
0e810240c9081c9dc9000001c8000001
"
}

SubProgram "d3d11 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
ConstBuffer "$Globals" 176 // 128 used size, 9 vars
Vector 112 [_Color] 4
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 1
SetTexture 1 [_ShadowMapTexture] 2D 0
SetTexture 2 [unity_Lightmap] 2D 2
// 15 instructions, 2 temp regs, 0 temp arrays:
// ALU 10 float, 0 int, 0 uint
// TEX 3 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0
eefiecedncdcodfnbkhncijfbfihhompegfhmiooabaaaaaadeadaaaaadaaaaaa
cmaaaaaaleaaaaaaoiaaaaaaejfdeheoiaaaaaaaaeaaaaaaaiaaaaaagiaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaheaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaadadaaaaheaaaaaaabaaaaaaaaaaaaaaadaaaaaaabaaaaaa
amamaaaaheaaaaaaacaaaaaaaaaaaaaaadaaaaaaacaaaaaaapalaaaafdfgfpfa
epfdejfeejepeoaafeeffiedepepfceeaaklklklepfdeheocmaaaaaaabaaaaaa
aiaaaaaacaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaafdfgfpfe
gbhcghgfheaaklklfdeieefceeacaaaaeaaaaaaajbaaaaaafjaaaaaeegiocaaa
aaaaaaaaaiaaaaaafkaaaaadaagabaaaaaaaaaaafkaaaaadaagabaaaabaaaaaa
fkaaaaadaagabaaaacaaaaaafibiaaaeaahabaaaaaaaaaaaffffaaaafibiaaae
aahabaaaabaaaaaaffffaaaafibiaaaeaahabaaaacaaaaaaffffaaaagcbaaaad
dcbabaaaabaaaaaagcbaaaadmcbabaaaabaaaaaagcbaaaadlcbabaaaacaaaaaa
gfaaaaadpccabaaaaaaaaaaagiaaaaacacaaaaaaaoaaaaahdcaabaaaaaaaaaaa
egbabaaaacaaaaaapgbpbaaaacaaaaaaefaaaaajpcaabaaaaaaaaaaaegaabaaa
aaaaaaaaeghobaaaabaaaaaaaagabaaaaaaaaaaaaaaaaaahccaabaaaaaaaaaaa
akaabaaaaaaaaaaaakaabaaaaaaaaaaaefaaaaajpcaabaaaabaaaaaaogbkbaaa
abaaaaaaeghobaaaacaaaaaaaagabaaaacaaaaaadiaaaaahocaabaaaaaaaaaaa
fgafbaaaaaaaaaaaagajbaaaabaaaaaadiaaaaahicaabaaaabaaaaaadkaabaaa
abaaaaaaabeaaaaaaaaaaaebdiaaaaahhcaabaaaabaaaaaaegacbaaaabaaaaaa
pgapbaaaabaaaaaaddaaaaahocaabaaaaaaaaaaafgaobaaaaaaaaaaaagajbaaa
abaaaaaadiaaaaahhcaabaaaabaaaaaaagaabaaaaaaaaaaaegacbaaaabaaaaaa
deaaaaahhcaabaaaaaaaaaaajgahbaaaaaaaaaaaegacbaaaabaaaaaaefaaaaaj
pcaabaaaabaaaaaaegbabaaaabaaaaaaeghobaaaaaaaaaaaaagabaaaabaaaaaa
diaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaaegiccaaaaaaaaaaaahaaaaaa
diaaaaahhccabaaaaaaaaaaaegacbaaaaaaaaaaaegacbaaaabaaaaaadgaaaaaf
iccabaaaaaaaaaaaabeaaaaaaaaaaaaadoaaaaab"
}

SubProgram "gles " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
"!!GLES"
}

SubProgram "glesdesktop " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
"!!GLES"
}

SubProgram "flash " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Vector 0 [_Color]
SetTexture 0 [_MainTex] 2D
SetTexture 2 [_ShadowMapTexture] 2D
SetTexture 3 [unity_Lightmap] 2D
"agal_ps
c1 8.0 2.0 0.0 0.0
[bc]
ciaaaaaaabaaapacaaaaaaoeaeaaaaaaaaaaaaaaafaababb tex r1, v0, s0 <2d wrap linear point>
aeaaaaaaaaaaapacacaaaaoeaeaaaaaaacaaaappaeaaaaaa div r0, v2, v2.w
ciaaaaaaadaaapacaaaaaafeacaaaaaaacaaaaaaafaababb tex r3, r0.xyyy, s2 <2d wrap linear point>
ciaaaaaaaaaaapacabaaaaoeaeaaaaaaadaaaaaaafaababb tex r0, v1, s3 <2d wrap linear point>
adaaaaaaacaaahacaaaaaakeacaaaaaaadaaaaaaacaaaaaa mul r2.xyz, r0.xyzz, r3.x
adaaaaaaaaaaahacaaaaaappacaaaaaaaaaaaakeacaaaaaa mul r0.xyz, r0.w, r0.xyzz
adaaaaaaaaaaahacaaaaaakeacaaaaaaabaaaaaaabaaaaaa mul r0.xyz, r0.xyzz, c1.x
adaaaaaaacaaahacacaaaakeacaaaaaaabaaaaffabaaaaaa mul r2.xyz, r2.xyzz, c1.y
agaaaaaaacaaahacaaaaaakeacaaaaaaacaaaakeacaaaaaa min r2.xyz, r0.xyzz, r2.xyzz
adaaaaaaaaaaahacaaaaaakeacaaaaaaadaaaaaaacaaaaaa mul r0.xyz, r0.xyzz, r3.x
ahaaaaaaaaaaahacacaaaakeacaaaaaaaaaaaakeacaaaaaa max r0.xyz, r2.xyzz, r0.xyzz
adaaaaaaabaaahacabaaaakeacaaaaaaaaaaaaoeabaaaaaa mul r1.xyz, r1.xyzz, c0
adaaaaaaaaaaahacabaaaakeacaaaaaaaaaaaakeacaaaaaa mul r0.xyz, r1.xyzz, r0.xyzz
aaaaaaaaaaaaaiacabaaaakkabaaaaaaaaaaaaaaaaaaaaaa mov r0.w, c1.z
aaaaaaaaaaaaapadaaaaaaoeacaaaaaaaaaaaaaaaaaaaaaa mov o0, r0
"
}

}
	}
	Pass {
		Name "FORWARD"
		Tags { "LightMode" = "ForwardAdd" }
		ZWrite Off Blend One One Fog { Color (0,0,0,0) }
Program "vp" {
// Vertex combos: 5
//   opengl - ALU: 25 to 34
//   d3d9 - ALU: 28 to 37
//   d3d11 - ALU: 12 to 14, TEX: 0 to 0, FLOW: 1 to 1
//   d3d11_9x - ALU: 12 to 14, TEX: 0 to 0, FLOW: 1 to 1
SubProgram "opengl " {
Keywords { "POINT" }
Bind "vertex" Vertex
Bind "tangent" ATTR14
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 17 [_WorldSpaceCameraPos]
Vector 18 [_WorldSpaceLightPos0]
Matrix 5 [_Object2World]
Matrix 9 [_World2Object]
Vector 19 [unity_Scale]
Matrix 13 [_LightMatrix0]
Vector 20 [_MainTex_ST]
"!!ARBvp1.0
# 33 ALU
PARAM c[21] = { { 1 },
		state.matrix.mvp,
		program.local[5..20] };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
MOV R1.xyz, c[17];
MOV R1.w, c[0].x;
MOV R0.xyz, vertex.attrib[14];
DP4 R2.z, R1, c[11];
DP4 R2.y, R1, c[10];
DP4 R2.x, R1, c[9];
MAD R2.xyz, R2, c[19].w, -vertex.position;
MUL R1.xyz, vertex.normal.zxyw, R0.yzxw;
MAD R1.xyz, vertex.normal.yzxw, R0.zxyw, -R1;
MOV R0, c[18];
MUL R1.xyz, R1, vertex.attrib[14].w;
DP4 R3.z, R0, c[11];
DP4 R3.x, R0, c[9];
DP4 R3.y, R0, c[10];
MAD R0.xyz, R3, c[19].w, -vertex.position;
DP3 result.texcoord[1].y, R0, R1;
DP3 result.texcoord[1].z, vertex.normal, R0;
DP3 result.texcoord[1].x, R0, vertex.attrib[14];
DP4 R0.w, vertex.position, c[8];
DP4 R0.z, vertex.position, c[7];
DP4 R0.x, vertex.position, c[5];
DP4 R0.y, vertex.position, c[6];
DP3 result.texcoord[2].y, R1, R2;
DP3 result.texcoord[2].z, vertex.normal, R2;
DP3 result.texcoord[2].x, vertex.attrib[14], R2;
DP4 result.texcoord[3].z, R0, c[15];
DP4 result.texcoord[3].y, R0, c[14];
DP4 result.texcoord[3].x, R0, c[13];
MAD result.texcoord[0].xy, vertex.texcoord[0], c[20], c[20].zwzw;
DP4 result.position.w, vertex.position, c[4];
DP4 result.position.z, vertex.position, c[3];
DP4 result.position.y, vertex.position, c[2];
DP4 result.position.x, vertex.position, c[1];
END
# 33 instructions, 4 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "POINT" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Matrix 0 [glstate_matrix_mvp]
Vector 16 [_WorldSpaceCameraPos]
Vector 17 [_WorldSpaceLightPos0]
Matrix 4 [_Object2World]
Matrix 8 [_World2Object]
Vector 18 [unity_Scale]
Matrix 12 [_LightMatrix0]
Vector 19 [_MainTex_ST]
"vs_2_0
; 36 ALU
def c20, 1.00000000, 0, 0, 0
dcl_position0 v0
dcl_tangent0 v1
dcl_normal0 v2
dcl_texcoord0 v3
mov r0.w, c20.x
mov r0.xyz, c16
dp4 r1.z, r0, c10
dp4 r1.y, r0, c9
dp4 r1.x, r0, c8
mad r3.xyz, r1, c18.w, -v0
mov r0.xyz, v1
mul r1.xyz, v2.zxyw, r0.yzxw
mov r0.xyz, v1
mad r1.xyz, v2.yzxw, r0.zxyw, -r1
mul r2.xyz, r1, v1.w
mov r0, c10
dp4 r4.z, c17, r0
mov r0, c9
dp4 r4.y, c17, r0
mov r1, c8
dp4 r4.x, c17, r1
mad r0.xyz, r4, c18.w, -v0
dp3 oT1.y, r0, r2
dp3 oT1.z, v2, r0
dp3 oT1.x, r0, v1
dp4 r0.w, v0, c7
dp4 r0.z, v0, c6
dp4 r0.x, v0, c4
dp4 r0.y, v0, c5
dp3 oT2.y, r2, r3
dp3 oT2.z, v2, r3
dp3 oT2.x, v1, r3
dp4 oT3.z, r0, c14
dp4 oT3.y, r0, c13
dp4 oT3.x, r0, c12
mad oT0.xy, v3, c19, c19.zwzw
dp4 oPos.w, v0, c3
dp4 oPos.z, v0, c2
dp4 oPos.y, v0, c1
dp4 oPos.x, v0, c0
"
}

SubProgram "xbox360 " {
Keywords { "POINT" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Matrix 15 [_LightMatrix0] 4
Vector 19 [_MainTex_ST]
Matrix 6 [_Object2World] 4
Matrix 10 [_World2Object] 4
Vector 0 [_WorldSpaceCameraPos]
Vector 1 [_WorldSpaceLightPos0]
Matrix 2 [glstate_matrix_mvp] 4
Vector 14 [unity_Scale]
// Shader Timing Estimate, in Cycles/64 vertex vector:
// ALU: 41.33 (31 instructions), vertex: 32, texture: 0,
//   sequencer: 18,  8 GPRs, 24 threads,
// Performance (if enough threads): ~41 cycles per vector
// * Vertex cycle estimates are assuming 3 vfetch_minis for every vfetch_full,
//     with <= 32 bytes per vfetch_full group.

"vs_360
backbbabaaaaacbeaaaaabomaaaaaaaaaaaaaaceaaaaaaaaaaaaabkmaaaaaaaa
aaaaaaaaaaaaabieaaaaaabmaaaaabhhpppoadaaaaaaaaaiaaaaaabmaaaaaaaa
aaaaabhaaaaaaalmaaacaaapaaaeaaaaaaaaaammaaaaaaaaaaaaaanmaaacaabd
aaabaaaaaaaaaaoiaaaaaaaaaaaaaapiaaacaaagaaaeaaaaaaaaaammaaaaaaaa
aaaaabagaaacaaakaaaeaaaaaaaaaammaaaaaaaaaaaaabbeaaacaaaaaaabaaaa
aaaaabcmaaaaaaaaaaaaabdmaaacaaabaaabaaaaaaaaaaoiaaaaaaaaaaaaabfb
aaacaaacaaaeaaaaaaaaaammaaaaaaaaaaaaabgeaaacaaaoaaabaaaaaaaaaaoi
aaaaaaaafpemgjghgiheengbhehcgjhidaaaklklaaadaaadaaaeaaaeaaabaaaa
aaaaaaaafpengbgjgofegfhifpfdfeaaaaabaaadaaabaaaeaaabaaaaaaaaaaaa
fpepgcgkgfgdhedcfhgphcgmgeaafpfhgphcgmgedcepgcgkgfgdheaafpfhgphc
gmgefdhagbgdgfedgbgngfhcgbfagphdaaklklklaaabaaadaaabaaadaaabaaaa
aaaaaaaafpfhgphcgmgefdhagbgdgfemgjghgihefagphddaaaghgmhdhegbhegf
fpgngbhehcgjhifpgnhghaaahfgogjhehjfpfdgdgbgmgfaahghdfpddfpdaaadc
codacodcdadddfddcodaaaklaaaaaaaaaaaaabomaadbaaahaaaaaaaaaaaaaaaa
aaaacmieaaaaaaabaaaaaaaeaaaaaaaiaaaaacjaaabaaaafaaaagaagaaaadaah
aacafaaiaaaadafaaaabhbfbaaaehcfcaaahhdfdaaaabacdaaaaaabnaaaaaabo
aaaababpaaaaaacaaaaaaacbaaaabaccaaaabachpaffeaafaaaabcaamcaaaaaa
aaaaeaajaaaabcaameaaaaaaaaaagaangabdbcaabcaaaaaaaaaagabjgabpbcaa
bcaaaaaaaaaadacfaaaaccaaaaaaaaaaafpigaaaaaaaagiiaaaaaaaaafpifaaa
aaaaagiiaaaaaaaaafpicaaaaaaaaoiiaaaaaaaaafpibaaaaaaaapmiaaaaaaaa
miapaaaaaabliiaakbagafaamiapaaaaaamgiiaaklagaeaamiapaaaaaalbdeje
klagadaamiapiadoaagmaadeklagacaamiahaaaaaaleblaacbanabaamiahaaad
aamamgmaalamaaanmiahaaadaalelbleclalaaadmiahaaaeaalogfaaobacafaa
miahaaahaamamgleclamabaamiapaaaaaabliiaakbagajaamiapaaaaaamgiiaa
klagaiaamiahaaahaalelbleclalabahmiahaaaeabgflomaolacafaemiahaaad
aamagmleclakaaadmiahaaadabmablmakladaoagmiahaaaeaamablaaobaeafaa
miahaaahaamagmleclakabahmiapaaaaaalbdejeklagahaamiapaaaaaagmejhk
klagagaamiahaaagabmablmaklahaoagmiabiaabaaloloaapaagafaamiaciaab
aaloloaapaaeagaamiaeiaabaaloloaapaagacaamiabiaacaaloloaapaadafaa
miaciaacaaloloaapaaeadaamiaeiaacaaloloaapaadacaamiadiaaaaalalabk
ilabbdbdmiahaaabaalbleaakbaabcaamiahaaabaamgmaleklaabbabmiahaaaa
aagmleleklaabaabmiahiaadaablmaleklaaapaaaaaaaaaaaaaaaaaaaaaaaaaa
"
}

SubProgram "ps3 " {
Keywords { "POINT" }
Matrix 256 [glstate_matrix_mvp]
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 467 [_WorldSpaceCameraPos]
Vector 466 [_WorldSpaceLightPos0]
Matrix 260 [_Object2World]
Matrix 264 [_World2Object]
Vector 465 [unity_Scale]
Matrix 268 [_LightMatrix0]
Vector 464 [_MainTex_ST]
"sce_vp_rsx // 32 instructions using 5 registers
[Configuration]
8
0000002041050500
[Microcode]
512
00009c6c005d200d8186c0836041fffc00011c6c00400e0c0106c0836041dffc
00019c6c005d300c0186c0836041dffc401f9c6c011d0808010400d740619f9c
401f9c6c01d0300d8106c0c360403f80401f9c6c01d0200d8106c0c360405f80
401f9c6c01d0100d8106c0c360409f80401f9c6c01d0000d8106c0c360411f80
00001c6c01d0700d8106c0c360403ffc00001c6c01d0600d8106c0c360405ffc
00001c6c01d0500d8106c0c360409ffc00001c6c01d0400d8106c0c360411ffc
00021c6c01d0a00d8286c0c360405ffc00021c6c01d0900d8286c0c360409ffc
00021c6c01d0800d8286c0c360411ffc00009c6c0190a00c0686c0c360405ffc
00009c6c0190900c0686c0c360409ffc00009c6c0190800c0686c0c360411ffc
00019c6c00800243011842436041dffc00011c6c010002308121826301a1dffc
401f9c6c01d0e00d8086c0c360405fa8401f9c6c01d0d00d8086c0c360409fa8
401f9c6c01d0c00d8086c0c360411fa800001c6c011d100c08bfc0e30041dffc
00009c6c011d100c02bfc0e30041dffc401f9c6c0140020c0106004360405fa0
401f9c6c01400e0c0086008360411fa000011c6c00800e0c04bfc0836041dffc
401f9c6c0140020c0106014360405fa4401f9c6c01400e0c0106014360411fa4
401f9c6c0140000c0086024360409fa0401f9c6c0140000c0486014360409fa5
"
}

SubProgram "d3d11 " {
Keywords { "POINT" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 160 // 160 used size, 8 vars
Matrix 48 [_LightMatrix0] 4
Vector 144 [_MainTex_ST] 4
ConstBuffer "UnityPerCamera" 128 // 76 used size, 8 vars
Vector 64 [_WorldSpaceCameraPos] 3
ConstBuffer "UnityLighting" 400 // 16 used size, 16 vars
Vector 0 [_WorldSpaceLightPos0] 4
ConstBuffer "UnityPerDraw" 336 // 336 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
Matrix 192 [_Object2World] 4
Matrix 256 [_World2Object] 4
Vector 320 [unity_Scale] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityLighting" 2
BindCB "UnityPerDraw" 3
// 33 instructions, 2 temp regs, 0 temp arrays:
// ALU 14 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0
eefiecedlgmpoopbcgheknafngcjigmdnlnlonpkabaaaaaapiagaaaaadaaaaaa
cmaaaaaapeaaaaaajeabaaaaejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaa
abaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaljaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfc
enebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheojiaaaaaaafaaaaaa
aiaaaaaaiaaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaabaaaaaaadamaaaaimaaaaaaabaaaaaaaaaaaaaa
adaaaaaaacaaaaaaahaiaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaa
ahaiaaaaimaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaahaiaaaafdfgfpfa
epfdejfeejepeoaafeeffiedepepfceeaaklklklfdeieefcfmafaaaaeaaaabaa
fhabaaaafjaaaaaeegiocaaaaaaaaaaaakaaaaaafjaaaaaeegiocaaaabaaaaaa
afaaaaaafjaaaaaeegiocaaaacaaaaaaabaaaaaafjaaaaaeegiocaaaadaaaaaa
bfaaaaaafpaaaaadpcbabaaaaaaaaaaafpaaaaadpcbabaaaabaaaaaafpaaaaad
hcbabaaaacaaaaaafpaaaaaddcbabaaaadaaaaaaghaaaaaepccabaaaaaaaaaaa
abaaaaaagfaaaaaddccabaaaabaaaaaagfaaaaadhccabaaaacaaaaaagfaaaaad
hccabaaaadaaaaaagfaaaaadhccabaaaaeaaaaaagiaaaaacacaaaaaadiaaaaai
pcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaadaaaaaaabaaaaaadcaaaaak
pcaabaaaaaaaaaaaegiocaaaadaaaaaaaaaaaaaaagbabaaaaaaaaaaaegaobaaa
aaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaacaaaaaakgbkbaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaakpccabaaaaaaaaaaaegiocaaaadaaaaaa
adaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaaldccabaaaabaaaaaa
egbabaaaadaaaaaaegiacaaaaaaaaaaaajaaaaaaogikcaaaaaaaaaaaajaaaaaa
diaaaaahhcaabaaaaaaaaaaajgbebaaaabaaaaaacgbjbaaaacaaaaaadcaaaaak
hcaabaaaaaaaaaaajgbebaaaacaaaaaacgbjbaaaabaaaaaaegacbaiaebaaaaaa
aaaaaaaadiaaaaahhcaabaaaaaaaaaaaegacbaaaaaaaaaaapgbpbaaaabaaaaaa
diaaaaajhcaabaaaabaaaaaafgifcaaaacaaaaaaaaaaaaaaegiccaaaadaaaaaa
bbaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaa
acaaaaaaaaaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaa
adaaaaaabcaaaaaakgikcaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaadcaaaaal
hcaabaaaabaaaaaaegiccaaaadaaaaaabdaaaaaapgipcaaaacaaaaaaaaaaaaaa
egacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegacbaaaabaaaaaapgipcaaa
adaaaaaabeaaaaaaegbcbaiaebaaaaaaaaaaaaaabaaaaaahcccabaaaacaaaaaa
egacbaaaaaaaaaaaegacbaaaabaaaaaabaaaaaahbccabaaaacaaaaaaegbcbaaa
abaaaaaaegacbaaaabaaaaaabaaaaaaheccabaaaacaaaaaaegbcbaaaacaaaaaa
egacbaaaabaaaaaadiaaaaajhcaabaaaabaaaaaafgifcaaaabaaaaaaaeaaaaaa
egiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaa
baaaaaaaagiacaaaabaaaaaaaeaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaa
abaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaaabaaaaaaaeaaaaaaegacbaaa
abaaaaaaaaaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaaegiccaaaadaaaaaa
bdaaaaaadcaaaaalhcaabaaaabaaaaaaegacbaaaabaaaaaapgipcaaaadaaaaaa
beaaaaaaegbcbaiaebaaaaaaaaaaaaaabaaaaaahcccabaaaadaaaaaaegacbaaa
aaaaaaaaegacbaaaabaaaaaabaaaaaahbccabaaaadaaaaaaegbcbaaaabaaaaaa
egacbaaaabaaaaaabaaaaaaheccabaaaadaaaaaaegbcbaaaacaaaaaaegacbaaa
abaaaaaadiaaaaaipcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaadaaaaaa
anaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaamaaaaaaagbabaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaa
aoaaaaaakgbkbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaa
egiocaaaadaaaaaaapaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadiaaaaai
hcaabaaaabaaaaaafgafbaaaaaaaaaaaegiccaaaaaaaaaaaaeaaaaaadcaaaaak
hcaabaaaabaaaaaaegiccaaaaaaaaaaaadaaaaaaagaabaaaaaaaaaaaegacbaaa
abaaaaaadcaaaaakhcaabaaaaaaaaaaaegiccaaaaaaaaaaaafaaaaaakgakbaaa
aaaaaaaaegacbaaaabaaaaaadcaaaaakhccabaaaaeaaaaaaegiccaaaaaaaaaaa
agaaaaaapgapbaaaaaaaaaaaegacbaaaaaaaaaaadoaaaaab"
}

SubProgram "gles " {
Keywords { "POINT" }
"!!GLES
#define SHADER_API_GLES 1
#define tex2D texture2D


#ifdef VERTEX
#define gl_ModelViewProjectionMatrix glstate_matrix_mvp
uniform mat4 glstate_matrix_mvp;

varying highp vec3 xlv_TEXCOORD3;
varying mediump vec3 xlv_TEXCOORD2;
varying mediump vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp vec4 _MainTex_ST;
uniform highp mat4 _LightMatrix0;
uniform highp vec4 unity_Scale;
uniform highp mat4 _World2Object;
uniform highp mat4 _Object2World;

uniform highp vec4 _WorldSpaceLightPos0;
uniform highp vec3 _WorldSpaceCameraPos;
attribute vec4 _glesTANGENT;
attribute vec4 _glesMultiTexCoord0;
attribute vec3 _glesNormal;
attribute vec4 _glesVertex;
void main ()
{
  vec4 tmpvar_1;
  tmpvar_1.xyz = normalize(_glesTANGENT.xyz);
  tmpvar_1.w = _glesTANGENT.w;
  vec3 tmpvar_2;
  tmpvar_2 = normalize(_glesNormal);
  mediump vec3 tmpvar_3;
  mediump vec3 tmpvar_4;
  highp vec3 tmpvar_5;
  highp vec3 tmpvar_6;
  tmpvar_5 = tmpvar_1.xyz;
  tmpvar_6 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_7;
  tmpvar_7[0].x = tmpvar_5.x;
  tmpvar_7[0].y = tmpvar_6.x;
  tmpvar_7[0].z = tmpvar_2.x;
  tmpvar_7[1].x = tmpvar_5.y;
  tmpvar_7[1].y = tmpvar_6.y;
  tmpvar_7[1].z = tmpvar_2.y;
  tmpvar_7[2].x = tmpvar_5.z;
  tmpvar_7[2].y = tmpvar_6.z;
  tmpvar_7[2].z = tmpvar_2.z;
  highp vec3 tmpvar_8;
  tmpvar_8 = (tmpvar_7 * (((_World2Object * _WorldSpaceLightPos0).xyz * unity_Scale.w) - _glesVertex.xyz));
  tmpvar_3 = tmpvar_8;
  highp vec4 tmpvar_9;
  tmpvar_9.w = 1.0;
  tmpvar_9.xyz = _WorldSpaceCameraPos;
  highp vec3 tmpvar_10;
  tmpvar_10 = (tmpvar_7 * (((_World2Object * tmpvar_9).xyz * unity_Scale.w) - _glesVertex.xyz));
  tmpvar_4 = tmpvar_10;
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = tmpvar_3;
  xlv_TEXCOORD2 = tmpvar_4;
  xlv_TEXCOORD3 = (_LightMatrix0 * (_Object2World * _glesVertex)).xyz;
}



#endif
#ifdef FRAGMENT

varying highp vec3 xlv_TEXCOORD3;
varying mediump vec3 xlv_TEXCOORD2;
varying mediump vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform mediump float _Gloss;
uniform mediump float _Shininess;
uniform lowp vec4 _Color;
uniform sampler2D _SpecMap;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform sampler2D _LightTexture0;
uniform lowp vec4 _LightColor0;
void main ()
{
  lowp vec4 c_1;
  lowp vec3 lightDir_2;
  mediump vec3 tmpvar_3;
  mediump vec3 tmpvar_4;
  lowp vec4 tmpvar_5;
  tmpvar_5 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_6;
  tmpvar_6 = (tmpvar_5.xyz * _Color.xyz);
  tmpvar_3 = tmpvar_6;
  lowp vec4 tmpvar_7;
  tmpvar_7 = texture2D (_SpecMap, xlv_TEXCOORD0);
  mediump vec3 tmpvar_8;
  tmpvar_8 = ((tmpvar_5.w * tmpvar_7.xyz) * _Gloss);
  lowp vec3 tmpvar_9;
  tmpvar_9 = ((texture2D (_BumpMap, xlv_TEXCOORD0).xyz * 2.0) - 1.0);
  tmpvar_4 = tmpvar_9;
  mediump vec3 tmpvar_10;
  tmpvar_10 = normalize(xlv_TEXCOORD1);
  lightDir_2 = tmpvar_10;
  highp float tmpvar_11;
  tmpvar_11 = dot (xlv_TEXCOORD3, xlv_TEXCOORD3);
  lowp vec4 tmpvar_12;
  tmpvar_12 = texture2D (_LightTexture0, vec2(tmpvar_11));
  mediump vec3 lightDir_13;
  lightDir_13 = lightDir_2;
  mediump float atten_14;
  atten_14 = tmpvar_12.w;
  mediump vec4 c_15;
  mediump vec3 specCol_16;
  highp float nh_17;
  mediump float tmpvar_18;
  tmpvar_18 = max (0.0, dot (tmpvar_4, normalize((lightDir_13 + normalize(xlv_TEXCOORD2)))));
  nh_17 = tmpvar_18;
  mediump float arg1_19;
  arg1_19 = (32.0 * _Shininess);
  highp vec3 tmpvar_20;
  tmpvar_20 = (pow (nh_17, arg1_19) * tmpvar_8);
  specCol_16 = tmpvar_20;
  c_15.xyz = ((((tmpvar_3 * _LightColor0.xyz) * max (0.0, dot (tmpvar_4, lightDir_13))) + (_LightColor0.xyz * specCol_16)) * (atten_14 * 2.0));
  c_15.w = 0.0;
  c_1.xyz = c_15.xyz;
  c_1.w = 0.0;
  gl_FragData[0] = c_1;
}



#endif"
}

SubProgram "glesdesktop " {
Keywords { "POINT" }
"!!GLES
#define SHADER_API_GLES 1
#define tex2D texture2D


#ifdef VERTEX
#define gl_ModelViewProjectionMatrix glstate_matrix_mvp
uniform mat4 glstate_matrix_mvp;

varying highp vec3 xlv_TEXCOORD3;
varying mediump vec3 xlv_TEXCOORD2;
varying mediump vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp vec4 _MainTex_ST;
uniform highp mat4 _LightMatrix0;
uniform highp vec4 unity_Scale;
uniform highp mat4 _World2Object;
uniform highp mat4 _Object2World;

uniform highp vec4 _WorldSpaceLightPos0;
uniform highp vec3 _WorldSpaceCameraPos;
attribute vec4 _glesTANGENT;
attribute vec4 _glesMultiTexCoord0;
attribute vec3 _glesNormal;
attribute vec4 _glesVertex;
void main ()
{
  vec4 tmpvar_1;
  tmpvar_1.xyz = normalize(_glesTANGENT.xyz);
  tmpvar_1.w = _glesTANGENT.w;
  vec3 tmpvar_2;
  tmpvar_2 = normalize(_glesNormal);
  mediump vec3 tmpvar_3;
  mediump vec3 tmpvar_4;
  highp vec3 tmpvar_5;
  highp vec3 tmpvar_6;
  tmpvar_5 = tmpvar_1.xyz;
  tmpvar_6 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_7;
  tmpvar_7[0].x = tmpvar_5.x;
  tmpvar_7[0].y = tmpvar_6.x;
  tmpvar_7[0].z = tmpvar_2.x;
  tmpvar_7[1].x = tmpvar_5.y;
  tmpvar_7[1].y = tmpvar_6.y;
  tmpvar_7[1].z = tmpvar_2.y;
  tmpvar_7[2].x = tmpvar_5.z;
  tmpvar_7[2].y = tmpvar_6.z;
  tmpvar_7[2].z = tmpvar_2.z;
  highp vec3 tmpvar_8;
  tmpvar_8 = (tmpvar_7 * (((_World2Object * _WorldSpaceLightPos0).xyz * unity_Scale.w) - _glesVertex.xyz));
  tmpvar_3 = tmpvar_8;
  highp vec4 tmpvar_9;
  tmpvar_9.w = 1.0;
  tmpvar_9.xyz = _WorldSpaceCameraPos;
  highp vec3 tmpvar_10;
  tmpvar_10 = (tmpvar_7 * (((_World2Object * tmpvar_9).xyz * unity_Scale.w) - _glesVertex.xyz));
  tmpvar_4 = tmpvar_10;
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = tmpvar_3;
  xlv_TEXCOORD2 = tmpvar_4;
  xlv_TEXCOORD3 = (_LightMatrix0 * (_Object2World * _glesVertex)).xyz;
}



#endif
#ifdef FRAGMENT

varying highp vec3 xlv_TEXCOORD3;
varying mediump vec3 xlv_TEXCOORD2;
varying mediump vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform mediump float _Gloss;
uniform mediump float _Shininess;
uniform lowp vec4 _Color;
uniform sampler2D _SpecMap;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform sampler2D _LightTexture0;
uniform lowp vec4 _LightColor0;
void main ()
{
  lowp vec4 c_1;
  lowp vec3 lightDir_2;
  mediump vec3 tmpvar_3;
  mediump vec3 tmpvar_4;
  lowp vec4 tmpvar_5;
  tmpvar_5 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_6;
  tmpvar_6 = (tmpvar_5.xyz * _Color.xyz);
  tmpvar_3 = tmpvar_6;
  lowp vec4 tmpvar_7;
  tmpvar_7 = texture2D (_SpecMap, xlv_TEXCOORD0);
  mediump vec3 tmpvar_8;
  tmpvar_8 = ((tmpvar_5.w * tmpvar_7.xyz) * _Gloss);
  lowp vec3 normal_9;
  normal_9.xy = ((texture2D (_BumpMap, xlv_TEXCOORD0).wy * 2.0) - 1.0);
  normal_9.z = sqrt(((1.0 - (normal_9.x * normal_9.x)) - (normal_9.y * normal_9.y)));
  tmpvar_4 = normal_9;
  mediump vec3 tmpvar_10;
  tmpvar_10 = normalize(xlv_TEXCOORD1);
  lightDir_2 = tmpvar_10;
  highp float tmpvar_11;
  tmpvar_11 = dot (xlv_TEXCOORD3, xlv_TEXCOORD3);
  lowp vec4 tmpvar_12;
  tmpvar_12 = texture2D (_LightTexture0, vec2(tmpvar_11));
  mediump vec3 lightDir_13;
  lightDir_13 = lightDir_2;
  mediump float atten_14;
  atten_14 = tmpvar_12.w;
  mediump vec4 c_15;
  mediump vec3 specCol_16;
  highp float nh_17;
  mediump float tmpvar_18;
  tmpvar_18 = max (0.0, dot (tmpvar_4, normalize((lightDir_13 + normalize(xlv_TEXCOORD2)))));
  nh_17 = tmpvar_18;
  mediump float arg1_19;
  arg1_19 = (32.0 * _Shininess);
  highp vec3 tmpvar_20;
  tmpvar_20 = (pow (nh_17, arg1_19) * tmpvar_8);
  specCol_16 = tmpvar_20;
  c_15.xyz = ((((tmpvar_3 * _LightColor0.xyz) * max (0.0, dot (tmpvar_4, lightDir_13))) + (_LightColor0.xyz * specCol_16)) * (atten_14 * 2.0));
  c_15.w = 0.0;
  c_1.xyz = c_15.xyz;
  c_1.w = 0.0;
  gl_FragData[0] = c_1;
}



#endif"
}

SubProgram "flash " {
Keywords { "POINT" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Matrix 0 [glstate_matrix_mvp]
Vector 16 [_WorldSpaceCameraPos]
Vector 17 [_WorldSpaceLightPos0]
Matrix 4 [_Object2World]
Matrix 8 [_World2Object]
Vector 18 [unity_Scale]
Matrix 12 [_LightMatrix0]
Vector 19 [_MainTex_ST]
"agal_vs
c20 1.0 0.0 0.0 0.0
[bc]
aaaaaaaaaaaaaiacbeaaaaaaabaaaaaaaaaaaaaaaaaaaaaa mov r0.w, c20.x
aaaaaaaaaaaaahacbaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, c16
bdaaaaaaabaaaeacaaaaaaoeacaaaaaaakaaaaoeabaaaaaa dp4 r1.z, r0, c10
bdaaaaaaabaaacacaaaaaaoeacaaaaaaajaaaaoeabaaaaaa dp4 r1.y, r0, c9
bdaaaaaaabaaabacaaaaaaoeacaaaaaaaiaaaaoeabaaaaaa dp4 r1.x, r0, c8
adaaaaaaacaaahacabaaaakeacaaaaaabcaaaappabaaaaaa mul r2.xyz, r1.xyzz, c18.w
acaaaaaaadaaahacacaaaakeacaaaaaaaaaaaaoeaaaaaaaa sub r3.xyz, r2.xyzz, a0
aaaaaaaaaaaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, a5
adaaaaaaabaaahacabaaaancaaaaaaaaaaaaaaajacaaaaaa mul r1.xyz, a1.zxyw, r0.yzxx
aaaaaaaaaaaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, a5
adaaaaaaaeaaahacabaaaamjaaaaaaaaaaaaaafcacaaaaaa mul r4.xyz, a1.yzxw, r0.zxyy
acaaaaaaabaaahacaeaaaakeacaaaaaaabaaaakeacaaaaaa sub r1.xyz, r4.xyzz, r1.xyzz
adaaaaaaacaaahacabaaaakeacaaaaaaafaaaappaaaaaaaa mul r2.xyz, r1.xyzz, a5.w
aaaaaaaaaaaaapacakaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0, c10
bdaaaaaaaeaaaeacbbaaaaoeabaaaaaaaaaaaaoeacaaaaaa dp4 r4.z, c17, r0
aaaaaaaaaaaaapacajaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0, c9
bdaaaaaaaeaaacacbbaaaaoeabaaaaaaaaaaaaoeacaaaaaa dp4 r4.y, c17, r0
aaaaaaaaabaaapacaiaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r1, c8
bdaaaaaaaeaaabacbbaaaaoeabaaaaaaabaaaaoeacaaaaaa dp4 r4.x, c17, r1
adaaaaaaabaaahacaeaaaakeacaaaaaabcaaaappabaaaaaa mul r1.xyz, r4.xyzz, c18.w
acaaaaaaaaaaahacabaaaakeacaaaaaaaaaaaaoeaaaaaaaa sub r0.xyz, r1.xyzz, a0
bcaaaaaaabaaacaeaaaaaakeacaaaaaaacaaaakeacaaaaaa dp3 v1.y, r0.xyzz, r2.xyzz
bcaaaaaaabaaaeaeabaaaaoeaaaaaaaaaaaaaakeacaaaaaa dp3 v1.z, a1, r0.xyzz
bcaaaaaaabaaabaeaaaaaakeacaaaaaaafaaaaoeaaaaaaaa dp3 v1.x, r0.xyzz, a5
bdaaaaaaaaaaaiacaaaaaaoeaaaaaaaaahaaaaoeabaaaaaa dp4 r0.w, a0, c7
bdaaaaaaaaaaaeacaaaaaaoeaaaaaaaaagaaaaoeabaaaaaa dp4 r0.z, a0, c6
bdaaaaaaaaaaabacaaaaaaoeaaaaaaaaaeaaaaoeabaaaaaa dp4 r0.x, a0, c4
bdaaaaaaaaaaacacaaaaaaoeaaaaaaaaafaaaaoeabaaaaaa dp4 r0.y, a0, c5
bcaaaaaaacaaacaeacaaaakeacaaaaaaadaaaakeacaaaaaa dp3 v2.y, r2.xyzz, r3.xyzz
bcaaaaaaacaaaeaeabaaaaoeaaaaaaaaadaaaakeacaaaaaa dp3 v2.z, a1, r3.xyzz
bcaaaaaaacaaabaeafaaaaoeaaaaaaaaadaaaakeacaaaaaa dp3 v2.x, a5, r3.xyzz
bdaaaaaaadaaaeaeaaaaaaoeacaaaaaaaoaaaaoeabaaaaaa dp4 v3.z, r0, c14
bdaaaaaaadaaacaeaaaaaaoeacaaaaaaanaaaaoeabaaaaaa dp4 v3.y, r0, c13
bdaaaaaaadaaabaeaaaaaaoeacaaaaaaamaaaaoeabaaaaaa dp4 v3.x, r0, c12
adaaaaaaaaaaadacadaaaaoeaaaaaaaabdaaaaoeabaaaaaa mul r0.xy, a3, c19
abaaaaaaaaaaadaeaaaaaafeacaaaaaabdaaaaooabaaaaaa add v0.xy, r0.xyyy, c19.zwzw
bdaaaaaaaaaaaiadaaaaaaoeaaaaaaaaadaaaaoeabaaaaaa dp4 o0.w, a0, c3
bdaaaaaaaaaaaeadaaaaaaoeaaaaaaaaacaaaaoeabaaaaaa dp4 o0.z, a0, c2
bdaaaaaaaaaaacadaaaaaaoeaaaaaaaaabaaaaoeabaaaaaa dp4 o0.y, a0, c1
bdaaaaaaaaaaabadaaaaaaoeaaaaaaaaaaaaaaoeabaaaaaa dp4 o0.x, a0, c0
aaaaaaaaaaaaamaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v0.zw, c0
aaaaaaaaabaaaiaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v1.w, c0
aaaaaaaaacaaaiaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v2.w, c0
aaaaaaaaadaaaiaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v3.w, c0
"
}

SubProgram "d3d11_9x " {
Keywords { "POINT" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 160 // 160 used size, 8 vars
Matrix 48 [_LightMatrix0] 4
Vector 144 [_MainTex_ST] 4
ConstBuffer "UnityPerCamera" 128 // 76 used size, 8 vars
Vector 64 [_WorldSpaceCameraPos] 3
ConstBuffer "UnityLighting" 400 // 16 used size, 16 vars
Vector 0 [_WorldSpaceLightPos0] 4
ConstBuffer "UnityPerDraw" 336 // 336 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
Matrix 192 [_Object2World] 4
Matrix 256 [_World2Object] 4
Vector 320 [unity_Scale] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityLighting" 2
BindCB "UnityPerDraw" 3
// 33 instructions, 2 temp regs, 0 temp arrays:
// ALU 14 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0_level_9_3
eefiecedijcbcpehncccipennobfnmfkecknhmjlabaaaaaadiakaaaaaeaaaaaa
daaaaaaagmadaaaanaaiaaaajiajaaaaebgpgodjdeadaaaadeadaaaaaaacpopp
meacaaaahaaaaaaaagaaceaaaaaagmaaaaaagmaaaaaaceaaabaagmaaaaaaadaa
aeaaabaaaaaaaaaaaaaaajaaabaaafaaaaaaaaaaabaaaeaaabaaagaaaaaaaaaa
acaaaaaaabaaahaaaaaaaaaaadaaaaaaaeaaaiaaaaaaaaaaadaaamaaajaaamaa
aaaaaaaaaaaaaaaaabacpoppbpaaaaacafaaaaiaaaaaapjabpaaaaacafaaabia
abaaapjabpaaaaacafaaaciaacaaapjabpaaaaacafaaadiaadaaapjaaeaaaaae
aaaaadoaadaaoejaafaaoekaafaaookaabaaaaacaaaaapiaahaaoekaafaaaaad
abaaahiaaaaaffiabbaaoekaaeaaaaaeabaaahiabaaaoekaaaaaaaiaabaaoeia
aeaaaaaeaaaaahiabcaaoekaaaaakkiaabaaoeiaaeaaaaaeaaaaahiabdaaoeka
aaaappiaaaaaoeiaaeaaaaaeaaaaahiaaaaaoeiabeaappkaaaaaoejbaiaaaaad
abaaaboaabaaoejaaaaaoeiaabaaaaacabaaahiaabaaoejaafaaaaadacaaahia
abaamjiaacaancjaaeaaaaaeabaaahiaacaamjjaabaanciaacaaoeibafaaaaad
abaaahiaabaaoeiaabaappjaaiaaaaadabaaacoaabaaoeiaaaaaoeiaaiaaaaad
abaaaeoaacaaoejaaaaaoeiaabaaaaacaaaaahiaagaaoekaafaaaaadacaaahia
aaaaffiabbaaoekaaeaaaaaeaaaaaliabaaakekaaaaaaaiaacaakeiaaeaaaaae
aaaaahiabcaaoekaaaaakkiaaaaapeiaacaaaaadaaaaahiaaaaaoeiabdaaoeka
aeaaaaaeaaaaahiaaaaaoeiabeaappkaaaaaoejbaiaaaaadacaaaboaabaaoeja
aaaaoeiaaiaaaaadacaaacoaabaaoeiaaaaaoeiaaiaaaaadacaaaeoaacaaoeja
aaaaoeiaafaaaaadaaaaapiaaaaaffjaanaaoekaaeaaaaaeaaaaapiaamaaoeka
aaaaaajaaaaaoeiaaeaaaaaeaaaaapiaaoaaoekaaaaakkjaaaaaoeiaaeaaaaae
aaaaapiaapaaoekaaaaappjaaaaaoeiaafaaaaadabaaahiaaaaaffiaacaaoeka
aeaaaaaeabaaahiaabaaoekaaaaaaaiaabaaoeiaaeaaaaaeaaaaahiaadaaoeka
aaaakkiaabaaoeiaaeaaaaaeadaaahoaaeaaoekaaaaappiaaaaaoeiaafaaaaad
aaaaapiaaaaaffjaajaaoekaaeaaaaaeaaaaapiaaiaaoekaaaaaaajaaaaaoeia
aeaaaaaeaaaaapiaakaaoekaaaaakkjaaaaaoeiaaeaaaaaeaaaaapiaalaaoeka
aaaappjaaaaaoeiaaeaaaaaeaaaaadmaaaaappiaaaaaoekaaaaaoeiaabaaaaac
aaaaammaaaaaoeiappppaaaafdeieefcfmafaaaaeaaaabaafhabaaaafjaaaaae
egiocaaaaaaaaaaaakaaaaaafjaaaaaeegiocaaaabaaaaaaafaaaaaafjaaaaae
egiocaaaacaaaaaaabaaaaaafjaaaaaeegiocaaaadaaaaaabfaaaaaafpaaaaad
pcbabaaaaaaaaaaafpaaaaadpcbabaaaabaaaaaafpaaaaadhcbabaaaacaaaaaa
fpaaaaaddcbabaaaadaaaaaaghaaaaaepccabaaaaaaaaaaaabaaaaaagfaaaaad
dccabaaaabaaaaaagfaaaaadhccabaaaacaaaaaagfaaaaadhccabaaaadaaaaaa
gfaaaaadhccabaaaaeaaaaaagiaaaaacacaaaaaadiaaaaaipcaabaaaaaaaaaaa
fgbfbaaaaaaaaaaaegiocaaaadaaaaaaabaaaaaadcaaaaakpcaabaaaaaaaaaaa
egiocaaaadaaaaaaaaaaaaaaagbabaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaak
pcaabaaaaaaaaaaaegiocaaaadaaaaaaacaaaaaakgbkbaaaaaaaaaaaegaobaaa
aaaaaaaadcaaaaakpccabaaaaaaaaaaaegiocaaaadaaaaaaadaaaaaapgbpbaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaaldccabaaaabaaaaaaegbabaaaadaaaaaa
egiacaaaaaaaaaaaajaaaaaaogikcaaaaaaaaaaaajaaaaaadiaaaaahhcaabaaa
aaaaaaaajgbebaaaabaaaaaacgbjbaaaacaaaaaadcaaaaakhcaabaaaaaaaaaaa
jgbebaaaacaaaaaacgbjbaaaabaaaaaaegacbaiaebaaaaaaaaaaaaaadiaaaaah
hcaabaaaaaaaaaaaegacbaaaaaaaaaaapgbpbaaaabaaaaaadiaaaaajhcaabaaa
abaaaaaafgifcaaaacaaaaaaaaaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaal
hcaabaaaabaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaaacaaaaaaaaaaaaaa
egacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabcaaaaaa
kgikcaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaa
egiccaaaadaaaaaabdaaaaaapgipcaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaa
dcaaaaalhcaabaaaabaaaaaaegacbaaaabaaaaaapgipcaaaadaaaaaabeaaaaaa
egbcbaiaebaaaaaaaaaaaaaabaaaaaahcccabaaaacaaaaaaegacbaaaaaaaaaaa
egacbaaaabaaaaaabaaaaaahbccabaaaacaaaaaaegbcbaaaabaaaaaaegacbaaa
abaaaaaabaaaaaaheccabaaaacaaaaaaegbcbaaaacaaaaaaegacbaaaabaaaaaa
diaaaaajhcaabaaaabaaaaaafgifcaaaabaaaaaaaeaaaaaaegiccaaaadaaaaaa
bbaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaa
abaaaaaaaeaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaa
adaaaaaabcaaaaaakgikcaaaabaaaaaaaeaaaaaaegacbaaaabaaaaaaaaaaaaai
hcaabaaaabaaaaaaegacbaaaabaaaaaaegiccaaaadaaaaaabdaaaaaadcaaaaal
hcaabaaaabaaaaaaegacbaaaabaaaaaapgipcaaaadaaaaaabeaaaaaaegbcbaia
ebaaaaaaaaaaaaaabaaaaaahcccabaaaadaaaaaaegacbaaaaaaaaaaaegacbaaa
abaaaaaabaaaaaahbccabaaaadaaaaaaegbcbaaaabaaaaaaegacbaaaabaaaaaa
baaaaaaheccabaaaadaaaaaaegbcbaaaacaaaaaaegacbaaaabaaaaaadiaaaaai
pcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaadaaaaaaanaaaaaadcaaaaak
pcaabaaaaaaaaaaaegiocaaaadaaaaaaamaaaaaaagbabaaaaaaaaaaaegaobaaa
aaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaaoaaaaaakgbkbaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaa
apaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadiaaaaaihcaabaaaabaaaaaa
fgafbaaaaaaaaaaaegiccaaaaaaaaaaaaeaaaaaadcaaaaakhcaabaaaabaaaaaa
egiccaaaaaaaaaaaadaaaaaaagaabaaaaaaaaaaaegacbaaaabaaaaaadcaaaaak
hcaabaaaaaaaaaaaegiccaaaaaaaaaaaafaaaaaakgakbaaaaaaaaaaaegacbaaa
abaaaaaadcaaaaakhccabaaaaeaaaaaaegiccaaaaaaaaaaaagaaaaaapgapbaaa
aaaaaaaaegacbaaaaaaaaaaadoaaaaabejfdeheomaaaaaaaagaaaaaaaiaaaaaa
jiaaaaaaaaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaa
acaaaaaaahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaa
laaaaaaaabaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaljaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofe
aaeoepfcenebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheojiaaaaaa
afaaaaaaaiaaaaaaiaaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaa
imaaaaaaaaaaaaaaaaaaaaaaadaaaaaaabaaaaaaadamaaaaimaaaaaaabaaaaaa
aaaaaaaaadaaaaaaacaaaaaaahaiaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaa
adaaaaaaahaiaaaaimaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaahaiaaaa
fdfgfpfaepfdejfeejepeoaafeeffiedepepfceeaaklklkl"
}

SubProgram "opengl " {
Keywords { "DIRECTIONAL" }
Bind "vertex" Vertex
Bind "tangent" ATTR14
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 9 [_WorldSpaceCameraPos]
Vector 10 [_WorldSpaceLightPos0]
Matrix 5 [_World2Object]
Vector 11 [unity_Scale]
Vector 12 [_MainTex_ST]
"!!ARBvp1.0
# 25 ALU
PARAM c[13] = { { 1 },
		state.matrix.mvp,
		program.local[5..12] };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
MOV R1.xyz, c[9];
MOV R1.w, c[0].x;
MOV R0.xyz, vertex.attrib[14];
DP4 R2.z, R1, c[7];
DP4 R2.y, R1, c[6];
DP4 R2.x, R1, c[5];
MAD R2.xyz, R2, c[11].w, -vertex.position;
MUL R1.xyz, vertex.normal.zxyw, R0.yzxw;
MAD R1.xyz, vertex.normal.yzxw, R0.zxyw, -R1;
MOV R0, c[10];
MUL R1.xyz, R1, vertex.attrib[14].w;
DP4 R3.z, R0, c[7];
DP4 R3.y, R0, c[6];
DP4 R3.x, R0, c[5];
DP3 result.texcoord[1].y, R3, R1;
DP3 result.texcoord[2].y, R1, R2;
DP3 result.texcoord[1].z, vertex.normal, R3;
DP3 result.texcoord[1].x, R3, vertex.attrib[14];
DP3 result.texcoord[2].z, vertex.normal, R2;
DP3 result.texcoord[2].x, vertex.attrib[14], R2;
MAD result.texcoord[0].xy, vertex.texcoord[0], c[12], c[12].zwzw;
DP4 result.position.w, vertex.position, c[4];
DP4 result.position.z, vertex.position, c[3];
DP4 result.position.y, vertex.position, c[2];
DP4 result.position.x, vertex.position, c[1];
END
# 25 instructions, 4 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "DIRECTIONAL" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Matrix 0 [glstate_matrix_mvp]
Vector 8 [_WorldSpaceCameraPos]
Vector 9 [_WorldSpaceLightPos0]
Matrix 4 [_World2Object]
Vector 10 [unity_Scale]
Vector 11 [_MainTex_ST]
"vs_2_0
; 28 ALU
def c12, 1.00000000, 0, 0, 0
dcl_position0 v0
dcl_tangent0 v1
dcl_normal0 v2
dcl_texcoord0 v3
mov r0.w, c12.x
mov r0.xyz, c8
dp4 r1.z, r0, c6
dp4 r1.y, r0, c5
dp4 r1.x, r0, c4
mad r3.xyz, r1, c10.w, -v0
mov r0.xyz, v1
mul r1.xyz, v2.zxyw, r0.yzxw
mov r0.xyz, v1
mad r1.xyz, v2.yzxw, r0.zxyw, -r1
mul r2.xyz, r1, v1.w
mov r0, c6
dp4 r4.z, c9, r0
mov r0, c5
mov r1, c4
dp4 r4.y, c9, r0
dp4 r4.x, c9, r1
dp3 oT1.y, r4, r2
dp3 oT2.y, r2, r3
dp3 oT1.z, v2, r4
dp3 oT1.x, r4, v1
dp3 oT2.z, v2, r3
dp3 oT2.x, v1, r3
mad oT0.xy, v3, c11, c11.zwzw
dp4 oPos.w, v0, c3
dp4 oPos.z, v0, c2
dp4 oPos.y, v0, c1
dp4 oPos.x, v0, c0
"
}

SubProgram "xbox360 " {
Keywords { "DIRECTIONAL" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 11 [_MainTex_ST]
Matrix 6 [_World2Object] 4
Vector 0 [_WorldSpaceCameraPos]
Vector 1 [_WorldSpaceLightPos0]
Matrix 2 [glstate_matrix_mvp] 4
Vector 10 [unity_Scale]
// Shader Timing Estimate, in Cycles/64 vertex vector:
// ALU: 29.33 (22 instructions), vertex: 32, texture: 0,
//   sequencer: 14,  7 GPRs, 27 threads,
// Performance (if enough threads): ~32 cycles per vector
// * Vertex cycle estimates are assuming 3 vfetch_minis for every vfetch_full,
//     with <= 32 bytes per vfetch_full group.

"vs_360
backbbabaaaaabmiaaaaabheaaaaaaaaaaaaaaceaaaaaaaaaaaaabgiaaaaaaaa
aaaaaaaaaaaaabeaaaaaaabmaaaaabddpppoadaaaaaaaaagaaaaaabmaaaaaaaa
aaaaabcmaaaaaajeaaacaaalaaabaaaaaaaaaakaaaaaaaaaaaaaaalaaaacaaag
aaaeaaaaaaaaaamaaaaaaaaaaaaaaanaaaacaaaaaaabaaaaaaaaaaoiaaaaaaaa
aaaaaapiaaacaaabaaabaaaaaaaaaakaaaaaaaaaaaaaabanaaacaaacaaaeaaaa
aaaaaamaaaaaaaaaaaaaabcaaaacaaakaaabaaaaaaaaaakaaaaaaaaafpengbgj
gofegfhifpfdfeaaaaabaaadaaabaaaeaaabaaaaaaaaaaaafpfhgphcgmgedcep
gcgkgfgdheaaklklaaadaaadaaaeaaaeaaabaaaaaaaaaaaafpfhgphcgmgefdha
gbgdgfedgbgngfhcgbfagphdaaklklklaaabaaadaaabaaadaaabaaaaaaaaaaaa
fpfhgphcgmgefdhagbgdgfemgjghgihefagphddaaaghgmhdhegbhegffpgngbhe
hcgjhifpgnhghaaahfgogjhehjfpfdgdgbgmgfaahghdfpddfpdaaadccodacodc
dadddfddcodaaaklaaaaaaaaaaaaabheaacbaaagaaaaaaaaaaaaaaaaaaaacagd
aaaaaaabaaaaaaaeaaaaaaahaaaaacjaaabaaaaeaaaagaafaaaadaagaadafaah
aaaadafaaaabhbfbaaaehcfcaaaababnaaaaaabhaaaaaabiaaaababjaaaaaabk
aaaaaablaaaababmpaffeaaeaaaabcaamcaaaaaaaaaaeaaiaaaabcaameaaaaaa
aaaagaamgabcbcaabcaaaaaaaaaagabiaaaaccaaaaaaaaaaafpicaaaaaaaagii
aaaaaaaaafpieaaaaaaaagiiaaaaaaaaafpibaaaaaaaaoiiaaaaaaaaafpiaaaa
aaaaapmiaaaaaaaamiapaaadaabliiaakbacafaamiapaaadaamgiiaaklacaead
miapaaadaalbdejeklacadadmiapiadoaagmaadeklacacadmiahaaafaaleblaa
cbajabaamiahaaadaamamgmaalaiaaajmiahaaagaalelbleclahaaadmiahaaad
aalogfaaobabaeaamiahaaafaamamgleclaiabafmiahaaafaalelbleclahabaf
miahaaadabgflomaolabaeadmiahaaagaamagmleclagaaagmiahaaacabmablma
klagakacmiahaaadaamablaaobadaeaamiahaaafaamagmleclagabafmiabiaab
aaloloaapaafaeaamiaciaabaaloloaapaadafaamiaeiaabaaloloaapaafabaa
miabiaacaaloloaapaacaeaamiaciaacaaloloaapaadacaamiaeiaacaaloloaa
paacabaamiadiaaaaalalabkilaaalalaaaaaaaaaaaaaaaaaaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "DIRECTIONAL" }
Matrix 256 [glstate_matrix_mvp]
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 467 [_WorldSpaceCameraPos]
Vector 466 [_WorldSpaceLightPos0]
Matrix 260 [_World2Object]
Vector 465 [unity_Scale]
Vector 464 [_MainTex_ST]
"sce_vp_rsx // 24 instructions using 4 registers
[Configuration]
8
0000001841050400
[Microcode]
384
00001c6c005d200d8186c0836041fffc00009c6c00400e0c0106c0836041dffc
00011c6c005d300c0186c0836041dffc401f9c6c011d0808010400d740619f9c
401f9c6c01d0300d8106c0c360403f80401f9c6c01d0200d8106c0c360405f80
401f9c6c01d0100d8106c0c360409f80401f9c6c01d0000d8106c0c360411f80
00019c6c01d0600d8086c0c360405ffc00019c6c01d0500d8086c0c360409ffc
00019c6c01d0400d8086c0c360411ffc00001c6c0190600c0486c0c360405ffc
00001c6c0190500c0486c0c360409ffc00001c6c0190400c0486c0c360411ffc
00011c6c00800243011841436041dffc00009c6c01000230812181630121dffc
00001c6c011d100c00bfc0e30041dffc401f9c6c0140020c0106034360405fa0
401f9c6c01400e0c0686008360411fa000009c6c00800e0c02bfc0836041dffc
401f9c6c0140020c0106004360405fa4401f9c6c01400e0c0106004360411fa4
401f9c6c0140000c0686014360409fa0401f9c6c0140000c0286004360409fa5
"
}

SubProgram "d3d11 " {
Keywords { "DIRECTIONAL" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 96 // 96 used size, 7 vars
Vector 80 [_MainTex_ST] 4
ConstBuffer "UnityPerCamera" 128 // 76 used size, 8 vars
Vector 64 [_WorldSpaceCameraPos] 3
ConstBuffer "UnityLighting" 400 // 16 used size, 16 vars
Vector 0 [_WorldSpaceLightPos0] 4
ConstBuffer "UnityPerDraw" 336 // 336 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
Matrix 256 [_World2Object] 4
Vector 320 [unity_Scale] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityLighting" 2
BindCB "UnityPerDraw" 3
// 24 instructions, 2 temp regs, 0 temp arrays:
// ALU 12 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0
eefieceddadbmnnknlpboeoiggjhnpjeckedjolbabaaaaaahiafaaaaadaaaaaa
cmaaaaaapeaaaaaahmabaaaaejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaa
abaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaljaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfc
enebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheoiaaaaaaaaeaaaaaa
aiaaaaaagiaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaheaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaabaaaaaaadamaaaaheaaaaaaabaaaaaaaaaaaaaa
adaaaaaaacaaaaaaahaiaaaaheaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaa
ahaiaaaafdfgfpfaepfdejfeejepeoaafeeffiedepepfceeaaklklklfdeieefc
peadaaaaeaaaabaapnaaaaaafjaaaaaeegiocaaaaaaaaaaaagaaaaaafjaaaaae
egiocaaaabaaaaaaafaaaaaafjaaaaaeegiocaaaacaaaaaaabaaaaaafjaaaaae
egiocaaaadaaaaaabfaaaaaafpaaaaadpcbabaaaaaaaaaaafpaaaaadpcbabaaa
abaaaaaafpaaaaadhcbabaaaacaaaaaafpaaaaaddcbabaaaadaaaaaaghaaaaae
pccabaaaaaaaaaaaabaaaaaagfaaaaaddccabaaaabaaaaaagfaaaaadhccabaaa
acaaaaaagfaaaaadhccabaaaadaaaaaagiaaaaacacaaaaaadiaaaaaipcaabaaa
aaaaaaaafgbfbaaaaaaaaaaaegiocaaaadaaaaaaabaaaaaadcaaaaakpcaabaaa
aaaaaaaaegiocaaaadaaaaaaaaaaaaaaagbabaaaaaaaaaaaegaobaaaaaaaaaaa
dcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaacaaaaaakgbkbaaaaaaaaaaa
egaobaaaaaaaaaaadcaaaaakpccabaaaaaaaaaaaegiocaaaadaaaaaaadaaaaaa
pgbpbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaaldccabaaaabaaaaaaegbabaaa
adaaaaaaegiacaaaaaaaaaaaafaaaaaaogikcaaaaaaaaaaaafaaaaaadiaaaaah
hcaabaaaaaaaaaaajgbebaaaabaaaaaacgbjbaaaacaaaaaadcaaaaakhcaabaaa
aaaaaaaajgbebaaaacaaaaaacgbjbaaaabaaaaaaegacbaiaebaaaaaaaaaaaaaa
diaaaaahhcaabaaaaaaaaaaaegacbaaaaaaaaaaapgbpbaaaabaaaaaadiaaaaaj
hcaabaaaabaaaaaafgifcaaaacaaaaaaaaaaaaaaegiccaaaadaaaaaabbaaaaaa
dcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaaacaaaaaa
aaaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaa
bcaaaaaakgikcaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaa
abaaaaaaegiccaaaadaaaaaabdaaaaaapgipcaaaacaaaaaaaaaaaaaaegacbaaa
abaaaaaabaaaaaahcccabaaaacaaaaaaegacbaaaaaaaaaaaegacbaaaabaaaaaa
baaaaaahbccabaaaacaaaaaaegbcbaaaabaaaaaaegacbaaaabaaaaaabaaaaaah
eccabaaaacaaaaaaegbcbaaaacaaaaaaegacbaaaabaaaaaadiaaaaajhcaabaaa
abaaaaaafgifcaaaabaaaaaaaeaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaal
hcaabaaaabaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaaabaaaaaaaeaaaaaa
egacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabcaaaaaa
kgikcaaaabaaaaaaaeaaaaaaegacbaaaabaaaaaaaaaaaaaihcaabaaaabaaaaaa
egacbaaaabaaaaaaegiccaaaadaaaaaabdaaaaaadcaaaaalhcaabaaaabaaaaaa
egacbaaaabaaaaaapgipcaaaadaaaaaabeaaaaaaegbcbaiaebaaaaaaaaaaaaaa
baaaaaahcccabaaaadaaaaaaegacbaaaaaaaaaaaegacbaaaabaaaaaabaaaaaah
bccabaaaadaaaaaaegbcbaaaabaaaaaaegacbaaaabaaaaaabaaaaaaheccabaaa
adaaaaaaegbcbaaaacaaaaaaegacbaaaabaaaaaadoaaaaab"
}

SubProgram "gles " {
Keywords { "DIRECTIONAL" }
"!!GLES
#define SHADER_API_GLES 1
#define tex2D texture2D


#ifdef VERTEX
#define gl_ModelViewProjectionMatrix glstate_matrix_mvp
uniform mat4 glstate_matrix_mvp;

varying mediump vec3 xlv_TEXCOORD2;
varying mediump vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 unity_Scale;
uniform highp mat4 _World2Object;

uniform lowp vec4 _WorldSpaceLightPos0;
uniform highp vec3 _WorldSpaceCameraPos;
attribute vec4 _glesTANGENT;
attribute vec4 _glesMultiTexCoord0;
attribute vec3 _glesNormal;
attribute vec4 _glesVertex;
void main ()
{
  vec4 tmpvar_1;
  tmpvar_1.xyz = normalize(_glesTANGENT.xyz);
  tmpvar_1.w = _glesTANGENT.w;
  vec3 tmpvar_2;
  tmpvar_2 = normalize(_glesNormal);
  mediump vec3 tmpvar_3;
  mediump vec3 tmpvar_4;
  highp vec3 tmpvar_5;
  highp vec3 tmpvar_6;
  tmpvar_5 = tmpvar_1.xyz;
  tmpvar_6 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_7;
  tmpvar_7[0].x = tmpvar_5.x;
  tmpvar_7[0].y = tmpvar_6.x;
  tmpvar_7[0].z = tmpvar_2.x;
  tmpvar_7[1].x = tmpvar_5.y;
  tmpvar_7[1].y = tmpvar_6.y;
  tmpvar_7[1].z = tmpvar_2.y;
  tmpvar_7[2].x = tmpvar_5.z;
  tmpvar_7[2].y = tmpvar_6.z;
  tmpvar_7[2].z = tmpvar_2.z;
  highp vec3 tmpvar_8;
  tmpvar_8 = (tmpvar_7 * (_World2Object * _WorldSpaceLightPos0).xyz);
  tmpvar_3 = tmpvar_8;
  highp vec4 tmpvar_9;
  tmpvar_9.w = 1.0;
  tmpvar_9.xyz = _WorldSpaceCameraPos;
  highp vec3 tmpvar_10;
  tmpvar_10 = (tmpvar_7 * (((_World2Object * tmpvar_9).xyz * unity_Scale.w) - _glesVertex.xyz));
  tmpvar_4 = tmpvar_10;
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = tmpvar_3;
  xlv_TEXCOORD2 = tmpvar_4;
}



#endif
#ifdef FRAGMENT

varying mediump vec3 xlv_TEXCOORD2;
varying mediump vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform mediump float _Gloss;
uniform mediump float _Shininess;
uniform lowp vec4 _Color;
uniform sampler2D _SpecMap;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform lowp vec4 _LightColor0;
void main ()
{
  lowp vec4 c_1;
  lowp vec3 lightDir_2;
  mediump vec3 tmpvar_3;
  mediump vec3 tmpvar_4;
  lowp vec4 tmpvar_5;
  tmpvar_5 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_6;
  tmpvar_6 = (tmpvar_5.xyz * _Color.xyz);
  tmpvar_3 = tmpvar_6;
  lowp vec4 tmpvar_7;
  tmpvar_7 = texture2D (_SpecMap, xlv_TEXCOORD0);
  mediump vec3 tmpvar_8;
  tmpvar_8 = ((tmpvar_5.w * tmpvar_7.xyz) * _Gloss);
  lowp vec3 tmpvar_9;
  tmpvar_9 = ((texture2D (_BumpMap, xlv_TEXCOORD0).xyz * 2.0) - 1.0);
  tmpvar_4 = tmpvar_9;
  lightDir_2 = xlv_TEXCOORD1;
  mediump vec3 lightDir_10;
  lightDir_10 = lightDir_2;
  mediump vec4 c_11;
  mediump vec3 specCol_12;
  highp float nh_13;
  mediump float tmpvar_14;
  tmpvar_14 = max (0.0, dot (tmpvar_4, normalize((lightDir_10 + normalize(xlv_TEXCOORD2)))));
  nh_13 = tmpvar_14;
  mediump float arg1_15;
  arg1_15 = (32.0 * _Shininess);
  highp vec3 tmpvar_16;
  tmpvar_16 = (pow (nh_13, arg1_15) * tmpvar_8);
  specCol_12 = tmpvar_16;
  c_11.xyz = ((((tmpvar_3 * _LightColor0.xyz) * max (0.0, dot (tmpvar_4, lightDir_10))) + (_LightColor0.xyz * specCol_12)) * 2.0);
  c_11.w = 0.0;
  c_1.xyz = c_11.xyz;
  c_1.w = 0.0;
  gl_FragData[0] = c_1;
}



#endif"
}

SubProgram "glesdesktop " {
Keywords { "DIRECTIONAL" }
"!!GLES
#define SHADER_API_GLES 1
#define tex2D texture2D


#ifdef VERTEX
#define gl_ModelViewProjectionMatrix glstate_matrix_mvp
uniform mat4 glstate_matrix_mvp;

varying mediump vec3 xlv_TEXCOORD2;
varying mediump vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 unity_Scale;
uniform highp mat4 _World2Object;

uniform lowp vec4 _WorldSpaceLightPos0;
uniform highp vec3 _WorldSpaceCameraPos;
attribute vec4 _glesTANGENT;
attribute vec4 _glesMultiTexCoord0;
attribute vec3 _glesNormal;
attribute vec4 _glesVertex;
void main ()
{
  vec4 tmpvar_1;
  tmpvar_1.xyz = normalize(_glesTANGENT.xyz);
  tmpvar_1.w = _glesTANGENT.w;
  vec3 tmpvar_2;
  tmpvar_2 = normalize(_glesNormal);
  mediump vec3 tmpvar_3;
  mediump vec3 tmpvar_4;
  highp vec3 tmpvar_5;
  highp vec3 tmpvar_6;
  tmpvar_5 = tmpvar_1.xyz;
  tmpvar_6 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_7;
  tmpvar_7[0].x = tmpvar_5.x;
  tmpvar_7[0].y = tmpvar_6.x;
  tmpvar_7[0].z = tmpvar_2.x;
  tmpvar_7[1].x = tmpvar_5.y;
  tmpvar_7[1].y = tmpvar_6.y;
  tmpvar_7[1].z = tmpvar_2.y;
  tmpvar_7[2].x = tmpvar_5.z;
  tmpvar_7[2].y = tmpvar_6.z;
  tmpvar_7[2].z = tmpvar_2.z;
  highp vec3 tmpvar_8;
  tmpvar_8 = (tmpvar_7 * (_World2Object * _WorldSpaceLightPos0).xyz);
  tmpvar_3 = tmpvar_8;
  highp vec4 tmpvar_9;
  tmpvar_9.w = 1.0;
  tmpvar_9.xyz = _WorldSpaceCameraPos;
  highp vec3 tmpvar_10;
  tmpvar_10 = (tmpvar_7 * (((_World2Object * tmpvar_9).xyz * unity_Scale.w) - _glesVertex.xyz));
  tmpvar_4 = tmpvar_10;
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = tmpvar_3;
  xlv_TEXCOORD2 = tmpvar_4;
}



#endif
#ifdef FRAGMENT

varying mediump vec3 xlv_TEXCOORD2;
varying mediump vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform mediump float _Gloss;
uniform mediump float _Shininess;
uniform lowp vec4 _Color;
uniform sampler2D _SpecMap;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform lowp vec4 _LightColor0;
void main ()
{
  lowp vec4 c_1;
  lowp vec3 lightDir_2;
  mediump vec3 tmpvar_3;
  mediump vec3 tmpvar_4;
  lowp vec4 tmpvar_5;
  tmpvar_5 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_6;
  tmpvar_6 = (tmpvar_5.xyz * _Color.xyz);
  tmpvar_3 = tmpvar_6;
  lowp vec4 tmpvar_7;
  tmpvar_7 = texture2D (_SpecMap, xlv_TEXCOORD0);
  mediump vec3 tmpvar_8;
  tmpvar_8 = ((tmpvar_5.w * tmpvar_7.xyz) * _Gloss);
  lowp vec3 normal_9;
  normal_9.xy = ((texture2D (_BumpMap, xlv_TEXCOORD0).wy * 2.0) - 1.0);
  normal_9.z = sqrt(((1.0 - (normal_9.x * normal_9.x)) - (normal_9.y * normal_9.y)));
  tmpvar_4 = normal_9;
  lightDir_2 = xlv_TEXCOORD1;
  mediump vec3 lightDir_10;
  lightDir_10 = lightDir_2;
  mediump vec4 c_11;
  mediump vec3 specCol_12;
  highp float nh_13;
  mediump float tmpvar_14;
  tmpvar_14 = max (0.0, dot (tmpvar_4, normalize((lightDir_10 + normalize(xlv_TEXCOORD2)))));
  nh_13 = tmpvar_14;
  mediump float arg1_15;
  arg1_15 = (32.0 * _Shininess);
  highp vec3 tmpvar_16;
  tmpvar_16 = (pow (nh_13, arg1_15) * tmpvar_8);
  specCol_12 = tmpvar_16;
  c_11.xyz = ((((tmpvar_3 * _LightColor0.xyz) * max (0.0, dot (tmpvar_4, lightDir_10))) + (_LightColor0.xyz * specCol_12)) * 2.0);
  c_11.w = 0.0;
  c_1.xyz = c_11.xyz;
  c_1.w = 0.0;
  gl_FragData[0] = c_1;
}



#endif"
}

SubProgram "flash " {
Keywords { "DIRECTIONAL" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Matrix 0 [glstate_matrix_mvp]
Vector 8 [_WorldSpaceCameraPos]
Vector 9 [_WorldSpaceLightPos0]
Matrix 4 [_World2Object]
Vector 10 [unity_Scale]
Vector 11 [_MainTex_ST]
"agal_vs
c12 1.0 0.0 0.0 0.0
[bc]
aaaaaaaaaaaaaiacamaaaaaaabaaaaaaaaaaaaaaaaaaaaaa mov r0.w, c12.x
aaaaaaaaaaaaahacaiaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, c8
bdaaaaaaabaaaeacaaaaaaoeacaaaaaaagaaaaoeabaaaaaa dp4 r1.z, r0, c6
bdaaaaaaabaaacacaaaaaaoeacaaaaaaafaaaaoeabaaaaaa dp4 r1.y, r0, c5
bdaaaaaaabaaabacaaaaaaoeacaaaaaaaeaaaaoeabaaaaaa dp4 r1.x, r0, c4
adaaaaaaacaaahacabaaaakeacaaaaaaakaaaappabaaaaaa mul r2.xyz, r1.xyzz, c10.w
acaaaaaaadaaahacacaaaakeacaaaaaaaaaaaaoeaaaaaaaa sub r3.xyz, r2.xyzz, a0
aaaaaaaaaaaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, a5
adaaaaaaabaaahacabaaaancaaaaaaaaaaaaaaajacaaaaaa mul r1.xyz, a1.zxyw, r0.yzxx
aaaaaaaaaaaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, a5
adaaaaaaaeaaahacabaaaamjaaaaaaaaaaaaaafcacaaaaaa mul r4.xyz, a1.yzxw, r0.zxyy
acaaaaaaabaaahacaeaaaakeacaaaaaaabaaaakeacaaaaaa sub r1.xyz, r4.xyzz, r1.xyzz
adaaaaaaacaaahacabaaaakeacaaaaaaafaaaappaaaaaaaa mul r2.xyz, r1.xyzz, a5.w
aaaaaaaaaaaaapacagaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0, c6
bdaaaaaaaeaaaeacajaaaaoeabaaaaaaaaaaaaoeacaaaaaa dp4 r4.z, c9, r0
aaaaaaaaaaaaapacafaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0, c5
aaaaaaaaabaaapacaeaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r1, c4
bdaaaaaaaeaaacacajaaaaoeabaaaaaaaaaaaaoeacaaaaaa dp4 r4.y, c9, r0
bdaaaaaaaeaaabacajaaaaoeabaaaaaaabaaaaoeacaaaaaa dp4 r4.x, c9, r1
bcaaaaaaabaaacaeaeaaaakeacaaaaaaacaaaakeacaaaaaa dp3 v1.y, r4.xyzz, r2.xyzz
bcaaaaaaacaaacaeacaaaakeacaaaaaaadaaaakeacaaaaaa dp3 v2.y, r2.xyzz, r3.xyzz
bcaaaaaaabaaaeaeabaaaaoeaaaaaaaaaeaaaakeacaaaaaa dp3 v1.z, a1, r4.xyzz
bcaaaaaaabaaabaeaeaaaakeacaaaaaaafaaaaoeaaaaaaaa dp3 v1.x, r4.xyzz, a5
bcaaaaaaacaaaeaeabaaaaoeaaaaaaaaadaaaakeacaaaaaa dp3 v2.z, a1, r3.xyzz
bcaaaaaaacaaabaeafaaaaoeaaaaaaaaadaaaakeacaaaaaa dp3 v2.x, a5, r3.xyzz
adaaaaaaaaaaadacadaaaaoeaaaaaaaaalaaaaoeabaaaaaa mul r0.xy, a3, c11
abaaaaaaaaaaadaeaaaaaafeacaaaaaaalaaaaooabaaaaaa add v0.xy, r0.xyyy, c11.zwzw
bdaaaaaaaaaaaiadaaaaaaoeaaaaaaaaadaaaaoeabaaaaaa dp4 o0.w, a0, c3
bdaaaaaaaaaaaeadaaaaaaoeaaaaaaaaacaaaaoeabaaaaaa dp4 o0.z, a0, c2
bdaaaaaaaaaaacadaaaaaaoeaaaaaaaaabaaaaoeabaaaaaa dp4 o0.y, a0, c1
bdaaaaaaaaaaabadaaaaaaoeaaaaaaaaaaaaaaoeabaaaaaa dp4 o0.x, a0, c0
aaaaaaaaaaaaamaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v0.zw, c0
aaaaaaaaabaaaiaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v1.w, c0
aaaaaaaaacaaaiaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v2.w, c0
"
}

SubProgram "d3d11_9x " {
Keywords { "DIRECTIONAL" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 96 // 96 used size, 7 vars
Vector 80 [_MainTex_ST] 4
ConstBuffer "UnityPerCamera" 128 // 76 used size, 8 vars
Vector 64 [_WorldSpaceCameraPos] 3
ConstBuffer "UnityLighting" 400 // 16 used size, 16 vars
Vector 0 [_WorldSpaceLightPos0] 4
ConstBuffer "UnityPerDraw" 336 // 336 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
Matrix 256 [_World2Object] 4
Vector 320 [unity_Scale] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityLighting" 2
BindCB "UnityPerDraw" 3
// 24 instructions, 2 temp regs, 0 temp arrays:
// ALU 12 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0_level_9_3
eefiecedcmcpfdeiginfkkmbnedjcpfcnecmaapfabaaaaaaaaaiaaaaaeaaaaaa
daaaaaaaleacaaaalaagaaaahiahaaaaebgpgodjhmacaaaahmacaaaaaaacpopp
biacaaaageaaaaaaafaaceaaaaaagaaaaaaagaaaaaaaceaaabaagaaaaaaaafaa
abaaabaaaaaaaaaaabaaaeaaabaaacaaaaaaaaaaacaaaaaaabaaadaaaaaaaaaa
adaaaaaaaeaaaeaaaaaaaaaaadaabaaaafaaaiaaaaaaaaaaaaaaaaaaabacpopp
bpaaaaacafaaaaiaaaaaapjabpaaaaacafaaabiaabaaapjabpaaaaacafaaacia
acaaapjabpaaaaacafaaadiaadaaapjaaeaaaaaeaaaaadoaadaaoejaabaaoeka
abaaookaabaaaaacaaaaapiaadaaoekaafaaaaadabaaahiaaaaaffiaajaaoeka
aeaaaaaeabaaahiaaiaaoekaaaaaaaiaabaaoeiaaeaaaaaeaaaaahiaakaaoeka
aaaakkiaabaaoeiaaeaaaaaeaaaaahiaalaaoekaaaaappiaaaaaoeiaaiaaaaad
abaaaboaabaaoejaaaaaoeiaabaaaaacabaaahiaabaaoejaafaaaaadacaaahia
abaamjiaacaancjaaeaaaaaeabaaahiaacaamjjaabaanciaacaaoeibafaaaaad
abaaahiaabaaoeiaabaappjaaiaaaaadabaaacoaabaaoeiaaaaaoeiaaiaaaaad
abaaaeoaacaaoejaaaaaoeiaabaaaaacaaaaahiaacaaoekaafaaaaadacaaahia
aaaaffiaajaaoekaaeaaaaaeaaaaaliaaiaakekaaaaaaaiaacaakeiaaeaaaaae
aaaaahiaakaaoekaaaaakkiaaaaapeiaacaaaaadaaaaahiaaaaaoeiaalaaoeka
aeaaaaaeaaaaahiaaaaaoeiaamaappkaaaaaoejbaiaaaaadacaaaboaabaaoeja
aaaaoeiaaiaaaaadacaaacoaabaaoeiaaaaaoeiaaiaaaaadacaaaeoaacaaoeja
aaaaoeiaafaaaaadaaaaapiaaaaaffjaafaaoekaaeaaaaaeaaaaapiaaeaaoeka
aaaaaajaaaaaoeiaaeaaaaaeaaaaapiaagaaoekaaaaakkjaaaaaoeiaaeaaaaae
aaaaapiaahaaoekaaaaappjaaaaaoeiaaeaaaaaeaaaaadmaaaaappiaaaaaoeka
aaaaoeiaabaaaaacaaaaammaaaaaoeiappppaaaafdeieefcpeadaaaaeaaaabaa
pnaaaaaafjaaaaaeegiocaaaaaaaaaaaagaaaaaafjaaaaaeegiocaaaabaaaaaa
afaaaaaafjaaaaaeegiocaaaacaaaaaaabaaaaaafjaaaaaeegiocaaaadaaaaaa
bfaaaaaafpaaaaadpcbabaaaaaaaaaaafpaaaaadpcbabaaaabaaaaaafpaaaaad
hcbabaaaacaaaaaafpaaaaaddcbabaaaadaaaaaaghaaaaaepccabaaaaaaaaaaa
abaaaaaagfaaaaaddccabaaaabaaaaaagfaaaaadhccabaaaacaaaaaagfaaaaad
hccabaaaadaaaaaagiaaaaacacaaaaaadiaaaaaipcaabaaaaaaaaaaafgbfbaaa
aaaaaaaaegiocaaaadaaaaaaabaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaa
adaaaaaaaaaaaaaaagbabaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaa
aaaaaaaaegiocaaaadaaaaaaacaaaaaakgbkbaaaaaaaaaaaegaobaaaaaaaaaaa
dcaaaaakpccabaaaaaaaaaaaegiocaaaadaaaaaaadaaaaaapgbpbaaaaaaaaaaa
egaobaaaaaaaaaaadcaaaaaldccabaaaabaaaaaaegbabaaaadaaaaaaegiacaaa
aaaaaaaaafaaaaaaogikcaaaaaaaaaaaafaaaaaadiaaaaahhcaabaaaaaaaaaaa
jgbebaaaabaaaaaacgbjbaaaacaaaaaadcaaaaakhcaabaaaaaaaaaaajgbebaaa
acaaaaaacgbjbaaaabaaaaaaegacbaiaebaaaaaaaaaaaaaadiaaaaahhcaabaaa
aaaaaaaaegacbaaaaaaaaaaapgbpbaaaabaaaaaadiaaaaajhcaabaaaabaaaaaa
fgifcaaaacaaaaaaaaaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaa
abaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaaacaaaaaaaaaaaaaaegacbaaa
abaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaa
acaaaaaaaaaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaa
adaaaaaabdaaaaaapgipcaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaabaaaaaah
cccabaaaacaaaaaaegacbaaaaaaaaaaaegacbaaaabaaaaaabaaaaaahbccabaaa
acaaaaaaegbcbaaaabaaaaaaegacbaaaabaaaaaabaaaaaaheccabaaaacaaaaaa
egbcbaaaacaaaaaaegacbaaaabaaaaaadiaaaaajhcaabaaaabaaaaaafgifcaaa
abaaaaaaaeaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaaabaaaaaa
egiccaaaadaaaaaabaaaaaaaagiacaaaabaaaaaaaeaaaaaaegacbaaaabaaaaaa
dcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaaabaaaaaa
aeaaaaaaegacbaaaabaaaaaaaaaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaa
egiccaaaadaaaaaabdaaaaaadcaaaaalhcaabaaaabaaaaaaegacbaaaabaaaaaa
pgipcaaaadaaaaaabeaaaaaaegbcbaiaebaaaaaaaaaaaaaabaaaaaahcccabaaa
adaaaaaaegacbaaaaaaaaaaaegacbaaaabaaaaaabaaaaaahbccabaaaadaaaaaa
egbcbaaaabaaaaaaegacbaaaabaaaaaabaaaaaaheccabaaaadaaaaaaegbcbaaa
acaaaaaaegacbaaaabaaaaaadoaaaaabejfdeheomaaaaaaaagaaaaaaaiaaaaaa
jiaaaaaaaaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaa
acaaaaaaahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaa
laaaaaaaabaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaljaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofe
aaeoepfcenebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheoiaaaaaaa
aeaaaaaaaiaaaaaagiaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaa
heaaaaaaaaaaaaaaaaaaaaaaadaaaaaaabaaaaaaadamaaaaheaaaaaaabaaaaaa
aaaaaaaaadaaaaaaacaaaaaaahaiaaaaheaaaaaaacaaaaaaaaaaaaaaadaaaaaa
adaaaaaaahaiaaaafdfgfpfaepfdejfeejepeoaafeeffiedepepfceeaaklklkl
"
}

SubProgram "opengl " {
Keywords { "SPOT" }
Bind "vertex" Vertex
Bind "tangent" ATTR14
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 17 [_WorldSpaceCameraPos]
Vector 18 [_WorldSpaceLightPos0]
Matrix 5 [_Object2World]
Matrix 9 [_World2Object]
Vector 19 [unity_Scale]
Matrix 13 [_LightMatrix0]
Vector 20 [_MainTex_ST]
"!!ARBvp1.0
# 34 ALU
PARAM c[21] = { { 1 },
		state.matrix.mvp,
		program.local[5..20] };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
MOV R1.xyz, c[17];
MOV R1.w, c[0].x;
MOV R0.xyz, vertex.attrib[14];
DP4 R2.z, R1, c[11];
DP4 R2.y, R1, c[10];
DP4 R2.x, R1, c[9];
MAD R2.xyz, R2, c[19].w, -vertex.position;
MUL R1.xyz, vertex.normal.zxyw, R0.yzxw;
MAD R1.xyz, vertex.normal.yzxw, R0.zxyw, -R1;
MOV R0, c[18];
MUL R1.xyz, R1, vertex.attrib[14].w;
DP4 R3.z, R0, c[11];
DP4 R3.x, R0, c[9];
DP4 R3.y, R0, c[10];
MAD R0.xyz, R3, c[19].w, -vertex.position;
DP4 R0.w, vertex.position, c[8];
DP3 result.texcoord[1].y, R0, R1;
DP3 result.texcoord[1].z, vertex.normal, R0;
DP3 result.texcoord[1].x, R0, vertex.attrib[14];
DP4 R0.z, vertex.position, c[7];
DP4 R0.x, vertex.position, c[5];
DP4 R0.y, vertex.position, c[6];
DP3 result.texcoord[2].y, R1, R2;
DP3 result.texcoord[2].z, vertex.normal, R2;
DP3 result.texcoord[2].x, vertex.attrib[14], R2;
DP4 result.texcoord[3].w, R0, c[16];
DP4 result.texcoord[3].z, R0, c[15];
DP4 result.texcoord[3].y, R0, c[14];
DP4 result.texcoord[3].x, R0, c[13];
MAD result.texcoord[0].xy, vertex.texcoord[0], c[20], c[20].zwzw;
DP4 result.position.w, vertex.position, c[4];
DP4 result.position.z, vertex.position, c[3];
DP4 result.position.y, vertex.position, c[2];
DP4 result.position.x, vertex.position, c[1];
END
# 34 instructions, 4 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "SPOT" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Matrix 0 [glstate_matrix_mvp]
Vector 16 [_WorldSpaceCameraPos]
Vector 17 [_WorldSpaceLightPos0]
Matrix 4 [_Object2World]
Matrix 8 [_World2Object]
Vector 18 [unity_Scale]
Matrix 12 [_LightMatrix0]
Vector 19 [_MainTex_ST]
"vs_2_0
; 37 ALU
def c20, 1.00000000, 0, 0, 0
dcl_position0 v0
dcl_tangent0 v1
dcl_normal0 v2
dcl_texcoord0 v3
mov r0.w, c20.x
mov r0.xyz, c16
dp4 r1.z, r0, c10
dp4 r1.y, r0, c9
dp4 r1.x, r0, c8
mad r3.xyz, r1, c18.w, -v0
mov r0.xyz, v1
mul r1.xyz, v2.zxyw, r0.yzxw
mov r0.xyz, v1
mad r1.xyz, v2.yzxw, r0.zxyw, -r1
mul r2.xyz, r1, v1.w
mov r0, c10
dp4 r4.z, c17, r0
mov r0, c9
dp4 r4.y, c17, r0
mov r1, c8
dp4 r4.x, c17, r1
mad r0.xyz, r4, c18.w, -v0
dp4 r0.w, v0, c7
dp3 oT1.y, r0, r2
dp3 oT1.z, v2, r0
dp3 oT1.x, r0, v1
dp4 r0.z, v0, c6
dp4 r0.x, v0, c4
dp4 r0.y, v0, c5
dp3 oT2.y, r2, r3
dp3 oT2.z, v2, r3
dp3 oT2.x, v1, r3
dp4 oT3.w, r0, c15
dp4 oT3.z, r0, c14
dp4 oT3.y, r0, c13
dp4 oT3.x, r0, c12
mad oT0.xy, v3, c19, c19.zwzw
dp4 oPos.w, v0, c3
dp4 oPos.z, v0, c2
dp4 oPos.y, v0, c1
dp4 oPos.x, v0, c0
"
}

SubProgram "xbox360 " {
Keywords { "SPOT" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Matrix 15 [_LightMatrix0] 4
Vector 19 [_MainTex_ST]
Matrix 6 [_Object2World] 4
Matrix 10 [_World2Object] 4
Vector 0 [_WorldSpaceCameraPos]
Vector 1 [_WorldSpaceLightPos0]
Matrix 2 [glstate_matrix_mvp] 4
Vector 14 [unity_Scale]
// Shader Timing Estimate, in Cycles/64 vertex vector:
// ALU: 41.33 (31 instructions), vertex: 32, texture: 0,
//   sequencer: 18,  8 GPRs, 24 threads,
// Performance (if enough threads): ~41 cycles per vector
// * Vertex cycle estimates are assuming 3 vfetch_minis for every vfetch_full,
//     with <= 32 bytes per vfetch_full group.

"vs_360
backbbabaaaaacbeaaaaabomaaaaaaaaaaaaaaceaaaaaaaaaaaaabkmaaaaaaaa
aaaaaaaaaaaaabieaaaaaabmaaaaabhhpppoadaaaaaaaaaiaaaaaabmaaaaaaaa
aaaaabhaaaaaaalmaaacaaapaaaeaaaaaaaaaammaaaaaaaaaaaaaanmaaacaabd
aaabaaaaaaaaaaoiaaaaaaaaaaaaaapiaaacaaagaaaeaaaaaaaaaammaaaaaaaa
aaaaabagaaacaaakaaaeaaaaaaaaaammaaaaaaaaaaaaabbeaaacaaaaaaabaaaa
aaaaabcmaaaaaaaaaaaaabdmaaacaaabaaabaaaaaaaaaaoiaaaaaaaaaaaaabfb
aaacaaacaaaeaaaaaaaaaammaaaaaaaaaaaaabgeaaacaaaoaaabaaaaaaaaaaoi
aaaaaaaafpemgjghgiheengbhehcgjhidaaaklklaaadaaadaaaeaaaeaaabaaaa
aaaaaaaafpengbgjgofegfhifpfdfeaaaaabaaadaaabaaaeaaabaaaaaaaaaaaa
fpepgcgkgfgdhedcfhgphcgmgeaafpfhgphcgmgedcepgcgkgfgdheaafpfhgphc
gmgefdhagbgdgfedgbgngfhcgbfagphdaaklklklaaabaaadaaabaaadaaabaaaa
aaaaaaaafpfhgphcgmgefdhagbgdgfemgjghgihefagphddaaaghgmhdhegbhegf
fpgngbhehcgjhifpgnhghaaahfgogjhehjfpfdgdgbgmgfaahghdfpddfpdaaadc
codacodcdadddfddcodaaaklaaaaaaaaaaaaabomaadbaaahaaaaaaaaaaaaaaaa
aaaadaieaaaaaaabaaaaaaaeaaaaaaaiaaaaacjaaabaaaafaaaagaagaaaadaah
aadafaaiaaaadafaaaabhbfbaaaehcfcaaahpdfdaaaabacdaaaaaabnaaaaaabo
aaaababpaaaaaacaaaaaaacbaaaabaccaaaabachpaffeaafaaaabcaamcaaaaaa
aaaaeaajaaaabcaameaaaaaaaaaagaangabdbcaabcaaaaaaaaaagabjgabpbcaa
bcaaaaaaaaaadacfaaaaccaaaaaaaaaaafpigaaaaaaaagiiaaaaaaaaafpifaaa
aaaaagiiaaaaaaaaafpicaaaaaaaaoiiaaaaaaaaafpiaaaaaaaaapmiaaaaaaaa
miapaaabaabliiaakbagafaamiapaaabaamgiiaaklagaeabmiapaaabaalbdeje
klagadabmiapiadoaagmaadeklagacabmiahaaabaaleblaacbanabaamiahaaad
aamamgmaalamaaanmiahaaadaalelbleclalaaadmiahaaaeaalogfaaobacafaa
miahaaahaamamgleclamababmiapaaabaabliiaakbagajaamiapaaabaamgiiaa
klagaiabmiahaaahaalelbleclalabahmiahaaaeabgflomaolacafaemiahaaad
aamagmleclakaaadmiahaaadabmablmakladaoagmiahaaaeaamablaaobaeafaa
miahaaahaamagmleclakabahmiapaaabaalbdejeklagahabmiapaaabaagmnaje
klagagabmiahaaagabmablmaklahaoagmiabiaabaaloloaapaagafaamiaciaab
aaloloaapaaeagaamiaeiaabaaloloaapaagacaamiabiaacaaloloaapaadafaa
miaciaacaaloloaapaaeadaamiaeiaacaaloloaapaadacaamiadiaaaaalalabk
ilaabdbdmiapaaaaaamgiiaakbabbcaamiapaaaaaabliiaaklabbbaamiapaaaa
aalbdejeklabbaaamiapiaadaagmaadeklabapaaaaaaaaaaaaaaaaaaaaaaaaaa
"
}

SubProgram "ps3 " {
Keywords { "SPOT" }
Matrix 256 [glstate_matrix_mvp]
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 467 [_WorldSpaceCameraPos]
Vector 466 [_WorldSpaceLightPos0]
Matrix 260 [_Object2World]
Matrix 264 [_World2Object]
Vector 465 [unity_Scale]
Matrix 268 [_LightMatrix0]
Vector 464 [_MainTex_ST]
"sce_vp_rsx // 33 instructions using 5 registers
[Configuration]
8
0000002141050500
[Microcode]
528
00009c6c005d200d8186c0836041fffc00011c6c00400e0c0106c0836041dffc
00019c6c005d300c0186c0836041dffc401f9c6c011d0808010400d740619f9c
401f9c6c01d0300d8106c0c360403f80401f9c6c01d0200d8106c0c360405f80
401f9c6c01d0100d8106c0c360409f80401f9c6c01d0000d8106c0c360411f80
00001c6c01d0700d8106c0c360403ffc00001c6c01d0600d8106c0c360405ffc
00001c6c01d0500d8106c0c360409ffc00001c6c01d0400d8106c0c360411ffc
00021c6c01d0a00d8286c0c360405ffc00021c6c01d0900d8286c0c360409ffc
00021c6c01d0800d8286c0c360411ffc00009c6c0190a00c0686c0c360405ffc
00009c6c0190900c0686c0c360409ffc00009c6c0190800c0686c0c360411ffc
00019c6c00800243011842436041dffc00011c6c010002308121826301a1dffc
401f9c6c01d0f00d8086c0c360403fa8401f9c6c01d0e00d8086c0c360405fa8
401f9c6c01d0d00d8086c0c360409fa8401f9c6c01d0c00d8086c0c360411fa8
00001c6c011d100c08bfc0e30041dffc00009c6c011d100c02bfc0e30041dffc
401f9c6c0140020c0106004360405fa0401f9c6c01400e0c0086008360411fa0
00011c6c00800e0c04bfc0836041dffc401f9c6c0140020c0106014360405fa4
401f9c6c01400e0c0106014360411fa4401f9c6c0140000c0086024360409fa0
401f9c6c0140000c0486014360409fa5
"
}

SubProgram "d3d11 " {
Keywords { "SPOT" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 160 // 160 used size, 8 vars
Matrix 48 [_LightMatrix0] 4
Vector 144 [_MainTex_ST] 4
ConstBuffer "UnityPerCamera" 128 // 76 used size, 8 vars
Vector 64 [_WorldSpaceCameraPos] 3
ConstBuffer "UnityLighting" 400 // 16 used size, 16 vars
Vector 0 [_WorldSpaceLightPos0] 4
ConstBuffer "UnityPerDraw" 336 // 336 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
Matrix 192 [_Object2World] 4
Matrix 256 [_World2Object] 4
Vector 320 [unity_Scale] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityLighting" 2
BindCB "UnityPerDraw" 3
// 33 instructions, 2 temp regs, 0 temp arrays:
// ALU 14 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0
eefiecedngihocidihbbfolijamjafhfmjianhggabaaaaaapiagaaaaadaaaaaa
cmaaaaaapeaaaaaajeabaaaaejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaa
abaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaljaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfc
enebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheojiaaaaaaafaaaaaa
aiaaaaaaiaaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaabaaaaaaadamaaaaimaaaaaaabaaaaaaaaaaaaaa
adaaaaaaacaaaaaaahaiaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaa
ahaiaaaaimaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaafdfgfpfa
epfdejfeejepeoaafeeffiedepepfceeaaklklklfdeieefcfmafaaaaeaaaabaa
fhabaaaafjaaaaaeegiocaaaaaaaaaaaakaaaaaafjaaaaaeegiocaaaabaaaaaa
afaaaaaafjaaaaaeegiocaaaacaaaaaaabaaaaaafjaaaaaeegiocaaaadaaaaaa
bfaaaaaafpaaaaadpcbabaaaaaaaaaaafpaaaaadpcbabaaaabaaaaaafpaaaaad
hcbabaaaacaaaaaafpaaaaaddcbabaaaadaaaaaaghaaaaaepccabaaaaaaaaaaa
abaaaaaagfaaaaaddccabaaaabaaaaaagfaaaaadhccabaaaacaaaaaagfaaaaad
hccabaaaadaaaaaagfaaaaadpccabaaaaeaaaaaagiaaaaacacaaaaaadiaaaaai
pcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaadaaaaaaabaaaaaadcaaaaak
pcaabaaaaaaaaaaaegiocaaaadaaaaaaaaaaaaaaagbabaaaaaaaaaaaegaobaaa
aaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaacaaaaaakgbkbaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaakpccabaaaaaaaaaaaegiocaaaadaaaaaa
adaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaaldccabaaaabaaaaaa
egbabaaaadaaaaaaegiacaaaaaaaaaaaajaaaaaaogikcaaaaaaaaaaaajaaaaaa
diaaaaahhcaabaaaaaaaaaaajgbebaaaabaaaaaacgbjbaaaacaaaaaadcaaaaak
hcaabaaaaaaaaaaajgbebaaaacaaaaaacgbjbaaaabaaaaaaegacbaiaebaaaaaa
aaaaaaaadiaaaaahhcaabaaaaaaaaaaaegacbaaaaaaaaaaapgbpbaaaabaaaaaa
diaaaaajhcaabaaaabaaaaaafgifcaaaacaaaaaaaaaaaaaaegiccaaaadaaaaaa
bbaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaa
acaaaaaaaaaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaa
adaaaaaabcaaaaaakgikcaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaadcaaaaal
hcaabaaaabaaaaaaegiccaaaadaaaaaabdaaaaaapgipcaaaacaaaaaaaaaaaaaa
egacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegacbaaaabaaaaaapgipcaaa
adaaaaaabeaaaaaaegbcbaiaebaaaaaaaaaaaaaabaaaaaahcccabaaaacaaaaaa
egacbaaaaaaaaaaaegacbaaaabaaaaaabaaaaaahbccabaaaacaaaaaaegbcbaaa
abaaaaaaegacbaaaabaaaaaabaaaaaaheccabaaaacaaaaaaegbcbaaaacaaaaaa
egacbaaaabaaaaaadiaaaaajhcaabaaaabaaaaaafgifcaaaabaaaaaaaeaaaaaa
egiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaa
baaaaaaaagiacaaaabaaaaaaaeaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaa
abaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaaabaaaaaaaeaaaaaaegacbaaa
abaaaaaaaaaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaaegiccaaaadaaaaaa
bdaaaaaadcaaaaalhcaabaaaabaaaaaaegacbaaaabaaaaaapgipcaaaadaaaaaa
beaaaaaaegbcbaiaebaaaaaaaaaaaaaabaaaaaahcccabaaaadaaaaaaegacbaaa
aaaaaaaaegacbaaaabaaaaaabaaaaaahbccabaaaadaaaaaaegbcbaaaabaaaaaa
egacbaaaabaaaaaabaaaaaaheccabaaaadaaaaaaegbcbaaaacaaaaaaegacbaaa
abaaaaaadiaaaaaipcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaadaaaaaa
anaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaamaaaaaaagbabaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaa
aoaaaaaakgbkbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaa
egiocaaaadaaaaaaapaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadiaaaaai
pcaabaaaabaaaaaafgafbaaaaaaaaaaaegiocaaaaaaaaaaaaeaaaaaadcaaaaak
pcaabaaaabaaaaaaegiocaaaaaaaaaaaadaaaaaaagaabaaaaaaaaaaaegaobaaa
abaaaaaadcaaaaakpcaabaaaabaaaaaaegiocaaaaaaaaaaaafaaaaaakgakbaaa
aaaaaaaaegaobaaaabaaaaaadcaaaaakpccabaaaaeaaaaaaegiocaaaaaaaaaaa
agaaaaaapgapbaaaaaaaaaaaegaobaaaabaaaaaadoaaaaab"
}

SubProgram "gles " {
Keywords { "SPOT" }
"!!GLES
#define SHADER_API_GLES 1
#define tex2D texture2D


#ifdef VERTEX
#define gl_ModelViewProjectionMatrix glstate_matrix_mvp
uniform mat4 glstate_matrix_mvp;

varying highp vec4 xlv_TEXCOORD3;
varying mediump vec3 xlv_TEXCOORD2;
varying mediump vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp vec4 _MainTex_ST;
uniform highp mat4 _LightMatrix0;
uniform highp vec4 unity_Scale;
uniform highp mat4 _World2Object;
uniform highp mat4 _Object2World;

uniform highp vec4 _WorldSpaceLightPos0;
uniform highp vec3 _WorldSpaceCameraPos;
attribute vec4 _glesTANGENT;
attribute vec4 _glesMultiTexCoord0;
attribute vec3 _glesNormal;
attribute vec4 _glesVertex;
void main ()
{
  vec4 tmpvar_1;
  tmpvar_1.xyz = normalize(_glesTANGENT.xyz);
  tmpvar_1.w = _glesTANGENT.w;
  vec3 tmpvar_2;
  tmpvar_2 = normalize(_glesNormal);
  mediump vec3 tmpvar_3;
  mediump vec3 tmpvar_4;
  highp vec3 tmpvar_5;
  highp vec3 tmpvar_6;
  tmpvar_5 = tmpvar_1.xyz;
  tmpvar_6 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_7;
  tmpvar_7[0].x = tmpvar_5.x;
  tmpvar_7[0].y = tmpvar_6.x;
  tmpvar_7[0].z = tmpvar_2.x;
  tmpvar_7[1].x = tmpvar_5.y;
  tmpvar_7[1].y = tmpvar_6.y;
  tmpvar_7[1].z = tmpvar_2.y;
  tmpvar_7[2].x = tmpvar_5.z;
  tmpvar_7[2].y = tmpvar_6.z;
  tmpvar_7[2].z = tmpvar_2.z;
  highp vec3 tmpvar_8;
  tmpvar_8 = (tmpvar_7 * (((_World2Object * _WorldSpaceLightPos0).xyz * unity_Scale.w) - _glesVertex.xyz));
  tmpvar_3 = tmpvar_8;
  highp vec4 tmpvar_9;
  tmpvar_9.w = 1.0;
  tmpvar_9.xyz = _WorldSpaceCameraPos;
  highp vec3 tmpvar_10;
  tmpvar_10 = (tmpvar_7 * (((_World2Object * tmpvar_9).xyz * unity_Scale.w) - _glesVertex.xyz));
  tmpvar_4 = tmpvar_10;
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = tmpvar_3;
  xlv_TEXCOORD2 = tmpvar_4;
  xlv_TEXCOORD3 = (_LightMatrix0 * (_Object2World * _glesVertex));
}



#endif
#ifdef FRAGMENT

varying highp vec4 xlv_TEXCOORD3;
varying mediump vec3 xlv_TEXCOORD2;
varying mediump vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform mediump float _Gloss;
uniform mediump float _Shininess;
uniform lowp vec4 _Color;
uniform sampler2D _SpecMap;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform sampler2D _LightTextureB0;
uniform sampler2D _LightTexture0;
uniform lowp vec4 _LightColor0;
void main ()
{
  lowp vec4 c_1;
  lowp vec3 lightDir_2;
  mediump vec3 tmpvar_3;
  mediump vec3 tmpvar_4;
  lowp vec4 tmpvar_5;
  tmpvar_5 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_6;
  tmpvar_6 = (tmpvar_5.xyz * _Color.xyz);
  tmpvar_3 = tmpvar_6;
  lowp vec4 tmpvar_7;
  tmpvar_7 = texture2D (_SpecMap, xlv_TEXCOORD0);
  mediump vec3 tmpvar_8;
  tmpvar_8 = ((tmpvar_5.w * tmpvar_7.xyz) * _Gloss);
  lowp vec3 tmpvar_9;
  tmpvar_9 = ((texture2D (_BumpMap, xlv_TEXCOORD0).xyz * 2.0) - 1.0);
  tmpvar_4 = tmpvar_9;
  mediump vec3 tmpvar_10;
  tmpvar_10 = normalize(xlv_TEXCOORD1);
  lightDir_2 = tmpvar_10;
  lowp vec4 tmpvar_11;
  highp vec2 P_12;
  P_12 = ((xlv_TEXCOORD3.xy / xlv_TEXCOORD3.w) + 0.5);
  tmpvar_11 = texture2D (_LightTexture0, P_12);
  highp float tmpvar_13;
  tmpvar_13 = dot (xlv_TEXCOORD3.xyz, xlv_TEXCOORD3.xyz);
  lowp vec4 tmpvar_14;
  tmpvar_14 = texture2D (_LightTextureB0, vec2(tmpvar_13));
  mediump vec3 lightDir_15;
  lightDir_15 = lightDir_2;
  mediump float atten_16;
  atten_16 = ((float((xlv_TEXCOORD3.z > 0.0)) * tmpvar_11.w) * tmpvar_14.w);
  mediump vec4 c_17;
  mediump vec3 specCol_18;
  highp float nh_19;
  mediump float tmpvar_20;
  tmpvar_20 = max (0.0, dot (tmpvar_4, normalize((lightDir_15 + normalize(xlv_TEXCOORD2)))));
  nh_19 = tmpvar_20;
  mediump float arg1_21;
  arg1_21 = (32.0 * _Shininess);
  highp vec3 tmpvar_22;
  tmpvar_22 = (pow (nh_19, arg1_21) * tmpvar_8);
  specCol_18 = tmpvar_22;
  c_17.xyz = ((((tmpvar_3 * _LightColor0.xyz) * max (0.0, dot (tmpvar_4, lightDir_15))) + (_LightColor0.xyz * specCol_18)) * (atten_16 * 2.0));
  c_17.w = 0.0;
  c_1.xyz = c_17.xyz;
  c_1.w = 0.0;
  gl_FragData[0] = c_1;
}



#endif"
}

SubProgram "glesdesktop " {
Keywords { "SPOT" }
"!!GLES
#define SHADER_API_GLES 1
#define tex2D texture2D


#ifdef VERTEX
#define gl_ModelViewProjectionMatrix glstate_matrix_mvp
uniform mat4 glstate_matrix_mvp;

varying highp vec4 xlv_TEXCOORD3;
varying mediump vec3 xlv_TEXCOORD2;
varying mediump vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp vec4 _MainTex_ST;
uniform highp mat4 _LightMatrix0;
uniform highp vec4 unity_Scale;
uniform highp mat4 _World2Object;
uniform highp mat4 _Object2World;

uniform highp vec4 _WorldSpaceLightPos0;
uniform highp vec3 _WorldSpaceCameraPos;
attribute vec4 _glesTANGENT;
attribute vec4 _glesMultiTexCoord0;
attribute vec3 _glesNormal;
attribute vec4 _glesVertex;
void main ()
{
  vec4 tmpvar_1;
  tmpvar_1.xyz = normalize(_glesTANGENT.xyz);
  tmpvar_1.w = _glesTANGENT.w;
  vec3 tmpvar_2;
  tmpvar_2 = normalize(_glesNormal);
  mediump vec3 tmpvar_3;
  mediump vec3 tmpvar_4;
  highp vec3 tmpvar_5;
  highp vec3 tmpvar_6;
  tmpvar_5 = tmpvar_1.xyz;
  tmpvar_6 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_7;
  tmpvar_7[0].x = tmpvar_5.x;
  tmpvar_7[0].y = tmpvar_6.x;
  tmpvar_7[0].z = tmpvar_2.x;
  tmpvar_7[1].x = tmpvar_5.y;
  tmpvar_7[1].y = tmpvar_6.y;
  tmpvar_7[1].z = tmpvar_2.y;
  tmpvar_7[2].x = tmpvar_5.z;
  tmpvar_7[2].y = tmpvar_6.z;
  tmpvar_7[2].z = tmpvar_2.z;
  highp vec3 tmpvar_8;
  tmpvar_8 = (tmpvar_7 * (((_World2Object * _WorldSpaceLightPos0).xyz * unity_Scale.w) - _glesVertex.xyz));
  tmpvar_3 = tmpvar_8;
  highp vec4 tmpvar_9;
  tmpvar_9.w = 1.0;
  tmpvar_9.xyz = _WorldSpaceCameraPos;
  highp vec3 tmpvar_10;
  tmpvar_10 = (tmpvar_7 * (((_World2Object * tmpvar_9).xyz * unity_Scale.w) - _glesVertex.xyz));
  tmpvar_4 = tmpvar_10;
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = tmpvar_3;
  xlv_TEXCOORD2 = tmpvar_4;
  xlv_TEXCOORD3 = (_LightMatrix0 * (_Object2World * _glesVertex));
}



#endif
#ifdef FRAGMENT

varying highp vec4 xlv_TEXCOORD3;
varying mediump vec3 xlv_TEXCOORD2;
varying mediump vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform mediump float _Gloss;
uniform mediump float _Shininess;
uniform lowp vec4 _Color;
uniform sampler2D _SpecMap;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform sampler2D _LightTextureB0;
uniform sampler2D _LightTexture0;
uniform lowp vec4 _LightColor0;
void main ()
{
  lowp vec4 c_1;
  lowp vec3 lightDir_2;
  mediump vec3 tmpvar_3;
  mediump vec3 tmpvar_4;
  lowp vec4 tmpvar_5;
  tmpvar_5 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_6;
  tmpvar_6 = (tmpvar_5.xyz * _Color.xyz);
  tmpvar_3 = tmpvar_6;
  lowp vec4 tmpvar_7;
  tmpvar_7 = texture2D (_SpecMap, xlv_TEXCOORD0);
  mediump vec3 tmpvar_8;
  tmpvar_8 = ((tmpvar_5.w * tmpvar_7.xyz) * _Gloss);
  lowp vec3 normal_9;
  normal_9.xy = ((texture2D (_BumpMap, xlv_TEXCOORD0).wy * 2.0) - 1.0);
  normal_9.z = sqrt(((1.0 - (normal_9.x * normal_9.x)) - (normal_9.y * normal_9.y)));
  tmpvar_4 = normal_9;
  mediump vec3 tmpvar_10;
  tmpvar_10 = normalize(xlv_TEXCOORD1);
  lightDir_2 = tmpvar_10;
  lowp vec4 tmpvar_11;
  highp vec2 P_12;
  P_12 = ((xlv_TEXCOORD3.xy / xlv_TEXCOORD3.w) + 0.5);
  tmpvar_11 = texture2D (_LightTexture0, P_12);
  highp float tmpvar_13;
  tmpvar_13 = dot (xlv_TEXCOORD3.xyz, xlv_TEXCOORD3.xyz);
  lowp vec4 tmpvar_14;
  tmpvar_14 = texture2D (_LightTextureB0, vec2(tmpvar_13));
  mediump vec3 lightDir_15;
  lightDir_15 = lightDir_2;
  mediump float atten_16;
  atten_16 = ((float((xlv_TEXCOORD3.z > 0.0)) * tmpvar_11.w) * tmpvar_14.w);
  mediump vec4 c_17;
  mediump vec3 specCol_18;
  highp float nh_19;
  mediump float tmpvar_20;
  tmpvar_20 = max (0.0, dot (tmpvar_4, normalize((lightDir_15 + normalize(xlv_TEXCOORD2)))));
  nh_19 = tmpvar_20;
  mediump float arg1_21;
  arg1_21 = (32.0 * _Shininess);
  highp vec3 tmpvar_22;
  tmpvar_22 = (pow (nh_19, arg1_21) * tmpvar_8);
  specCol_18 = tmpvar_22;
  c_17.xyz = ((((tmpvar_3 * _LightColor0.xyz) * max (0.0, dot (tmpvar_4, lightDir_15))) + (_LightColor0.xyz * specCol_18)) * (atten_16 * 2.0));
  c_17.w = 0.0;
  c_1.xyz = c_17.xyz;
  c_1.w = 0.0;
  gl_FragData[0] = c_1;
}



#endif"
}

SubProgram "flash " {
Keywords { "SPOT" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Matrix 0 [glstate_matrix_mvp]
Vector 16 [_WorldSpaceCameraPos]
Vector 17 [_WorldSpaceLightPos0]
Matrix 4 [_Object2World]
Matrix 8 [_World2Object]
Vector 18 [unity_Scale]
Matrix 12 [_LightMatrix0]
Vector 19 [_MainTex_ST]
"agal_vs
c20 1.0 0.0 0.0 0.0
[bc]
aaaaaaaaaaaaaiacbeaaaaaaabaaaaaaaaaaaaaaaaaaaaaa mov r0.w, c20.x
aaaaaaaaaaaaahacbaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, c16
bdaaaaaaabaaaeacaaaaaaoeacaaaaaaakaaaaoeabaaaaaa dp4 r1.z, r0, c10
bdaaaaaaabaaacacaaaaaaoeacaaaaaaajaaaaoeabaaaaaa dp4 r1.y, r0, c9
bdaaaaaaabaaabacaaaaaaoeacaaaaaaaiaaaaoeabaaaaaa dp4 r1.x, r0, c8
adaaaaaaacaaahacabaaaakeacaaaaaabcaaaappabaaaaaa mul r2.xyz, r1.xyzz, c18.w
acaaaaaaadaaahacacaaaakeacaaaaaaaaaaaaoeaaaaaaaa sub r3.xyz, r2.xyzz, a0
aaaaaaaaaaaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, a5
adaaaaaaabaaahacabaaaancaaaaaaaaaaaaaaajacaaaaaa mul r1.xyz, a1.zxyw, r0.yzxx
aaaaaaaaaaaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, a5
adaaaaaaaeaaahacabaaaamjaaaaaaaaaaaaaafcacaaaaaa mul r4.xyz, a1.yzxw, r0.zxyy
acaaaaaaabaaahacaeaaaakeacaaaaaaabaaaakeacaaaaaa sub r1.xyz, r4.xyzz, r1.xyzz
adaaaaaaacaaahacabaaaakeacaaaaaaafaaaappaaaaaaaa mul r2.xyz, r1.xyzz, a5.w
aaaaaaaaaaaaapacakaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0, c10
bdaaaaaaaeaaaeacbbaaaaoeabaaaaaaaaaaaaoeacaaaaaa dp4 r4.z, c17, r0
aaaaaaaaaaaaapacajaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0, c9
bdaaaaaaaeaaacacbbaaaaoeabaaaaaaaaaaaaoeacaaaaaa dp4 r4.y, c17, r0
aaaaaaaaabaaapacaiaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r1, c8
bdaaaaaaaeaaabacbbaaaaoeabaaaaaaabaaaaoeacaaaaaa dp4 r4.x, c17, r1
adaaaaaaabaaahacaeaaaakeacaaaaaabcaaaappabaaaaaa mul r1.xyz, r4.xyzz, c18.w
acaaaaaaaaaaahacabaaaakeacaaaaaaaaaaaaoeaaaaaaaa sub r0.xyz, r1.xyzz, a0
bdaaaaaaaaaaaiacaaaaaaoeaaaaaaaaahaaaaoeabaaaaaa dp4 r0.w, a0, c7
bcaaaaaaabaaacaeaaaaaakeacaaaaaaacaaaakeacaaaaaa dp3 v1.y, r0.xyzz, r2.xyzz
bcaaaaaaabaaaeaeabaaaaoeaaaaaaaaaaaaaakeacaaaaaa dp3 v1.z, a1, r0.xyzz
bcaaaaaaabaaabaeaaaaaakeacaaaaaaafaaaaoeaaaaaaaa dp3 v1.x, r0.xyzz, a5
bdaaaaaaaaaaaeacaaaaaaoeaaaaaaaaagaaaaoeabaaaaaa dp4 r0.z, a0, c6
bdaaaaaaaaaaabacaaaaaaoeaaaaaaaaaeaaaaoeabaaaaaa dp4 r0.x, a0, c4
bdaaaaaaaaaaacacaaaaaaoeaaaaaaaaafaaaaoeabaaaaaa dp4 r0.y, a0, c5
bcaaaaaaacaaacaeacaaaakeacaaaaaaadaaaakeacaaaaaa dp3 v2.y, r2.xyzz, r3.xyzz
bcaaaaaaacaaaeaeabaaaaoeaaaaaaaaadaaaakeacaaaaaa dp3 v2.z, a1, r3.xyzz
bcaaaaaaacaaabaeafaaaaoeaaaaaaaaadaaaakeacaaaaaa dp3 v2.x, a5, r3.xyzz
bdaaaaaaadaaaiaeaaaaaaoeacaaaaaaapaaaaoeabaaaaaa dp4 v3.w, r0, c15
bdaaaaaaadaaaeaeaaaaaaoeacaaaaaaaoaaaaoeabaaaaaa dp4 v3.z, r0, c14
bdaaaaaaadaaacaeaaaaaaoeacaaaaaaanaaaaoeabaaaaaa dp4 v3.y, r0, c13
bdaaaaaaadaaabaeaaaaaaoeacaaaaaaamaaaaoeabaaaaaa dp4 v3.x, r0, c12
adaaaaaaaaaaadacadaaaaoeaaaaaaaabdaaaaoeabaaaaaa mul r0.xy, a3, c19
abaaaaaaaaaaadaeaaaaaafeacaaaaaabdaaaaooabaaaaaa add v0.xy, r0.xyyy, c19.zwzw
bdaaaaaaaaaaaiadaaaaaaoeaaaaaaaaadaaaaoeabaaaaaa dp4 o0.w, a0, c3
bdaaaaaaaaaaaeadaaaaaaoeaaaaaaaaacaaaaoeabaaaaaa dp4 o0.z, a0, c2
bdaaaaaaaaaaacadaaaaaaoeaaaaaaaaabaaaaoeabaaaaaa dp4 o0.y, a0, c1
bdaaaaaaaaaaabadaaaaaaoeaaaaaaaaaaaaaaoeabaaaaaa dp4 o0.x, a0, c0
aaaaaaaaaaaaamaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v0.zw, c0
aaaaaaaaabaaaiaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v1.w, c0
aaaaaaaaacaaaiaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v2.w, c0
"
}

SubProgram "d3d11_9x " {
Keywords { "SPOT" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 160 // 160 used size, 8 vars
Matrix 48 [_LightMatrix0] 4
Vector 144 [_MainTex_ST] 4
ConstBuffer "UnityPerCamera" 128 // 76 used size, 8 vars
Vector 64 [_WorldSpaceCameraPos] 3
ConstBuffer "UnityLighting" 400 // 16 used size, 16 vars
Vector 0 [_WorldSpaceLightPos0] 4
ConstBuffer "UnityPerDraw" 336 // 336 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
Matrix 192 [_Object2World] 4
Matrix 256 [_World2Object] 4
Vector 320 [unity_Scale] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityLighting" 2
BindCB "UnityPerDraw" 3
// 33 instructions, 2 temp regs, 0 temp arrays:
// ALU 14 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0_level_9_3
eefiecedbjnjaghkfkpecdbgbbmelipnjjmdbhdfabaaaaaadiakaaaaaeaaaaaa
daaaaaaagmadaaaanaaiaaaajiajaaaaebgpgodjdeadaaaadeadaaaaaaacpopp
meacaaaahaaaaaaaagaaceaaaaaagmaaaaaagmaaaaaaceaaabaagmaaaaaaadaa
aeaaabaaaaaaaaaaaaaaajaaabaaafaaaaaaaaaaabaaaeaaabaaagaaaaaaaaaa
acaaaaaaabaaahaaaaaaaaaaadaaaaaaaeaaaiaaaaaaaaaaadaaamaaajaaamaa
aaaaaaaaaaaaaaaaabacpoppbpaaaaacafaaaaiaaaaaapjabpaaaaacafaaabia
abaaapjabpaaaaacafaaaciaacaaapjabpaaaaacafaaadiaadaaapjaaeaaaaae
aaaaadoaadaaoejaafaaoekaafaaookaabaaaaacaaaaapiaahaaoekaafaaaaad
abaaahiaaaaaffiabbaaoekaaeaaaaaeabaaahiabaaaoekaaaaaaaiaabaaoeia
aeaaaaaeaaaaahiabcaaoekaaaaakkiaabaaoeiaaeaaaaaeaaaaahiabdaaoeka
aaaappiaaaaaoeiaaeaaaaaeaaaaahiaaaaaoeiabeaappkaaaaaoejbaiaaaaad
abaaaboaabaaoejaaaaaoeiaabaaaaacabaaahiaabaaoejaafaaaaadacaaahia
abaamjiaacaancjaaeaaaaaeabaaahiaacaamjjaabaanciaacaaoeibafaaaaad
abaaahiaabaaoeiaabaappjaaiaaaaadabaaacoaabaaoeiaaaaaoeiaaiaaaaad
abaaaeoaacaaoejaaaaaoeiaabaaaaacaaaaahiaagaaoekaafaaaaadacaaahia
aaaaffiabbaaoekaaeaaaaaeaaaaaliabaaakekaaaaaaaiaacaakeiaaeaaaaae
aaaaahiabcaaoekaaaaakkiaaaaapeiaacaaaaadaaaaahiaaaaaoeiabdaaoeka
aeaaaaaeaaaaahiaaaaaoeiabeaappkaaaaaoejbaiaaaaadacaaaboaabaaoeja
aaaaoeiaaiaaaaadacaaacoaabaaoeiaaaaaoeiaaiaaaaadacaaaeoaacaaoeja
aaaaoeiaafaaaaadaaaaapiaaaaaffjaanaaoekaaeaaaaaeaaaaapiaamaaoeka
aaaaaajaaaaaoeiaaeaaaaaeaaaaapiaaoaaoekaaaaakkjaaaaaoeiaaeaaaaae
aaaaapiaapaaoekaaaaappjaaaaaoeiaafaaaaadabaaapiaaaaaffiaacaaoeka
aeaaaaaeabaaapiaabaaoekaaaaaaaiaabaaoeiaaeaaaaaeabaaapiaadaaoeka
aaaakkiaabaaoeiaaeaaaaaeadaaapoaaeaaoekaaaaappiaabaaoeiaafaaaaad
aaaaapiaaaaaffjaajaaoekaaeaaaaaeaaaaapiaaiaaoekaaaaaaajaaaaaoeia
aeaaaaaeaaaaapiaakaaoekaaaaakkjaaaaaoeiaaeaaaaaeaaaaapiaalaaoeka
aaaappjaaaaaoeiaaeaaaaaeaaaaadmaaaaappiaaaaaoekaaaaaoeiaabaaaaac
aaaaammaaaaaoeiappppaaaafdeieefcfmafaaaaeaaaabaafhabaaaafjaaaaae
egiocaaaaaaaaaaaakaaaaaafjaaaaaeegiocaaaabaaaaaaafaaaaaafjaaaaae
egiocaaaacaaaaaaabaaaaaafjaaaaaeegiocaaaadaaaaaabfaaaaaafpaaaaad
pcbabaaaaaaaaaaafpaaaaadpcbabaaaabaaaaaafpaaaaadhcbabaaaacaaaaaa
fpaaaaaddcbabaaaadaaaaaaghaaaaaepccabaaaaaaaaaaaabaaaaaagfaaaaad
dccabaaaabaaaaaagfaaaaadhccabaaaacaaaaaagfaaaaadhccabaaaadaaaaaa
gfaaaaadpccabaaaaeaaaaaagiaaaaacacaaaaaadiaaaaaipcaabaaaaaaaaaaa
fgbfbaaaaaaaaaaaegiocaaaadaaaaaaabaaaaaadcaaaaakpcaabaaaaaaaaaaa
egiocaaaadaaaaaaaaaaaaaaagbabaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaak
pcaabaaaaaaaaaaaegiocaaaadaaaaaaacaaaaaakgbkbaaaaaaaaaaaegaobaaa
aaaaaaaadcaaaaakpccabaaaaaaaaaaaegiocaaaadaaaaaaadaaaaaapgbpbaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaaldccabaaaabaaaaaaegbabaaaadaaaaaa
egiacaaaaaaaaaaaajaaaaaaogikcaaaaaaaaaaaajaaaaaadiaaaaahhcaabaaa
aaaaaaaajgbebaaaabaaaaaacgbjbaaaacaaaaaadcaaaaakhcaabaaaaaaaaaaa
jgbebaaaacaaaaaacgbjbaaaabaaaaaaegacbaiaebaaaaaaaaaaaaaadiaaaaah
hcaabaaaaaaaaaaaegacbaaaaaaaaaaapgbpbaaaabaaaaaadiaaaaajhcaabaaa
abaaaaaafgifcaaaacaaaaaaaaaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaal
hcaabaaaabaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaaacaaaaaaaaaaaaaa
egacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabcaaaaaa
kgikcaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaa
egiccaaaadaaaaaabdaaaaaapgipcaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaa
dcaaaaalhcaabaaaabaaaaaaegacbaaaabaaaaaapgipcaaaadaaaaaabeaaaaaa
egbcbaiaebaaaaaaaaaaaaaabaaaaaahcccabaaaacaaaaaaegacbaaaaaaaaaaa
egacbaaaabaaaaaabaaaaaahbccabaaaacaaaaaaegbcbaaaabaaaaaaegacbaaa
abaaaaaabaaaaaaheccabaaaacaaaaaaegbcbaaaacaaaaaaegacbaaaabaaaaaa
diaaaaajhcaabaaaabaaaaaafgifcaaaabaaaaaaaeaaaaaaegiccaaaadaaaaaa
bbaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaa
abaaaaaaaeaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaa
adaaaaaabcaaaaaakgikcaaaabaaaaaaaeaaaaaaegacbaaaabaaaaaaaaaaaaai
hcaabaaaabaaaaaaegacbaaaabaaaaaaegiccaaaadaaaaaabdaaaaaadcaaaaal
hcaabaaaabaaaaaaegacbaaaabaaaaaapgipcaaaadaaaaaabeaaaaaaegbcbaia
ebaaaaaaaaaaaaaabaaaaaahcccabaaaadaaaaaaegacbaaaaaaaaaaaegacbaaa
abaaaaaabaaaaaahbccabaaaadaaaaaaegbcbaaaabaaaaaaegacbaaaabaaaaaa
baaaaaaheccabaaaadaaaaaaegbcbaaaacaaaaaaegacbaaaabaaaaaadiaaaaai
pcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaadaaaaaaanaaaaaadcaaaaak
pcaabaaaaaaaaaaaegiocaaaadaaaaaaamaaaaaaagbabaaaaaaaaaaaegaobaaa
aaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaaoaaaaaakgbkbaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaa
apaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadiaaaaaipcaabaaaabaaaaaa
fgafbaaaaaaaaaaaegiocaaaaaaaaaaaaeaaaaaadcaaaaakpcaabaaaabaaaaaa
egiocaaaaaaaaaaaadaaaaaaagaabaaaaaaaaaaaegaobaaaabaaaaaadcaaaaak
pcaabaaaabaaaaaaegiocaaaaaaaaaaaafaaaaaakgakbaaaaaaaaaaaegaobaaa
abaaaaaadcaaaaakpccabaaaaeaaaaaaegiocaaaaaaaaaaaagaaaaaapgapbaaa
aaaaaaaaegaobaaaabaaaaaadoaaaaabejfdeheomaaaaaaaagaaaaaaaiaaaaaa
jiaaaaaaaaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaa
acaaaaaaahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaa
laaaaaaaabaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaljaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofe
aaeoepfcenebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheojiaaaaaa
afaaaaaaaiaaaaaaiaaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaa
imaaaaaaaaaaaaaaaaaaaaaaadaaaaaaabaaaaaaadamaaaaimaaaaaaabaaaaaa
aaaaaaaaadaaaaaaacaaaaaaahaiaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaa
adaaaaaaahaiaaaaimaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaa
fdfgfpfaepfdejfeejepeoaafeeffiedepepfceeaaklklkl"
}

SubProgram "opengl " {
Keywords { "POINT_COOKIE" }
Bind "vertex" Vertex
Bind "tangent" ATTR14
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 17 [_WorldSpaceCameraPos]
Vector 18 [_WorldSpaceLightPos0]
Matrix 5 [_Object2World]
Matrix 9 [_World2Object]
Vector 19 [unity_Scale]
Matrix 13 [_LightMatrix0]
Vector 20 [_MainTex_ST]
"!!ARBvp1.0
# 33 ALU
PARAM c[21] = { { 1 },
		state.matrix.mvp,
		program.local[5..20] };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
MOV R1.xyz, c[17];
MOV R1.w, c[0].x;
MOV R0.xyz, vertex.attrib[14];
DP4 R2.z, R1, c[11];
DP4 R2.y, R1, c[10];
DP4 R2.x, R1, c[9];
MAD R2.xyz, R2, c[19].w, -vertex.position;
MUL R1.xyz, vertex.normal.zxyw, R0.yzxw;
MAD R1.xyz, vertex.normal.yzxw, R0.zxyw, -R1;
MOV R0, c[18];
MUL R1.xyz, R1, vertex.attrib[14].w;
DP4 R3.z, R0, c[11];
DP4 R3.x, R0, c[9];
DP4 R3.y, R0, c[10];
MAD R0.xyz, R3, c[19].w, -vertex.position;
DP3 result.texcoord[1].y, R0, R1;
DP3 result.texcoord[1].z, vertex.normal, R0;
DP3 result.texcoord[1].x, R0, vertex.attrib[14];
DP4 R0.w, vertex.position, c[8];
DP4 R0.z, vertex.position, c[7];
DP4 R0.x, vertex.position, c[5];
DP4 R0.y, vertex.position, c[6];
DP3 result.texcoord[2].y, R1, R2;
DP3 result.texcoord[2].z, vertex.normal, R2;
DP3 result.texcoord[2].x, vertex.attrib[14], R2;
DP4 result.texcoord[3].z, R0, c[15];
DP4 result.texcoord[3].y, R0, c[14];
DP4 result.texcoord[3].x, R0, c[13];
MAD result.texcoord[0].xy, vertex.texcoord[0], c[20], c[20].zwzw;
DP4 result.position.w, vertex.position, c[4];
DP4 result.position.z, vertex.position, c[3];
DP4 result.position.y, vertex.position, c[2];
DP4 result.position.x, vertex.position, c[1];
END
# 33 instructions, 4 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "POINT_COOKIE" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Matrix 0 [glstate_matrix_mvp]
Vector 16 [_WorldSpaceCameraPos]
Vector 17 [_WorldSpaceLightPos0]
Matrix 4 [_Object2World]
Matrix 8 [_World2Object]
Vector 18 [unity_Scale]
Matrix 12 [_LightMatrix0]
Vector 19 [_MainTex_ST]
"vs_2_0
; 36 ALU
def c20, 1.00000000, 0, 0, 0
dcl_position0 v0
dcl_tangent0 v1
dcl_normal0 v2
dcl_texcoord0 v3
mov r0.w, c20.x
mov r0.xyz, c16
dp4 r1.z, r0, c10
dp4 r1.y, r0, c9
dp4 r1.x, r0, c8
mad r3.xyz, r1, c18.w, -v0
mov r0.xyz, v1
mul r1.xyz, v2.zxyw, r0.yzxw
mov r0.xyz, v1
mad r1.xyz, v2.yzxw, r0.zxyw, -r1
mul r2.xyz, r1, v1.w
mov r0, c10
dp4 r4.z, c17, r0
mov r0, c9
dp4 r4.y, c17, r0
mov r1, c8
dp4 r4.x, c17, r1
mad r0.xyz, r4, c18.w, -v0
dp3 oT1.y, r0, r2
dp3 oT1.z, v2, r0
dp3 oT1.x, r0, v1
dp4 r0.w, v0, c7
dp4 r0.z, v0, c6
dp4 r0.x, v0, c4
dp4 r0.y, v0, c5
dp3 oT2.y, r2, r3
dp3 oT2.z, v2, r3
dp3 oT2.x, v1, r3
dp4 oT3.z, r0, c14
dp4 oT3.y, r0, c13
dp4 oT3.x, r0, c12
mad oT0.xy, v3, c19, c19.zwzw
dp4 oPos.w, v0, c3
dp4 oPos.z, v0, c2
dp4 oPos.y, v0, c1
dp4 oPos.x, v0, c0
"
}

SubProgram "xbox360 " {
Keywords { "POINT_COOKIE" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Matrix 15 [_LightMatrix0] 4
Vector 19 [_MainTex_ST]
Matrix 6 [_Object2World] 4
Matrix 10 [_World2Object] 4
Vector 0 [_WorldSpaceCameraPos]
Vector 1 [_WorldSpaceLightPos0]
Matrix 2 [glstate_matrix_mvp] 4
Vector 14 [unity_Scale]
// Shader Timing Estimate, in Cycles/64 vertex vector:
// ALU: 41.33 (31 instructions), vertex: 32, texture: 0,
//   sequencer: 18,  8 GPRs, 24 threads,
// Performance (if enough threads): ~41 cycles per vector
// * Vertex cycle estimates are assuming 3 vfetch_minis for every vfetch_full,
//     with <= 32 bytes per vfetch_full group.

"vs_360
backbbabaaaaacbeaaaaabomaaaaaaaaaaaaaaceaaaaaaaaaaaaabkmaaaaaaaa
aaaaaaaaaaaaabieaaaaaabmaaaaabhhpppoadaaaaaaaaaiaaaaaabmaaaaaaaa
aaaaabhaaaaaaalmaaacaaapaaaeaaaaaaaaaammaaaaaaaaaaaaaanmaaacaabd
aaabaaaaaaaaaaoiaaaaaaaaaaaaaapiaaacaaagaaaeaaaaaaaaaammaaaaaaaa
aaaaabagaaacaaakaaaeaaaaaaaaaammaaaaaaaaaaaaabbeaaacaaaaaaabaaaa
aaaaabcmaaaaaaaaaaaaabdmaaacaaabaaabaaaaaaaaaaoiaaaaaaaaaaaaabfb
aaacaaacaaaeaaaaaaaaaammaaaaaaaaaaaaabgeaaacaaaoaaabaaaaaaaaaaoi
aaaaaaaafpemgjghgiheengbhehcgjhidaaaklklaaadaaadaaaeaaaeaaabaaaa
aaaaaaaafpengbgjgofegfhifpfdfeaaaaabaaadaaabaaaeaaabaaaaaaaaaaaa
fpepgcgkgfgdhedcfhgphcgmgeaafpfhgphcgmgedcepgcgkgfgdheaafpfhgphc
gmgefdhagbgdgfedgbgngfhcgbfagphdaaklklklaaabaaadaaabaaadaaabaaaa
aaaaaaaafpfhgphcgmgefdhagbgdgfemgjghgihefagphddaaaghgmhdhegbhegf
fpgngbhehcgjhifpgnhghaaahfgogjhehjfpfdgdgbgmgfaahghdfpddfpdaaadc
codacodcdadddfddcodaaaklaaaaaaaaaaaaabomaadbaaahaaaaaaaaaaaaaaaa
aaaacmieaaaaaaabaaaaaaaeaaaaaaaiaaaaacjaaabaaaafaaaagaagaaaadaah
aacafaaiaaaadafaaaabhbfbaaaehcfcaaahhdfdaaaabacdaaaaaabnaaaaaabo
aaaababpaaaaaacaaaaaaacbaaaabaccaaaabachpaffeaafaaaabcaamcaaaaaa
aaaaeaajaaaabcaameaaaaaaaaaagaangabdbcaabcaaaaaaaaaagabjgabpbcaa
bcaaaaaaaaaadacfaaaaccaaaaaaaaaaafpigaaaaaaaagiiaaaaaaaaafpifaaa
aaaaagiiaaaaaaaaafpicaaaaaaaaoiiaaaaaaaaafpibaaaaaaaapmiaaaaaaaa
miapaaaaaabliiaakbagafaamiapaaaaaamgiiaaklagaeaamiapaaaaaalbdeje
klagadaamiapiadoaagmaadeklagacaamiahaaaaaaleblaacbanabaamiahaaad
aamamgmaalamaaanmiahaaadaalelbleclalaaadmiahaaaeaalogfaaobacafaa
miahaaahaamamgleclamabaamiapaaaaaabliiaakbagajaamiapaaaaaamgiiaa
klagaiaamiahaaahaalelbleclalabahmiahaaaeabgflomaolacafaemiahaaad
aamagmleclakaaadmiahaaadabmablmakladaoagmiahaaaeaamablaaobaeafaa
miahaaahaamagmleclakabahmiapaaaaaalbdejeklagahaamiapaaaaaagmejhk
klagagaamiahaaagabmablmaklahaoagmiabiaabaaloloaapaagafaamiaciaab
aaloloaapaaeagaamiaeiaabaaloloaapaagacaamiabiaacaaloloaapaadafaa
miaciaacaaloloaapaaeadaamiaeiaacaaloloaapaadacaamiadiaaaaalalabk
ilabbdbdmiahaaabaalbleaakbaabcaamiahaaabaamgmaleklaabbabmiahaaaa
aagmleleklaabaabmiahiaadaablmaleklaaapaaaaaaaaaaaaaaaaaaaaaaaaaa
"
}

SubProgram "ps3 " {
Keywords { "POINT_COOKIE" }
Matrix 256 [glstate_matrix_mvp]
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 467 [_WorldSpaceCameraPos]
Vector 466 [_WorldSpaceLightPos0]
Matrix 260 [_Object2World]
Matrix 264 [_World2Object]
Vector 465 [unity_Scale]
Matrix 268 [_LightMatrix0]
Vector 464 [_MainTex_ST]
"sce_vp_rsx // 32 instructions using 5 registers
[Configuration]
8
0000002041050500
[Microcode]
512
00009c6c005d200d8186c0836041fffc00011c6c00400e0c0106c0836041dffc
00019c6c005d300c0186c0836041dffc401f9c6c011d0808010400d740619f9c
401f9c6c01d0300d8106c0c360403f80401f9c6c01d0200d8106c0c360405f80
401f9c6c01d0100d8106c0c360409f80401f9c6c01d0000d8106c0c360411f80
00001c6c01d0700d8106c0c360403ffc00001c6c01d0600d8106c0c360405ffc
00001c6c01d0500d8106c0c360409ffc00001c6c01d0400d8106c0c360411ffc
00021c6c01d0a00d8286c0c360405ffc00021c6c01d0900d8286c0c360409ffc
00021c6c01d0800d8286c0c360411ffc00009c6c0190a00c0686c0c360405ffc
00009c6c0190900c0686c0c360409ffc00009c6c0190800c0686c0c360411ffc
00019c6c00800243011842436041dffc00011c6c010002308121826301a1dffc
401f9c6c01d0e00d8086c0c360405fa8401f9c6c01d0d00d8086c0c360409fa8
401f9c6c01d0c00d8086c0c360411fa800001c6c011d100c08bfc0e30041dffc
00009c6c011d100c02bfc0e30041dffc401f9c6c0140020c0106004360405fa0
401f9c6c01400e0c0086008360411fa000011c6c00800e0c04bfc0836041dffc
401f9c6c0140020c0106014360405fa4401f9c6c01400e0c0106014360411fa4
401f9c6c0140000c0086024360409fa0401f9c6c0140000c0486014360409fa5
"
}

SubProgram "d3d11 " {
Keywords { "POINT_COOKIE" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 160 // 160 used size, 8 vars
Matrix 48 [_LightMatrix0] 4
Vector 144 [_MainTex_ST] 4
ConstBuffer "UnityPerCamera" 128 // 76 used size, 8 vars
Vector 64 [_WorldSpaceCameraPos] 3
ConstBuffer "UnityLighting" 400 // 16 used size, 16 vars
Vector 0 [_WorldSpaceLightPos0] 4
ConstBuffer "UnityPerDraw" 336 // 336 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
Matrix 192 [_Object2World] 4
Matrix 256 [_World2Object] 4
Vector 320 [unity_Scale] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityLighting" 2
BindCB "UnityPerDraw" 3
// 33 instructions, 2 temp regs, 0 temp arrays:
// ALU 14 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0
eefiecedlgmpoopbcgheknafngcjigmdnlnlonpkabaaaaaapiagaaaaadaaaaaa
cmaaaaaapeaaaaaajeabaaaaejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaa
abaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaljaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfc
enebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheojiaaaaaaafaaaaaa
aiaaaaaaiaaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaabaaaaaaadamaaaaimaaaaaaabaaaaaaaaaaaaaa
adaaaaaaacaaaaaaahaiaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaa
ahaiaaaaimaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaahaiaaaafdfgfpfa
epfdejfeejepeoaafeeffiedepepfceeaaklklklfdeieefcfmafaaaaeaaaabaa
fhabaaaafjaaaaaeegiocaaaaaaaaaaaakaaaaaafjaaaaaeegiocaaaabaaaaaa
afaaaaaafjaaaaaeegiocaaaacaaaaaaabaaaaaafjaaaaaeegiocaaaadaaaaaa
bfaaaaaafpaaaaadpcbabaaaaaaaaaaafpaaaaadpcbabaaaabaaaaaafpaaaaad
hcbabaaaacaaaaaafpaaaaaddcbabaaaadaaaaaaghaaaaaepccabaaaaaaaaaaa
abaaaaaagfaaaaaddccabaaaabaaaaaagfaaaaadhccabaaaacaaaaaagfaaaaad
hccabaaaadaaaaaagfaaaaadhccabaaaaeaaaaaagiaaaaacacaaaaaadiaaaaai
pcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaadaaaaaaabaaaaaadcaaaaak
pcaabaaaaaaaaaaaegiocaaaadaaaaaaaaaaaaaaagbabaaaaaaaaaaaegaobaaa
aaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaacaaaaaakgbkbaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaakpccabaaaaaaaaaaaegiocaaaadaaaaaa
adaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaaldccabaaaabaaaaaa
egbabaaaadaaaaaaegiacaaaaaaaaaaaajaaaaaaogikcaaaaaaaaaaaajaaaaaa
diaaaaahhcaabaaaaaaaaaaajgbebaaaabaaaaaacgbjbaaaacaaaaaadcaaaaak
hcaabaaaaaaaaaaajgbebaaaacaaaaaacgbjbaaaabaaaaaaegacbaiaebaaaaaa
aaaaaaaadiaaaaahhcaabaaaaaaaaaaaegacbaaaaaaaaaaapgbpbaaaabaaaaaa
diaaaaajhcaabaaaabaaaaaafgifcaaaacaaaaaaaaaaaaaaegiccaaaadaaaaaa
bbaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaa
acaaaaaaaaaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaa
adaaaaaabcaaaaaakgikcaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaadcaaaaal
hcaabaaaabaaaaaaegiccaaaadaaaaaabdaaaaaapgipcaaaacaaaaaaaaaaaaaa
egacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegacbaaaabaaaaaapgipcaaa
adaaaaaabeaaaaaaegbcbaiaebaaaaaaaaaaaaaabaaaaaahcccabaaaacaaaaaa
egacbaaaaaaaaaaaegacbaaaabaaaaaabaaaaaahbccabaaaacaaaaaaegbcbaaa
abaaaaaaegacbaaaabaaaaaabaaaaaaheccabaaaacaaaaaaegbcbaaaacaaaaaa
egacbaaaabaaaaaadiaaaaajhcaabaaaabaaaaaafgifcaaaabaaaaaaaeaaaaaa
egiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaa
baaaaaaaagiacaaaabaaaaaaaeaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaa
abaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaaabaaaaaaaeaaaaaaegacbaaa
abaaaaaaaaaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaaegiccaaaadaaaaaa
bdaaaaaadcaaaaalhcaabaaaabaaaaaaegacbaaaabaaaaaapgipcaaaadaaaaaa
beaaaaaaegbcbaiaebaaaaaaaaaaaaaabaaaaaahcccabaaaadaaaaaaegacbaaa
aaaaaaaaegacbaaaabaaaaaabaaaaaahbccabaaaadaaaaaaegbcbaaaabaaaaaa
egacbaaaabaaaaaabaaaaaaheccabaaaadaaaaaaegbcbaaaacaaaaaaegacbaaa
abaaaaaadiaaaaaipcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaadaaaaaa
anaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaamaaaaaaagbabaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaa
aoaaaaaakgbkbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaa
egiocaaaadaaaaaaapaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadiaaaaai
hcaabaaaabaaaaaafgafbaaaaaaaaaaaegiccaaaaaaaaaaaaeaaaaaadcaaaaak
hcaabaaaabaaaaaaegiccaaaaaaaaaaaadaaaaaaagaabaaaaaaaaaaaegacbaaa
abaaaaaadcaaaaakhcaabaaaaaaaaaaaegiccaaaaaaaaaaaafaaaaaakgakbaaa
aaaaaaaaegacbaaaabaaaaaadcaaaaakhccabaaaaeaaaaaaegiccaaaaaaaaaaa
agaaaaaapgapbaaaaaaaaaaaegacbaaaaaaaaaaadoaaaaab"
}

SubProgram "gles " {
Keywords { "POINT_COOKIE" }
"!!GLES
#define SHADER_API_GLES 1
#define tex2D texture2D


#ifdef VERTEX
#define gl_ModelViewProjectionMatrix glstate_matrix_mvp
uniform mat4 glstate_matrix_mvp;

varying highp vec3 xlv_TEXCOORD3;
varying mediump vec3 xlv_TEXCOORD2;
varying mediump vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp vec4 _MainTex_ST;
uniform highp mat4 _LightMatrix0;
uniform highp vec4 unity_Scale;
uniform highp mat4 _World2Object;
uniform highp mat4 _Object2World;

uniform highp vec4 _WorldSpaceLightPos0;
uniform highp vec3 _WorldSpaceCameraPos;
attribute vec4 _glesTANGENT;
attribute vec4 _glesMultiTexCoord0;
attribute vec3 _glesNormal;
attribute vec4 _glesVertex;
void main ()
{
  vec4 tmpvar_1;
  tmpvar_1.xyz = normalize(_glesTANGENT.xyz);
  tmpvar_1.w = _glesTANGENT.w;
  vec3 tmpvar_2;
  tmpvar_2 = normalize(_glesNormal);
  mediump vec3 tmpvar_3;
  mediump vec3 tmpvar_4;
  highp vec3 tmpvar_5;
  highp vec3 tmpvar_6;
  tmpvar_5 = tmpvar_1.xyz;
  tmpvar_6 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_7;
  tmpvar_7[0].x = tmpvar_5.x;
  tmpvar_7[0].y = tmpvar_6.x;
  tmpvar_7[0].z = tmpvar_2.x;
  tmpvar_7[1].x = tmpvar_5.y;
  tmpvar_7[1].y = tmpvar_6.y;
  tmpvar_7[1].z = tmpvar_2.y;
  tmpvar_7[2].x = tmpvar_5.z;
  tmpvar_7[2].y = tmpvar_6.z;
  tmpvar_7[2].z = tmpvar_2.z;
  highp vec3 tmpvar_8;
  tmpvar_8 = (tmpvar_7 * (((_World2Object * _WorldSpaceLightPos0).xyz * unity_Scale.w) - _glesVertex.xyz));
  tmpvar_3 = tmpvar_8;
  highp vec4 tmpvar_9;
  tmpvar_9.w = 1.0;
  tmpvar_9.xyz = _WorldSpaceCameraPos;
  highp vec3 tmpvar_10;
  tmpvar_10 = (tmpvar_7 * (((_World2Object * tmpvar_9).xyz * unity_Scale.w) - _glesVertex.xyz));
  tmpvar_4 = tmpvar_10;
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = tmpvar_3;
  xlv_TEXCOORD2 = tmpvar_4;
  xlv_TEXCOORD3 = (_LightMatrix0 * (_Object2World * _glesVertex)).xyz;
}



#endif
#ifdef FRAGMENT

varying highp vec3 xlv_TEXCOORD3;
varying mediump vec3 xlv_TEXCOORD2;
varying mediump vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform mediump float _Gloss;
uniform mediump float _Shininess;
uniform lowp vec4 _Color;
uniform sampler2D _SpecMap;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform sampler2D _LightTextureB0;
uniform samplerCube _LightTexture0;
uniform lowp vec4 _LightColor0;
void main ()
{
  lowp vec4 c_1;
  lowp vec3 lightDir_2;
  mediump vec3 tmpvar_3;
  mediump vec3 tmpvar_4;
  lowp vec4 tmpvar_5;
  tmpvar_5 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_6;
  tmpvar_6 = (tmpvar_5.xyz * _Color.xyz);
  tmpvar_3 = tmpvar_6;
  lowp vec4 tmpvar_7;
  tmpvar_7 = texture2D (_SpecMap, xlv_TEXCOORD0);
  mediump vec3 tmpvar_8;
  tmpvar_8 = ((tmpvar_5.w * tmpvar_7.xyz) * _Gloss);
  lowp vec3 tmpvar_9;
  tmpvar_9 = ((texture2D (_BumpMap, xlv_TEXCOORD0).xyz * 2.0) - 1.0);
  tmpvar_4 = tmpvar_9;
  mediump vec3 tmpvar_10;
  tmpvar_10 = normalize(xlv_TEXCOORD1);
  lightDir_2 = tmpvar_10;
  highp float tmpvar_11;
  tmpvar_11 = dot (xlv_TEXCOORD3, xlv_TEXCOORD3);
  lowp vec4 tmpvar_12;
  tmpvar_12 = texture2D (_LightTextureB0, vec2(tmpvar_11));
  lowp vec4 tmpvar_13;
  tmpvar_13 = textureCube (_LightTexture0, xlv_TEXCOORD3);
  mediump vec3 lightDir_14;
  lightDir_14 = lightDir_2;
  mediump float atten_15;
  atten_15 = (tmpvar_12.w * tmpvar_13.w);
  mediump vec4 c_16;
  mediump vec3 specCol_17;
  highp float nh_18;
  mediump float tmpvar_19;
  tmpvar_19 = max (0.0, dot (tmpvar_4, normalize((lightDir_14 + normalize(xlv_TEXCOORD2)))));
  nh_18 = tmpvar_19;
  mediump float arg1_20;
  arg1_20 = (32.0 * _Shininess);
  highp vec3 tmpvar_21;
  tmpvar_21 = (pow (nh_18, arg1_20) * tmpvar_8);
  specCol_17 = tmpvar_21;
  c_16.xyz = ((((tmpvar_3 * _LightColor0.xyz) * max (0.0, dot (tmpvar_4, lightDir_14))) + (_LightColor0.xyz * specCol_17)) * (atten_15 * 2.0));
  c_16.w = 0.0;
  c_1.xyz = c_16.xyz;
  c_1.w = 0.0;
  gl_FragData[0] = c_1;
}



#endif"
}

SubProgram "glesdesktop " {
Keywords { "POINT_COOKIE" }
"!!GLES
#define SHADER_API_GLES 1
#define tex2D texture2D


#ifdef VERTEX
#define gl_ModelViewProjectionMatrix glstate_matrix_mvp
uniform mat4 glstate_matrix_mvp;

varying highp vec3 xlv_TEXCOORD3;
varying mediump vec3 xlv_TEXCOORD2;
varying mediump vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp vec4 _MainTex_ST;
uniform highp mat4 _LightMatrix0;
uniform highp vec4 unity_Scale;
uniform highp mat4 _World2Object;
uniform highp mat4 _Object2World;

uniform highp vec4 _WorldSpaceLightPos0;
uniform highp vec3 _WorldSpaceCameraPos;
attribute vec4 _glesTANGENT;
attribute vec4 _glesMultiTexCoord0;
attribute vec3 _glesNormal;
attribute vec4 _glesVertex;
void main ()
{
  vec4 tmpvar_1;
  tmpvar_1.xyz = normalize(_glesTANGENT.xyz);
  tmpvar_1.w = _glesTANGENT.w;
  vec3 tmpvar_2;
  tmpvar_2 = normalize(_glesNormal);
  mediump vec3 tmpvar_3;
  mediump vec3 tmpvar_4;
  highp vec3 tmpvar_5;
  highp vec3 tmpvar_6;
  tmpvar_5 = tmpvar_1.xyz;
  tmpvar_6 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_7;
  tmpvar_7[0].x = tmpvar_5.x;
  tmpvar_7[0].y = tmpvar_6.x;
  tmpvar_7[0].z = tmpvar_2.x;
  tmpvar_7[1].x = tmpvar_5.y;
  tmpvar_7[1].y = tmpvar_6.y;
  tmpvar_7[1].z = tmpvar_2.y;
  tmpvar_7[2].x = tmpvar_5.z;
  tmpvar_7[2].y = tmpvar_6.z;
  tmpvar_7[2].z = tmpvar_2.z;
  highp vec3 tmpvar_8;
  tmpvar_8 = (tmpvar_7 * (((_World2Object * _WorldSpaceLightPos0).xyz * unity_Scale.w) - _glesVertex.xyz));
  tmpvar_3 = tmpvar_8;
  highp vec4 tmpvar_9;
  tmpvar_9.w = 1.0;
  tmpvar_9.xyz = _WorldSpaceCameraPos;
  highp vec3 tmpvar_10;
  tmpvar_10 = (tmpvar_7 * (((_World2Object * tmpvar_9).xyz * unity_Scale.w) - _glesVertex.xyz));
  tmpvar_4 = tmpvar_10;
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = tmpvar_3;
  xlv_TEXCOORD2 = tmpvar_4;
  xlv_TEXCOORD3 = (_LightMatrix0 * (_Object2World * _glesVertex)).xyz;
}



#endif
#ifdef FRAGMENT

varying highp vec3 xlv_TEXCOORD3;
varying mediump vec3 xlv_TEXCOORD2;
varying mediump vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform mediump float _Gloss;
uniform mediump float _Shininess;
uniform lowp vec4 _Color;
uniform sampler2D _SpecMap;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform sampler2D _LightTextureB0;
uniform samplerCube _LightTexture0;
uniform lowp vec4 _LightColor0;
void main ()
{
  lowp vec4 c_1;
  lowp vec3 lightDir_2;
  mediump vec3 tmpvar_3;
  mediump vec3 tmpvar_4;
  lowp vec4 tmpvar_5;
  tmpvar_5 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_6;
  tmpvar_6 = (tmpvar_5.xyz * _Color.xyz);
  tmpvar_3 = tmpvar_6;
  lowp vec4 tmpvar_7;
  tmpvar_7 = texture2D (_SpecMap, xlv_TEXCOORD0);
  mediump vec3 tmpvar_8;
  tmpvar_8 = ((tmpvar_5.w * tmpvar_7.xyz) * _Gloss);
  lowp vec3 normal_9;
  normal_9.xy = ((texture2D (_BumpMap, xlv_TEXCOORD0).wy * 2.0) - 1.0);
  normal_9.z = sqrt(((1.0 - (normal_9.x * normal_9.x)) - (normal_9.y * normal_9.y)));
  tmpvar_4 = normal_9;
  mediump vec3 tmpvar_10;
  tmpvar_10 = normalize(xlv_TEXCOORD1);
  lightDir_2 = tmpvar_10;
  highp float tmpvar_11;
  tmpvar_11 = dot (xlv_TEXCOORD3, xlv_TEXCOORD3);
  lowp vec4 tmpvar_12;
  tmpvar_12 = texture2D (_LightTextureB0, vec2(tmpvar_11));
  lowp vec4 tmpvar_13;
  tmpvar_13 = textureCube (_LightTexture0, xlv_TEXCOORD3);
  mediump vec3 lightDir_14;
  lightDir_14 = lightDir_2;
  mediump float atten_15;
  atten_15 = (tmpvar_12.w * tmpvar_13.w);
  mediump vec4 c_16;
  mediump vec3 specCol_17;
  highp float nh_18;
  mediump float tmpvar_19;
  tmpvar_19 = max (0.0, dot (tmpvar_4, normalize((lightDir_14 + normalize(xlv_TEXCOORD2)))));
  nh_18 = tmpvar_19;
  mediump float arg1_20;
  arg1_20 = (32.0 * _Shininess);
  highp vec3 tmpvar_21;
  tmpvar_21 = (pow (nh_18, arg1_20) * tmpvar_8);
  specCol_17 = tmpvar_21;
  c_16.xyz = ((((tmpvar_3 * _LightColor0.xyz) * max (0.0, dot (tmpvar_4, lightDir_14))) + (_LightColor0.xyz * specCol_17)) * (atten_15 * 2.0));
  c_16.w = 0.0;
  c_1.xyz = c_16.xyz;
  c_1.w = 0.0;
  gl_FragData[0] = c_1;
}



#endif"
}

SubProgram "flash " {
Keywords { "POINT_COOKIE" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Matrix 0 [glstate_matrix_mvp]
Vector 16 [_WorldSpaceCameraPos]
Vector 17 [_WorldSpaceLightPos0]
Matrix 4 [_Object2World]
Matrix 8 [_World2Object]
Vector 18 [unity_Scale]
Matrix 12 [_LightMatrix0]
Vector 19 [_MainTex_ST]
"agal_vs
c20 1.0 0.0 0.0 0.0
[bc]
aaaaaaaaaaaaaiacbeaaaaaaabaaaaaaaaaaaaaaaaaaaaaa mov r0.w, c20.x
aaaaaaaaaaaaahacbaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, c16
bdaaaaaaabaaaeacaaaaaaoeacaaaaaaakaaaaoeabaaaaaa dp4 r1.z, r0, c10
bdaaaaaaabaaacacaaaaaaoeacaaaaaaajaaaaoeabaaaaaa dp4 r1.y, r0, c9
bdaaaaaaabaaabacaaaaaaoeacaaaaaaaiaaaaoeabaaaaaa dp4 r1.x, r0, c8
adaaaaaaacaaahacabaaaakeacaaaaaabcaaaappabaaaaaa mul r2.xyz, r1.xyzz, c18.w
acaaaaaaadaaahacacaaaakeacaaaaaaaaaaaaoeaaaaaaaa sub r3.xyz, r2.xyzz, a0
aaaaaaaaaaaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, a5
adaaaaaaabaaahacabaaaancaaaaaaaaaaaaaaajacaaaaaa mul r1.xyz, a1.zxyw, r0.yzxx
aaaaaaaaaaaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, a5
adaaaaaaaeaaahacabaaaamjaaaaaaaaaaaaaafcacaaaaaa mul r4.xyz, a1.yzxw, r0.zxyy
acaaaaaaabaaahacaeaaaakeacaaaaaaabaaaakeacaaaaaa sub r1.xyz, r4.xyzz, r1.xyzz
adaaaaaaacaaahacabaaaakeacaaaaaaafaaaappaaaaaaaa mul r2.xyz, r1.xyzz, a5.w
aaaaaaaaaaaaapacakaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0, c10
bdaaaaaaaeaaaeacbbaaaaoeabaaaaaaaaaaaaoeacaaaaaa dp4 r4.z, c17, r0
aaaaaaaaaaaaapacajaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0, c9
bdaaaaaaaeaaacacbbaaaaoeabaaaaaaaaaaaaoeacaaaaaa dp4 r4.y, c17, r0
aaaaaaaaabaaapacaiaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r1, c8
bdaaaaaaaeaaabacbbaaaaoeabaaaaaaabaaaaoeacaaaaaa dp4 r4.x, c17, r1
adaaaaaaabaaahacaeaaaakeacaaaaaabcaaaappabaaaaaa mul r1.xyz, r4.xyzz, c18.w
acaaaaaaaaaaahacabaaaakeacaaaaaaaaaaaaoeaaaaaaaa sub r0.xyz, r1.xyzz, a0
bcaaaaaaabaaacaeaaaaaakeacaaaaaaacaaaakeacaaaaaa dp3 v1.y, r0.xyzz, r2.xyzz
bcaaaaaaabaaaeaeabaaaaoeaaaaaaaaaaaaaakeacaaaaaa dp3 v1.z, a1, r0.xyzz
bcaaaaaaabaaabaeaaaaaakeacaaaaaaafaaaaoeaaaaaaaa dp3 v1.x, r0.xyzz, a5
bdaaaaaaaaaaaiacaaaaaaoeaaaaaaaaahaaaaoeabaaaaaa dp4 r0.w, a0, c7
bdaaaaaaaaaaaeacaaaaaaoeaaaaaaaaagaaaaoeabaaaaaa dp4 r0.z, a0, c6
bdaaaaaaaaaaabacaaaaaaoeaaaaaaaaaeaaaaoeabaaaaaa dp4 r0.x, a0, c4
bdaaaaaaaaaaacacaaaaaaoeaaaaaaaaafaaaaoeabaaaaaa dp4 r0.y, a0, c5
bcaaaaaaacaaacaeacaaaakeacaaaaaaadaaaakeacaaaaaa dp3 v2.y, r2.xyzz, r3.xyzz
bcaaaaaaacaaaeaeabaaaaoeaaaaaaaaadaaaakeacaaaaaa dp3 v2.z, a1, r3.xyzz
bcaaaaaaacaaabaeafaaaaoeaaaaaaaaadaaaakeacaaaaaa dp3 v2.x, a5, r3.xyzz
bdaaaaaaadaaaeaeaaaaaaoeacaaaaaaaoaaaaoeabaaaaaa dp4 v3.z, r0, c14
bdaaaaaaadaaacaeaaaaaaoeacaaaaaaanaaaaoeabaaaaaa dp4 v3.y, r0, c13
bdaaaaaaadaaabaeaaaaaaoeacaaaaaaamaaaaoeabaaaaaa dp4 v3.x, r0, c12
adaaaaaaaaaaadacadaaaaoeaaaaaaaabdaaaaoeabaaaaaa mul r0.xy, a3, c19
abaaaaaaaaaaadaeaaaaaafeacaaaaaabdaaaaooabaaaaaa add v0.xy, r0.xyyy, c19.zwzw
bdaaaaaaaaaaaiadaaaaaaoeaaaaaaaaadaaaaoeabaaaaaa dp4 o0.w, a0, c3
bdaaaaaaaaaaaeadaaaaaaoeaaaaaaaaacaaaaoeabaaaaaa dp4 o0.z, a0, c2
bdaaaaaaaaaaacadaaaaaaoeaaaaaaaaabaaaaoeabaaaaaa dp4 o0.y, a0, c1
bdaaaaaaaaaaabadaaaaaaoeaaaaaaaaaaaaaaoeabaaaaaa dp4 o0.x, a0, c0
aaaaaaaaaaaaamaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v0.zw, c0
aaaaaaaaabaaaiaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v1.w, c0
aaaaaaaaacaaaiaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v2.w, c0
aaaaaaaaadaaaiaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v3.w, c0
"
}

SubProgram "d3d11_9x " {
Keywords { "POINT_COOKIE" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 160 // 160 used size, 8 vars
Matrix 48 [_LightMatrix0] 4
Vector 144 [_MainTex_ST] 4
ConstBuffer "UnityPerCamera" 128 // 76 used size, 8 vars
Vector 64 [_WorldSpaceCameraPos] 3
ConstBuffer "UnityLighting" 400 // 16 used size, 16 vars
Vector 0 [_WorldSpaceLightPos0] 4
ConstBuffer "UnityPerDraw" 336 // 336 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
Matrix 192 [_Object2World] 4
Matrix 256 [_World2Object] 4
Vector 320 [unity_Scale] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityLighting" 2
BindCB "UnityPerDraw" 3
// 33 instructions, 2 temp regs, 0 temp arrays:
// ALU 14 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0_level_9_3
eefiecedijcbcpehncccipennobfnmfkecknhmjlabaaaaaadiakaaaaaeaaaaaa
daaaaaaagmadaaaanaaiaaaajiajaaaaebgpgodjdeadaaaadeadaaaaaaacpopp
meacaaaahaaaaaaaagaaceaaaaaagmaaaaaagmaaaaaaceaaabaagmaaaaaaadaa
aeaaabaaaaaaaaaaaaaaajaaabaaafaaaaaaaaaaabaaaeaaabaaagaaaaaaaaaa
acaaaaaaabaaahaaaaaaaaaaadaaaaaaaeaaaiaaaaaaaaaaadaaamaaajaaamaa
aaaaaaaaaaaaaaaaabacpoppbpaaaaacafaaaaiaaaaaapjabpaaaaacafaaabia
abaaapjabpaaaaacafaaaciaacaaapjabpaaaaacafaaadiaadaaapjaaeaaaaae
aaaaadoaadaaoejaafaaoekaafaaookaabaaaaacaaaaapiaahaaoekaafaaaaad
abaaahiaaaaaffiabbaaoekaaeaaaaaeabaaahiabaaaoekaaaaaaaiaabaaoeia
aeaaaaaeaaaaahiabcaaoekaaaaakkiaabaaoeiaaeaaaaaeaaaaahiabdaaoeka
aaaappiaaaaaoeiaaeaaaaaeaaaaahiaaaaaoeiabeaappkaaaaaoejbaiaaaaad
abaaaboaabaaoejaaaaaoeiaabaaaaacabaaahiaabaaoejaafaaaaadacaaahia
abaamjiaacaancjaaeaaaaaeabaaahiaacaamjjaabaanciaacaaoeibafaaaaad
abaaahiaabaaoeiaabaappjaaiaaaaadabaaacoaabaaoeiaaaaaoeiaaiaaaaad
abaaaeoaacaaoejaaaaaoeiaabaaaaacaaaaahiaagaaoekaafaaaaadacaaahia
aaaaffiabbaaoekaaeaaaaaeaaaaaliabaaakekaaaaaaaiaacaakeiaaeaaaaae
aaaaahiabcaaoekaaaaakkiaaaaapeiaacaaaaadaaaaahiaaaaaoeiabdaaoeka
aeaaaaaeaaaaahiaaaaaoeiabeaappkaaaaaoejbaiaaaaadacaaaboaabaaoeja
aaaaoeiaaiaaaaadacaaacoaabaaoeiaaaaaoeiaaiaaaaadacaaaeoaacaaoeja
aaaaoeiaafaaaaadaaaaapiaaaaaffjaanaaoekaaeaaaaaeaaaaapiaamaaoeka
aaaaaajaaaaaoeiaaeaaaaaeaaaaapiaaoaaoekaaaaakkjaaaaaoeiaaeaaaaae
aaaaapiaapaaoekaaaaappjaaaaaoeiaafaaaaadabaaahiaaaaaffiaacaaoeka
aeaaaaaeabaaahiaabaaoekaaaaaaaiaabaaoeiaaeaaaaaeaaaaahiaadaaoeka
aaaakkiaabaaoeiaaeaaaaaeadaaahoaaeaaoekaaaaappiaaaaaoeiaafaaaaad
aaaaapiaaaaaffjaajaaoekaaeaaaaaeaaaaapiaaiaaoekaaaaaaajaaaaaoeia
aeaaaaaeaaaaapiaakaaoekaaaaakkjaaaaaoeiaaeaaaaaeaaaaapiaalaaoeka
aaaappjaaaaaoeiaaeaaaaaeaaaaadmaaaaappiaaaaaoekaaaaaoeiaabaaaaac
aaaaammaaaaaoeiappppaaaafdeieefcfmafaaaaeaaaabaafhabaaaafjaaaaae
egiocaaaaaaaaaaaakaaaaaafjaaaaaeegiocaaaabaaaaaaafaaaaaafjaaaaae
egiocaaaacaaaaaaabaaaaaafjaaaaaeegiocaaaadaaaaaabfaaaaaafpaaaaad
pcbabaaaaaaaaaaafpaaaaadpcbabaaaabaaaaaafpaaaaadhcbabaaaacaaaaaa
fpaaaaaddcbabaaaadaaaaaaghaaaaaepccabaaaaaaaaaaaabaaaaaagfaaaaad
dccabaaaabaaaaaagfaaaaadhccabaaaacaaaaaagfaaaaadhccabaaaadaaaaaa
gfaaaaadhccabaaaaeaaaaaagiaaaaacacaaaaaadiaaaaaipcaabaaaaaaaaaaa
fgbfbaaaaaaaaaaaegiocaaaadaaaaaaabaaaaaadcaaaaakpcaabaaaaaaaaaaa
egiocaaaadaaaaaaaaaaaaaaagbabaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaak
pcaabaaaaaaaaaaaegiocaaaadaaaaaaacaaaaaakgbkbaaaaaaaaaaaegaobaaa
aaaaaaaadcaaaaakpccabaaaaaaaaaaaegiocaaaadaaaaaaadaaaaaapgbpbaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaaldccabaaaabaaaaaaegbabaaaadaaaaaa
egiacaaaaaaaaaaaajaaaaaaogikcaaaaaaaaaaaajaaaaaadiaaaaahhcaabaaa
aaaaaaaajgbebaaaabaaaaaacgbjbaaaacaaaaaadcaaaaakhcaabaaaaaaaaaaa
jgbebaaaacaaaaaacgbjbaaaabaaaaaaegacbaiaebaaaaaaaaaaaaaadiaaaaah
hcaabaaaaaaaaaaaegacbaaaaaaaaaaapgbpbaaaabaaaaaadiaaaaajhcaabaaa
abaaaaaafgifcaaaacaaaaaaaaaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaal
hcaabaaaabaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaaacaaaaaaaaaaaaaa
egacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabcaaaaaa
kgikcaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaa
egiccaaaadaaaaaabdaaaaaapgipcaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaa
dcaaaaalhcaabaaaabaaaaaaegacbaaaabaaaaaapgipcaaaadaaaaaabeaaaaaa
egbcbaiaebaaaaaaaaaaaaaabaaaaaahcccabaaaacaaaaaaegacbaaaaaaaaaaa
egacbaaaabaaaaaabaaaaaahbccabaaaacaaaaaaegbcbaaaabaaaaaaegacbaaa
abaaaaaabaaaaaaheccabaaaacaaaaaaegbcbaaaacaaaaaaegacbaaaabaaaaaa
diaaaaajhcaabaaaabaaaaaafgifcaaaabaaaaaaaeaaaaaaegiccaaaadaaaaaa
bbaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaa
abaaaaaaaeaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaa
adaaaaaabcaaaaaakgikcaaaabaaaaaaaeaaaaaaegacbaaaabaaaaaaaaaaaaai
hcaabaaaabaaaaaaegacbaaaabaaaaaaegiccaaaadaaaaaabdaaaaaadcaaaaal
hcaabaaaabaaaaaaegacbaaaabaaaaaapgipcaaaadaaaaaabeaaaaaaegbcbaia
ebaaaaaaaaaaaaaabaaaaaahcccabaaaadaaaaaaegacbaaaaaaaaaaaegacbaaa
abaaaaaabaaaaaahbccabaaaadaaaaaaegbcbaaaabaaaaaaegacbaaaabaaaaaa
baaaaaaheccabaaaadaaaaaaegbcbaaaacaaaaaaegacbaaaabaaaaaadiaaaaai
pcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaadaaaaaaanaaaaaadcaaaaak
pcaabaaaaaaaaaaaegiocaaaadaaaaaaamaaaaaaagbabaaaaaaaaaaaegaobaaa
aaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaaoaaaaaakgbkbaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaa
apaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadiaaaaaihcaabaaaabaaaaaa
fgafbaaaaaaaaaaaegiccaaaaaaaaaaaaeaaaaaadcaaaaakhcaabaaaabaaaaaa
egiccaaaaaaaaaaaadaaaaaaagaabaaaaaaaaaaaegacbaaaabaaaaaadcaaaaak
hcaabaaaaaaaaaaaegiccaaaaaaaaaaaafaaaaaakgakbaaaaaaaaaaaegacbaaa
abaaaaaadcaaaaakhccabaaaaeaaaaaaegiccaaaaaaaaaaaagaaaaaapgapbaaa
aaaaaaaaegacbaaaaaaaaaaadoaaaaabejfdeheomaaaaaaaagaaaaaaaiaaaaaa
jiaaaaaaaaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaa
acaaaaaaahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaa
laaaaaaaabaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaljaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofe
aaeoepfcenebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheojiaaaaaa
afaaaaaaaiaaaaaaiaaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaa
imaaaaaaaaaaaaaaaaaaaaaaadaaaaaaabaaaaaaadamaaaaimaaaaaaabaaaaaa
aaaaaaaaadaaaaaaacaaaaaaahaiaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaa
adaaaaaaahaiaaaaimaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaahaiaaaa
fdfgfpfaepfdejfeejepeoaafeeffiedepepfceeaaklklkl"
}

SubProgram "opengl " {
Keywords { "DIRECTIONAL_COOKIE" }
Bind "vertex" Vertex
Bind "tangent" ATTR14
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 17 [_WorldSpaceCameraPos]
Vector 18 [_WorldSpaceLightPos0]
Matrix 5 [_Object2World]
Matrix 9 [_World2Object]
Vector 19 [unity_Scale]
Matrix 13 [_LightMatrix0]
Vector 20 [_MainTex_ST]
"!!ARBvp1.0
# 31 ALU
PARAM c[21] = { { 1 },
		state.matrix.mvp,
		program.local[5..20] };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
MOV R1.xyz, c[17];
MOV R1.w, c[0].x;
MOV R0.xyz, vertex.attrib[14];
DP4 R2.z, R1, c[11];
DP4 R2.y, R1, c[10];
DP4 R2.x, R1, c[9];
MAD R2.xyz, R2, c[19].w, -vertex.position;
MUL R1.xyz, vertex.normal.zxyw, R0.yzxw;
MAD R1.xyz, vertex.normal.yzxw, R0.zxyw, -R1;
MOV R0, c[18];
MUL R1.xyz, R1, vertex.attrib[14].w;
DP4 R3.z, R0, c[11];
DP4 R3.y, R0, c[10];
DP4 R3.x, R0, c[9];
DP4 R0.w, vertex.position, c[8];
DP4 R0.z, vertex.position, c[7];
DP4 R0.x, vertex.position, c[5];
DP4 R0.y, vertex.position, c[6];
DP3 result.texcoord[1].y, R3, R1;
DP3 result.texcoord[2].y, R1, R2;
DP3 result.texcoord[1].z, vertex.normal, R3;
DP3 result.texcoord[1].x, R3, vertex.attrib[14];
DP3 result.texcoord[2].z, vertex.normal, R2;
DP3 result.texcoord[2].x, vertex.attrib[14], R2;
DP4 result.texcoord[3].y, R0, c[14];
DP4 result.texcoord[3].x, R0, c[13];
MAD result.texcoord[0].xy, vertex.texcoord[0], c[20], c[20].zwzw;
DP4 result.position.w, vertex.position, c[4];
DP4 result.position.z, vertex.position, c[3];
DP4 result.position.y, vertex.position, c[2];
DP4 result.position.x, vertex.position, c[1];
END
# 31 instructions, 4 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "DIRECTIONAL_COOKIE" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Matrix 0 [glstate_matrix_mvp]
Vector 16 [_WorldSpaceCameraPos]
Vector 17 [_WorldSpaceLightPos0]
Matrix 4 [_Object2World]
Matrix 8 [_World2Object]
Vector 18 [unity_Scale]
Matrix 12 [_LightMatrix0]
Vector 19 [_MainTex_ST]
"vs_2_0
; 34 ALU
def c20, 1.00000000, 0, 0, 0
dcl_position0 v0
dcl_tangent0 v1
dcl_normal0 v2
dcl_texcoord0 v3
mov r0.w, c20.x
mov r0.xyz, c16
dp4 r1.z, r0, c10
dp4 r1.y, r0, c9
dp4 r1.x, r0, c8
mad r3.xyz, r1, c18.w, -v0
mov r0.xyz, v1
mul r1.xyz, v2.zxyw, r0.yzxw
mov r0.xyz, v1
mad r1.xyz, v2.yzxw, r0.zxyw, -r1
mul r2.xyz, r1, v1.w
mov r0, c10
dp4 r4.z, c17, r0
mov r0, c9
dp4 r4.y, c17, r0
mov r1, c8
dp4 r4.x, c17, r1
dp4 r0.w, v0, c7
dp4 r0.z, v0, c6
dp4 r0.x, v0, c4
dp4 r0.y, v0, c5
dp3 oT1.y, r4, r2
dp3 oT2.y, r2, r3
dp3 oT1.z, v2, r4
dp3 oT1.x, r4, v1
dp3 oT2.z, v2, r3
dp3 oT2.x, v1, r3
dp4 oT3.y, r0, c13
dp4 oT3.x, r0, c12
mad oT0.xy, v3, c19, c19.zwzw
dp4 oPos.w, v0, c3
dp4 oPos.z, v0, c2
dp4 oPos.y, v0, c1
dp4 oPos.x, v0, c0
"
}

SubProgram "xbox360 " {
Keywords { "DIRECTIONAL_COOKIE" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Matrix 15 [_LightMatrix0] 4
Vector 19 [_MainTex_ST]
Matrix 6 [_Object2World] 4
Matrix 10 [_World2Object] 4
Vector 0 [_WorldSpaceCameraPos]
Vector 1 [_WorldSpaceLightPos0]
Matrix 2 [glstate_matrix_mvp] 4
Vector 14 [unity_Scale]
// Shader Timing Estimate, in Cycles/64 vertex vector:
// ALU: 40.00 (30 instructions), vertex: 32, texture: 0,
//   sequencer: 18,  8 GPRs, 24 threads,
// Performance (if enough threads): ~40 cycles per vector
// * Vertex cycle estimates are assuming 3 vfetch_minis for every vfetch_full,
//     with <= 32 bytes per vfetch_full group.

"vs_360
backbbabaaaaacbeaaaaaboaaaaaaaaaaaaaaaceaaaaaaaaaaaaabkmaaaaaaaa
aaaaaaaaaaaaabieaaaaaabmaaaaabhhpppoadaaaaaaaaaiaaaaaabmaaaaaaaa
aaaaabhaaaaaaalmaaacaaapaaaeaaaaaaaaaammaaaaaaaaaaaaaanmaaacaabd
aaabaaaaaaaaaaoiaaaaaaaaaaaaaapiaaacaaagaaaeaaaaaaaaaammaaaaaaaa
aaaaabagaaacaaakaaaeaaaaaaaaaammaaaaaaaaaaaaabbeaaacaaaaaaabaaaa
aaaaabcmaaaaaaaaaaaaabdmaaacaaabaaabaaaaaaaaaaoiaaaaaaaaaaaaabfb
aaacaaacaaaeaaaaaaaaaammaaaaaaaaaaaaabgeaaacaaaoaaabaaaaaaaaaaoi
aaaaaaaafpemgjghgiheengbhehcgjhidaaaklklaaadaaadaaaeaaaeaaabaaaa
aaaaaaaafpengbgjgofegfhifpfdfeaaaaabaaadaaabaaaeaaabaaaaaaaaaaaa
fpepgcgkgfgdhedcfhgphcgmgeaafpfhgphcgmgedcepgcgkgfgdheaafpfhgphc
gmgefdhagbgdgfedgbgngfhcgbfagphdaaklklklaaabaaadaaabaaadaaabaaaa
aaaaaaaafpfhgphcgmgefdhagbgdgfemgjghgihefagphddaaaghgmhdhegbhegf
fpgngbhehcgjhifpgnhghaaahfgogjhehjfpfdgdgbgmgfaahghdfpddfpdaaadc
codacodcdadddfddcodaaaklaaaaaaaaaaaaaboaaadbaaahaaaaaaaaaaaaaaaa
aaaaciieaaaaaaabaaaaaaaeaaaaaaaiaaaaacjaaabaaaafaaaagaagaaaadaah
aacafaaiaaaadafaaaabhbfbaaaehcfcaaahddfdaaaabaccaaaaaabmaaaaaabn
aaaababoaaaaaabpaaaaaacaaaaabacbaaaabacgpaffeaafaaaabcaamcaaaaaa
aaaaeaajaaaabcaameaaaaaaaaaagaangabdbcaabcaaaaaaaaaagabjgabpbcaa
bcaaaaaaaaaacacfaaaaccaaaaaaaaaaafpihaaaaaaaagiiaaaaaaaaafpifaaa
aaaaagiiaaaaaaaaafpicaaaaaaaaoiiaaaaaaaaafpibaaaaaaaapmiaaaaaaaa
miapaaaaaabliiaakbahafaamiapaaaaaamgiiaaklahaeaamiapaaaaaalbdeje
klahadaamiapiadoaagmaadeklahacaamiahaaaaaaleblaacbanabaamiahaaad
aamamgmaalamaaanmiahaaadaalelbleclalaaadmiahaaaeaalogfaaobacafaa
miahaaagaamamgleclamabaamiapaaaaaabliiaakbahajaamiapaaaaaamgiiaa
klahaiaamiahaaagaalelbleclalabagmiahaaaeabgflomaolacafaemiahaaad
aamagmleclakaaadmiahaaadabmablmakladaoahmiahaaaeaamablaaobaeafaa
miahaaagaamagmleclakabagmiapaaaaaalbdejeklahahaamiapaaaaaagmojkk
klahagaamiabiaabaaloloaapaagafaamiaciaabaaloloaapaaeagaamiaeiaab
aaloloaapaagacaamiabiaacaaloloaapaadafaamiaciaacaaloloaapaaeadaa
miaeiaacaaloloaapaadacaamiadiaaaaalalabkilabbdbdmiadaaabaalblaaa
kbaabcaamiadaaabaabllalaklaabbabmiadaaaaaagmlalaklaabaabmiadiaad
aamglalaklaaapaaaaaaaaaaaaaaaaaaaaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "DIRECTIONAL_COOKIE" }
Matrix 256 [glstate_matrix_mvp]
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 467 [_WorldSpaceCameraPos]
Vector 466 [_WorldSpaceLightPos0]
Matrix 260 [_Object2World]
Matrix 264 [_World2Object]
Vector 465 [unity_Scale]
Matrix 268 [_LightMatrix0]
Vector 464 [_MainTex_ST]
"sce_vp_rsx // 30 instructions using 5 registers
[Configuration]
8
0000001e41050500
[Microcode]
480
00009c6c005d200d8186c0836041fffc00011c6c00400e0c0106c0836041dffc
00019c6c005d300c0186c0836041dffc401f9c6c011d0808010400d740619f9c
401f9c6c01d0300d8106c0c360403f80401f9c6c01d0200d8106c0c360405f80
401f9c6c01d0100d8106c0c360409f80401f9c6c01d0000d8106c0c360411f80
00001c6c01d0700d8106c0c360403ffc00001c6c01d0600d8106c0c360405ffc
00001c6c01d0500d8106c0c360409ffc00001c6c01d0400d8106c0c360411ffc
00021c6c01d0a00d8286c0c360405ffc00021c6c01d0900d8286c0c360409ffc
00021c6c01d0800d8286c0c360411ffc00009c6c0190a00c0686c0c360405ffc
00009c6c0190900c0686c0c360409ffc00009c6c0190800c0686c0c360411ffc
00019c6c00800243011842436041dffc00011c6c010002308121826301a1dffc
401f9c6c01d0d00d8086c0c360409fa8401f9c6c01d0c00d8086c0c360411fa8
00001c6c011d100c02bfc0e30041dffc401f9c6c0140020c0106044360405fa0
401f9c6c01400e0c0886008360411fa000009c6c00800e0c04bfc0836041dffc
401f9c6c0140020c0106004360405fa4401f9c6c01400e0c0106004360411fa4
401f9c6c0140000c0886014360409fa0401f9c6c0140000c0286004360409fa5
"
}

SubProgram "d3d11 " {
Keywords { "DIRECTIONAL_COOKIE" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 160 // 160 used size, 8 vars
Matrix 48 [_LightMatrix0] 4
Vector 144 [_MainTex_ST] 4
ConstBuffer "UnityPerCamera" 128 // 76 used size, 8 vars
Vector 64 [_WorldSpaceCameraPos] 3
ConstBuffer "UnityLighting" 400 // 16 used size, 16 vars
Vector 0 [_WorldSpaceLightPos0] 4
ConstBuffer "UnityPerDraw" 336 // 336 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
Matrix 192 [_Object2World] 4
Matrix 256 [_World2Object] 4
Vector 320 [unity_Scale] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityLighting" 2
BindCB "UnityPerDraw" 3
// 32 instructions, 2 temp regs, 0 temp arrays:
// ALU 14 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0
eefiecedphhnegepknobncaagdhgmenkmhbajmibabaaaaaammagaaaaadaaaaaa
cmaaaaaapeaaaaaajeabaaaaejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaa
abaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaljaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfc
enebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheojiaaaaaaafaaaaaa
aiaaaaaaiaaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaabaaaaaaadamaaaaimaaaaaaadaaaaaaaaaaaaaa
adaaaaaaabaaaaaaamadaaaaimaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahaiaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaahaiaaaafdfgfpfa
epfdejfeejepeoaafeeffiedepepfceeaaklklklfdeieefcdaafaaaaeaaaabaa
emabaaaafjaaaaaeegiocaaaaaaaaaaaakaaaaaafjaaaaaeegiocaaaabaaaaaa
afaaaaaafjaaaaaeegiocaaaacaaaaaaabaaaaaafjaaaaaeegiocaaaadaaaaaa
bfaaaaaafpaaaaadpcbabaaaaaaaaaaafpaaaaadpcbabaaaabaaaaaafpaaaaad
hcbabaaaacaaaaaafpaaaaaddcbabaaaadaaaaaaghaaaaaepccabaaaaaaaaaaa
abaaaaaagfaaaaaddccabaaaabaaaaaagfaaaaadmccabaaaabaaaaaagfaaaaad
hccabaaaacaaaaaagfaaaaadhccabaaaadaaaaaagiaaaaacacaaaaaadiaaaaai
pcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaadaaaaaaabaaaaaadcaaaaak
pcaabaaaaaaaaaaaegiocaaaadaaaaaaaaaaaaaaagbabaaaaaaaaaaaegaobaaa
aaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaacaaaaaakgbkbaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaakpccabaaaaaaaaaaaegiocaaaadaaaaaa
adaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadiaaaaaipcaabaaaaaaaaaaa
fgbfbaaaaaaaaaaaegiocaaaadaaaaaaanaaaaaadcaaaaakpcaabaaaaaaaaaaa
egiocaaaadaaaaaaamaaaaaaagbabaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaak
pcaabaaaaaaaaaaaegiocaaaadaaaaaaaoaaaaaakgbkbaaaaaaaaaaaegaobaaa
aaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaapaaaaaapgbpbaaa
aaaaaaaaegaobaaaaaaaaaaadiaaaaaidcaabaaaabaaaaaafgafbaaaaaaaaaaa
egiacaaaaaaaaaaaaeaaaaaadcaaaaakdcaabaaaaaaaaaaaegiacaaaaaaaaaaa
adaaaaaaagaabaaaaaaaaaaaegaabaaaabaaaaaadcaaaaakdcaabaaaaaaaaaaa
egiacaaaaaaaaaaaafaaaaaakgakbaaaaaaaaaaaegaabaaaaaaaaaaadcaaaaak
mccabaaaabaaaaaaagiecaaaaaaaaaaaagaaaaaapgapbaaaaaaaaaaaagaebaaa
aaaaaaaadcaaaaaldccabaaaabaaaaaaegbabaaaadaaaaaaegiacaaaaaaaaaaa
ajaaaaaaogikcaaaaaaaaaaaajaaaaaadiaaaaahhcaabaaaaaaaaaaajgbebaaa
abaaaaaacgbjbaaaacaaaaaadcaaaaakhcaabaaaaaaaaaaajgbebaaaacaaaaaa
cgbjbaaaabaaaaaaegacbaiaebaaaaaaaaaaaaaadiaaaaahhcaabaaaaaaaaaaa
egacbaaaaaaaaaaapgbpbaaaabaaaaaadiaaaaajhcaabaaaabaaaaaafgifcaaa
acaaaaaaaaaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaaabaaaaaa
egiccaaaadaaaaaabaaaaaaaagiacaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaa
dcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaaacaaaaaa
aaaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaa
bdaaaaaapgipcaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaabaaaaaahcccabaaa
acaaaaaaegacbaaaaaaaaaaaegacbaaaabaaaaaabaaaaaahbccabaaaacaaaaaa
egbcbaaaabaaaaaaegacbaaaabaaaaaabaaaaaaheccabaaaacaaaaaaegbcbaaa
acaaaaaaegacbaaaabaaaaaadiaaaaajhcaabaaaabaaaaaafgifcaaaabaaaaaa
aeaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaa
adaaaaaabaaaaaaaagiacaaaabaaaaaaaeaaaaaaegacbaaaabaaaaaadcaaaaal
hcaabaaaabaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaaabaaaaaaaeaaaaaa
egacbaaaabaaaaaaaaaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaaegiccaaa
adaaaaaabdaaaaaadcaaaaalhcaabaaaabaaaaaaegacbaaaabaaaaaapgipcaaa
adaaaaaabeaaaaaaegbcbaiaebaaaaaaaaaaaaaabaaaaaahcccabaaaadaaaaaa
egacbaaaaaaaaaaaegacbaaaabaaaaaabaaaaaahbccabaaaadaaaaaaegbcbaaa
abaaaaaaegacbaaaabaaaaaabaaaaaaheccabaaaadaaaaaaegbcbaaaacaaaaaa
egacbaaaabaaaaaadoaaaaab"
}

SubProgram "gles " {
Keywords { "DIRECTIONAL_COOKIE" }
"!!GLES
#define SHADER_API_GLES 1
#define tex2D texture2D


#ifdef VERTEX
#define gl_ModelViewProjectionMatrix glstate_matrix_mvp
uniform mat4 glstate_matrix_mvp;

varying highp vec2 xlv_TEXCOORD3;
varying mediump vec3 xlv_TEXCOORD2;
varying mediump vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp vec4 _MainTex_ST;
uniform highp mat4 _LightMatrix0;
uniform highp vec4 unity_Scale;
uniform highp mat4 _World2Object;
uniform highp mat4 _Object2World;

uniform lowp vec4 _WorldSpaceLightPos0;
uniform highp vec3 _WorldSpaceCameraPos;
attribute vec4 _glesTANGENT;
attribute vec4 _glesMultiTexCoord0;
attribute vec3 _glesNormal;
attribute vec4 _glesVertex;
void main ()
{
  vec4 tmpvar_1;
  tmpvar_1.xyz = normalize(_glesTANGENT.xyz);
  tmpvar_1.w = _glesTANGENT.w;
  vec3 tmpvar_2;
  tmpvar_2 = normalize(_glesNormal);
  mediump vec3 tmpvar_3;
  mediump vec3 tmpvar_4;
  highp vec3 tmpvar_5;
  highp vec3 tmpvar_6;
  tmpvar_5 = tmpvar_1.xyz;
  tmpvar_6 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_7;
  tmpvar_7[0].x = tmpvar_5.x;
  tmpvar_7[0].y = tmpvar_6.x;
  tmpvar_7[0].z = tmpvar_2.x;
  tmpvar_7[1].x = tmpvar_5.y;
  tmpvar_7[1].y = tmpvar_6.y;
  tmpvar_7[1].z = tmpvar_2.y;
  tmpvar_7[2].x = tmpvar_5.z;
  tmpvar_7[2].y = tmpvar_6.z;
  tmpvar_7[2].z = tmpvar_2.z;
  highp vec3 tmpvar_8;
  tmpvar_8 = (tmpvar_7 * (_World2Object * _WorldSpaceLightPos0).xyz);
  tmpvar_3 = tmpvar_8;
  highp vec4 tmpvar_9;
  tmpvar_9.w = 1.0;
  tmpvar_9.xyz = _WorldSpaceCameraPos;
  highp vec3 tmpvar_10;
  tmpvar_10 = (tmpvar_7 * (((_World2Object * tmpvar_9).xyz * unity_Scale.w) - _glesVertex.xyz));
  tmpvar_4 = tmpvar_10;
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = tmpvar_3;
  xlv_TEXCOORD2 = tmpvar_4;
  xlv_TEXCOORD3 = (_LightMatrix0 * (_Object2World * _glesVertex)).xy;
}



#endif
#ifdef FRAGMENT

varying highp vec2 xlv_TEXCOORD3;
varying mediump vec3 xlv_TEXCOORD2;
varying mediump vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform mediump float _Gloss;
uniform mediump float _Shininess;
uniform lowp vec4 _Color;
uniform sampler2D _SpecMap;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform sampler2D _LightTexture0;
uniform lowp vec4 _LightColor0;
void main ()
{
  lowp vec4 c_1;
  lowp vec3 lightDir_2;
  mediump vec3 tmpvar_3;
  mediump vec3 tmpvar_4;
  lowp vec4 tmpvar_5;
  tmpvar_5 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_6;
  tmpvar_6 = (tmpvar_5.xyz * _Color.xyz);
  tmpvar_3 = tmpvar_6;
  lowp vec4 tmpvar_7;
  tmpvar_7 = texture2D (_SpecMap, xlv_TEXCOORD0);
  mediump vec3 tmpvar_8;
  tmpvar_8 = ((tmpvar_5.w * tmpvar_7.xyz) * _Gloss);
  lowp vec3 tmpvar_9;
  tmpvar_9 = ((texture2D (_BumpMap, xlv_TEXCOORD0).xyz * 2.0) - 1.0);
  tmpvar_4 = tmpvar_9;
  lightDir_2 = xlv_TEXCOORD1;
  lowp vec4 tmpvar_10;
  tmpvar_10 = texture2D (_LightTexture0, xlv_TEXCOORD3);
  mediump vec3 lightDir_11;
  lightDir_11 = lightDir_2;
  mediump float atten_12;
  atten_12 = tmpvar_10.w;
  mediump vec4 c_13;
  mediump vec3 specCol_14;
  highp float nh_15;
  mediump float tmpvar_16;
  tmpvar_16 = max (0.0, dot (tmpvar_4, normalize((lightDir_11 + normalize(xlv_TEXCOORD2)))));
  nh_15 = tmpvar_16;
  mediump float arg1_17;
  arg1_17 = (32.0 * _Shininess);
  highp vec3 tmpvar_18;
  tmpvar_18 = (pow (nh_15, arg1_17) * tmpvar_8);
  specCol_14 = tmpvar_18;
  c_13.xyz = ((((tmpvar_3 * _LightColor0.xyz) * max (0.0, dot (tmpvar_4, lightDir_11))) + (_LightColor0.xyz * specCol_14)) * (atten_12 * 2.0));
  c_13.w = 0.0;
  c_1.xyz = c_13.xyz;
  c_1.w = 0.0;
  gl_FragData[0] = c_1;
}



#endif"
}

SubProgram "glesdesktop " {
Keywords { "DIRECTIONAL_COOKIE" }
"!!GLES
#define SHADER_API_GLES 1
#define tex2D texture2D


#ifdef VERTEX
#define gl_ModelViewProjectionMatrix glstate_matrix_mvp
uniform mat4 glstate_matrix_mvp;

varying highp vec2 xlv_TEXCOORD3;
varying mediump vec3 xlv_TEXCOORD2;
varying mediump vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp vec4 _MainTex_ST;
uniform highp mat4 _LightMatrix0;
uniform highp vec4 unity_Scale;
uniform highp mat4 _World2Object;
uniform highp mat4 _Object2World;

uniform lowp vec4 _WorldSpaceLightPos0;
uniform highp vec3 _WorldSpaceCameraPos;
attribute vec4 _glesTANGENT;
attribute vec4 _glesMultiTexCoord0;
attribute vec3 _glesNormal;
attribute vec4 _glesVertex;
void main ()
{
  vec4 tmpvar_1;
  tmpvar_1.xyz = normalize(_glesTANGENT.xyz);
  tmpvar_1.w = _glesTANGENT.w;
  vec3 tmpvar_2;
  tmpvar_2 = normalize(_glesNormal);
  mediump vec3 tmpvar_3;
  mediump vec3 tmpvar_4;
  highp vec3 tmpvar_5;
  highp vec3 tmpvar_6;
  tmpvar_5 = tmpvar_1.xyz;
  tmpvar_6 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_7;
  tmpvar_7[0].x = tmpvar_5.x;
  tmpvar_7[0].y = tmpvar_6.x;
  tmpvar_7[0].z = tmpvar_2.x;
  tmpvar_7[1].x = tmpvar_5.y;
  tmpvar_7[1].y = tmpvar_6.y;
  tmpvar_7[1].z = tmpvar_2.y;
  tmpvar_7[2].x = tmpvar_5.z;
  tmpvar_7[2].y = tmpvar_6.z;
  tmpvar_7[2].z = tmpvar_2.z;
  highp vec3 tmpvar_8;
  tmpvar_8 = (tmpvar_7 * (_World2Object * _WorldSpaceLightPos0).xyz);
  tmpvar_3 = tmpvar_8;
  highp vec4 tmpvar_9;
  tmpvar_9.w = 1.0;
  tmpvar_9.xyz = _WorldSpaceCameraPos;
  highp vec3 tmpvar_10;
  tmpvar_10 = (tmpvar_7 * (((_World2Object * tmpvar_9).xyz * unity_Scale.w) - _glesVertex.xyz));
  tmpvar_4 = tmpvar_10;
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = tmpvar_3;
  xlv_TEXCOORD2 = tmpvar_4;
  xlv_TEXCOORD3 = (_LightMatrix0 * (_Object2World * _glesVertex)).xy;
}



#endif
#ifdef FRAGMENT

varying highp vec2 xlv_TEXCOORD3;
varying mediump vec3 xlv_TEXCOORD2;
varying mediump vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform mediump float _Gloss;
uniform mediump float _Shininess;
uniform lowp vec4 _Color;
uniform sampler2D _SpecMap;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform sampler2D _LightTexture0;
uniform lowp vec4 _LightColor0;
void main ()
{
  lowp vec4 c_1;
  lowp vec3 lightDir_2;
  mediump vec3 tmpvar_3;
  mediump vec3 tmpvar_4;
  lowp vec4 tmpvar_5;
  tmpvar_5 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_6;
  tmpvar_6 = (tmpvar_5.xyz * _Color.xyz);
  tmpvar_3 = tmpvar_6;
  lowp vec4 tmpvar_7;
  tmpvar_7 = texture2D (_SpecMap, xlv_TEXCOORD0);
  mediump vec3 tmpvar_8;
  tmpvar_8 = ((tmpvar_5.w * tmpvar_7.xyz) * _Gloss);
  lowp vec3 normal_9;
  normal_9.xy = ((texture2D (_BumpMap, xlv_TEXCOORD0).wy * 2.0) - 1.0);
  normal_9.z = sqrt(((1.0 - (normal_9.x * normal_9.x)) - (normal_9.y * normal_9.y)));
  tmpvar_4 = normal_9;
  lightDir_2 = xlv_TEXCOORD1;
  lowp vec4 tmpvar_10;
  tmpvar_10 = texture2D (_LightTexture0, xlv_TEXCOORD3);
  mediump vec3 lightDir_11;
  lightDir_11 = lightDir_2;
  mediump float atten_12;
  atten_12 = tmpvar_10.w;
  mediump vec4 c_13;
  mediump vec3 specCol_14;
  highp float nh_15;
  mediump float tmpvar_16;
  tmpvar_16 = max (0.0, dot (tmpvar_4, normalize((lightDir_11 + normalize(xlv_TEXCOORD2)))));
  nh_15 = tmpvar_16;
  mediump float arg1_17;
  arg1_17 = (32.0 * _Shininess);
  highp vec3 tmpvar_18;
  tmpvar_18 = (pow (nh_15, arg1_17) * tmpvar_8);
  specCol_14 = tmpvar_18;
  c_13.xyz = ((((tmpvar_3 * _LightColor0.xyz) * max (0.0, dot (tmpvar_4, lightDir_11))) + (_LightColor0.xyz * specCol_14)) * (atten_12 * 2.0));
  c_13.w = 0.0;
  c_1.xyz = c_13.xyz;
  c_1.w = 0.0;
  gl_FragData[0] = c_1;
}



#endif"
}

SubProgram "flash " {
Keywords { "DIRECTIONAL_COOKIE" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Matrix 0 [glstate_matrix_mvp]
Vector 16 [_WorldSpaceCameraPos]
Vector 17 [_WorldSpaceLightPos0]
Matrix 4 [_Object2World]
Matrix 8 [_World2Object]
Vector 18 [unity_Scale]
Matrix 12 [_LightMatrix0]
Vector 19 [_MainTex_ST]
"agal_vs
c20 1.0 0.0 0.0 0.0
[bc]
aaaaaaaaaaaaaiacbeaaaaaaabaaaaaaaaaaaaaaaaaaaaaa mov r0.w, c20.x
aaaaaaaaaaaaahacbaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, c16
bdaaaaaaabaaaeacaaaaaaoeacaaaaaaakaaaaoeabaaaaaa dp4 r1.z, r0, c10
bdaaaaaaabaaacacaaaaaaoeacaaaaaaajaaaaoeabaaaaaa dp4 r1.y, r0, c9
bdaaaaaaabaaabacaaaaaaoeacaaaaaaaiaaaaoeabaaaaaa dp4 r1.x, r0, c8
adaaaaaaacaaahacabaaaakeacaaaaaabcaaaappabaaaaaa mul r2.xyz, r1.xyzz, c18.w
acaaaaaaadaaahacacaaaakeacaaaaaaaaaaaaoeaaaaaaaa sub r3.xyz, r2.xyzz, a0
aaaaaaaaaaaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, a5
adaaaaaaabaaahacabaaaancaaaaaaaaaaaaaaajacaaaaaa mul r1.xyz, a1.zxyw, r0.yzxx
aaaaaaaaaaaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, a5
adaaaaaaaeaaahacabaaaamjaaaaaaaaaaaaaafcacaaaaaa mul r4.xyz, a1.yzxw, r0.zxyy
acaaaaaaabaaahacaeaaaakeacaaaaaaabaaaakeacaaaaaa sub r1.xyz, r4.xyzz, r1.xyzz
adaaaaaaacaaahacabaaaakeacaaaaaaafaaaappaaaaaaaa mul r2.xyz, r1.xyzz, a5.w
aaaaaaaaaaaaapacakaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0, c10
bdaaaaaaaeaaaeacbbaaaaoeabaaaaaaaaaaaaoeacaaaaaa dp4 r4.z, c17, r0
aaaaaaaaaaaaapacajaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0, c9
bdaaaaaaaeaaacacbbaaaaoeabaaaaaaaaaaaaoeacaaaaaa dp4 r4.y, c17, r0
aaaaaaaaabaaapacaiaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r1, c8
bdaaaaaaaeaaabacbbaaaaoeabaaaaaaabaaaaoeacaaaaaa dp4 r4.x, c17, r1
bdaaaaaaaaaaaiacaaaaaaoeaaaaaaaaahaaaaoeabaaaaaa dp4 r0.w, a0, c7
bdaaaaaaaaaaaeacaaaaaaoeaaaaaaaaagaaaaoeabaaaaaa dp4 r0.z, a0, c6
bdaaaaaaaaaaabacaaaaaaoeaaaaaaaaaeaaaaoeabaaaaaa dp4 r0.x, a0, c4
bdaaaaaaaaaaacacaaaaaaoeaaaaaaaaafaaaaoeabaaaaaa dp4 r0.y, a0, c5
bcaaaaaaabaaacaeaeaaaakeacaaaaaaacaaaakeacaaaaaa dp3 v1.y, r4.xyzz, r2.xyzz
bcaaaaaaacaaacaeacaaaakeacaaaaaaadaaaakeacaaaaaa dp3 v2.y, r2.xyzz, r3.xyzz
bcaaaaaaabaaaeaeabaaaaoeaaaaaaaaaeaaaakeacaaaaaa dp3 v1.z, a1, r4.xyzz
bcaaaaaaabaaabaeaeaaaakeacaaaaaaafaaaaoeaaaaaaaa dp3 v1.x, r4.xyzz, a5
bcaaaaaaacaaaeaeabaaaaoeaaaaaaaaadaaaakeacaaaaaa dp3 v2.z, a1, r3.xyzz
bcaaaaaaacaaabaeafaaaaoeaaaaaaaaadaaaakeacaaaaaa dp3 v2.x, a5, r3.xyzz
bdaaaaaaadaaacaeaaaaaaoeacaaaaaaanaaaaoeabaaaaaa dp4 v3.y, r0, c13
bdaaaaaaadaaabaeaaaaaaoeacaaaaaaamaaaaoeabaaaaaa dp4 v3.x, r0, c12
adaaaaaaaaaaadacadaaaaoeaaaaaaaabdaaaaoeabaaaaaa mul r0.xy, a3, c19
abaaaaaaaaaaadaeaaaaaafeacaaaaaabdaaaaooabaaaaaa add v0.xy, r0.xyyy, c19.zwzw
bdaaaaaaaaaaaiadaaaaaaoeaaaaaaaaadaaaaoeabaaaaaa dp4 o0.w, a0, c3
bdaaaaaaaaaaaeadaaaaaaoeaaaaaaaaacaaaaoeabaaaaaa dp4 o0.z, a0, c2
bdaaaaaaaaaaacadaaaaaaoeaaaaaaaaabaaaaoeabaaaaaa dp4 o0.y, a0, c1
bdaaaaaaaaaaabadaaaaaaoeaaaaaaaaaaaaaaoeabaaaaaa dp4 o0.x, a0, c0
aaaaaaaaaaaaamaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v0.zw, c0
aaaaaaaaabaaaiaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v1.w, c0
aaaaaaaaacaaaiaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v2.w, c0
aaaaaaaaadaaamaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v3.zw, c0
"
}

SubProgram "d3d11_9x " {
Keywords { "DIRECTIONAL_COOKIE" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 160 // 160 used size, 8 vars
Matrix 48 [_LightMatrix0] 4
Vector 144 [_MainTex_ST] 4
ConstBuffer "UnityPerCamera" 128 // 76 used size, 8 vars
Vector 64 [_WorldSpaceCameraPos] 3
ConstBuffer "UnityLighting" 400 // 16 used size, 16 vars
Vector 0 [_WorldSpaceLightPos0] 4
ConstBuffer "UnityPerDraw" 336 // 336 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
Matrix 192 [_Object2World] 4
Matrix 256 [_World2Object] 4
Vector 320 [unity_Scale] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityLighting" 2
BindCB "UnityPerDraw" 3
// 32 instructions, 2 temp regs, 0 temp arrays:
// ALU 14 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0_level_9_3
eefiecedgdmopicnijgpmnldmjninmgkhbnngeamabaaaaaapiajaaaaaeaaaaaa
daaaaaaafiadaaaajaaiaaaafiajaaaaebgpgodjcaadaaaacaadaaaaaaacpopp
laacaaaahaaaaaaaagaaceaaaaaagmaaaaaagmaaaaaaceaaabaagmaaaaaaadaa
aeaaabaaaaaaaaaaaaaaajaaabaaafaaaaaaaaaaabaaaeaaabaaagaaaaaaaaaa
acaaaaaaabaaahaaaaaaaaaaadaaaaaaaeaaaiaaaaaaaaaaadaaamaaajaaamaa
aaaaaaaaaaaaaaaaabacpoppbpaaaaacafaaaaiaaaaaapjabpaaaaacafaaabia
abaaapjabpaaaaacafaaaciaacaaapjabpaaaaacafaaadiaadaaapjaaeaaaaae
aaaaadoaadaaoejaafaaoekaafaaookaabaaaaacaaaaapiaahaaoekaafaaaaad
abaaahiaaaaaffiabbaaoekaaeaaaaaeabaaahiabaaaoekaaaaaaaiaabaaoeia
aeaaaaaeaaaaahiabcaaoekaaaaakkiaabaaoeiaaeaaaaaeaaaaahiabdaaoeka
aaaappiaaaaaoeiaaiaaaaadabaaaboaabaaoejaaaaaoeiaabaaaaacabaaahia
abaaoejaafaaaaadacaaahiaabaamjiaacaancjaaeaaaaaeabaaahiaacaamjja
abaanciaacaaoeibafaaaaadabaaahiaabaaoeiaabaappjaaiaaaaadabaaacoa
abaaoeiaaaaaoeiaaiaaaaadabaaaeoaacaaoejaaaaaoeiaabaaaaacaaaaahia
agaaoekaafaaaaadacaaahiaaaaaffiabbaaoekaaeaaaaaeaaaaaliabaaakeka
aaaaaaiaacaakeiaaeaaaaaeaaaaahiabcaaoekaaaaakkiaaaaapeiaacaaaaad
aaaaahiaaaaaoeiabdaaoekaaeaaaaaeaaaaahiaaaaaoeiabeaappkaaaaaoejb
aiaaaaadacaaaboaabaaoejaaaaaoeiaaiaaaaadacaaacoaabaaoeiaaaaaoeia
aiaaaaadacaaaeoaacaaoejaaaaaoeiaafaaaaadaaaaapiaaaaaffjaanaaoeka
aeaaaaaeaaaaapiaamaaoekaaaaaaajaaaaaoeiaaeaaaaaeaaaaapiaaoaaoeka
aaaakkjaaaaaoeiaaeaaaaaeaaaaapiaapaaoekaaaaappjaaaaaoeiaafaaaaad
abaaadiaaaaaffiaacaaobkaaeaaaaaeaaaaadiaabaaobkaaaaaaaiaabaaoeia
aeaaaaaeaaaaadiaadaaobkaaaaakkiaaaaaoeiaaeaaaaaeaaaaamoaaeaabeka
aaaappiaaaaaeeiaafaaaaadaaaaapiaaaaaffjaajaaoekaaeaaaaaeaaaaapia
aiaaoekaaaaaaajaaaaaoeiaaeaaaaaeaaaaapiaakaaoekaaaaakkjaaaaaoeia
aeaaaaaeaaaaapiaalaaoekaaaaappjaaaaaoeiaaeaaaaaeaaaaadmaaaaappia
aaaaoekaaaaaoeiaabaaaaacaaaaammaaaaaoeiappppaaaafdeieefcdaafaaaa
eaaaabaaemabaaaafjaaaaaeegiocaaaaaaaaaaaakaaaaaafjaaaaaeegiocaaa
abaaaaaaafaaaaaafjaaaaaeegiocaaaacaaaaaaabaaaaaafjaaaaaeegiocaaa
adaaaaaabfaaaaaafpaaaaadpcbabaaaaaaaaaaafpaaaaadpcbabaaaabaaaaaa
fpaaaaadhcbabaaaacaaaaaafpaaaaaddcbabaaaadaaaaaaghaaaaaepccabaaa
aaaaaaaaabaaaaaagfaaaaaddccabaaaabaaaaaagfaaaaadmccabaaaabaaaaaa
gfaaaaadhccabaaaacaaaaaagfaaaaadhccabaaaadaaaaaagiaaaaacacaaaaaa
diaaaaaipcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaadaaaaaaabaaaaaa
dcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaaaaaaaaaagbabaaaaaaaaaaa
egaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaacaaaaaa
kgbkbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpccabaaaaaaaaaaaegiocaaa
adaaaaaaadaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadiaaaaaipcaabaaa
aaaaaaaafgbfbaaaaaaaaaaaegiocaaaadaaaaaaanaaaaaadcaaaaakpcaabaaa
aaaaaaaaegiocaaaadaaaaaaamaaaaaaagbabaaaaaaaaaaaegaobaaaaaaaaaaa
dcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaaoaaaaaakgbkbaaaaaaaaaaa
egaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaapaaaaaa
pgbpbaaaaaaaaaaaegaobaaaaaaaaaaadiaaaaaidcaabaaaabaaaaaafgafbaaa
aaaaaaaaegiacaaaaaaaaaaaaeaaaaaadcaaaaakdcaabaaaaaaaaaaaegiacaaa
aaaaaaaaadaaaaaaagaabaaaaaaaaaaaegaabaaaabaaaaaadcaaaaakdcaabaaa
aaaaaaaaegiacaaaaaaaaaaaafaaaaaakgakbaaaaaaaaaaaegaabaaaaaaaaaaa
dcaaaaakmccabaaaabaaaaaaagiecaaaaaaaaaaaagaaaaaapgapbaaaaaaaaaaa
agaebaaaaaaaaaaadcaaaaaldccabaaaabaaaaaaegbabaaaadaaaaaaegiacaaa
aaaaaaaaajaaaaaaogikcaaaaaaaaaaaajaaaaaadiaaaaahhcaabaaaaaaaaaaa
jgbebaaaabaaaaaacgbjbaaaacaaaaaadcaaaaakhcaabaaaaaaaaaaajgbebaaa
acaaaaaacgbjbaaaabaaaaaaegacbaiaebaaaaaaaaaaaaaadiaaaaahhcaabaaa
aaaaaaaaegacbaaaaaaaaaaapgbpbaaaabaaaaaadiaaaaajhcaabaaaabaaaaaa
fgifcaaaacaaaaaaaaaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaa
abaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaaacaaaaaaaaaaaaaaegacbaaa
abaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaa
acaaaaaaaaaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaa
adaaaaaabdaaaaaapgipcaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaabaaaaaah
cccabaaaacaaaaaaegacbaaaaaaaaaaaegacbaaaabaaaaaabaaaaaahbccabaaa
acaaaaaaegbcbaaaabaaaaaaegacbaaaabaaaaaabaaaaaaheccabaaaacaaaaaa
egbcbaaaacaaaaaaegacbaaaabaaaaaadiaaaaajhcaabaaaabaaaaaafgifcaaa
abaaaaaaaeaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaaabaaaaaa
egiccaaaadaaaaaabaaaaaaaagiacaaaabaaaaaaaeaaaaaaegacbaaaabaaaaaa
dcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaaabaaaaaa
aeaaaaaaegacbaaaabaaaaaaaaaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaa
egiccaaaadaaaaaabdaaaaaadcaaaaalhcaabaaaabaaaaaaegacbaaaabaaaaaa
pgipcaaaadaaaaaabeaaaaaaegbcbaiaebaaaaaaaaaaaaaabaaaaaahcccabaaa
adaaaaaaegacbaaaaaaaaaaaegacbaaaabaaaaaabaaaaaahbccabaaaadaaaaaa
egbcbaaaabaaaaaaegacbaaaabaaaaaabaaaaaaheccabaaaadaaaaaaegbcbaaa
acaaaaaaegacbaaaabaaaaaadoaaaaabejfdeheomaaaaaaaagaaaaaaaiaaaaaa
jiaaaaaaaaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaa
acaaaaaaahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaa
laaaaaaaabaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaljaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofe
aaeoepfcenebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheojiaaaaaa
afaaaaaaaiaaaaaaiaaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaa
imaaaaaaaaaaaaaaaaaaaaaaadaaaaaaabaaaaaaadamaaaaimaaaaaaadaaaaaa
aaaaaaaaadaaaaaaabaaaaaaamadaaaaimaaaaaaabaaaaaaaaaaaaaaadaaaaaa
acaaaaaaahaiaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaahaiaaaa
fdfgfpfaepfdejfeejepeoaafeeffiedepepfceeaaklklkl"
}

}
Program "fp" {
// Fragment combos: 5
//   opengl - ALU: 32 to 43, TEX: 3 to 5
//   d3d9 - ALU: 34 to 45, TEX: 3 to 5
//   d3d11 - ALU: 20 to 30, TEX: 3 to 5, FLOW: 1 to 1
//   d3d11_9x - ALU: 20 to 30, TEX: 3 to 5, FLOW: 1 to 1
SubProgram "opengl " {
Keywords { "POINT" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_LightTexture0] 2D
"!!ARBfp1.0
OPTION ARB_precision_hint_fastest;
# 37 ALU, 4 TEX
PARAM c[5] = { program.local[0..3],
		{ 0, 2, 1, 32 } };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
TEMP R4;
TEX R0, fragment.texcoord[0], texture[0], 2D;
TEX R2.yw, fragment.texcoord[0], texture[2], 2D;
TEX R1.xyz, fragment.texcoord[0], texture[1], 2D;
MAD R2.xy, R2.wyzw, c[4].y, -c[4].z;
MUL R2.z, R2.y, R2.y;
MAD R2.z, -R2.x, R2.x, -R2;
MUL R1.xyz, R0.w, R1;
DP3 R1.w, fragment.texcoord[3], fragment.texcoord[3];
DP3 R2.w, fragment.texcoord[1], fragment.texcoord[1];
RSQ R2.w, R2.w;
ADD R2.z, R2, c[4];
RSQ R2.z, R2.z;
MUL R0.xyz, R0, c[1];
MUL R3.xyz, R2.w, fragment.texcoord[1];
DP3 R3.w, fragment.texcoord[2], fragment.texcoord[2];
RSQ R2.w, R3.w;
MAD R4.xyz, R2.w, fragment.texcoord[2], R3;
DP3 R2.w, R4, R4;
RSQ R2.w, R2.w;
RCP R2.z, R2.z;
MUL R4.xyz, R2.w, R4;
DP3 R2.w, R2, R3;
DP3 R2.y, R2, R4;
MAX R2.z, R2.y, c[4].x;
MOV R2.y, c[4].w;
MUL R0.w, R2.y, c[2].x;
MAX R2.x, R2.w, c[4];
MUL R1.xyz, R1, c[3].x;
POW R0.w, R2.z, R0.w;
MUL R1.xyz, R0.w, R1;
MUL R1.xyz, R1, c[0];
MUL R0.xyz, R0, c[0];
MAD R0.xyz, R0, R2.x, R1;
MOV result.color.w, c[4].x;
TEX R1.w, R1.w, texture[3], 2D;
MUL R0.xyz, R1.w, R0;
MUL result.color.xyz, R0, c[4].y;
END
# 37 instructions, 5 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "POINT" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_LightTexture0] 2D
"ps_2_0
; 39 ALU, 4 TEX
dcl_2d s0
dcl_2d s1
dcl_2d s2
dcl_2d s3
def c4, 2.00000000, -1.00000000, 1.00000000, 0.00000000
def c5, 32.00000000, 0, 0, 0
dcl t0.xy
dcl t1.xyz
dcl t2.xyz
dcl t3.xyz
texld r2, t0, s0
texld r3, t0, s1
dp3_pp r1.x, t1, t1
rsq_pp r4.x, r1.x
dp3 r0.x, t3, t3
mov r0.xy, r0.x
dp3_pp r1.x, t2, t2
mul_pp r2.xyz, r2, c1
mul_pp r4.xyz, r4.x, t1
rsq_pp r1.x, r1.x
mad_pp r5.xyz, r1.x, t2, r4
dp3_pp r1.x, r5, r5
rsq_pp r1.x, r1.x
mul_pp r1.xyz, r1.x, r5
mul_pp r2.xyz, r2, c0
texld r7, r0, s3
texld r0, t0, s2
mov r0.x, r0.w
mad_pp r6.xy, r0, c4.x, c4.y
mul_pp r0.x, r6.y, r6.y
mad_pp r0.x, -r6, r6, -r0
add_pp r0.x, r0, c4.z
rsq_pp r0.x, r0.x
rcp_pp r6.z, r0.x
dp3_pp r1.x, r6, r1
mov_pp r0.x, c2
mul_pp r0.x, c5, r0
max_pp r1.x, r1, c4.w
pow r5.x, r1.x, r0.x
mul_pp r0.xyz, r2.w, r3
mul_pp r1.xyz, r0, c3.x
mov r0.x, r5.x
mul r0.xyz, r0.x, r1
mul_pp r1.xyz, r0, c0
dp3_pp r0.x, r6, r4
max_pp r0.x, r0, c4.w
mad_pp r0.xyz, r2, r0.x, r1
mul_pp r0.xyz, r7.x, r0
mul_pp r0.xyz, r0, c4.x
mov_pp r0.w, c4
mov_pp oC0, r0
"
}

SubProgram "xbox360 " {
Keywords { "POINT" }
Vector 1 [_Color]
Float 3 [_Gloss]
Vector 0 [_LightColor0]
Float 2 [_Shininess]
SetTexture 0 [_LightTexture0] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_SpecMap] 2D
// Shader Timing Estimate, in Cycles/64 pixel vector:
// ALU: 28.00 (21 instructions), vertex: 0, texture: 16,
//   sequencer: 12, interpolator: 16;    7 GPRs, 27 threads,
// Performance (if enough threads): ~28 cycles per vector
// * Texture cycle estimates are assuming an 8bit/component texture with no
//     aniso or trilinear filtering.

"ps_360
backbbaaaaaaabneaaaaabjmaaaaaaaaaaaaaaceaaaaabhmaaaaabkeaaaaaaaa
aaaaaaaaaaaaabfeaaaaaabmaaaaabeippppadaaaaaaaaaiaaaaaabmaaaaaaaa
aaaaabebaaaaaalmaaadaaacaaabaaaaaaaaaamiaaaaaaaaaaaaaaniaaacaaab
aaabaaaaaaaaaaoaaaaaaaaaaaaaaapaaaacaaadaaabaaaaaaaaaapiaaaaaaaa
aaaaabaiaaacaaaaaaabaaaaaaaaaaoaaaaaaaaaaaaaabbfaaadaaaaaaabaaaa
aaaaaamiaaaaaaaaaaaaabceaaadaaabaaabaaaaaaaaaamiaaaaaaaaaaaaabcn
aaacaaacaaabaaaaaaaaaapiaaaaaaaaaaaaabdiaaadaaadaaabaaaaaaaaaami
aaaaaaaafpechfgnhaengbhaaaklklklaaaeaaamaaabaaabaaabaaaaaaaaaaaa
fpedgpgmgphcaaklaaabaaadaaabaaaeaaabaaaaaaaaaaaafpehgmgphdhdaakl
aaaaaaadaaabaaabaaabaaaaaaaaaaaafpemgjghgiheedgpgmgphcdaaafpemgj
ghgihefegfhihehfhcgfdaaafpengbgjgofegfhiaafpfdgigjgogjgogfhdhdaa
fpfdhagfgdengbhaaahahdfpddfpdaaadccodacodcdadddfddcodaaaaaaaaaaa
aaaaaaabaaaaaaaaaaaaaaaaaaaaaabeabpmaabaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaeaaaaaabfmbaaaagaaaaaaaaaeaaaaaaaaaaaacmieaaapaaap
aaaaaaabaaaadafaaaaahbfbaaaahcfcaaaahdfdaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaalpiaaaaaaaaaaaaadpiaaaaaecaaaaaaabfefaadaaaabcaameaaaaaa
aaaagaaigaaobcaabcaaaaaaaaaagabecabkbcaaccaaaaaamiaeaaaaaaloloaa
paadadaabadigaabbpbppoiiaaaaeaaakiaidaabbpbppppiaaaaeaaabacidaab
bpbppompaaaaeaaababiaaabbpbppgecaaaaeaaamiaiaaaeaagmblaacbacppaa
miaiaaabaaloloaapaacacaamiaiaaacaaloloaapaababaafiihacadaalelebl
oaadadicfibhabaeaablmablobacabibmiahaaafaagmmamaolabacaemiagaaac
aambgmaakaadppaamiaiaaabaelclcmgnbacacppbecbaaacaalololbpaafafaa
fibhacabaablmagmobaaagickaihacafaamagmblobafacibkibbaeacaalomdeb
naaeacabkiccaeacaalomdicnaafacabkiedaeacaalalbmaicacppabeaehacae
aamamalbkbaeaaicmiapaaaaaaaaomaaobaeacaadiboababaapmgmblkbabadaa
miahaaabaabfgmaaobababaamiahaaaaaamamamaklabaaaamiahmaaaaagmmaaa
obadaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "POINT" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_LightTexture0] 2D
"sce_fp_rsx // 44 instructions using 4 registers
[Configuration]
24
ffffffff0003c020000ffff1000000000000840004000000
[Offsets]
4
_LightColor0 2 0
000002a0000000e0
_Color 1 0
000000b0
_Shininess 1 0
00000180
_Gloss 1 0
00000080
[Microcode]
704
ee000100c8011c9dc8000001c8003fe110020500c8001c9dc8000001c8000001
94001704c8011c9dc8000001c8003fe1ae883940c8011c9dc8000029c800bfe1
068a0440ce001c9d00020000aa020000000040000000bf800000000000000000
9e001700c8011c9dc8000001c8003fe108820240fe001c9d00020000c8000001
0000000000000000000000000000000010820240ab141c9cab140000c8000001
0e800240c8001c9dc8020001c800000100000000000000000000000000000000
108c044001141c9e01140000c90400030e800240c9001c9dc8020001c8000001
000000000000000000000000000000008e021702c8011c9dc8000001c8003fe1
ce8c3940c8011c9dc8000029c800bfe10e8e024055041c9dc8040001c8000001
108c0340c9181c9d00020000c800000100003f80000000000000000000000000
088a3b40ff183c9dff180001c80000010e020340c9101c9dc9180001c8000001
028c0540c9141c9dc9100001c80000011082014000021c9cc8000001c8000001
000000000000000000000000000000001080090001181c9c00020000c8000001
0000000000000000000000000000000002041706fe041c9dc8000001c8000001
0e843940c8041c9dc8000029c800000102840540c9141c9dc9080001c8000001
02020900c9081c9d00020000c800000100000000000000000000000000000000
02860240ff041c9d00020000c800000100004200000000000000000000000000
10001d00c8041c9dc8000001c800000110000200c8001c9d010c0000c8000001
0e800240c9001c9dff000001c800000108001c00fe001c9dc8000001c8000001
0e82020054001c9dc91c0001c80000011080014000021c9cc8000001c8000001
000000000000000000000000000000000e800440c9041c9dc8020001c9000001
000000000000000000000000000000000e81024000081c9cc9001001c8000001
"
}

SubProgram "d3d11 " {
Keywords { "POINT" }
ConstBuffer "$Globals" 160 // 136 used size, 8 vars
Vector 16 [_LightColor0] 4
Vector 112 [_Color] 4
Float 128 [_Shininess]
Float 132 [_Gloss]
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 1
SetTexture 1 [_SpecMap] 2D 3
SetTexture 2 [_BumpMap] 2D 2
SetTexture 3 [_LightTexture0] 2D 0
// 36 instructions, 3 temp regs, 0 temp arrays:
// ALU 25 float, 0 int, 0 uint
// TEX 4 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0
eefiecedfibkklkomnijpmbpjdafombecechodhdabaaaaaapmafaaaaadaaaaaa
cmaaaaaammaaaaaaaaabaaaaejfdeheojiaaaaaaafaaaaaaaiaaaaaaiaaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaadadaaaaimaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaahahaaaaimaaaaaa
adaaaaaaaaaaaaaaadaaaaaaaeaaaaaaahahaaaafdfgfpfaepfdejfeejepeoaa
feeffiedepepfceeaaklklklepfdeheocmaaaaaaabaaaaaaaiaaaaaacaaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaafdfgfpfegbhcghgfheaaklkl
fdeieefcpeaeaaaaeaaaaaaadnabaaaafjaaaaaeegiocaaaaaaaaaaaajaaaaaa
fkaaaaadaagabaaaaaaaaaaafkaaaaadaagabaaaabaaaaaafkaaaaadaagabaaa
acaaaaaafkaaaaadaagabaaaadaaaaaafibiaaaeaahabaaaaaaaaaaaffffaaaa
fibiaaaeaahabaaaabaaaaaaffffaaaafibiaaaeaahabaaaacaaaaaaffffaaaa
fibiaaaeaahabaaaadaaaaaaffffaaaagcbaaaaddcbabaaaabaaaaaagcbaaaad
hcbabaaaacaaaaaagcbaaaadhcbabaaaadaaaaaagcbaaaadhcbabaaaaeaaaaaa
gfaaaaadpccabaaaaaaaaaaagiaaaaacadaaaaaabaaaaaahbcaabaaaaaaaaaaa
egbcbaaaadaaaaaaegbcbaaaadaaaaaaeeaaaaafbcaabaaaaaaaaaaaakaabaaa
aaaaaaaabaaaaaahccaabaaaaaaaaaaaegbcbaaaacaaaaaaegbcbaaaacaaaaaa
eeaaaaafccaabaaaaaaaaaaabkaabaaaaaaaaaaadiaaaaahocaabaaaaaaaaaaa
fgafbaaaaaaaaaaaagbjbaaaacaaaaaadcaaaaajhcaabaaaabaaaaaaegbcbaaa
adaaaaaaagaabaaaaaaaaaaajgahbaaaaaaaaaaabaaaaaahbcaabaaaaaaaaaaa
egacbaaaabaaaaaaegacbaaaabaaaaaaeeaaaaafbcaabaaaaaaaaaaaakaabaaa
aaaaaaaadiaaaaahhcaabaaaabaaaaaaagaabaaaaaaaaaaaegacbaaaabaaaaaa
efaaaaajpcaabaaaacaaaaaaegbabaaaabaaaaaaeghobaaaacaaaaaaaagabaaa
acaaaaaadcaaaaapdcaabaaaacaaaaaahgapbaaaacaaaaaaaceaaaaaaaaaaaea
aaaaaaeaaaaaaaaaaaaaaaaaaceaaaaaaaaaialpaaaaialpaaaaaaaaaaaaaaaa
dcaaaaakbcaabaaaaaaaaaaaakaabaiaebaaaaaaacaaaaaaakaabaaaacaaaaaa
abeaaaaaaaaaiadpdcaaaaakbcaabaaaaaaaaaaabkaabaiaebaaaaaaacaaaaaa
bkaabaaaacaaaaaaakaabaaaaaaaaaaaelaaaaafecaabaaaacaaaaaaakaabaaa
aaaaaaaabaaaaaahbcaabaaaaaaaaaaaegacbaaaacaaaaaaegacbaaaabaaaaaa
baaaaaahccaabaaaaaaaaaaaegacbaaaacaaaaaajgahbaaaaaaaaaaadeaaaaak
dcaabaaaaaaaaaaaegaabaaaaaaaaaaaaceaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaacpaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaadiaaaaaiecaabaaa
aaaaaaaaakiacaaaaaaaaaaaaiaaaaaaabeaaaaaaaaaaaecdiaaaaahbcaabaaa
aaaaaaaaakaabaaaaaaaaaaackaabaaaaaaaaaaabjaaaaafbcaabaaaaaaaaaaa
akaabaaaaaaaaaaaefaaaaajpcaabaaaabaaaaaaegbabaaaabaaaaaaeghobaaa
abaaaaaaaagabaaaadaaaaaaefaaaaajpcaabaaaacaaaaaaegbabaaaabaaaaaa
eghobaaaaaaaaaaaaagabaaaabaaaaaadiaaaaahhcaabaaaabaaaaaaegacbaaa
abaaaaaapgapbaaaacaaaaaadiaaaaaihcaabaaaacaaaaaaegacbaaaacaaaaaa
egiccaaaaaaaaaaaahaaaaaadiaaaaaihcaabaaaacaaaaaaegacbaaaacaaaaaa
egiccaaaaaaaaaaaabaaaaaadiaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaa
fgifcaaaaaaaaaaaaiaaaaaadiaaaaahncaabaaaaaaaaaaaagaabaaaaaaaaaaa
agajbaaaabaaaaaadiaaaaaincaabaaaaaaaaaaaagaobaaaaaaaaaaaagijcaaa
aaaaaaaaabaaaaaadcaaaaajhcaabaaaaaaaaaaaegacbaaaacaaaaaafgafbaaa
aaaaaaaaigadbaaaaaaaaaaabaaaaaahicaabaaaaaaaaaaaegbcbaaaaeaaaaaa
egbcbaaaaeaaaaaaefaaaaajpcaabaaaabaaaaaapgapbaaaaaaaaaaaeghobaaa
adaaaaaaaagabaaaaaaaaaaaaaaaaaahicaabaaaaaaaaaaaakaabaaaabaaaaaa
akaabaaaabaaaaaadiaaaaahhccabaaaaaaaaaaapgapbaaaaaaaaaaaegacbaaa
aaaaaaaadgaaaaaficcabaaaaaaaaaaaabeaaaaaaaaaaaaadoaaaaab"
}

SubProgram "gles " {
Keywords { "POINT" }
"!!GLES"
}

SubProgram "glesdesktop " {
Keywords { "POINT" }
"!!GLES"
}

SubProgram "flash " {
Keywords { "POINT" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_LightTexture0] 2D
"agal_ps
c4 2.0 -1.0 1.0 0.0
c5 32.0 0.0 0.0 0.0
[bc]
ciaaaaaaabaaapacaaaaaaoeaeaaaaaaacaaaaaaafaababb tex r1, v0, s2 <2d wrap linear point>
ciaaaaaaacaaapacaaaaaaoeaeaaaaaaaaaaaaaaafaababb tex r2, v0, s0 <2d wrap linear point>
ciaaaaaaadaaapacaaaaaaoeaeaaaaaaabaaaaaaafaababb tex r3, v0, s1 <2d wrap linear point>
bcaaaaaaabaaabacabaaaaoeaeaaaaaaabaaaaoeaeaaaaaa dp3 r1.x, v1, v1
akaaaaaaaeaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r4.x, r1.x
bcaaaaaaaaaaabacadaaaaoeaeaaaaaaadaaaaoeaeaaaaaa dp3 r0.x, v3, v3
aaaaaaaaaaaaadacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa mov r0.xy, r0.x
bcaaaaaaabaaabacacaaaaoeaeaaaaaaacaaaaoeaeaaaaaa dp3 r1.x, v2, v2
adaaaaaaacaaahacacaaaakeacaaaaaaabaaaaoeabaaaaaa mul r2.xyz, r2.xyzz, c1
adaaaaaaaeaaahacaeaaaaaaacaaaaaaabaaaaoeaeaaaaaa mul r4.xyz, r4.x, v1
akaaaaaaabaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r1.x, r1.x
adaaaaaaafaaahacabaaaaaaacaaaaaaacaaaaoeaeaaaaaa mul r5.xyz, r1.x, v2
abaaaaaaafaaahacafaaaakeacaaaaaaaeaaaakeacaaaaaa add r5.xyz, r5.xyzz, r4.xyzz
bcaaaaaaabaaabacafaaaakeacaaaaaaafaaaakeacaaaaaa dp3 r1.x, r5.xyzz, r5.xyzz
akaaaaaaabaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r1.x, r1.x
adaaaaaaacaaahacacaaaakeacaaaaaaaaaaaaoeabaaaaaa mul r2.xyz, r2.xyzz, c0
ciaaaaaaaaaaapacaaaaaafeacaaaaaaadaaaaaaafaababb tex r0, r0.xyyy, s3 <2d wrap linear point>
aaaaaaaaaaaaacacabaaaaffacaaaaaaaaaaaaaaaaaaaaaa mov r0.y, r1.y
aaaaaaaaaaaaabacabaaaappacaaaaaaaaaaaaaaaaaaaaaa mov r0.x, r1.w
adaaaaaaagaaadacaaaaaafeacaaaaaaaeaaaaaaabaaaaaa mul r6.xy, r0.xyyy, c4.x
abaaaaaaagaaadacagaaaafeacaaaaaaaeaaaaffabaaaaaa add r6.xy, r6.xyyy, c4.y
adaaaaaaaaaaabacagaaaaffacaaaaaaagaaaaffacaaaaaa mul r0.x, r6.y, r6.y
bfaaaaaaadaaaiacagaaaaaaacaaaaaaaaaaaaaaaaaaaaaa neg r3.w, r6.x
adaaaaaaadaaaiacadaaaappacaaaaaaagaaaaaaacaaaaaa mul r3.w, r3.w, r6.x
acaaaaaaaaaaabacadaaaappacaaaaaaaaaaaaaaacaaaaaa sub r0.x, r3.w, r0.x
abaaaaaaaaaaabacaaaaaaaaacaaaaaaaeaaaakkabaaaaaa add r0.x, r0.x, c4.z
akaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r0.x, r0.x
afaaaaaaagaaaeacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rcp r6.z, r0.x
adaaaaaaabaaahacabaaaaaaacaaaaaaafaaaakeacaaaaaa mul r1.xyz, r1.x, r5.xyzz
bcaaaaaaabaaabacagaaaakeacaaaaaaabaaaakeacaaaaaa dp3 r1.x, r6.xyzz, r1.xyzz
aaaaaaaaaaaaabacacaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0.x, c2
adaaaaaaaaaaabacafaaaaoeabaaaaaaaaaaaaaaacaaaaaa mul r0.x, c5, r0.x
ahaaaaaaabaaabacabaaaaaaacaaaaaaaeaaaappabaaaaaa max r1.x, r1.x, c4.w
alaaaaaaafaaapacabaaaaaaacaaaaaaaaaaaaaaacaaaaaa pow r5, r1.x, r0.x
adaaaaaaaaaaahacacaaaappacaaaaaaadaaaakeacaaaaaa mul r0.xyz, r2.w, r3.xyzz
adaaaaaaabaaahacaaaaaakeacaaaaaaadaaaaaaabaaaaaa mul r1.xyz, r0.xyzz, c3.x
aaaaaaaaaaaaabacafaaaaaaacaaaaaaaaaaaaaaaaaaaaaa mov r0.x, r5.x
adaaaaaaaaaaahacaaaaaaaaacaaaaaaabaaaakeacaaaaaa mul r0.xyz, r0.x, r1.xyzz
adaaaaaaabaaahacaaaaaakeacaaaaaaaaaaaaoeabaaaaaa mul r1.xyz, r0.xyzz, c0
bcaaaaaaaaaaabacagaaaakeacaaaaaaaeaaaakeacaaaaaa dp3 r0.x, r6.xyzz, r4.xyzz
ahaaaaaaaaaaabacaaaaaaaaacaaaaaaaeaaaappabaaaaaa max r0.x, r0.x, c4.w
adaaaaaaaaaaahacacaaaakeacaaaaaaaaaaaaaaacaaaaaa mul r0.xyz, r2.xyzz, r0.x
abaaaaaaaaaaahacaaaaaakeacaaaaaaabaaaakeacaaaaaa add r0.xyz, r0.xyzz, r1.xyzz
adaaaaaaaaaaahacaaaaaappacaaaaaaaaaaaakeacaaaaaa mul r0.xyz, r0.w, r0.xyzz
adaaaaaaaaaaahacaaaaaakeacaaaaaaaeaaaaaaabaaaaaa mul r0.xyz, r0.xyzz, c4.x
aaaaaaaaaaaaaiacaeaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0.w, c4
aaaaaaaaaaaaapadaaaaaaoeacaaaaaaaaaaaaaaaaaaaaaa mov o0, r0
"
}

SubProgram "d3d11_9x " {
Keywords { "POINT" }
ConstBuffer "$Globals" 160 // 136 used size, 8 vars
Vector 16 [_LightColor0] 4
Vector 112 [_Color] 4
Float 128 [_Shininess]
Float 132 [_Gloss]
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 1
SetTexture 1 [_SpecMap] 2D 3
SetTexture 2 [_BumpMap] 2D 2
SetTexture 3 [_LightTexture0] 2D 0
// 36 instructions, 3 temp regs, 0 temp arrays:
// ALU 25 float, 0 int, 0 uint
// TEX 4 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0_level_9_3
eefiecediicnedcdhkgflkehihhocifmepmhgdilabaaaaaapaaiaaaaaeaaaaaa
daaaaaaacaadaaaabmaiaaaalmaiaaaaebgpgodjoiacaaaaoiacaaaaaaacpppp
jmacaaaaemaaaaaaacaadeaaaaaaemaaaaaaemaaaeaaceaaaaaaemaaadaaaaaa
aaababaaacacacaaabadadaaaaaaabaaabaaaaaaaaaaaaaaaaaaahaaacaaabaa
aaaaaaaaabacppppfbaaaaafadaaapkaaaaaaaeaaaaaialpaaaaiadpaaaaaaaa
fbaaaaafaeaaapkaaaaaaaecaaaaaaaaaaaaaaaaaaaaaaaabpaaaaacaaaaaaia
aaaaadlabpaaaaacaaaaaaiaabaachlabpaaaaacaaaaaaiaacaachlabpaaaaac
aaaaaaiaadaaahlabpaaaaacaaaaaajaaaaiapkabpaaaaacaaaaaajaabaiapka
bpaaaaacaaaaaajaacaiapkabpaaaaacaaaaaajaadaiapkaaiaaaaadaaaaciia
acaaoelaacaaoelaahaaaaacaaaacbiaaaaappiaceaaaaacabaachiaabaaoela
aeaaaaaeaaaachiaacaaoelaaaaaaaiaabaaoeiaceaaaaacacaachiaaaaaoeia
aiaaaaadaaaaadiaadaaoelaadaaoelaecaaaaadadaacpiaaaaaoelaacaioeka
ecaaaaadaaaacpiaaaaaoeiaaaaioekaaeaaaaaeadaacdiaadaaohiaadaaaaka
adaaffkaaeaaaaaeabaaciiaadaaaaiaadaaaaibadaakkkaaeaaaaaeabaaciia
adaaffiaadaaffibabaappiaahaaaaacabaaciiaabaappiaagaaaaacadaaceia
abaappiaaiaaaaadabaaciiaadaaoeiaacaaoeiaaiaaaaadaaaacciaadaaoeia
abaaoeiaalaaaaadabaacbiaaaaaffiaadaappkaalaaaaadaaaaaciaabaappia
adaappkaabaaaaacacaaabiaacaaaakaafaaaaadaaaaaeiaacaaaaiaaeaaaaka
caaaaaadabaaaciaaaaaffiaaaaakkiaecaaaaadacaacpiaaaaaoelaabaioeka
ecaaaaadadaacpiaaaaaoelaadaioekaafaaaaadaaaacoiaacaappiaadaajaia
afaaaaadacaachiaacaaoeiaabaaoekaafaaaaadacaachiaacaaoeiaaaaaoeka
afaaaaadaaaacoiaaaaaoeiaacaaffkaafaaaaadaaaacoiaaaaaoeiaabaaffia
afaaaaadaaaacoiaaaaaoeiaaaaajakaaeaaaaaeaaaacoiaacaajaiaabaaaaia
aaaaoeiaacaaaaadaaaacbiaaaaaaaiaaaaaaaiaafaaaaadaaaachiaaaaaaaia
aaaapjiaabaaaaacaaaaciiaadaappkaabaaaaacaaaicpiaaaaaoeiappppaaaa
fdeieefcpeaeaaaaeaaaaaaadnabaaaafjaaaaaeegiocaaaaaaaaaaaajaaaaaa
fkaaaaadaagabaaaaaaaaaaafkaaaaadaagabaaaabaaaaaafkaaaaadaagabaaa
acaaaaaafkaaaaadaagabaaaadaaaaaafibiaaaeaahabaaaaaaaaaaaffffaaaa
fibiaaaeaahabaaaabaaaaaaffffaaaafibiaaaeaahabaaaacaaaaaaffffaaaa
fibiaaaeaahabaaaadaaaaaaffffaaaagcbaaaaddcbabaaaabaaaaaagcbaaaad
hcbabaaaacaaaaaagcbaaaadhcbabaaaadaaaaaagcbaaaadhcbabaaaaeaaaaaa
gfaaaaadpccabaaaaaaaaaaagiaaaaacadaaaaaabaaaaaahbcaabaaaaaaaaaaa
egbcbaaaadaaaaaaegbcbaaaadaaaaaaeeaaaaafbcaabaaaaaaaaaaaakaabaaa
aaaaaaaabaaaaaahccaabaaaaaaaaaaaegbcbaaaacaaaaaaegbcbaaaacaaaaaa
eeaaaaafccaabaaaaaaaaaaabkaabaaaaaaaaaaadiaaaaahocaabaaaaaaaaaaa
fgafbaaaaaaaaaaaagbjbaaaacaaaaaadcaaaaajhcaabaaaabaaaaaaegbcbaaa
adaaaaaaagaabaaaaaaaaaaajgahbaaaaaaaaaaabaaaaaahbcaabaaaaaaaaaaa
egacbaaaabaaaaaaegacbaaaabaaaaaaeeaaaaafbcaabaaaaaaaaaaaakaabaaa
aaaaaaaadiaaaaahhcaabaaaabaaaaaaagaabaaaaaaaaaaaegacbaaaabaaaaaa
efaaaaajpcaabaaaacaaaaaaegbabaaaabaaaaaaeghobaaaacaaaaaaaagabaaa
acaaaaaadcaaaaapdcaabaaaacaaaaaahgapbaaaacaaaaaaaceaaaaaaaaaaaea
aaaaaaeaaaaaaaaaaaaaaaaaaceaaaaaaaaaialpaaaaialpaaaaaaaaaaaaaaaa
dcaaaaakbcaabaaaaaaaaaaaakaabaiaebaaaaaaacaaaaaaakaabaaaacaaaaaa
abeaaaaaaaaaiadpdcaaaaakbcaabaaaaaaaaaaabkaabaiaebaaaaaaacaaaaaa
bkaabaaaacaaaaaaakaabaaaaaaaaaaaelaaaaafecaabaaaacaaaaaaakaabaaa
aaaaaaaabaaaaaahbcaabaaaaaaaaaaaegacbaaaacaaaaaaegacbaaaabaaaaaa
baaaaaahccaabaaaaaaaaaaaegacbaaaacaaaaaajgahbaaaaaaaaaaadeaaaaak
dcaabaaaaaaaaaaaegaabaaaaaaaaaaaaceaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaacpaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaadiaaaaaiecaabaaa
aaaaaaaaakiacaaaaaaaaaaaaiaaaaaaabeaaaaaaaaaaaecdiaaaaahbcaabaaa
aaaaaaaaakaabaaaaaaaaaaackaabaaaaaaaaaaabjaaaaafbcaabaaaaaaaaaaa
akaabaaaaaaaaaaaefaaaaajpcaabaaaabaaaaaaegbabaaaabaaaaaaeghobaaa
abaaaaaaaagabaaaadaaaaaaefaaaaajpcaabaaaacaaaaaaegbabaaaabaaaaaa
eghobaaaaaaaaaaaaagabaaaabaaaaaadiaaaaahhcaabaaaabaaaaaaegacbaaa
abaaaaaapgapbaaaacaaaaaadiaaaaaihcaabaaaacaaaaaaegacbaaaacaaaaaa
egiccaaaaaaaaaaaahaaaaaadiaaaaaihcaabaaaacaaaaaaegacbaaaacaaaaaa
egiccaaaaaaaaaaaabaaaaaadiaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaa
fgifcaaaaaaaaaaaaiaaaaaadiaaaaahncaabaaaaaaaaaaaagaabaaaaaaaaaaa
agajbaaaabaaaaaadiaaaaaincaabaaaaaaaaaaaagaobaaaaaaaaaaaagijcaaa
aaaaaaaaabaaaaaadcaaaaajhcaabaaaaaaaaaaaegacbaaaacaaaaaafgafbaaa
aaaaaaaaigadbaaaaaaaaaaabaaaaaahicaabaaaaaaaaaaaegbcbaaaaeaaaaaa
egbcbaaaaeaaaaaaefaaaaajpcaabaaaabaaaaaapgapbaaaaaaaaaaaeghobaaa
adaaaaaaaagabaaaaaaaaaaaaaaaaaahicaabaaaaaaaaaaaakaabaaaabaaaaaa
akaabaaaabaaaaaadiaaaaahhccabaaaaaaaaaaapgapbaaaaaaaaaaaegacbaaa
aaaaaaaadgaaaaaficcabaaaaaaaaaaaabeaaaaaaaaaaaaadoaaaaabejfdeheo
jiaaaaaaafaaaaaaaiaaaaaaiaaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaa
apaaaaaaimaaaaaaaaaaaaaaaaaaaaaaadaaaaaaabaaaaaaadadaaaaimaaaaaa
abaaaaaaaaaaaaaaadaaaaaaacaaaaaaahahaaaaimaaaaaaacaaaaaaaaaaaaaa
adaaaaaaadaaaaaaahahaaaaimaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaa
ahahaaaafdfgfpfaepfdejfeejepeoaafeeffiedepepfceeaaklklklepfdeheo
cmaaaaaaabaaaaaaaiaaaaaacaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaaaaaaaaa
apaaaaaafdfgfpfegbhcghgfheaaklkl"
}

SubProgram "opengl " {
Keywords { "DIRECTIONAL" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
"!!ARBfp1.0
OPTION ARB_precision_hint_fastest;
# 32 ALU, 3 TEX
PARAM c[5] = { program.local[0..3],
		{ 0, 2, 1, 32 } };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
TEX R0, fragment.texcoord[0], texture[0], 2D;
TEX R2.yw, fragment.texcoord[0], texture[2], 2D;
TEX R1.xyz, fragment.texcoord[0], texture[1], 2D;
MAD R2.xy, R2.wyzw, c[4].y, -c[4].z;
MUL R1.xyz, R0.w, R1;
DP3 R2.z, fragment.texcoord[2], fragment.texcoord[2];
MUL R1.w, R2.y, R2.y;
MAD R1.w, -R2.x, R2.x, -R1;
ADD R1.w, R1, c[4].z;
MOV R2.w, c[4];
MUL R0.xyz, R0, c[1];
RSQ R2.z, R2.z;
MOV R3.xyz, fragment.texcoord[1];
MAD R3.xyz, R2.z, fragment.texcoord[2], R3;
DP3 R2.z, R3, R3;
RSQ R2.z, R2.z;
RSQ R1.w, R1.w;
MUL R3.xyz, R2.z, R3;
RCP R2.z, R1.w;
DP3 R1.w, R2, R3;
MAX R1.w, R1, c[4].x;
MUL R0.w, R2, c[2].x;
POW R0.w, R1.w, R0.w;
MUL R1.xyz, R1, c[3].x;
MUL R1.xyz, R0.w, R1;
DP3 R0.w, R2, fragment.texcoord[1];
MUL R1.xyz, R1, c[0];
MUL R0.xyz, R0, c[0];
MAX R0.w, R0, c[4].x;
MAD R0.xyz, R0, R0.w, R1;
MUL result.color.xyz, R0, c[4].y;
MOV result.color.w, c[4].x;
END
# 32 instructions, 4 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "DIRECTIONAL" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
"ps_2_0
; 34 ALU, 3 TEX
dcl_2d s0
dcl_2d s1
dcl_2d s2
def c4, 2.00000000, -1.00000000, 1.00000000, 0.00000000
def c5, 32.00000000, 0, 0, 0
dcl t0.xy
dcl t1.xyz
dcl t2.xyz
texld r0, t0, s2
texld r2, t0, s0
texld r3, t0, s1
mov r0.x, r0.w
mad_pp r5.xy, r0, c4.x, c4.y
mul_pp r0.x, r5.y, r5.y
mad_pp r0.x, -r5, r5, -r0
dp3_pp r1.x, t2, t2
add_pp r0.x, r0, c4.z
rsq_pp r0.x, r0.x
rcp_pp r5.z, r0.x
mov_pp r0.x, c2
mul_pp r2.xyz, r2, c1
rsq_pp r1.x, r1.x
mov_pp r4.xyz, t1
mad_pp r4.xyz, r1.x, t2, r4
dp3_pp r1.x, r4, r4
rsq_pp r1.x, r1.x
mul_pp r1.xyz, r1.x, r4
dp3_pp r1.x, r5, r1
mul_pp r0.x, c5, r0
max_pp r1.x, r1, c4.w
pow r4.x, r1.x, r0.x
mul_pp r0.xyz, r2.w, r3
mul_pp r1.xyz, r0, c3.x
mov r0.x, r4.x
mul r0.xyz, r0.x, r1
mul_pp r1.xyz, r0, c0
dp3_pp r0.x, r5, t1
max_pp r0.x, r0, c4.w
mul_pp r2.xyz, r2, c0
mad_pp r0.xyz, r2, r0.x, r1
mul_pp r0.xyz, r0, c4.x
mov_pp r0.w, c4
mov_pp oC0, r0
"
}

SubProgram "xbox360 " {
Keywords { "DIRECTIONAL" }
Vector 1 [_Color]
Float 3 [_Gloss]
Vector 0 [_LightColor0]
Float 2 [_Shininess]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_BumpMap] 2D
SetTexture 2 [_SpecMap] 2D
// Shader Timing Estimate, in Cycles/64 pixel vector:
// ALU: 22.67 (17 instructions), vertex: 0, texture: 12,
//   sequencer: 10, interpolator: 12;    5 GPRs, 36 threads,
// Performance (if enough threads): ~22 cycles per vector
// * Texture cycle estimates are assuming an 8bit/component texture with no
//     aniso or trilinear filtering.

"ps_360
backbbaaaaaaablaaaaaabgaaaaaaaaaaaaaaaceaaaaabfmaaaaabieaaaaaaaa
aaaaaaaaaaaaabdeaaaaaabmaaaaabcfppppadaaaaaaaaahaaaaaabmaaaaaaaa
aaaaabboaaaaaakiaaadaaabaaabaaaaaaaaaaleaaaaaaaaaaaaaameaaacaaab
aaabaaaaaaaaaammaaaaaaaaaaaaaanmaaacaaadaaabaaaaaaaaaaoeaaaaaaaa
aaaaaapeaaacaaaaaaabaaaaaaaaaammaaaaaaaaaaaaababaaadaaaaaaabaaaa
aaaaaaleaaaaaaaaaaaaabakaaacaaacaaabaaaaaaaaaaoeaaaaaaaaaaaaabbf
aaadaaacaaabaaaaaaaaaaleaaaaaaaafpechfgnhaengbhaaaklklklaaaeaaam
aaabaaabaaabaaaaaaaaaaaafpedgpgmgphcaaklaaabaaadaaabaaaeaaabaaaa
aaaaaaaafpehgmgphdhdaaklaaaaaaadaaabaaabaaabaaaaaaaaaaaafpemgjgh
giheedgpgmgphcdaaafpengbgjgofegfhiaafpfdgigjgogjgogfhdhdaafpfdha
gfgdengbhaaahahdfpddfpdaaadccodacodcdadddfddcodaaaklklklaaaaaaaa
aaaaaaabaaaaaaaaaaaaaaaaaaaaaabeabpmaabaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaeaaaaaabcabaaaaeaaaaaaaaaeaaaaaaaaaaaacagdaaahaaah
aaaaaaabaaaadafaaaaahbfbaaaahcfcaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaeaaaaaaaaaaaaaaalpiaaaaadpiaaaaa
ecaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaabfdaadaaaabcaameaaaaaaaaaagaag
gaambcaabcaaaaaaaaaafabcaaaaccaaaaaaaaaabacidaabbpbppoiiaaaaeaaa
babieaabbpbpppnjaaaaeaaabaaiaaabbpbppgecaaaaeaaamiaiaaaeaagmgmaa
cbacppaabeciaaabaalololbpaacacaamiadaaaeaagngmmgilaepopofiihabad
aablmablobaaadibmiahaaacaablmamaolabacabmiaiaaacaegngnblnbaeaepo
kaeiaeabaaloloblpaacacicficbababaaloloblpaaeabibkiboacabaapmlbeb
mbacababkiccacabaamdloicnaabaeabkiedacabaalalbmaicabpoabeaehabae
aamamalbkbacaaibmiapaaaaaaaaomaaobaeabaadiboababaapmgmblkbadadaa
miahaaabaabfgmaaobababaamiahaaaaaamamamaklabaaaamiahmaaaaamamaaa
oaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "DIRECTIONAL" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
"sce_fp_rsx // 40 instructions using 3 registers
[Configuration]
24
ffffffff0001c0200007fff9000000000000840003000000
[Offsets]
4
_LightColor0 2 0
0000027000000220
_Color 1 0
00000020
_Shininess 1 0
00000130
_Gloss 1 0
00000070
[Microcode]
640
9e041700c8011c9dc8000001c8003fe10e880240c8081c9dc8020001c8000001
0000000000000000000000000000000094001704c8011c9dc8000001c8003fe1
06820440ce001c9daa0200005402000100000000000040000000bf8000000000
10800240c8081c9d00020000c800000100000000000000000000000000000000
8e021702c8011c9dc8000001c8003fe1ce803940c8011c9dc8000029c800bfe1
0e860240ff001c9dc8040001c800000110800240ab041c9cab040000c8000001
ae840140c8011c9dc8000001c8003fe11080044001041c9e01040000c9000003
0e800340c9081c9dc9000001c800000110800340c9001c9d00020000c8000001
00003f8000000000000000000000000008823b40ff003c9dff000001c8000001
028a014000021c9cc8000001c800000100000000000000000000000000000000
1088024001141c9c00020000c800000100004200000000000000000000000000
0e803940c9001c9dc8000029c800000102800540c9041c9dc9000001c8000001
1080014000021c9cc8000001c800000100000000000000000000000000000000
02840540c9041c9dc9080001c80000011000090001001c9c00020000c8000001
0000000000000000000000000000000002001d00fe001c9dc8000001c8000001
02840900c9081c9d00020000c800000100000000000000000000000000000000
1000020000001c9cc9100001c80000010e800240c9101c9dc8020001c8000001
000000000000000000000000000000000e800240c9001c9d01080000c8000001
08001c00fe001c9dc8000001c80000010e82020054001c9dc90c0001c8000001
0e810440c9041c9dc8021001c900000100000000000000000000000000000000
"
}

SubProgram "d3d11 " {
Keywords { "DIRECTIONAL" }
ConstBuffer "$Globals" 96 // 72 used size, 7 vars
Vector 16 [_LightColor0] 4
Vector 48 [_Color] 4
Float 64 [_Shininess]
Float 68 [_Gloss]
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 0
SetTexture 1 [_SpecMap] 2D 2
SetTexture 2 [_BumpMap] 2D 1
// 30 instructions, 3 temp regs, 0 temp arrays:
// ALU 20 float, 0 int, 0 uint
// TEX 3 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0
eefiecedhfjpljncoklmjgbgopaacekndnpndlbgabaaaaaabeafaaaaadaaaaaa
cmaaaaaaleaaaaaaoiaaaaaaejfdeheoiaaaaaaaaeaaaaaaaiaaaaaagiaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaheaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaadadaaaaheaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaaheaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaahahaaaafdfgfpfa
epfdejfeejepeoaafeeffiedepepfceeaaklklklepfdeheocmaaaaaaabaaaaaa
aiaaaaaacaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaafdfgfpfe
gbhcghgfheaaklklfdeieefcceaeaaaaeaaaaaaaajabaaaafjaaaaaeegiocaaa
aaaaaaaaafaaaaaafkaaaaadaagabaaaaaaaaaaafkaaaaadaagabaaaabaaaaaa
fkaaaaadaagabaaaacaaaaaafibiaaaeaahabaaaaaaaaaaaffffaaaafibiaaae
aahabaaaabaaaaaaffffaaaafibiaaaeaahabaaaacaaaaaaffffaaaagcbaaaad
dcbabaaaabaaaaaagcbaaaadhcbabaaaacaaaaaagcbaaaadhcbabaaaadaaaaaa
gfaaaaadpccabaaaaaaaaaaagiaaaaacadaaaaaabaaaaaahbcaabaaaaaaaaaaa
egbcbaaaadaaaaaaegbcbaaaadaaaaaaeeaaaaafbcaabaaaaaaaaaaaakaabaaa
aaaaaaaadcaaaaajhcaabaaaaaaaaaaaegbcbaaaadaaaaaaagaabaaaaaaaaaaa
egbcbaaaacaaaaaabaaaaaahicaabaaaaaaaaaaaegacbaaaaaaaaaaaegacbaaa
aaaaaaaaeeaaaaaficaabaaaaaaaaaaadkaabaaaaaaaaaaadiaaaaahhcaabaaa
aaaaaaaapgapbaaaaaaaaaaaegacbaaaaaaaaaaaefaaaaajpcaabaaaabaaaaaa
egbabaaaabaaaaaaeghobaaaacaaaaaaaagabaaaabaaaaaadcaaaaapdcaabaaa
abaaaaaahgapbaaaabaaaaaaaceaaaaaaaaaaaeaaaaaaaeaaaaaaaaaaaaaaaaa
aceaaaaaaaaaialpaaaaialpaaaaaaaaaaaaaaaadcaaaaakicaabaaaaaaaaaaa
akaabaiaebaaaaaaabaaaaaaakaabaaaabaaaaaaabeaaaaaaaaaiadpdcaaaaak
icaabaaaaaaaaaaabkaabaiaebaaaaaaabaaaaaabkaabaaaabaaaaaadkaabaaa
aaaaaaaaelaaaaafecaabaaaabaaaaaadkaabaaaaaaaaaaabaaaaaahbcaabaaa
aaaaaaaaegacbaaaabaaaaaaegacbaaaaaaaaaaabaaaaaahccaabaaaaaaaaaaa
egacbaaaabaaaaaaegbcbaaaacaaaaaadeaaaaakdcaabaaaaaaaaaaaegaabaaa
aaaaaaaaaceaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaacpaaaaafbcaabaaa
aaaaaaaaakaabaaaaaaaaaaadiaaaaaiecaabaaaaaaaaaaaakiacaaaaaaaaaaa
aeaaaaaaabeaaaaaaaaaaaecdiaaaaahbcaabaaaaaaaaaaaakaabaaaaaaaaaaa
ckaabaaaaaaaaaaabjaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaaefaaaaaj
pcaabaaaabaaaaaaegbabaaaabaaaaaaeghobaaaabaaaaaaaagabaaaacaaaaaa
efaaaaajpcaabaaaacaaaaaaegbabaaaabaaaaaaeghobaaaaaaaaaaaaagabaaa
aaaaaaaadiaaaaahhcaabaaaabaaaaaaegacbaaaabaaaaaapgapbaaaacaaaaaa
diaaaaaihcaabaaaacaaaaaaegacbaaaacaaaaaaegiccaaaaaaaaaaaadaaaaaa
diaaaaaihcaabaaaacaaaaaaegacbaaaacaaaaaaegiccaaaaaaaaaaaabaaaaaa
diaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaafgifcaaaaaaaaaaaaeaaaaaa
diaaaaahncaabaaaaaaaaaaaagaabaaaaaaaaaaaagajbaaaabaaaaaadiaaaaai
ncaabaaaaaaaaaaaagaobaaaaaaaaaaaagijcaaaaaaaaaaaabaaaaaadcaaaaaj
hcaabaaaaaaaaaaaegacbaaaacaaaaaafgafbaaaaaaaaaaaigadbaaaaaaaaaaa
aaaaaaahhccabaaaaaaaaaaaegacbaaaaaaaaaaaegacbaaaaaaaaaaadgaaaaaf
iccabaaaaaaaaaaaabeaaaaaaaaaaaaadoaaaaab"
}

SubProgram "gles " {
Keywords { "DIRECTIONAL" }
"!!GLES"
}

SubProgram "glesdesktop " {
Keywords { "DIRECTIONAL" }
"!!GLES"
}

SubProgram "flash " {
Keywords { "DIRECTIONAL" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
"agal_ps
c4 2.0 -1.0 1.0 0.0
c5 32.0 0.0 0.0 0.0
[bc]
ciaaaaaaaaaaapacaaaaaaoeaeaaaaaaacaaaaaaafaababb tex r0, v0, s2 <2d wrap linear point>
ciaaaaaaacaaapacaaaaaaoeaeaaaaaaaaaaaaaaafaababb tex r2, v0, s0 <2d wrap linear point>
ciaaaaaaadaaapacaaaaaaoeaeaaaaaaabaaaaaaafaababb tex r3, v0, s1 <2d wrap linear point>
aaaaaaaaaaaaabacaaaaaappacaaaaaaaaaaaaaaaaaaaaaa mov r0.x, r0.w
adaaaaaaafaaadacaaaaaafeacaaaaaaaeaaaaaaabaaaaaa mul r5.xy, r0.xyyy, c4.x
abaaaaaaafaaadacafaaaafeacaaaaaaaeaaaaffabaaaaaa add r5.xy, r5.xyyy, c4.y
adaaaaaaaaaaabacafaaaaffacaaaaaaafaaaaffacaaaaaa mul r0.x, r5.y, r5.y
bfaaaaaaabaaabacafaaaaaaacaaaaaaaaaaaaaaaaaaaaaa neg r1.x, r5.x
adaaaaaaabaaabacabaaaaaaacaaaaaaafaaaaaaacaaaaaa mul r1.x, r1.x, r5.x
acaaaaaaaaaaabacabaaaaaaacaaaaaaaaaaaaaaacaaaaaa sub r0.x, r1.x, r0.x
bcaaaaaaabaaabacacaaaaoeaeaaaaaaacaaaaoeaeaaaaaa dp3 r1.x, v2, v2
abaaaaaaaaaaabacaaaaaaaaacaaaaaaaeaaaakkabaaaaaa add r0.x, r0.x, c4.z
akaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r0.x, r0.x
afaaaaaaafaaaeacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rcp r5.z, r0.x
aaaaaaaaaaaaabacacaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0.x, c2
adaaaaaaacaaahacacaaaakeacaaaaaaabaaaaoeabaaaaaa mul r2.xyz, r2.xyzz, c1
akaaaaaaabaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r1.x, r1.x
aaaaaaaaaeaaahacabaaaaoeaeaaaaaaaaaaaaaaaaaaaaaa mov r4.xyz, v1
adaaaaaaagaaahacabaaaaaaacaaaaaaacaaaaoeaeaaaaaa mul r6.xyz, r1.x, v2
abaaaaaaaeaaahacagaaaakeacaaaaaaaeaaaakeacaaaaaa add r4.xyz, r6.xyzz, r4.xyzz
bcaaaaaaabaaabacaeaaaakeacaaaaaaaeaaaakeacaaaaaa dp3 r1.x, r4.xyzz, r4.xyzz
akaaaaaaabaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r1.x, r1.x
adaaaaaaabaaahacabaaaaaaacaaaaaaaeaaaakeacaaaaaa mul r1.xyz, r1.x, r4.xyzz
bcaaaaaaabaaabacafaaaakeacaaaaaaabaaaakeacaaaaaa dp3 r1.x, r5.xyzz, r1.xyzz
adaaaaaaaaaaabacafaaaaoeabaaaaaaaaaaaaaaacaaaaaa mul r0.x, c5, r0.x
ahaaaaaaabaaabacabaaaaaaacaaaaaaaeaaaappabaaaaaa max r1.x, r1.x, c4.w
alaaaaaaaeaaapacabaaaaaaacaaaaaaaaaaaaaaacaaaaaa pow r4, r1.x, r0.x
adaaaaaaaaaaahacacaaaappacaaaaaaadaaaakeacaaaaaa mul r0.xyz, r2.w, r3.xyzz
adaaaaaaabaaahacaaaaaakeacaaaaaaadaaaaaaabaaaaaa mul r1.xyz, r0.xyzz, c3.x
aaaaaaaaaaaaabacaeaaaaaaacaaaaaaaaaaaaaaaaaaaaaa mov r0.x, r4.x
adaaaaaaaaaaahacaaaaaaaaacaaaaaaabaaaakeacaaaaaa mul r0.xyz, r0.x, r1.xyzz
adaaaaaaabaaahacaaaaaakeacaaaaaaaaaaaaoeabaaaaaa mul r1.xyz, r0.xyzz, c0
bcaaaaaaaaaaabacafaaaakeacaaaaaaabaaaaoeaeaaaaaa dp3 r0.x, r5.xyzz, v1
ahaaaaaaaaaaabacaaaaaaaaacaaaaaaaeaaaappabaaaaaa max r0.x, r0.x, c4.w
adaaaaaaacaaahacacaaaakeacaaaaaaaaaaaaoeabaaaaaa mul r2.xyz, r2.xyzz, c0
adaaaaaaaaaaahacacaaaakeacaaaaaaaaaaaaaaacaaaaaa mul r0.xyz, r2.xyzz, r0.x
abaaaaaaaaaaahacaaaaaakeacaaaaaaabaaaakeacaaaaaa add r0.xyz, r0.xyzz, r1.xyzz
adaaaaaaaaaaahacaaaaaakeacaaaaaaaeaaaaaaabaaaaaa mul r0.xyz, r0.xyzz, c4.x
aaaaaaaaaaaaaiacaeaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0.w, c4
aaaaaaaaaaaaapadaaaaaaoeacaaaaaaaaaaaaaaaaaaaaaa mov o0, r0
"
}

SubProgram "d3d11_9x " {
Keywords { "DIRECTIONAL" }
ConstBuffer "$Globals" 96 // 72 used size, 7 vars
Vector 16 [_LightColor0] 4
Vector 48 [_Color] 4
Float 64 [_Shininess]
Float 68 [_Gloss]
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 0
SetTexture 1 [_SpecMap] 2D 2
SetTexture 2 [_BumpMap] 2D 1
// 30 instructions, 3 temp regs, 0 temp arrays:
// ALU 20 float, 0 int, 0 uint
// TEX 3 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0_level_9_3
eefiecedadjkeafdnpmpbgojbmpbkakhhdeogibjabaaaaaalmahaaaaaeaaaaaa
daaaaaaaneacaaaaaaahaaaaiiahaaaaebgpgodjjmacaaaajmacaaaaaaacpppp
feacaaaaeiaaaaaaacaadaaaaaaaeiaaaaaaeiaaadaaceaaaaaaeiaaaaaaaaaa
acababaaabacacaaaaaaabaaabaaaaaaaaaaaaaaaaaaadaaacaaabaaaaaaaaaa
abacppppfbaaaaafadaaapkaaaaaaaeaaaaaialpaaaaiadpaaaaaaaafbaaaaaf
aeaaapkaaaaaaaecaaaaaaaaaaaaaaaaaaaaaaaabpaaaaacaaaaaaiaaaaaadla
bpaaaaacaaaaaaiaabaachlabpaaaaacaaaaaaiaacaachlabpaaaaacaaaaaaja
aaaiapkabpaaaaacaaaaaajaabaiapkabpaaaaacaaaaaajaacaiapkaaiaaaaad
aaaaciiaacaaoelaacaaoelaahaaaaacaaaacbiaaaaappiaabaaaaacabaaahia
acaaoelaaeaaaaaeaaaachiaabaaoeiaaaaaaaiaabaaoelaceaaaaacabaachia
aaaaoeiaecaaaaadaaaacpiaaaaaoelaabaioekaaeaaaaaeaaaacdiaaaaaohia
adaaaakaadaaffkaaeaaaaaeaaaaciiaaaaaaaiaaaaaaaibadaakkkaaeaaaaae
aaaaciiaaaaaffiaaaaaffibaaaappiaahaaaaacaaaaciiaaaaappiaagaaaaac
aaaaceiaaaaappiaaiaaaaadaaaaciiaaaaaoeiaabaaoeiaaiaaaaadaaaacbia
aaaaoeiaabaaoelaalaaaaadabaacbiaaaaaaaiaadaappkaalaaaaadabaaacia
aaaappiaadaappkaabaaaaacaaaaabiaacaaaakaafaaaaadaaaaabiaaaaaaaia
aeaaaakacaaaaaadacaaaiiaabaaffiaaaaaaaiaecaaaaadaaaacpiaaaaaoela
aaaioekaecaaaaadadaacpiaaaaaoelaacaioekaafaaaaadabaacoiaaaaappia
adaajaiaafaaaaadaaaachiaaaaaoeiaabaaoekaafaaaaadaaaachiaaaaaoeia
aaaaoekaafaaaaadabaacoiaabaaoeiaacaaffkaafaaaaadabaacoiaabaaoeia
acaappiaafaaaaadabaacoiaabaaoeiaaaaajakaaeaaaaaeaaaachiaaaaaoeia
abaaaaiaabaapjiaacaaaaadaaaachiaaaaaoeiaaaaaoeiaabaaaaacaaaaciia
adaappkaabaaaaacaaaicpiaaaaaoeiappppaaaafdeieefcceaeaaaaeaaaaaaa
ajabaaaafjaaaaaeegiocaaaaaaaaaaaafaaaaaafkaaaaadaagabaaaaaaaaaaa
fkaaaaadaagabaaaabaaaaaafkaaaaadaagabaaaacaaaaaafibiaaaeaahabaaa
aaaaaaaaffffaaaafibiaaaeaahabaaaabaaaaaaffffaaaafibiaaaeaahabaaa
acaaaaaaffffaaaagcbaaaaddcbabaaaabaaaaaagcbaaaadhcbabaaaacaaaaaa
gcbaaaadhcbabaaaadaaaaaagfaaaaadpccabaaaaaaaaaaagiaaaaacadaaaaaa
baaaaaahbcaabaaaaaaaaaaaegbcbaaaadaaaaaaegbcbaaaadaaaaaaeeaaaaaf
bcaabaaaaaaaaaaaakaabaaaaaaaaaaadcaaaaajhcaabaaaaaaaaaaaegbcbaaa
adaaaaaaagaabaaaaaaaaaaaegbcbaaaacaaaaaabaaaaaahicaabaaaaaaaaaaa
egacbaaaaaaaaaaaegacbaaaaaaaaaaaeeaaaaaficaabaaaaaaaaaaadkaabaaa
aaaaaaaadiaaaaahhcaabaaaaaaaaaaapgapbaaaaaaaaaaaegacbaaaaaaaaaaa
efaaaaajpcaabaaaabaaaaaaegbabaaaabaaaaaaeghobaaaacaaaaaaaagabaaa
abaaaaaadcaaaaapdcaabaaaabaaaaaahgapbaaaabaaaaaaaceaaaaaaaaaaaea
aaaaaaeaaaaaaaaaaaaaaaaaaceaaaaaaaaaialpaaaaialpaaaaaaaaaaaaaaaa
dcaaaaakicaabaaaaaaaaaaaakaabaiaebaaaaaaabaaaaaaakaabaaaabaaaaaa
abeaaaaaaaaaiadpdcaaaaakicaabaaaaaaaaaaabkaabaiaebaaaaaaabaaaaaa
bkaabaaaabaaaaaadkaabaaaaaaaaaaaelaaaaafecaabaaaabaaaaaadkaabaaa
aaaaaaaabaaaaaahbcaabaaaaaaaaaaaegacbaaaabaaaaaaegacbaaaaaaaaaaa
baaaaaahccaabaaaaaaaaaaaegacbaaaabaaaaaaegbcbaaaacaaaaaadeaaaaak
dcaabaaaaaaaaaaaegaabaaaaaaaaaaaaceaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaacpaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaadiaaaaaiecaabaaa
aaaaaaaaakiacaaaaaaaaaaaaeaaaaaaabeaaaaaaaaaaaecdiaaaaahbcaabaaa
aaaaaaaaakaabaaaaaaaaaaackaabaaaaaaaaaaabjaaaaafbcaabaaaaaaaaaaa
akaabaaaaaaaaaaaefaaaaajpcaabaaaabaaaaaaegbabaaaabaaaaaaeghobaaa
abaaaaaaaagabaaaacaaaaaaefaaaaajpcaabaaaacaaaaaaegbabaaaabaaaaaa
eghobaaaaaaaaaaaaagabaaaaaaaaaaadiaaaaahhcaabaaaabaaaaaaegacbaaa
abaaaaaapgapbaaaacaaaaaadiaaaaaihcaabaaaacaaaaaaegacbaaaacaaaaaa
egiccaaaaaaaaaaaadaaaaaadiaaaaaihcaabaaaacaaaaaaegacbaaaacaaaaaa
egiccaaaaaaaaaaaabaaaaaadiaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaa
fgifcaaaaaaaaaaaaeaaaaaadiaaaaahncaabaaaaaaaaaaaagaabaaaaaaaaaaa
agajbaaaabaaaaaadiaaaaaincaabaaaaaaaaaaaagaobaaaaaaaaaaaagijcaaa
aaaaaaaaabaaaaaadcaaaaajhcaabaaaaaaaaaaaegacbaaaacaaaaaafgafbaaa
aaaaaaaaigadbaaaaaaaaaaaaaaaaaahhccabaaaaaaaaaaaegacbaaaaaaaaaaa
egacbaaaaaaaaaaadgaaaaaficcabaaaaaaaaaaaabeaaaaaaaaaaaaadoaaaaab
ejfdeheoiaaaaaaaaeaaaaaaaiaaaaaagiaaaaaaaaaaaaaaabaaaaaaadaaaaaa
aaaaaaaaapaaaaaaheaaaaaaaaaaaaaaaaaaaaaaadaaaaaaabaaaaaaadadaaaa
heaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaaahahaaaaheaaaaaaacaaaaaa
aaaaaaaaadaaaaaaadaaaaaaahahaaaafdfgfpfaepfdejfeejepeoaafeeffied
epepfceeaaklklklepfdeheocmaaaaaaabaaaaaaaiaaaaaacaaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaaaaaaaaaapaaaaaafdfgfpfegbhcghgfheaaklkl"
}

SubProgram "opengl " {
Keywords { "SPOT" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_LightTexture0] 2D
SetTexture 4 [_LightTextureB0] 2D
"!!ARBfp1.0
OPTION ARB_precision_hint_fastest;
# 43 ALU, 5 TEX
PARAM c[6] = { program.local[0..3],
		{ 0, 0.5, 2, 1 },
		{ 32 } };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
TEMP R4;
TEX R2, fragment.texcoord[0], texture[0], 2D;
TEX R3.yw, fragment.texcoord[0], texture[2], 2D;
RCP R0.x, fragment.texcoord[3].w;
MAD R1.xy, fragment.texcoord[3], R0.x, c[4].y;
DP3 R1.z, fragment.texcoord[3], fragment.texcoord[3];
DP3 R3.x, fragment.texcoord[1], fragment.texcoord[1];
MUL R2.xyz, R2, c[1];
RSQ R3.x, R3.x;
MUL R2.xyz, R2, c[0];
MOV result.color.w, c[4].x;
TEX R0.w, R1, texture[3], 2D;
TEX R0.xyz, fragment.texcoord[0], texture[1], 2D;
TEX R1.w, R1.z, texture[4], 2D;
MAD R1.xy, R3.wyzw, c[4].z, -c[4].w;
MUL R1.z, R1.y, R1.y;
MAD R1.z, -R1.x, R1.x, -R1;
DP3 R3.w, fragment.texcoord[2], fragment.texcoord[2];
ADD R1.z, R1, c[4].w;
RSQ R1.z, R1.z;
MUL R0.xyz, R2.w, R0;
RCP R1.z, R1.z;
MUL R3.xyz, R3.x, fragment.texcoord[1];
RSQ R3.w, R3.w;
MAD R4.xyz, R3.w, fragment.texcoord[2], R3;
DP3 R3.w, R4, R4;
RSQ R3.w, R3.w;
DP3 R3.x, R1, R3;
MUL R4.xyz, R3.w, R4;
DP3 R1.y, R1, R4;
MAX R1.z, R1.y, c[4].x;
MOV R1.y, c[5].x;
MUL R1.y, R1, c[2].x;
POW R1.y, R1.z, R1.y;
MUL R0.xyz, R0, c[3].x;
MUL R0.xyz, R1.y, R0;
MUL R0.xyz, R0, c[0];
MAX R1.x, R3, c[4];
MAD R1.xyz, R2, R1.x, R0;
SLT R0.x, c[4], fragment.texcoord[3].z;
MUL R0.x, R0, R0.w;
MUL R0.x, R0, R1.w;
MUL R0.xyz, R0.x, R1;
MUL result.color.xyz, R0, c[4].z;
END
# 43 instructions, 5 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "SPOT" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_LightTexture0] 2D
SetTexture 4 [_LightTextureB0] 2D
"ps_2_0
; 45 ALU, 5 TEX
dcl_2d s0
dcl_2d s1
dcl_2d s2
dcl_2d s3
dcl_2d s4
def c4, 0.00000000, 1.00000000, 0.50000000, 32.00000000
def c5, 2.00000000, -1.00000000, 0, 0
dcl t0.xy
dcl t1.xyz
dcl t2.xyz
dcl t3
texld r3, t0, s1
rcp r1.x, t3.w
mad r2.xy, t3, r1.x, c4.z
dp3 r0.x, t3, t3
mov r1.xy, r0.x
texld r7, r1, s4
texld r0, r2, s3
texld r1, t0, s2
texld r2, t0, s0
dp3_pp r1.x, t1, t1
rsq_pp r4.x, r1.x
dp3_pp r1.x, t2, t2
mul_pp r2.xyz, r2, c1
mov r0.y, r1
mov r0.x, r1.w
mad_pp r5.xy, r0, c5.x, c5.y
mul_pp r0.x, r5.y, r5.y
mad_pp r0.x, -r5, r5, -r0
add_pp r0.x, r0, c4.y
rsq_pp r0.x, r0.x
rcp_pp r5.z, r0.x
mov_pp r0.x, c2
mul_pp r4.xyz, r4.x, t1
rsq_pp r1.x, r1.x
mad_pp r6.xyz, r1.x, t2, r4
dp3_pp r1.x, r6, r6
rsq_pp r1.x, r1.x
mul_pp r1.xyz, r1.x, r6
dp3_pp r1.x, r5, r1
mul_pp r0.x, c4.w, r0
max_pp r1.x, r1, c4
pow r6.x, r1.x, r0.x
mul_pp r0.xyz, r2.w, r3
mul_pp r1.xyz, r0, c3.x
mov r0.x, r6.x
mul r0.xyz, r0.x, r1
mul_pp r1.xyz, r0, c0
dp3_pp r0.x, r5, r4
max_pp r0.x, r0, c4
mul_pp r2.xyz, r2, c0
mad_pp r1.xyz, r2, r0.x, r1
cmp r0.x, -t3.z, c4, c4.y
mul_pp r0.x, r0, r0.w
mul_pp r0.x, r0, r7
mul_pp r0.xyz, r0.x, r1
mul_pp r0.xyz, r0, c5.x
mov_pp r0.w, c4.x
mov_pp oC0, r0
"
}

SubProgram "xbox360 " {
Keywords { "SPOT" }
Vector 1 [_Color]
Float 3 [_Gloss]
Vector 0 [_LightColor0]
Float 2 [_Shininess]
SetTexture 0 [_LightTexture0] 2D
SetTexture 1 [_LightTextureB0] 2D
SetTexture 2 [_MainTex] 2D
SetTexture 3 [_BumpMap] 2D
SetTexture 4 [_SpecMap] 2D
// Shader Timing Estimate, in Cycles/64 pixel vector:
// ALU: 30.67 (23 instructions), vertex: 0, texture: 20,
//   sequencer: 14, interpolator: 16;    9 GPRs, 21 threads,
// Performance (if enough threads): ~30 cycles per vector
// * Texture cycle estimates are assuming an 8bit/component texture with no
//     aniso or trilinear filtering.

"ps_360
backbbaaaaaaabpiaaaaabmmaaaaaaaaaaaaaaceaaaaabkaaaaaabmiaaaaaaaa
aaaaaaaaaaaaabhiaaaaaabmaaaaabgmppppadaaaaaaaaajaaaaaabmaaaaaaaa
aaaaabgfaaaaaanaaaadaaadaaabaaaaaaaaaanmaaaaaaaaaaaaaaomaaacaaab
aaabaaaaaaaaaapeaaaaaaaaaaaaabaeaaacaaadaaabaaaaaaaaabamaaaaaaaa
aaaaabbmaaacaaaaaaabaaaaaaaaaapeaaaaaaaaaaaaabcjaaadaaaaaaabaaaa
aaaaaanmaaaaaaaaaaaaabdiaaadaaabaaabaaaaaaaaaanmaaaaaaaaaaaaabei
aaadaaacaaabaaaaaaaaaanmaaaaaaaaaaaaabfbaaacaaacaaabaaaaaaaaabam
aaaaaaaaaaaaabfmaaadaaaeaaabaaaaaaaaaanmaaaaaaaafpechfgnhaengbha
aaklklklaaaeaaamaaabaaabaaabaaaaaaaaaaaafpedgpgmgphcaaklaaabaaad
aaabaaaeaaabaaaaaaaaaaaafpehgmgphdhdaaklaaaaaaadaaabaaabaaabaaaa
aaaaaaaafpemgjghgiheedgpgmgphcdaaafpemgjghgihefegfhihehfhcgfdaaa
fpemgjghgihefegfhihehfhcgfecdaaafpengbgjgofegfhiaafpfdgigjgogjgo
gfhdhdaafpfdhagfgdengbhaaahahdfpddfpdaaadccodacodcdadddfddcodaaa
aaaaaaaaaaaaaaabaaaaaaaaaaaaaaaaaaaaaabeabpmaabaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaeaaaaaabimbaaaaiaaaaaaaaaeaaaaaaaaaaaadaie
aaapaaapaaaaaaabaaaadafaaaaahbfbaaaahcfcaaaapdfdaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaadpaaaaaaaaaaaaaa
dpiaaaaaecaaaaaalpiaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaffagaaebaakbcaa
bcaaaaabaaaaaaaagaalmeaabcaaaaaaaaaagabbgabhbcaabcaaaaaaaaaadabn
aaaaccaaaaaaaaaaemieaaaaaaloloblpaadadadmiadaaaeaabllagmmlaaadpo
baeigaabbpbppoiiaaaaeaaabadiiaabbpbppompaaaaeaaakibiaaabbpbppodp
aaaaeaaabaaieaibbpbpppplaaaaeaaabacihaabbpbppgiiaaaaeaaamiacaaaa
aaloloaapaababaaficbaaaaaalololbpaacaciafibhaaafaalbmagmobaaabia
miaoaaaeaagmpmpmolaaacafcabcaaaaaamdmdmgpaaeaeadficoaaadaapmpmlb
kbahabiamiapaaaeaaaalaaaobaeaaaamiabaaaiaagmmgaaobaeaaaamialaaaa
aagcgcaaoaaiaiaamiagaaabaalmgmaakaaappaamiaeaaaaaelclcmgnbababpo
kaihabacaablmamgobahagiamiabaaabaalomdaapaafabaabeacaaabaamdmdgm
naaeabacambgadabaalmlbblicabpopoeaboabadaaabpmmgkbadaaibmiapaaab
aaaalaaaobadabaadiehaaacaamagmgmkbacadabmiahaaacaamamgaaobacaaaa
miahaaaaaamamabfklacaaabmiahmaaaaablmaaaobaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "SPOT" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_LightTexture0] 2D
SetTexture 4 [_LightTextureB0] 2D
"sce_fp_rsx // 53 instructions using 4 registers
[Configuration]
24
ffffffff0003c020000ffff1000000000000840004000000
[Offsets]
4
_LightColor0 2 0
0000031000000240
_Color 1 0
00000200
_Shininess 1 0
00000160
_Gloss 1 0
00000110
[Microcode]
848
94001704c8011c9dc8000001c8003fe106880440ce001c9d00020000aa020000
000040000000bf800000000000000000fe020100c8011c9dc8000001c8003fe1
08880240ab101c9cab100000c800000102820440c9101c9fc910000155100003
18043a0080041c9cfe040001c80000011080034001041c9c00020000c8000001
00003f8000000000000000000000000010000500c8041c9dc8040001c8000001
9e061700c8011c9dc8000001c8003fe1ae803940c8011c9dc8000029c800bfe1
1e7e7d00c8001c9dc8000001c8000001060203005c081c9d00020000c8000001
00003f0000000000000000000000000008883b40ff003c9dff000001c8000001
108a0240c80c1c9d00020000c800000100000000000000000000000000000000
10021706c8041c9dc8000001c8000001ce843940c8011c9dc8000029c800bfe1
088a0540c9101c9dc9000001c80000011084014000021c9cc8000001c8000001
000000000000000000000000000000000e000340c9001c9dc9080001c8000001
02021708fe001c9dc8000001c80000010e803940c8001c9dc8000029c8000001
08840540c9101c9dc9000001c800000110800240c9081c9d00020000c8000001
000042000000000000000000000000000200090055081c9d00020000c8000001
000000000000000000000000000000000e820240c80c1c9dc8020001c8000001
0000000000000000000000000000000002001d00c8001c9dc8000001c8000001
8e041702c8011c9dc8000001c8003fe10e820240c9041c9dc8020001c8000001
0000000000000000000000000000000002000200c8001c9dff000001c8000001
1080090055141c9d00020000c800000100000000000000000000000000000000
0e820240c9041c9dff000001c800000104021c00c8001c9dc8000001c8000001
0e800240ff141c9dc8080001c800000110800d0054041c9d00020000c8000001
000000000000000000000000000000000e800200aa041c9cc9000001c8000001
10800240c9001c9dc8040001c800000110800240c9001c9d00040000c8000001
0e800440c9001c9dc8020001c904000100000000000000000000000000000000
0e800240ff001c9dc9001001c80000011081014000021c9cc8000001c8000001
00000000000000000000000000000000
"
}

SubProgram "d3d11 " {
Keywords { "SPOT" }
ConstBuffer "$Globals" 160 // 136 used size, 8 vars
Vector 16 [_LightColor0] 4
Vector 112 [_Color] 4
Float 128 [_Shininess]
Float 132 [_Gloss]
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 2
SetTexture 1 [_SpecMap] 2D 4
SetTexture 2 [_BumpMap] 2D 3
SetTexture 3 [_LightTexture0] 2D 0
SetTexture 4 [_LightTextureB0] 2D 1
// 42 instructions, 3 temp regs, 0 temp arrays:
// ALU 29 float, 0 int, 1 uint
// TEX 5 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0
eefiecedkebbhheeibdhmnehlkhipfljgfcdafhfabaaaaaaneagaaaaadaaaaaa
cmaaaaaammaaaaaaaaabaaaaejfdeheojiaaaaaaafaaaaaaaiaaaaaaiaaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaadadaaaaimaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaahahaaaaimaaaaaa
adaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapapaaaafdfgfpfaepfdejfeejepeoaa
feeffiedepepfceeaaklklklepfdeheocmaaaaaaabaaaaaaaiaaaaaacaaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaafdfgfpfegbhcghgfheaaklkl
fdeieefcmmafaaaaeaaaaaaahdabaaaafjaaaaaeegiocaaaaaaaaaaaajaaaaaa
fkaaaaadaagabaaaaaaaaaaafkaaaaadaagabaaaabaaaaaafkaaaaadaagabaaa
acaaaaaafkaaaaadaagabaaaadaaaaaafkaaaaadaagabaaaaeaaaaaafibiaaae
aahabaaaaaaaaaaaffffaaaafibiaaaeaahabaaaabaaaaaaffffaaaafibiaaae
aahabaaaacaaaaaaffffaaaafibiaaaeaahabaaaadaaaaaaffffaaaafibiaaae
aahabaaaaeaaaaaaffffaaaagcbaaaaddcbabaaaabaaaaaagcbaaaadhcbabaaa
acaaaaaagcbaaaadhcbabaaaadaaaaaagcbaaaadpcbabaaaaeaaaaaagfaaaaad
pccabaaaaaaaaaaagiaaaaacadaaaaaabaaaaaahbcaabaaaaaaaaaaaegbcbaaa
adaaaaaaegbcbaaaadaaaaaaeeaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaa
baaaaaahccaabaaaaaaaaaaaegbcbaaaacaaaaaaegbcbaaaacaaaaaaeeaaaaaf
ccaabaaaaaaaaaaabkaabaaaaaaaaaaadiaaaaahocaabaaaaaaaaaaafgafbaaa
aaaaaaaaagbjbaaaacaaaaaadcaaaaajhcaabaaaabaaaaaaegbcbaaaadaaaaaa
agaabaaaaaaaaaaajgahbaaaaaaaaaaabaaaaaahbcaabaaaaaaaaaaaegacbaaa
abaaaaaaegacbaaaabaaaaaaeeaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaa
diaaaaahhcaabaaaabaaaaaaagaabaaaaaaaaaaaegacbaaaabaaaaaaefaaaaaj
pcaabaaaacaaaaaaegbabaaaabaaaaaaeghobaaaacaaaaaaaagabaaaadaaaaaa
dcaaaaapdcaabaaaacaaaaaahgapbaaaacaaaaaaaceaaaaaaaaaaaeaaaaaaaea
aaaaaaaaaaaaaaaaaceaaaaaaaaaialpaaaaialpaaaaaaaaaaaaaaaadcaaaaak
bcaabaaaaaaaaaaaakaabaiaebaaaaaaacaaaaaaakaabaaaacaaaaaaabeaaaaa
aaaaiadpdcaaaaakbcaabaaaaaaaaaaabkaabaiaebaaaaaaacaaaaaabkaabaaa
acaaaaaaakaabaaaaaaaaaaaelaaaaafecaabaaaacaaaaaaakaabaaaaaaaaaaa
baaaaaahbcaabaaaaaaaaaaaegacbaaaacaaaaaaegacbaaaabaaaaaabaaaaaah
ccaabaaaaaaaaaaaegacbaaaacaaaaaajgahbaaaaaaaaaaadeaaaaakdcaabaaa
aaaaaaaaegaabaaaaaaaaaaaaceaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
cpaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaadiaaaaaiecaabaaaaaaaaaaa
akiacaaaaaaaaaaaaiaaaaaaabeaaaaaaaaaaaecdiaaaaahbcaabaaaaaaaaaaa
akaabaaaaaaaaaaackaabaaaaaaaaaaabjaaaaafbcaabaaaaaaaaaaaakaabaaa
aaaaaaaaefaaaaajpcaabaaaabaaaaaaegbabaaaabaaaaaaeghobaaaabaaaaaa
aagabaaaaeaaaaaaefaaaaajpcaabaaaacaaaaaaegbabaaaabaaaaaaeghobaaa
aaaaaaaaaagabaaaacaaaaaadiaaaaahhcaabaaaabaaaaaaegacbaaaabaaaaaa
pgapbaaaacaaaaaadiaaaaaihcaabaaaacaaaaaaegacbaaaacaaaaaaegiccaaa
aaaaaaaaahaaaaaadiaaaaaihcaabaaaacaaaaaaegacbaaaacaaaaaaegiccaaa
aaaaaaaaabaaaaaadiaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaafgifcaaa
aaaaaaaaaiaaaaaadiaaaaahncaabaaaaaaaaaaaagaabaaaaaaaaaaaagajbaaa
abaaaaaadiaaaaaincaabaaaaaaaaaaaagaobaaaaaaaaaaaagijcaaaaaaaaaaa
abaaaaaadcaaaaajhcaabaaaaaaaaaaaegacbaaaacaaaaaafgafbaaaaaaaaaaa
igadbaaaaaaaaaaaaoaaaaahdcaabaaaabaaaaaaegbabaaaaeaaaaaapgbpbaaa
aeaaaaaaaaaaaaakdcaabaaaabaaaaaaegaabaaaabaaaaaaaceaaaaaaaaaaadp
aaaaaadpaaaaaaaaaaaaaaaaefaaaaajpcaabaaaabaaaaaaegaabaaaabaaaaaa
eghobaaaadaaaaaaaagabaaaaaaaaaaadbaaaaahicaabaaaaaaaaaaaabeaaaaa
aaaaaaaackbabaaaaeaaaaaaabaaaaahicaabaaaaaaaaaaadkaabaaaaaaaaaaa
abeaaaaaaaaaiadpdiaaaaahicaabaaaaaaaaaaadkaabaaaabaaaaaadkaabaaa
aaaaaaaabaaaaaahbcaabaaaabaaaaaaegbcbaaaaeaaaaaaegbcbaaaaeaaaaaa
efaaaaajpcaabaaaabaaaaaaagaabaaaabaaaaaaeghobaaaaeaaaaaaaagabaaa
abaaaaaaapaaaaahicaabaaaaaaaaaaapgapbaaaaaaaaaaaagaabaaaabaaaaaa
diaaaaahhccabaaaaaaaaaaapgapbaaaaaaaaaaaegacbaaaaaaaaaaadgaaaaaf
iccabaaaaaaaaaaaabeaaaaaaaaaaaaadoaaaaab"
}

SubProgram "gles " {
Keywords { "SPOT" }
"!!GLES"
}

SubProgram "glesdesktop " {
Keywords { "SPOT" }
"!!GLES"
}

SubProgram "flash " {
Keywords { "SPOT" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_LightTexture0] 2D
SetTexture 4 [_LightTextureB0] 2D
"agal_ps
c4 0.0 1.0 0.5 32.0
c5 2.0 -1.0 0.0 0.0
[bc]
ciaaaaaaaeaaapacaaaaaaoeaeaaaaaaacaaaaaaafaababb tex r4, v0, s2 <2d wrap linear point>
ciaaaaaaacaaapacaaaaaaoeaeaaaaaaaaaaaaaaafaababb tex r2, v0, s0 <2d wrap linear point>
ciaaaaaaadaaapacaaaaaaoeaeaaaaaaabaaaaaaafaababb tex r3, v0, s1 <2d wrap linear point>
afaaaaaaabaaabacadaaaappaeaaaaaaaaaaaaaaaaaaaaaa rcp r1.x, v3.w
bcaaaaaaaaaaabacadaaaaoeaeaaaaaaadaaaaoeaeaaaaaa dp3 r0.x, v3, v3
adaaaaaaabaaadacadaaaaoeaeaaaaaaabaaaaaaacaaaaaa mul r1.xy, v3, r1.x
abaaaaaaabaaadacabaaaafeacaaaaaaaeaaaakkabaaaaaa add r1.xy, r1.xyyy, c4.z
aaaaaaaaaaaaadacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa mov r0.xy, r0.x
adaaaaaaacaaahacacaaaakeacaaaaaaabaaaaoeabaaaaaa mul r2.xyz, r2.xyzz, c1
adaaaaaaacaaahacacaaaakeacaaaaaaaaaaaaoeabaaaaaa mul r2.xyz, r2.xyzz, c0
ciaaaaaaaaaaapacaaaaaafeacaaaaaaaeaaaaaaafaababb tex r0, r0.xyyy, s4 <2d wrap linear point>
ciaaaaaaabaaapacabaaaafeacaaaaaaadaaaaaaafaababb tex r1, r1.xyyy, s3 <2d wrap linear point>
bcaaaaaaabaaabacabaaaaoeaeaaaaaaabaaaaoeaeaaaaaa dp3 r1.x, v1, v1
akaaaaaaaeaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r4.x, r1.x
aaaaaaaaaaaaacacaeaaaaffacaaaaaaaaaaaaaaaaaaaaaa mov r0.y, r4.y
aaaaaaaaaaaaabacaeaaaappacaaaaaaaaaaaaaaaaaaaaaa mov r0.x, r4.w
adaaaaaaafaaadacaaaaaafeacaaaaaaafaaaaaaabaaaaaa mul r5.xy, r0.xyyy, c5.x
abaaaaaaafaaadacafaaaafeacaaaaaaafaaaaffabaaaaaa add r5.xy, r5.xyyy, c5.y
adaaaaaaaaaaabacafaaaaffacaaaaaaafaaaaffacaaaaaa mul r0.x, r5.y, r5.y
bfaaaaaaadaaaiacafaaaaaaacaaaaaaaaaaaaaaaaaaaaaa neg r3.w, r5.x
adaaaaaaadaaaiacadaaaappacaaaaaaafaaaaaaacaaaaaa mul r3.w, r3.w, r5.x
acaaaaaaaaaaabacadaaaappacaaaaaaaaaaaaaaacaaaaaa sub r0.x, r3.w, r0.x
bcaaaaaaabaaabacacaaaaoeaeaaaaaaacaaaaoeaeaaaaaa dp3 r1.x, v2, v2
abaaaaaaaaaaabacaaaaaaaaacaaaaaaaeaaaaffabaaaaaa add r0.x, r0.x, c4.y
akaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r0.x, r0.x
afaaaaaaafaaaeacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rcp r5.z, r0.x
aaaaaaaaaaaaabacacaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0.x, c2
adaaaaaaaeaaahacaeaaaaaaacaaaaaaabaaaaoeaeaaaaaa mul r4.xyz, r4.x, v1
akaaaaaaabaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r1.x, r1.x
adaaaaaaagaaahacabaaaaaaacaaaaaaacaaaaoeaeaaaaaa mul r6.xyz, r1.x, v2
abaaaaaaagaaahacagaaaakeacaaaaaaaeaaaakeacaaaaaa add r6.xyz, r6.xyzz, r4.xyzz
bcaaaaaaabaaabacagaaaakeacaaaaaaagaaaakeacaaaaaa dp3 r1.x, r6.xyzz, r6.xyzz
akaaaaaaabaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r1.x, r1.x
adaaaaaaabaaahacabaaaaaaacaaaaaaagaaaakeacaaaaaa mul r1.xyz, r1.x, r6.xyzz
bcaaaaaaabaaabacafaaaakeacaaaaaaabaaaakeacaaaaaa dp3 r1.x, r5.xyzz, r1.xyzz
adaaaaaaaaaaabacaeaaaappabaaaaaaaaaaaaaaacaaaaaa mul r0.x, c4.w, r0.x
ahaaaaaaabaaabacabaaaaaaacaaaaaaaeaaaaoeabaaaaaa max r1.x, r1.x, c4
alaaaaaaagaaapacabaaaaaaacaaaaaaaaaaaaaaacaaaaaa pow r6, r1.x, r0.x
adaaaaaaaaaaahacacaaaappacaaaaaaadaaaakeacaaaaaa mul r0.xyz, r2.w, r3.xyzz
adaaaaaaabaaahacaaaaaakeacaaaaaaadaaaaaaabaaaaaa mul r1.xyz, r0.xyzz, c3.x
aaaaaaaaaaaaabacagaaaaaaacaaaaaaaaaaaaaaaaaaaaaa mov r0.x, r6.x
adaaaaaaaaaaahacaaaaaaaaacaaaaaaabaaaakeacaaaaaa mul r0.xyz, r0.x, r1.xyzz
adaaaaaaabaaahacaaaaaakeacaaaaaaaaaaaaoeabaaaaaa mul r1.xyz, r0.xyzz, c0
bcaaaaaaaaaaabacafaaaakeacaaaaaaaeaaaakeacaaaaaa dp3 r0.x, r5.xyzz, r4.xyzz
ahaaaaaaaaaaabacaaaaaaaaacaaaaaaaeaaaaoeabaaaaaa max r0.x, r0.x, c4
adaaaaaaacaaahacacaaaakeacaaaaaaaaaaaaaaacaaaaaa mul r2.xyz, r2.xyzz, r0.x
abaaaaaaabaaahacacaaaakeacaaaaaaabaaaakeacaaaaaa add r1.xyz, r2.xyzz, r1.xyzz
bfaaaaaaacaaaeacadaaaakkaeaaaaaaaaaaaaaaaaaaaaaa neg r2.z, v3.z
ckaaaaaaaaaaabacacaaaakkacaaaaaaafaaaakkabaaaaaa slt r0.x, r2.z, c5.z
adaaaaaaaaaaabacaaaaaaaaacaaaaaaabaaaappacaaaaaa mul r0.x, r0.x, r1.w
adaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaappacaaaaaa mul r0.x, r0.x, r0.w
adaaaaaaaaaaahacaaaaaaaaacaaaaaaabaaaakeacaaaaaa mul r0.xyz, r0.x, r1.xyzz
adaaaaaaaaaaahacaaaaaakeacaaaaaaafaaaaaaabaaaaaa mul r0.xyz, r0.xyzz, c5.x
aaaaaaaaaaaaaiacaeaaaaaaabaaaaaaaaaaaaaaaaaaaaaa mov r0.w, c4.x
aaaaaaaaaaaaapadaaaaaaoeacaaaaaaaaaaaaaaaaaaaaaa mov o0, r0
"
}

SubProgram "d3d11_9x " {
Keywords { "SPOT" }
ConstBuffer "$Globals" 160 // 136 used size, 8 vars
Vector 16 [_LightColor0] 4
Vector 112 [_Color] 4
Float 128 [_Shininess]
Float 132 [_Gloss]
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 2
SetTexture 1 [_SpecMap] 2D 4
SetTexture 2 [_BumpMap] 2D 3
SetTexture 3 [_LightTexture0] 2D 0
SetTexture 4 [_LightTextureB0] 2D 1
// 42 instructions, 3 temp regs, 0 temp arrays:
// ALU 29 float, 0 int, 1 uint
// TEX 5 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0_level_9_3
eefiecedaemmnkoegcmdideilpgfggpdlggappamabaaaaaacmakaaaaaeaaaaaa
daaaaaaaieadaaaafiajaaaapiajaaaaebgpgodjemadaaaaemadaaaaaaacpppp
pmacaaaafaaaaaaaacaadiaaaaaafaaaaaaafaaaafaaceaaaaaafaaaadaaaaaa
aeababaaaaacacaaacadadaaabaeaeaaaaaaabaaabaaaaaaaaaaaaaaaaaaahaa
acaaabaaaaaaaaaaabacppppfbaaaaafadaaapkaaaaaaaeaaaaaialpaaaaiadp
aaaaaadpfbaaaaafaeaaapkaaaaaaaaaaaaaaaecaaaaaaaaaaaaaaaabpaaaaac
aaaaaaiaaaaaadlabpaaaaacaaaaaaiaabaachlabpaaaaacaaaaaaiaacaachla
bpaaaaacaaaaaaiaadaaaplabpaaaaacaaaaaajaaaaiapkabpaaaaacaaaaaaja
abaiapkabpaaaaacaaaaaajaacaiapkabpaaaaacaaaaaajaadaiapkabpaaaaac
aaaaaajaaeaiapkaaiaaaaadaaaaciiaacaaoelaacaaoelaahaaaaacaaaacbia
aaaappiaceaaaaacabaachiaabaaoelaaeaaaaaeaaaachiaacaaoelaaaaaaaia
abaaoeiaceaaaaacacaachiaaaaaoeiaagaaaaacabaaaiiaadaapplaaeaaaaae
aaaaadiaadaaoelaabaappiaadaappkaecaaaaadadaacpiaaaaaoelaadaioeka
ecaaaaadaaaacpiaaaaaoeiaaaaioekaaeaaaaaeaaaacdiaadaaohiaadaaaaka
adaaffkaaeaaaaaeabaaciiaaaaaaaiaaaaaaaibadaakkkaaeaaaaaeabaaciia
aaaaffiaaaaaffibabaappiaahaaaaacabaaciiaabaappiaagaaaaacaaaaceia
abaappiaaiaaaaadabaaciiaaaaaoeiaacaaoeiaaiaaaaadaaaacbiaaaaaoeia
abaaoeiaalaaaaadabaacbiaaaaaaaiaaeaaaakaalaaaaadaaaaabiaabaappia
aeaaaakaabaaaaacaaaaaciaaeaaffkaafaaaaadaaaaaciaaaaaffiaacaaaaka
caaaaaadabaaaciaaaaaaaiaaaaaffiaecaaaaadacaacpiaaaaaoelaacaioeka
ecaaaaadadaacpiaaaaaoelaaeaioekaafaaaaadaaaachiaacaappiaadaaoeia
afaaaaadacaachiaacaaoeiaabaaoekaafaaaaadacaachiaacaaoeiaaaaaoeka
afaaaaadaaaachiaaaaaoeiaacaaffkaafaaaaadaaaachiaaaaaoeiaabaaffia
afaaaaadaaaachiaaaaaoeiaaaaaoekaaeaaaaaeaaaachiaacaaoeiaabaaaaia
aaaaoeiaaiaaaaadabaaadiaadaaoelaadaaoelaecaaaaadabaacpiaabaaoeia
abaioekaafaaaaadaaaaciiaaaaappiaabaaaaiafiaaaaaeaaaaciiaadaakklb
aeaaaakaaaaappiaacaaaaadaaaaciiaaaaappiaaaaappiaafaaaaadaaaachia
aaaappiaaaaaoeiaabaaaaacaaaaciiaaeaaaakaabaaaaacaaaicpiaaaaaoeia
ppppaaaafdeieefcmmafaaaaeaaaaaaahdabaaaafjaaaaaeegiocaaaaaaaaaaa
ajaaaaaafkaaaaadaagabaaaaaaaaaaafkaaaaadaagabaaaabaaaaaafkaaaaad
aagabaaaacaaaaaafkaaaaadaagabaaaadaaaaaafkaaaaadaagabaaaaeaaaaaa
fibiaaaeaahabaaaaaaaaaaaffffaaaafibiaaaeaahabaaaabaaaaaaffffaaaa
fibiaaaeaahabaaaacaaaaaaffffaaaafibiaaaeaahabaaaadaaaaaaffffaaaa
fibiaaaeaahabaaaaeaaaaaaffffaaaagcbaaaaddcbabaaaabaaaaaagcbaaaad
hcbabaaaacaaaaaagcbaaaadhcbabaaaadaaaaaagcbaaaadpcbabaaaaeaaaaaa
gfaaaaadpccabaaaaaaaaaaagiaaaaacadaaaaaabaaaaaahbcaabaaaaaaaaaaa
egbcbaaaadaaaaaaegbcbaaaadaaaaaaeeaaaaafbcaabaaaaaaaaaaaakaabaaa
aaaaaaaabaaaaaahccaabaaaaaaaaaaaegbcbaaaacaaaaaaegbcbaaaacaaaaaa
eeaaaaafccaabaaaaaaaaaaabkaabaaaaaaaaaaadiaaaaahocaabaaaaaaaaaaa
fgafbaaaaaaaaaaaagbjbaaaacaaaaaadcaaaaajhcaabaaaabaaaaaaegbcbaaa
adaaaaaaagaabaaaaaaaaaaajgahbaaaaaaaaaaabaaaaaahbcaabaaaaaaaaaaa
egacbaaaabaaaaaaegacbaaaabaaaaaaeeaaaaafbcaabaaaaaaaaaaaakaabaaa
aaaaaaaadiaaaaahhcaabaaaabaaaaaaagaabaaaaaaaaaaaegacbaaaabaaaaaa
efaaaaajpcaabaaaacaaaaaaegbabaaaabaaaaaaeghobaaaacaaaaaaaagabaaa
adaaaaaadcaaaaapdcaabaaaacaaaaaahgapbaaaacaaaaaaaceaaaaaaaaaaaea
aaaaaaeaaaaaaaaaaaaaaaaaaceaaaaaaaaaialpaaaaialpaaaaaaaaaaaaaaaa
dcaaaaakbcaabaaaaaaaaaaaakaabaiaebaaaaaaacaaaaaaakaabaaaacaaaaaa
abeaaaaaaaaaiadpdcaaaaakbcaabaaaaaaaaaaabkaabaiaebaaaaaaacaaaaaa
bkaabaaaacaaaaaaakaabaaaaaaaaaaaelaaaaafecaabaaaacaaaaaaakaabaaa
aaaaaaaabaaaaaahbcaabaaaaaaaaaaaegacbaaaacaaaaaaegacbaaaabaaaaaa
baaaaaahccaabaaaaaaaaaaaegacbaaaacaaaaaajgahbaaaaaaaaaaadeaaaaak
dcaabaaaaaaaaaaaegaabaaaaaaaaaaaaceaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaacpaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaadiaaaaaiecaabaaa
aaaaaaaaakiacaaaaaaaaaaaaiaaaaaaabeaaaaaaaaaaaecdiaaaaahbcaabaaa
aaaaaaaaakaabaaaaaaaaaaackaabaaaaaaaaaaabjaaaaafbcaabaaaaaaaaaaa
akaabaaaaaaaaaaaefaaaaajpcaabaaaabaaaaaaegbabaaaabaaaaaaeghobaaa
abaaaaaaaagabaaaaeaaaaaaefaaaaajpcaabaaaacaaaaaaegbabaaaabaaaaaa
eghobaaaaaaaaaaaaagabaaaacaaaaaadiaaaaahhcaabaaaabaaaaaaegacbaaa
abaaaaaapgapbaaaacaaaaaadiaaaaaihcaabaaaacaaaaaaegacbaaaacaaaaaa
egiccaaaaaaaaaaaahaaaaaadiaaaaaihcaabaaaacaaaaaaegacbaaaacaaaaaa
egiccaaaaaaaaaaaabaaaaaadiaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaa
fgifcaaaaaaaaaaaaiaaaaaadiaaaaahncaabaaaaaaaaaaaagaabaaaaaaaaaaa
agajbaaaabaaaaaadiaaaaaincaabaaaaaaaaaaaagaobaaaaaaaaaaaagijcaaa
aaaaaaaaabaaaaaadcaaaaajhcaabaaaaaaaaaaaegacbaaaacaaaaaafgafbaaa
aaaaaaaaigadbaaaaaaaaaaaaoaaaaahdcaabaaaabaaaaaaegbabaaaaeaaaaaa
pgbpbaaaaeaaaaaaaaaaaaakdcaabaaaabaaaaaaegaabaaaabaaaaaaaceaaaaa
aaaaaadpaaaaaadpaaaaaaaaaaaaaaaaefaaaaajpcaabaaaabaaaaaaegaabaaa
abaaaaaaeghobaaaadaaaaaaaagabaaaaaaaaaaadbaaaaahicaabaaaaaaaaaaa
abeaaaaaaaaaaaaackbabaaaaeaaaaaaabaaaaahicaabaaaaaaaaaaadkaabaaa
aaaaaaaaabeaaaaaaaaaiadpdiaaaaahicaabaaaaaaaaaaadkaabaaaabaaaaaa
dkaabaaaaaaaaaaabaaaaaahbcaabaaaabaaaaaaegbcbaaaaeaaaaaaegbcbaaa
aeaaaaaaefaaaaajpcaabaaaabaaaaaaagaabaaaabaaaaaaeghobaaaaeaaaaaa
aagabaaaabaaaaaaapaaaaahicaabaaaaaaaaaaapgapbaaaaaaaaaaaagaabaaa
abaaaaaadiaaaaahhccabaaaaaaaaaaapgapbaaaaaaaaaaaegacbaaaaaaaaaaa
dgaaaaaficcabaaaaaaaaaaaabeaaaaaaaaaaaaadoaaaaabejfdeheojiaaaaaa
afaaaaaaaiaaaaaaiaaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaa
imaaaaaaaaaaaaaaaaaaaaaaadaaaaaaabaaaaaaadadaaaaimaaaaaaabaaaaaa
aaaaaaaaadaaaaaaacaaaaaaahahaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaa
adaaaaaaahahaaaaimaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapapaaaa
fdfgfpfaepfdejfeejepeoaafeeffiedepepfceeaaklklklepfdeheocmaaaaaa
abaaaaaaaiaaaaaacaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaa
fdfgfpfegbhcghgfheaaklkl"
}

SubProgram "opengl " {
Keywords { "POINT_COOKIE" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_LightTextureB0] 2D
SetTexture 4 [_LightTexture0] CUBE
"!!ARBfp1.0
OPTION ARB_precision_hint_fastest;
# 39 ALU, 5 TEX
PARAM c[5] = { program.local[0..3],
		{ 0, 2, 1, 32 } };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
TEMP R4;
TEX R3.yw, fragment.texcoord[0], texture[2], 2D;
TEX R2, fragment.texcoord[0], texture[0], 2D;
TEX R0.xyz, fragment.texcoord[0], texture[1], 2D;
TEX R1.w, fragment.texcoord[3], texture[4], CUBE;
MAD R1.xy, R3.wyzw, c[4].y, -c[4].z;
MUL R1.z, R1.y, R1.y;
MAD R1.z, -R1.x, R1.x, -R1;
DP3 R0.w, fragment.texcoord[3], fragment.texcoord[3];
DP3 R3.x, fragment.texcoord[1], fragment.texcoord[1];
RSQ R3.x, R3.x;
DP3 R3.w, fragment.texcoord[2], fragment.texcoord[2];
ADD R1.z, R1, c[4];
RSQ R1.z, R1.z;
MUL R0.xyz, R2.w, R0;
MUL R2.xyz, R2, c[1];
RCP R1.z, R1.z;
MUL R3.xyz, R3.x, fragment.texcoord[1];
RSQ R3.w, R3.w;
MAD R4.xyz, R3.w, fragment.texcoord[2], R3;
DP3 R3.w, R4, R4;
RSQ R3.w, R3.w;
DP3 R3.x, R1, R3;
MUL R4.xyz, R3.w, R4;
DP3 R1.y, R1, R4;
MAX R1.z, R1.y, c[4].x;
MOV R1.y, c[4].w;
MUL R1.y, R1, c[2].x;
POW R1.y, R1.z, R1.y;
MUL R0.xyz, R0, c[3].x;
MUL R0.xyz, R1.y, R0;
MUL R0.xyz, R0, c[0];
MAX R1.x, R3, c[4];
MUL R2.xyz, R2, c[0];
MAD R1.xyz, R2, R1.x, R0;
MOV result.color.w, c[4].x;
TEX R0.w, R0.w, texture[3], 2D;
MUL R0.x, R0.w, R1.w;
MUL R0.xyz, R0.x, R1;
MUL result.color.xyz, R0, c[4].y;
END
# 39 instructions, 5 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "POINT_COOKIE" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_LightTextureB0] 2D
SetTexture 4 [_LightTexture0] CUBE
"ps_2_0
; 41 ALU, 5 TEX
dcl_2d s0
dcl_2d s1
dcl_2d s2
dcl_2d s3
dcl_cube s4
def c4, 2.00000000, -1.00000000, 1.00000000, 0.00000000
def c5, 32.00000000, 0, 0, 0
dcl t0.xy
dcl t1.xyz
dcl t2.xyz
dcl t3.xyz
texld r1, t0, s2
texld r2, t0, s0
texld r3, t0, s1
dp3_pp r1.x, t1, t1
rsq_pp r4.x, r1.x
dp3 r0.x, t3, t3
mov r0.xy, r0.x
dp3_pp r1.x, t2, t2
mul_pp r2.xyz, r2, c1
mul_pp r4.xyz, r4.x, t1
rsq_pp r1.x, r1.x
mad_pp r6.xyz, r1.x, t2, r4
dp3_pp r1.x, r6, r6
rsq_pp r1.x, r1.x
mul_pp r2.xyz, r2, c0
texld r7, r0, s3
texld r0, t3, s4
mov r0.y, r1
mov r0.x, r1.w
mad_pp r5.xy, r0, c4.x, c4.y
mul_pp r0.x, r5.y, r5.y
mad_pp r0.x, -r5, r5, -r0
add_pp r0.x, r0, c4.z
rsq_pp r0.x, r0.x
rcp_pp r5.z, r0.x
mul_pp r1.xyz, r1.x, r6
dp3_pp r1.x, r5, r1
mov_pp r0.x, c2
mul_pp r0.x, c5, r0
max_pp r1.x, r1, c4.w
pow r6.x, r1.x, r0.x
mul_pp r0.xyz, r2.w, r3
mul_pp r1.xyz, r0, c3.x
mov r0.x, r6.x
mul r0.xyz, r0.x, r1
mul_pp r1.xyz, r0, c0
dp3_pp r0.x, r5, r4
max_pp r0.x, r0, c4.w
mad_pp r1.xyz, r2, r0.x, r1
mul r0.x, r7, r0.w
mul_pp r0.xyz, r0.x, r1
mul_pp r0.xyz, r0, c4.x
mov_pp r0.w, c4
mov_pp oC0, r0
"
}

SubProgram "xbox360 " {
Keywords { "POINT_COOKIE" }
Vector 1 [_Color]
Float 3 [_Gloss]
Vector 0 [_LightColor0]
Float 2 [_Shininess]
SetTexture 0 [_LightTexture0] CUBE
SetTexture 1 [_LightTextureB0] 2D
SetTexture 2 [_MainTex] 2D
SetTexture 3 [_BumpMap] 2D
SetTexture 4 [_SpecMap] 2D
// Shader Timing Estimate, in Cycles/64 pixel vector:
// ALU: 33.33 (25 instructions), vertex: 0, texture: 20,
//   sequencer: 14, interpolator: 16;    6 GPRs, 30 threads,
// Performance (if enough threads): ~33 cycles per vector
// * Texture cycle estimates are assuming an 8bit/component texture with no
//     aniso or trilinear filtering.

"ps_360
backbbaaaaaaacaiaaaaaboeaaaaaaaaaaaaaaceaaaaablaaaaaabniaaaaaaaa
aaaaaaaaaaaaabiiaaaaaabmaaaaabhmppppadaaaaaaaaajaaaaaabmaaaaaaaa
aaaaabhfaaaaaanaaaadaaadaaabaaaaaaaaaanmaaaaaaaaaaaaaaomaaacaaab
aaabaaaaaaaaaapeaaaaaaaaaaaaabaeaaacaaadaaabaaaaaaaaabamaaaaaaaa
aaaaabbmaaacaaaaaaabaaaaaaaaaapeaaaaaaaaaaaaabcjaaadaaaaaaabaaaa
aaaaabdiaaaaaaaaaaaaabeiaaadaaabaaabaaaaaaaaaanmaaaaaaaaaaaaabfi
aaadaaacaaabaaaaaaaaaanmaaaaaaaaaaaaabgbaaacaaacaaabaaaaaaaaabam
aaaaaaaaaaaaabgmaaadaaaeaaabaaaaaaaaaanmaaaaaaaafpechfgnhaengbha
aaklklklaaaeaaamaaabaaabaaabaaaaaaaaaaaafpedgpgmgphcaaklaaabaaad
aaabaaaeaaabaaaaaaaaaaaafpehgmgphdhdaaklaaaaaaadaaabaaabaaabaaaa
aaaaaaaafpemgjghgiheedgpgmgphcdaaafpemgjghgihefegfhihehfhcgfdaaa
aaaeaaaoaaabaaabaaabaaaaaaaaaaaafpemgjghgihefegfhihehfhcgfecdaaa
fpengbgjgofegfhiaafpfdgigjgogjgogfhdhdaafpfdhagfgdengbhaaahahdfp
ddfpdaaadccodacodcdadddfddcodaaaaaaaaaaaaaaaaaabaaaaaaaaaaaaaaaa
aaaaaabeabpmaabaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaeaaaaaabke
baaaafaaaaaaaaaeaaaaaaaaaaaacmieaaapaaapaaaaaaabaaaadafaaaaahbfb
aaaahcfcaaaahdfdaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaalpiaaaaaaaaaaaaadpmaaaaadpiaaaaaecaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaafaagaaedaakbcaabcaaaabfaaaaaaaagaanmeaabcaaaaaa
aaaagabdgabjbcaabcaaaaaaaaaadabpaaaaccaaaaaaaaaamiaiaaabaaloloaa
paadadaamiapaaadaakgmnaapcadadaaemeeaaaeaablblmgocadadidmiadaaae
aagnmgmgmladaapobadifaabbpbppompaaaaeaaabaeidaabbpbppoiiaaaaeaaa
baciaaabbpbppgecaaaaeaaajaaicaibbpbpphppaaaamaaapmbibacbbpbppbpp
aaaaeaaamiabaaaeaagmgmaacbacppaamiabaaafaablblaaobacabaamiaiaaab
aaloloaapaacacaamiaiaaacaaloloaapaababaafiihacadaablmablobaaadic
miaoaaaeaablpmaaobacabaafiihababaalelebloaafafibmiahaaafaablmabf
olabacaebeciaaabaalololbpaafafaafibgacacaambgmblkaabpoibmiaiaaab
aelclcblnbacacpokaihacafaamagmblobafacibkicbaeacaamdmdebnaaeacab
kiecaeacaalomdicnaafacabkiigaeacaalmlbmaicacpoabeaboacaeaaabpmmg
kbaeaaicmiapaaaaaaaalaaaobaeacaadiihabacaamagmgmkbadadaamiahaaac
aamablaaobacabaamiahaaaaaamamabfklacaaaamiahmaaaaagmmaaaobabaaaa
aaaaaaaaaaaaaaaaaaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "POINT_COOKIE" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_LightTextureB0] 2D
SetTexture 4 [_LightTexture0] CUBE
"sce_fp_rsx // 47 instructions using 4 registers
[Configuration]
24
ffffffff0003c020000ffff1000000000000840004000000
[Offsets]
4
_LightColor0 2 0
000002b000000240
_Color 1 0
000001a0
_Shininess 1 0
000001c0
_Gloss 1 0
00000130
[Microcode]
752
94001704c8011c9dc8000001c8003fe1ae843940c8011c9dc8000029c800bfe1
06860440ce001c9d00020000aa020000000040000000bf800000000000000000
ee000100c8011c9dc8000001c8003fe110840240ab0c1c9cab0c0000c8000001
02000500c8001c9dc8000001c800000110800440010c1c9e010c0000c9080003
0200170600001c9cc8000001c8000001ce823940c8011c9dc8000029c800bfe1
0e040340c9081c9dc9040001c80000019e061700c8011c9dc8000001c8003fe1
0e823940c8081c9dc8000029c800000108800340ff001c9d00020000c8000001
00003f8000000000000000000000000008863b40c9003c9d55000001c8000001
10800540c90c1c9dc9040001c80000018e041702c8011c9dc8000001c8003fe1
08800240fe0c1c9d00020000c800000100000000000000000000000000000000
10040900c9001c9d00020000c800000100000000000000000000000000000000
06000100c8001c9dc8000001c80000010e82024055001c9dc8080001c8000001
04001d00fe081c9dc8000001c80000010e8a0240c80c1c9dc8020001c8000001
000000000000000000000000000000000288014000021c9cc8000001c8000001
000000000000000000000000000000001088024001101c9c00020000c8000001
0000420000000000000000000000000002880540c90c1c9dc9080001c8000001
04000200c8001c9dff100001c8000001f0021708c8011c9dc8000001c8003fe1
04001c00aa001c9cc8000001c80000010e840240c9141c9dc8020001c8000001
000000000000000000000000000000000e820200aa001c9cc9040001c8000001
1080090001101c9c00020000c800000100000000000000000000000000000000
1082020000001c9cc8040001c80000010e800240c9081c9dff000001c8000001
0e800440c9041c9dc8020001c900000100000000000000000000000000000000
10800140c8021c9dc8000001c800000100000000000000000000000000000000
0e810240ff041c9dc9001001c8000001
"
}

SubProgram "d3d11 " {
Keywords { "POINT_COOKIE" }
ConstBuffer "$Globals" 160 // 136 used size, 8 vars
Vector 16 [_LightColor0] 4
Vector 112 [_Color] 4
Float 128 [_Shininess]
Float 132 [_Gloss]
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 2
SetTexture 1 [_SpecMap] 2D 4
SetTexture 2 [_BumpMap] 2D 3
SetTexture 3 [_LightTextureB0] 2D 1
SetTexture 4 [_LightTexture0] CUBE 0
// 37 instructions, 3 temp regs, 0 temp arrays:
// ALU 25 float, 0 int, 0 uint
// TEX 5 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0
eefiecedbdnnmikajhmdekgmpcpbiilaonoahnccabaaaaaadmagaaaaadaaaaaa
cmaaaaaammaaaaaaaaabaaaaejfdeheojiaaaaaaafaaaaaaaiaaaaaaiaaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaadadaaaaimaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaahahaaaaimaaaaaa
adaaaaaaaaaaaaaaadaaaaaaaeaaaaaaahahaaaafdfgfpfaepfdejfeejepeoaa
feeffiedepepfceeaaklklklepfdeheocmaaaaaaabaaaaaaaiaaaaaacaaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaafdfgfpfegbhcghgfheaaklkl
fdeieefcdeafaaaaeaaaaaaaenabaaaafjaaaaaeegiocaaaaaaaaaaaajaaaaaa
fkaaaaadaagabaaaaaaaaaaafkaaaaadaagabaaaabaaaaaafkaaaaadaagabaaa
acaaaaaafkaaaaadaagabaaaadaaaaaafkaaaaadaagabaaaaeaaaaaafibiaaae
aahabaaaaaaaaaaaffffaaaafibiaaaeaahabaaaabaaaaaaffffaaaafibiaaae
aahabaaaacaaaaaaffffaaaafibiaaaeaahabaaaadaaaaaaffffaaaafidaaaae
aahabaaaaeaaaaaaffffaaaagcbaaaaddcbabaaaabaaaaaagcbaaaadhcbabaaa
acaaaaaagcbaaaadhcbabaaaadaaaaaagcbaaaadhcbabaaaaeaaaaaagfaaaaad
pccabaaaaaaaaaaagiaaaaacadaaaaaabaaaaaahbcaabaaaaaaaaaaaegbcbaaa
adaaaaaaegbcbaaaadaaaaaaeeaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaa
baaaaaahccaabaaaaaaaaaaaegbcbaaaacaaaaaaegbcbaaaacaaaaaaeeaaaaaf
ccaabaaaaaaaaaaabkaabaaaaaaaaaaadiaaaaahocaabaaaaaaaaaaafgafbaaa
aaaaaaaaagbjbaaaacaaaaaadcaaaaajhcaabaaaabaaaaaaegbcbaaaadaaaaaa
agaabaaaaaaaaaaajgahbaaaaaaaaaaabaaaaaahbcaabaaaaaaaaaaaegacbaaa
abaaaaaaegacbaaaabaaaaaaeeaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaa
diaaaaahhcaabaaaabaaaaaaagaabaaaaaaaaaaaegacbaaaabaaaaaaefaaaaaj
pcaabaaaacaaaaaaegbabaaaabaaaaaaeghobaaaacaaaaaaaagabaaaadaaaaaa
dcaaaaapdcaabaaaacaaaaaahgapbaaaacaaaaaaaceaaaaaaaaaaaeaaaaaaaea
aaaaaaaaaaaaaaaaaceaaaaaaaaaialpaaaaialpaaaaaaaaaaaaaaaadcaaaaak
bcaabaaaaaaaaaaaakaabaiaebaaaaaaacaaaaaaakaabaaaacaaaaaaabeaaaaa
aaaaiadpdcaaaaakbcaabaaaaaaaaaaabkaabaiaebaaaaaaacaaaaaabkaabaaa
acaaaaaaakaabaaaaaaaaaaaelaaaaafecaabaaaacaaaaaaakaabaaaaaaaaaaa
baaaaaahbcaabaaaaaaaaaaaegacbaaaacaaaaaaegacbaaaabaaaaaabaaaaaah
ccaabaaaaaaaaaaaegacbaaaacaaaaaajgahbaaaaaaaaaaadeaaaaakdcaabaaa
aaaaaaaaegaabaaaaaaaaaaaaceaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
cpaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaadiaaaaaiecaabaaaaaaaaaaa
akiacaaaaaaaaaaaaiaaaaaaabeaaaaaaaaaaaecdiaaaaahbcaabaaaaaaaaaaa
akaabaaaaaaaaaaackaabaaaaaaaaaaabjaaaaafbcaabaaaaaaaaaaaakaabaaa
aaaaaaaaefaaaaajpcaabaaaabaaaaaaegbabaaaabaaaaaaeghobaaaabaaaaaa
aagabaaaaeaaaaaaefaaaaajpcaabaaaacaaaaaaegbabaaaabaaaaaaeghobaaa
aaaaaaaaaagabaaaacaaaaaadiaaaaahhcaabaaaabaaaaaaegacbaaaabaaaaaa
pgapbaaaacaaaaaadiaaaaaihcaabaaaacaaaaaaegacbaaaacaaaaaaegiccaaa
aaaaaaaaahaaaaaadiaaaaaihcaabaaaacaaaaaaegacbaaaacaaaaaaegiccaaa
aaaaaaaaabaaaaaadiaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaafgifcaaa
aaaaaaaaaiaaaaaadiaaaaahncaabaaaaaaaaaaaagaabaaaaaaaaaaaagajbaaa
abaaaaaadiaaaaaincaabaaaaaaaaaaaagaobaaaaaaaaaaaagijcaaaaaaaaaaa
abaaaaaadcaaaaajhcaabaaaaaaaaaaaegacbaaaacaaaaaafgafbaaaaaaaaaaa
igadbaaaaaaaaaaabaaaaaahicaabaaaaaaaaaaaegbcbaaaaeaaaaaaegbcbaaa
aeaaaaaaefaaaaajpcaabaaaabaaaaaapgapbaaaaaaaaaaaeghobaaaadaaaaaa
aagabaaaabaaaaaaefaaaaajpcaabaaaacaaaaaaegbcbaaaaeaaaaaaeghobaaa
aeaaaaaaaagabaaaaaaaaaaaapaaaaahicaabaaaaaaaaaaaagaabaaaabaaaaaa
pgapbaaaacaaaaaadiaaaaahhccabaaaaaaaaaaapgapbaaaaaaaaaaaegacbaaa
aaaaaaaadgaaaaaficcabaaaaaaaaaaaabeaaaaaaaaaaaaadoaaaaab"
}

SubProgram "gles " {
Keywords { "POINT_COOKIE" }
"!!GLES"
}

SubProgram "glesdesktop " {
Keywords { "POINT_COOKIE" }
"!!GLES"
}

SubProgram "flash " {
Keywords { "POINT_COOKIE" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_LightTextureB0] 2D
SetTexture 4 [_LightTexture0] CUBE
"agal_ps
c4 2.0 -1.0 1.0 0.0
c5 32.0 0.0 0.0 0.0
[bc]
ciaaaaaaaeaaapacaaaaaaoeaeaaaaaaacaaaaaaafaababb tex r4, v0, s2 <2d wrap linear point>
ciaaaaaaacaaapacaaaaaaoeaeaaaaaaaaaaaaaaafaababb tex r2, v0, s0 <2d wrap linear point>
ciaaaaaaadaaapacaaaaaaoeaeaaaaaaabaaaaaaafaababb tex r3, v0, s1 <2d wrap linear point>
bcaaaaaaaaaaabacadaaaaoeaeaaaaaaadaaaaoeaeaaaaaa dp3 r0.x, v3, v3
aaaaaaaaaaaaadacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa mov r0.xy, r0.x
adaaaaaaacaaahacacaaaakeacaaaaaaabaaaaoeabaaaaaa mul r2.xyz, r2.xyzz, c1
adaaaaaaacaaahacacaaaakeacaaaaaaaaaaaaoeabaaaaaa mul r2.xyz, r2.xyzz, c0
ciaaaaaaabaaapacaaaaaafeacaaaaaaadaaaaaaafaababb tex r1, r0.xyyy, s3 <2d wrap linear point>
ciaaaaaaaaaaapacadaaaaoeaeaaaaaaaeaaaaaaafbababb tex r0, v3, s4 <cube wrap linear point>
bcaaaaaaabaaabacabaaaaoeaeaaaaaaabaaaaoeaeaaaaaa dp3 r1.x, v1, v1
akaaaaaaaeaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r4.x, r1.x
aaaaaaaaaaaaacacaeaaaaffacaaaaaaaaaaaaaaaaaaaaaa mov r0.y, r4.y
aaaaaaaaaaaaabacaeaaaappacaaaaaaaaaaaaaaaaaaaaaa mov r0.x, r4.w
adaaaaaaafaaadacaaaaaafeacaaaaaaaeaaaaaaabaaaaaa mul r5.xy, r0.xyyy, c4.x
abaaaaaaafaaadacafaaaafeacaaaaaaaeaaaaffabaaaaaa add r5.xy, r5.xyyy, c4.y
adaaaaaaaaaaabacafaaaaffacaaaaaaafaaaaffacaaaaaa mul r0.x, r5.y, r5.y
bfaaaaaaadaaaiacafaaaaaaacaaaaaaaaaaaaaaaaaaaaaa neg r3.w, r5.x
adaaaaaaadaaaiacadaaaappacaaaaaaafaaaaaaacaaaaaa mul r3.w, r3.w, r5.x
acaaaaaaaaaaabacadaaaappacaaaaaaaaaaaaaaacaaaaaa sub r0.x, r3.w, r0.x
bcaaaaaaabaaabacacaaaaoeaeaaaaaaacaaaaoeaeaaaaaa dp3 r1.x, v2, v2
abaaaaaaaaaaabacaaaaaaaaacaaaaaaaeaaaakkabaaaaaa add r0.x, r0.x, c4.z
akaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r0.x, r0.x
afaaaaaaafaaaeacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rcp r5.z, r0.x
aaaaaaaaaaaaabacacaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0.x, c2
adaaaaaaaeaaahacaeaaaaaaacaaaaaaabaaaaoeaeaaaaaa mul r4.xyz, r4.x, v1
akaaaaaaabaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r1.x, r1.x
adaaaaaaagaaahacabaaaaaaacaaaaaaacaaaaoeaeaaaaaa mul r6.xyz, r1.x, v2
abaaaaaaagaaahacagaaaakeacaaaaaaaeaaaakeacaaaaaa add r6.xyz, r6.xyzz, r4.xyzz
bcaaaaaaabaaabacagaaaakeacaaaaaaagaaaakeacaaaaaa dp3 r1.x, r6.xyzz, r6.xyzz
akaaaaaaabaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r1.x, r1.x
adaaaaaaabaaahacabaaaaaaacaaaaaaagaaaakeacaaaaaa mul r1.xyz, r1.x, r6.xyzz
bcaaaaaaabaaabacafaaaakeacaaaaaaabaaaakeacaaaaaa dp3 r1.x, r5.xyzz, r1.xyzz
adaaaaaaaaaaabacafaaaaoeabaaaaaaaaaaaaaaacaaaaaa mul r0.x, c5, r0.x
ahaaaaaaabaaabacabaaaaaaacaaaaaaaeaaaappabaaaaaa max r1.x, r1.x, c4.w
alaaaaaaagaaapacabaaaaaaacaaaaaaaaaaaaaaacaaaaaa pow r6, r1.x, r0.x
adaaaaaaaaaaahacacaaaappacaaaaaaadaaaakeacaaaaaa mul r0.xyz, r2.w, r3.xyzz
adaaaaaaabaaahacaaaaaakeacaaaaaaadaaaaaaabaaaaaa mul r1.xyz, r0.xyzz, c3.x
aaaaaaaaaaaaabacagaaaaaaacaaaaaaaaaaaaaaaaaaaaaa mov r0.x, r6.x
adaaaaaaaaaaahacaaaaaaaaacaaaaaaabaaaakeacaaaaaa mul r0.xyz, r0.x, r1.xyzz
adaaaaaaabaaahacaaaaaakeacaaaaaaaaaaaaoeabaaaaaa mul r1.xyz, r0.xyzz, c0
bcaaaaaaaaaaabacafaaaakeacaaaaaaaeaaaakeacaaaaaa dp3 r0.x, r5.xyzz, r4.xyzz
ahaaaaaaaaaaabacaaaaaaaaacaaaaaaaeaaaappabaaaaaa max r0.x, r0.x, c4.w
adaaaaaaacaaahacacaaaakeacaaaaaaaaaaaaaaacaaaaaa mul r2.xyz, r2.xyzz, r0.x
abaaaaaaabaaahacacaaaakeacaaaaaaabaaaakeacaaaaaa add r1.xyz, r2.xyzz, r1.xyzz
adaaaaaaaaaaabacabaaaappacaaaaaaaaaaaappacaaaaaa mul r0.x, r1.w, r0.w
adaaaaaaaaaaahacaaaaaaaaacaaaaaaabaaaakeacaaaaaa mul r0.xyz, r0.x, r1.xyzz
adaaaaaaaaaaahacaaaaaakeacaaaaaaaeaaaaaaabaaaaaa mul r0.xyz, r0.xyzz, c4.x
aaaaaaaaaaaaaiacaeaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0.w, c4
aaaaaaaaaaaaapadaaaaaaoeacaaaaaaaaaaaaaaaaaaaaaa mov o0, r0
"
}

SubProgram "d3d11_9x " {
Keywords { "POINT_COOKIE" }
ConstBuffer "$Globals" 160 // 136 used size, 8 vars
Vector 16 [_LightColor0] 4
Vector 112 [_Color] 4
Float 128 [_Shininess]
Float 132 [_Gloss]
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 2
SetTexture 1 [_SpecMap] 2D 4
SetTexture 2 [_BumpMap] 2D 3
SetTexture 3 [_LightTextureB0] 2D 1
SetTexture 4 [_LightTexture0] CUBE 0
// 37 instructions, 3 temp regs, 0 temp arrays:
// ALU 25 float, 0 int, 0 uint
// TEX 5 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0_level_9_3
eefiecedaheailmpmepgefgohhkjacojbakkdlglabaaaaaafeajaaaaaeaaaaaa
daaaaaaaeeadaaaaiaaiaaaacaajaaaaebgpgodjamadaaaaamadaaaaaaacpppp
lmacaaaafaaaaaaaacaadiaaaaaafaaaaaaafaaaafaaceaaaaaafaaaaeaaaaaa
adababaaaaacacaaacadadaaabaeaeaaaaaaabaaabaaaaaaaaaaaaaaaaaaahaa
acaaabaaaaaaaaaaabacppppfbaaaaafadaaapkaaaaaaaeaaaaaialpaaaaiadp
aaaaaaaafbaaaaafaeaaapkaaaaaaaecaaaaaaaaaaaaaaaaaaaaaaaabpaaaaac
aaaaaaiaaaaaadlabpaaaaacaaaaaaiaabaachlabpaaaaacaaaaaaiaacaachla
bpaaaaacaaaaaaiaadaaahlabpaaaaacaaaaaajiaaaiapkabpaaaaacaaaaaaja
abaiapkabpaaaaacaaaaaajaacaiapkabpaaaaacaaaaaajaadaiapkabpaaaaac
aaaaaajaaeaiapkaaiaaaaadaaaaciiaacaaoelaacaaoelaahaaaaacaaaacbia
aaaappiaceaaaaacabaachiaabaaoelaaeaaaaaeaaaachiaacaaoelaaaaaaaia
abaaoeiaceaaaaacacaachiaaaaaoeiaecaaaaadaaaacpiaaaaaoelaadaioeka
aeaaaaaeaaaacdiaaaaaohiaadaaaakaadaaffkaaeaaaaaeaaaaciiaaaaaaaia
aaaaaaibadaakkkaaeaaaaaeaaaaciiaaaaaffiaaaaaffibaaaappiaahaaaaac
aaaaciiaaaaappiaagaaaaacaaaaceiaaaaappiaaiaaaaadaaaaciiaaaaaoeia
acaaoeiaaiaaaaadaaaacbiaaaaaoeiaabaaoeiaalaaaaadabaacbiaaaaaaaia
adaappkaalaaaaadabaaaciaaaaappiaadaappkaabaaaaacaaaaabiaacaaaaka
afaaaaadaaaaabiaaaaaaaiaaeaaaakacaaaaaadacaaabiaabaaffiaaaaaaaia
ecaaaaadaaaacpiaaaaaoelaacaioekaecaaaaadadaacpiaaaaaoelaaeaioeka
afaaaaadabaacoiaaaaappiaadaajaiaafaaaaadaaaachiaaaaaoeiaabaaoeka
afaaaaadaaaachiaaaaaoeiaaaaaoekaafaaaaadabaacoiaabaaoeiaacaaffka
afaaaaadabaacoiaabaaoeiaacaaaaiaafaaaaadabaacoiaabaaoeiaaaaajaka
aeaaaaaeaaaachiaaaaaoeiaabaaaaiaabaapjiaaiaaaaadabaaadiaadaaoela
adaaoelaecaaaaadacaaapiaadaaoelaaaaioekaecaaaaadabaaapiaabaaoeia
abaioekafkaaaaaeaaaaciiaabaaaaiaacaappiaadaappkaafaaaaadaaaachia
aaaappiaaaaaoeiaabaaaaacaaaaciiaadaappkaabaaaaacaaaicpiaaaaaoeia
ppppaaaafdeieefcdeafaaaaeaaaaaaaenabaaaafjaaaaaeegiocaaaaaaaaaaa
ajaaaaaafkaaaaadaagabaaaaaaaaaaafkaaaaadaagabaaaabaaaaaafkaaaaad
aagabaaaacaaaaaafkaaaaadaagabaaaadaaaaaafkaaaaadaagabaaaaeaaaaaa
fibiaaaeaahabaaaaaaaaaaaffffaaaafibiaaaeaahabaaaabaaaaaaffffaaaa
fibiaaaeaahabaaaacaaaaaaffffaaaafibiaaaeaahabaaaadaaaaaaffffaaaa
fidaaaaeaahabaaaaeaaaaaaffffaaaagcbaaaaddcbabaaaabaaaaaagcbaaaad
hcbabaaaacaaaaaagcbaaaadhcbabaaaadaaaaaagcbaaaadhcbabaaaaeaaaaaa
gfaaaaadpccabaaaaaaaaaaagiaaaaacadaaaaaabaaaaaahbcaabaaaaaaaaaaa
egbcbaaaadaaaaaaegbcbaaaadaaaaaaeeaaaaafbcaabaaaaaaaaaaaakaabaaa
aaaaaaaabaaaaaahccaabaaaaaaaaaaaegbcbaaaacaaaaaaegbcbaaaacaaaaaa
eeaaaaafccaabaaaaaaaaaaabkaabaaaaaaaaaaadiaaaaahocaabaaaaaaaaaaa
fgafbaaaaaaaaaaaagbjbaaaacaaaaaadcaaaaajhcaabaaaabaaaaaaegbcbaaa
adaaaaaaagaabaaaaaaaaaaajgahbaaaaaaaaaaabaaaaaahbcaabaaaaaaaaaaa
egacbaaaabaaaaaaegacbaaaabaaaaaaeeaaaaafbcaabaaaaaaaaaaaakaabaaa
aaaaaaaadiaaaaahhcaabaaaabaaaaaaagaabaaaaaaaaaaaegacbaaaabaaaaaa
efaaaaajpcaabaaaacaaaaaaegbabaaaabaaaaaaeghobaaaacaaaaaaaagabaaa
adaaaaaadcaaaaapdcaabaaaacaaaaaahgapbaaaacaaaaaaaceaaaaaaaaaaaea
aaaaaaeaaaaaaaaaaaaaaaaaaceaaaaaaaaaialpaaaaialpaaaaaaaaaaaaaaaa
dcaaaaakbcaabaaaaaaaaaaaakaabaiaebaaaaaaacaaaaaaakaabaaaacaaaaaa
abeaaaaaaaaaiadpdcaaaaakbcaabaaaaaaaaaaabkaabaiaebaaaaaaacaaaaaa
bkaabaaaacaaaaaaakaabaaaaaaaaaaaelaaaaafecaabaaaacaaaaaaakaabaaa
aaaaaaaabaaaaaahbcaabaaaaaaaaaaaegacbaaaacaaaaaaegacbaaaabaaaaaa
baaaaaahccaabaaaaaaaaaaaegacbaaaacaaaaaajgahbaaaaaaaaaaadeaaaaak
dcaabaaaaaaaaaaaegaabaaaaaaaaaaaaceaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaacpaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaadiaaaaaiecaabaaa
aaaaaaaaakiacaaaaaaaaaaaaiaaaaaaabeaaaaaaaaaaaecdiaaaaahbcaabaaa
aaaaaaaaakaabaaaaaaaaaaackaabaaaaaaaaaaabjaaaaafbcaabaaaaaaaaaaa
akaabaaaaaaaaaaaefaaaaajpcaabaaaabaaaaaaegbabaaaabaaaaaaeghobaaa
abaaaaaaaagabaaaaeaaaaaaefaaaaajpcaabaaaacaaaaaaegbabaaaabaaaaaa
eghobaaaaaaaaaaaaagabaaaacaaaaaadiaaaaahhcaabaaaabaaaaaaegacbaaa
abaaaaaapgapbaaaacaaaaaadiaaaaaihcaabaaaacaaaaaaegacbaaaacaaaaaa
egiccaaaaaaaaaaaahaaaaaadiaaaaaihcaabaaaacaaaaaaegacbaaaacaaaaaa
egiccaaaaaaaaaaaabaaaaaadiaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaa
fgifcaaaaaaaaaaaaiaaaaaadiaaaaahncaabaaaaaaaaaaaagaabaaaaaaaaaaa
agajbaaaabaaaaaadiaaaaaincaabaaaaaaaaaaaagaobaaaaaaaaaaaagijcaaa
aaaaaaaaabaaaaaadcaaaaajhcaabaaaaaaaaaaaegacbaaaacaaaaaafgafbaaa
aaaaaaaaigadbaaaaaaaaaaabaaaaaahicaabaaaaaaaaaaaegbcbaaaaeaaaaaa
egbcbaaaaeaaaaaaefaaaaajpcaabaaaabaaaaaapgapbaaaaaaaaaaaeghobaaa
adaaaaaaaagabaaaabaaaaaaefaaaaajpcaabaaaacaaaaaaegbcbaaaaeaaaaaa
eghobaaaaeaaaaaaaagabaaaaaaaaaaaapaaaaahicaabaaaaaaaaaaaagaabaaa
abaaaaaapgapbaaaacaaaaaadiaaaaahhccabaaaaaaaaaaapgapbaaaaaaaaaaa
egacbaaaaaaaaaaadgaaaaaficcabaaaaaaaaaaaabeaaaaaaaaaaaaadoaaaaab
ejfdeheojiaaaaaaafaaaaaaaiaaaaaaiaaaaaaaaaaaaaaaabaaaaaaadaaaaaa
aaaaaaaaapaaaaaaimaaaaaaaaaaaaaaaaaaaaaaadaaaaaaabaaaaaaadadaaaa
imaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaaahahaaaaimaaaaaaacaaaaaa
aaaaaaaaadaaaaaaadaaaaaaahahaaaaimaaaaaaadaaaaaaaaaaaaaaadaaaaaa
aeaaaaaaahahaaaafdfgfpfaepfdejfeejepeoaafeeffiedepepfceeaaklklkl
epfdeheocmaaaaaaabaaaaaaaiaaaaaacaaaaaaaaaaaaaaaaaaaaaaaadaaaaaa
aaaaaaaaapaaaaaafdfgfpfegbhcghgfheaaklkl"
}

SubProgram "opengl " {
Keywords { "DIRECTIONAL_COOKIE" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_LightTexture0] 2D
"!!ARBfp1.0
OPTION ARB_precision_hint_fastest;
# 34 ALU, 4 TEX
PARAM c[5] = { program.local[0..3],
		{ 0, 2, 1, 32 } };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
TEX R0, fragment.texcoord[0], texture[0], 2D;
TEX R2.yw, fragment.texcoord[0], texture[2], 2D;
TEX R1.xyz, fragment.texcoord[0], texture[1], 2D;
TEX R1.w, fragment.texcoord[3], texture[3], 2D;
MAD R2.xy, R2.wyzw, c[4].y, -c[4].z;
MUL R1.xyz, R0.w, R1;
DP3 R2.w, fragment.texcoord[2], fragment.texcoord[2];
MUL R2.z, R2.y, R2.y;
MAD R2.z, -R2.x, R2.x, -R2;
ADD R2.z, R2, c[4];
RSQ R2.z, R2.z;
MUL R0.xyz, R0, c[1];
RSQ R2.w, R2.w;
MOV R3.xyz, fragment.texcoord[1];
MAD R3.xyz, R2.w, fragment.texcoord[2], R3;
DP3 R2.w, R3, R3;
RSQ R2.w, R2.w;
MUL R3.xyz, R2.w, R3;
RCP R2.z, R2.z;
DP3 R2.w, R2, R3;
MOV R3.x, c[4].w;
MAX R2.w, R2, c[4].x;
MUL R0.w, R3.x, c[2].x;
POW R0.w, R2.w, R0.w;
MUL R1.xyz, R1, c[3].x;
MUL R1.xyz, R0.w, R1;
DP3 R0.w, R2, fragment.texcoord[1];
MUL R1.xyz, R1, c[0];
MUL R0.xyz, R0, c[0];
MAX R0.w, R0, c[4].x;
MAD R0.xyz, R0, R0.w, R1;
MUL R0.xyz, R1.w, R0;
MUL result.color.xyz, R0, c[4].y;
MOV result.color.w, c[4].x;
END
# 34 instructions, 4 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "DIRECTIONAL_COOKIE" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_LightTexture0] 2D
"ps_2_0
; 36 ALU, 4 TEX
dcl_2d s0
dcl_2d s1
dcl_2d s2
dcl_2d s3
def c4, 2.00000000, -1.00000000, 1.00000000, 0.00000000
def c5, 32.00000000, 0, 0, 0
dcl t0.xy
dcl t1.xyz
dcl t2.xyz
dcl t3.xy
texld r1, t0, s2
texld r2, t0, s0
texld r3, t0, s1
texld r0, t3, s3
dp3_pp r1.x, t2, t2
mul_pp r2.xyz, r2, c1
mov r0.y, r1
mov r0.x, r1.w
mad_pp r4.xy, r0, c4.x, c4.y
mul_pp r0.x, r4.y, r4.y
mad_pp r0.x, -r4, r4, -r0
add_pp r0.x, r0, c4.z
rsq_pp r0.x, r0.x
rcp_pp r4.z, r0.x
mov_pp r0.x, c2
rsq_pp r1.x, r1.x
mov_pp r5.xyz, t1
mad_pp r5.xyz, r1.x, t2, r5
dp3_pp r1.x, r5, r5
rsq_pp r1.x, r1.x
mul_pp r1.xyz, r1.x, r5
dp3_pp r1.x, r4, r1
mul_pp r0.x, c5, r0
max_pp r1.x, r1, c4.w
pow r5.x, r1.x, r0.x
mul_pp r0.xyz, r2.w, r3
mul_pp r1.xyz, r0, c3.x
mov r0.x, r5.x
mul r0.xyz, r0.x, r1
mul_pp r1.xyz, r0, c0
dp3_pp r0.x, r4, t1
max_pp r0.x, r0, c4.w
mul_pp r2.xyz, r2, c0
mad_pp r0.xyz, r2, r0.x, r1
mul_pp r0.xyz, r0.w, r0
mul_pp r0.xyz, r0, c4.x
mov_pp r0.w, c4
mov_pp oC0, r0
"
}

SubProgram "xbox360 " {
Keywords { "DIRECTIONAL_COOKIE" }
Vector 1 [_Color]
Float 3 [_Gloss]
Vector 0 [_LightColor0]
Float 2 [_Shininess]
SetTexture 0 [_LightTexture0] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_SpecMap] 2D
// Shader Timing Estimate, in Cycles/64 pixel vector:
// ALU: 22.67 (17 instructions), vertex: 0, texture: 16,
//   sequencer: 10, interpolator: 16;    6 GPRs, 30 threads,
// Performance (if enough threads): ~22 cycles per vector
// * Texture cycle estimates are assuming an 8bit/component texture with no
//     aniso or trilinear filtering.

"ps_360
backbbaaaaaaabneaaaaabgmaaaaaaaaaaaaaaceaaaaabhmaaaaabkeaaaaaaaa
aaaaaaaaaaaaabfeaaaaaabmaaaaabeippppadaaaaaaaaaiaaaaaabmaaaaaaaa
aaaaabebaaaaaalmaaadaaacaaabaaaaaaaaaamiaaaaaaaaaaaaaaniaaacaaab
aaabaaaaaaaaaaoaaaaaaaaaaaaaaapaaaacaaadaaabaaaaaaaaaapiaaaaaaaa
aaaaabaiaaacaaaaaaabaaaaaaaaaaoaaaaaaaaaaaaaabbfaaadaaaaaaabaaaa
aaaaaamiaaaaaaaaaaaaabceaaadaaabaaabaaaaaaaaaamiaaaaaaaaaaaaabcn
aaacaaacaaabaaaaaaaaaapiaaaaaaaaaaaaabdiaaadaaadaaabaaaaaaaaaami
aaaaaaaafpechfgnhaengbhaaaklklklaaaeaaamaaabaaabaaabaaaaaaaaaaaa
fpedgpgmgphcaaklaaabaaadaaabaaaeaaabaaaaaaaaaaaafpehgmgphdhdaakl
aaaaaaadaaabaaabaaabaaaaaaaaaaaafpemgjghgiheedgpgmgphcdaaafpemgj
ghgihefegfhihehfhcgfdaaafpengbgjgofegfhiaafpfdgigjgogjgogfhdhdaa
fpfdhagfgdengbhaaahahdfpddfpdaaadccodacodcdadddfddcodaaaaaaaaaaa
aaaaaaabaaaaaaaaaaaaaaaaaaaaaabeabpmaabaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaeaaaaaabcmbaaaafaaaaaaaaaeaaaaaaaaaaaaciieaaapaaap
aaaaaaabaaaadafaaaaahbfbaaaahcfcaaaaddfdaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaalpiaaaaaaaaaaaaadpiaaaaaecaaaaaaaaffeaadaaaabcaameaaaaaa
aaaagaahgaanbcaabcaaaaaaaaaafabdaaaaccaaaaaaaaaabaaidagbbpbppppl
aaaaeaaabacidaabbpbppompaaaaeaaabadieaabbpbppoiiaaaaeaaababiaaab
bpbppgecaaaaeaaabeaiaaabaalologmnaacacacamihafaeaablmablmbaaaepp
fiihabadaalelebloaadadibmiahaaafaablmamaolabacabbeciaaabaalololb
paafafaafibgacacaambgmblkaadppibmiaiaaabaelclcmgnbacacppkaihacaf
aamagmblobafacibkibbacabaamdloebnaacababkiccacabaalomdicnaafacab
kiedacabaalalbmaicabppabeaehabafaamamalbkbacaaibmiapaaaaaaaaomaa
obafabaadiboababaapmgmblkbaeadaamiahaaabaabfgmaaobababaamiahaaaa
aamamamaklabaaaamiahmaaaaagmmaaaobadaaaaaaaaaaaaaaaaaaaaaaaaaaaa
"
}

SubProgram "ps3 " {
Keywords { "DIRECTIONAL_COOKIE" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_LightTexture0] 2D
"sce_fp_rsx // 43 instructions using 4 registers
[Configuration]
24
ffffffff0003c020000ffff9000000000000840004000000
[Offsets]
4
_LightColor0 2 0
00000280000001f0
_Color 1 0
00000020
_Shininess 1 0
00000100
_Gloss 1 0
00000070
[Microcode]
688
9e021700c8011c9dc8000001c8003fe10e840240c8041c9dc8020001c8000001
0000000000000000000000000000000094001704c8011c9dc8000001c8003fe1
068a0440ce001c9daa0200005402000100000000000040000000bf8000000000
10800240c8041c9d00020000c800000100000000000000000000000000000000
8e061702c8011c9dc8000001c8003fe10e880240ff001c9dc80c0001c8000001
10880240ab141c9cab140000c8000001ae820140c8011c9dc8000001c8003fe1
1086044001141c9e01140000c910000310800340c90c1c9daa020000c8000001
0000000000003f8000000000000000000286014000021c9cc8000001c8000001
00000000000000000000000000000000088a3b40ff003c9dff000001c8000001
06860100c90c1c9dc8000001c800000110840240010c1c9c00020000c8000001
0000420000000000000000000000000002860540c9141c9dc9040001c8000001
ce803940c8011c9dc8000029c800bfe10e800340c9041c9dc9000001c8000001
10800900010c1c9c00020000c800000100000000000000000000000000000000
0e803940c9001c9dc8000029c800000110880540c9141c9dc9000001c8000001
10020900c9101c9dc8020001c800000100000000000000000000000000000000
0e8a0240c9081c9dc8020001c800000100000000000000000000000000000000
10021d00fe041c9dc8000001c800000102020200fe041c9dff080001c8000001
08001c00c8041c9dc8000001c80000010e800240c9141c9dff000001c8000001
0e82020054001c9dc9100001c80000011080014000021c9cc8000001c8000001
000000000000000000000000000000000e800440c9041c9dc8020001c9000001
00000000000000000000000000000000f0001706c8011c9dc8000001c8003fe1
0e810240fe001c9dc9001001c8000001
"
}

SubProgram "d3d11 " {
Keywords { "DIRECTIONAL_COOKIE" }
ConstBuffer "$Globals" 160 // 136 used size, 8 vars
Vector 16 [_LightColor0] 4
Vector 112 [_Color] 4
Float 128 [_Shininess]
Float 132 [_Gloss]
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 1
SetTexture 1 [_SpecMap] 2D 3
SetTexture 2 [_BumpMap] 2D 2
SetTexture 3 [_LightTexture0] 2D 0
// 32 instructions, 3 temp regs, 0 temp arrays:
// ALU 21 float, 0 int, 0 uint
// TEX 4 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0
eefiecedmhfccipkfafcmppnhboeigkoninjgbikabaaaaaajeafaaaaadaaaaaa
cmaaaaaammaaaaaaaaabaaaaejfdeheojiaaaaaaafaaaaaaaiaaaaaaiaaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaadadaaaaimaaaaaaadaaaaaaaaaaaaaaadaaaaaaabaaaaaa
amamaaaaimaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaaahahaaaaimaaaaaa
acaaaaaaaaaaaaaaadaaaaaaadaaaaaaahahaaaafdfgfpfaepfdejfeejepeoaa
feeffiedepepfceeaaklklklepfdeheocmaaaaaaabaaaaaaaiaaaaaacaaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaafdfgfpfegbhcghgfheaaklkl
fdeieefcimaeaaaaeaaaaaaacdabaaaafjaaaaaeegiocaaaaaaaaaaaajaaaaaa
fkaaaaadaagabaaaaaaaaaaafkaaaaadaagabaaaabaaaaaafkaaaaadaagabaaa
acaaaaaafkaaaaadaagabaaaadaaaaaafibiaaaeaahabaaaaaaaaaaaffffaaaa
fibiaaaeaahabaaaabaaaaaaffffaaaafibiaaaeaahabaaaacaaaaaaffffaaaa
fibiaaaeaahabaaaadaaaaaaffffaaaagcbaaaaddcbabaaaabaaaaaagcbaaaad
mcbabaaaabaaaaaagcbaaaadhcbabaaaacaaaaaagcbaaaadhcbabaaaadaaaaaa
gfaaaaadpccabaaaaaaaaaaagiaaaaacadaaaaaabaaaaaahbcaabaaaaaaaaaaa
egbcbaaaadaaaaaaegbcbaaaadaaaaaaeeaaaaafbcaabaaaaaaaaaaaakaabaaa
aaaaaaaadcaaaaajhcaabaaaaaaaaaaaegbcbaaaadaaaaaaagaabaaaaaaaaaaa
egbcbaaaacaaaaaabaaaaaahicaabaaaaaaaaaaaegacbaaaaaaaaaaaegacbaaa
aaaaaaaaeeaaaaaficaabaaaaaaaaaaadkaabaaaaaaaaaaadiaaaaahhcaabaaa
aaaaaaaapgapbaaaaaaaaaaaegacbaaaaaaaaaaaefaaaaajpcaabaaaabaaaaaa
egbabaaaabaaaaaaeghobaaaacaaaaaaaagabaaaacaaaaaadcaaaaapdcaabaaa
abaaaaaahgapbaaaabaaaaaaaceaaaaaaaaaaaeaaaaaaaeaaaaaaaaaaaaaaaaa
aceaaaaaaaaaialpaaaaialpaaaaaaaaaaaaaaaadcaaaaakicaabaaaaaaaaaaa
akaabaiaebaaaaaaabaaaaaaakaabaaaabaaaaaaabeaaaaaaaaaiadpdcaaaaak
icaabaaaaaaaaaaabkaabaiaebaaaaaaabaaaaaabkaabaaaabaaaaaadkaabaaa
aaaaaaaaelaaaaafecaabaaaabaaaaaadkaabaaaaaaaaaaabaaaaaahbcaabaaa
aaaaaaaaegacbaaaabaaaaaaegacbaaaaaaaaaaabaaaaaahccaabaaaaaaaaaaa
egacbaaaabaaaaaaegbcbaaaacaaaaaadeaaaaakdcaabaaaaaaaaaaaegaabaaa
aaaaaaaaaceaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaacpaaaaafbcaabaaa
aaaaaaaaakaabaaaaaaaaaaadiaaaaaiecaabaaaaaaaaaaaakiacaaaaaaaaaaa
aiaaaaaaabeaaaaaaaaaaaecdiaaaaahbcaabaaaaaaaaaaaakaabaaaaaaaaaaa
ckaabaaaaaaaaaaabjaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaaefaaaaaj
pcaabaaaabaaaaaaegbabaaaabaaaaaaeghobaaaabaaaaaaaagabaaaadaaaaaa
efaaaaajpcaabaaaacaaaaaaegbabaaaabaaaaaaeghobaaaaaaaaaaaaagabaaa
abaaaaaadiaaaaahhcaabaaaabaaaaaaegacbaaaabaaaaaapgapbaaaacaaaaaa
diaaaaaihcaabaaaacaaaaaaegacbaaaacaaaaaaegiccaaaaaaaaaaaahaaaaaa
diaaaaaihcaabaaaacaaaaaaegacbaaaacaaaaaaegiccaaaaaaaaaaaabaaaaaa
diaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaafgifcaaaaaaaaaaaaiaaaaaa
diaaaaahncaabaaaaaaaaaaaagaabaaaaaaaaaaaagajbaaaabaaaaaadiaaaaai
ncaabaaaaaaaaaaaagaobaaaaaaaaaaaagijcaaaaaaaaaaaabaaaaaadcaaaaaj
hcaabaaaaaaaaaaaegacbaaaacaaaaaafgafbaaaaaaaaaaaigadbaaaaaaaaaaa
efaaaaajpcaabaaaabaaaaaaogbkbaaaabaaaaaaeghobaaaadaaaaaaaagabaaa
aaaaaaaaaaaaaaahicaabaaaaaaaaaaadkaabaaaabaaaaaadkaabaaaabaaaaaa
diaaaaahhccabaaaaaaaaaaapgapbaaaaaaaaaaaegacbaaaaaaaaaaadgaaaaaf
iccabaaaaaaaaaaaabeaaaaaaaaaaaaadoaaaaab"
}

SubProgram "gles " {
Keywords { "DIRECTIONAL_COOKIE" }
"!!GLES"
}

SubProgram "glesdesktop " {
Keywords { "DIRECTIONAL_COOKIE" }
"!!GLES"
}

SubProgram "flash " {
Keywords { "DIRECTIONAL_COOKIE" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_LightTexture0] 2D
"agal_ps
c4 2.0 -1.0 1.0 0.0
c5 32.0 0.0 0.0 0.0
[bc]
ciaaaaaaabaaapacaaaaaaoeaeaaaaaaacaaaaaaafaababb tex r1, v0, s2 <2d wrap linear point>
ciaaaaaaacaaapacaaaaaaoeaeaaaaaaaaaaaaaaafaababb tex r2, v0, s0 <2d wrap linear point>
ciaaaaaaadaaapacaaaaaaoeaeaaaaaaabaaaaaaafaababb tex r3, v0, s1 <2d wrap linear point>
ciaaaaaaaaaaapacadaaaaoeaeaaaaaaadaaaaaaafaababb tex r0, v3, s3 <2d wrap linear point>
bcaaaaaaabaaabacacaaaaoeaeaaaaaaacaaaaoeaeaaaaaa dp3 r1.x, v2, v2
adaaaaaaacaaahacacaaaakeacaaaaaaabaaaaoeabaaaaaa mul r2.xyz, r2.xyzz, c1
aaaaaaaaaaaaacacabaaaaffacaaaaaaaaaaaaaaaaaaaaaa mov r0.y, r1.y
aaaaaaaaaaaaabacabaaaappacaaaaaaaaaaaaaaaaaaaaaa mov r0.x, r1.w
adaaaaaaaeaaadacaaaaaafeacaaaaaaaeaaaaaaabaaaaaa mul r4.xy, r0.xyyy, c4.x
abaaaaaaaeaaadacaeaaaafeacaaaaaaaeaaaaffabaaaaaa add r4.xy, r4.xyyy, c4.y
adaaaaaaaaaaabacaeaaaaffacaaaaaaaeaaaaffacaaaaaa mul r0.x, r4.y, r4.y
bfaaaaaaadaaaiacaeaaaaaaacaaaaaaaaaaaaaaaaaaaaaa neg r3.w, r4.x
adaaaaaaadaaaiacadaaaappacaaaaaaaeaaaaaaacaaaaaa mul r3.w, r3.w, r4.x
acaaaaaaaaaaabacadaaaappacaaaaaaaaaaaaaaacaaaaaa sub r0.x, r3.w, r0.x
abaaaaaaaaaaabacaaaaaaaaacaaaaaaaeaaaakkabaaaaaa add r0.x, r0.x, c4.z
akaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r0.x, r0.x
afaaaaaaaeaaaeacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rcp r4.z, r0.x
aaaaaaaaaaaaabacacaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0.x, c2
akaaaaaaabaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r1.x, r1.x
aaaaaaaaafaaahacabaaaaoeaeaaaaaaaaaaaaaaaaaaaaaa mov r5.xyz, v1
adaaaaaaagaaahacabaaaaaaacaaaaaaacaaaaoeaeaaaaaa mul r6.xyz, r1.x, v2
abaaaaaaafaaahacagaaaakeacaaaaaaafaaaakeacaaaaaa add r5.xyz, r6.xyzz, r5.xyzz
bcaaaaaaabaaabacafaaaakeacaaaaaaafaaaakeacaaaaaa dp3 r1.x, r5.xyzz, r5.xyzz
akaaaaaaabaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r1.x, r1.x
adaaaaaaabaaahacabaaaaaaacaaaaaaafaaaakeacaaaaaa mul r1.xyz, r1.x, r5.xyzz
bcaaaaaaabaaabacaeaaaakeacaaaaaaabaaaakeacaaaaaa dp3 r1.x, r4.xyzz, r1.xyzz
adaaaaaaaaaaabacafaaaaoeabaaaaaaaaaaaaaaacaaaaaa mul r0.x, c5, r0.x
ahaaaaaaabaaabacabaaaaaaacaaaaaaaeaaaappabaaaaaa max r1.x, r1.x, c4.w
alaaaaaaafaaapacabaaaaaaacaaaaaaaaaaaaaaacaaaaaa pow r5, r1.x, r0.x
adaaaaaaaaaaahacacaaaappacaaaaaaadaaaakeacaaaaaa mul r0.xyz, r2.w, r3.xyzz
adaaaaaaabaaahacaaaaaakeacaaaaaaadaaaaaaabaaaaaa mul r1.xyz, r0.xyzz, c3.x
aaaaaaaaaaaaabacafaaaaaaacaaaaaaaaaaaaaaaaaaaaaa mov r0.x, r5.x
adaaaaaaaaaaahacaaaaaaaaacaaaaaaabaaaakeacaaaaaa mul r0.xyz, r0.x, r1.xyzz
adaaaaaaabaaahacaaaaaakeacaaaaaaaaaaaaoeabaaaaaa mul r1.xyz, r0.xyzz, c0
bcaaaaaaaaaaabacaeaaaakeacaaaaaaabaaaaoeaeaaaaaa dp3 r0.x, r4.xyzz, v1
ahaaaaaaaaaaabacaaaaaaaaacaaaaaaaeaaaappabaaaaaa max r0.x, r0.x, c4.w
adaaaaaaacaaahacacaaaakeacaaaaaaaaaaaaoeabaaaaaa mul r2.xyz, r2.xyzz, c0
adaaaaaaaaaaahacacaaaakeacaaaaaaaaaaaaaaacaaaaaa mul r0.xyz, r2.xyzz, r0.x
abaaaaaaaaaaahacaaaaaakeacaaaaaaabaaaakeacaaaaaa add r0.xyz, r0.xyzz, r1.xyzz
adaaaaaaaaaaahacaaaaaappacaaaaaaaaaaaakeacaaaaaa mul r0.xyz, r0.w, r0.xyzz
adaaaaaaaaaaahacaaaaaakeacaaaaaaaeaaaaaaabaaaaaa mul r0.xyz, r0.xyzz, c4.x
aaaaaaaaaaaaaiacaeaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0.w, c4
aaaaaaaaaaaaapadaaaaaaoeacaaaaaaaaaaaaaaaaaaaaaa mov o0, r0
"
}

SubProgram "d3d11_9x " {
Keywords { "DIRECTIONAL_COOKIE" }
ConstBuffer "$Globals" 160 // 136 used size, 8 vars
Vector 16 [_LightColor0] 4
Vector 112 [_Color] 4
Float 128 [_Shininess]
Float 132 [_Gloss]
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 1
SetTexture 1 [_SpecMap] 2D 3
SetTexture 2 [_BumpMap] 2D 2
SetTexture 3 [_LightTexture0] 2D 0
// 32 instructions, 3 temp regs, 0 temp arrays:
// ALU 21 float, 0 int, 0 uint
// TEX 4 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0_level_9_3
eefiecedhhjfijnabgdghjkhfjefdmhecakjbmhnabaaaaaahiaiaaaaaeaaaaaa
daaaaaaabaadaaaakeahaaaaeeaiaaaaebgpgodjniacaaaaniacaaaaaaacpppp
imacaaaaemaaaaaaacaadeaaaaaaemaaaaaaemaaaeaaceaaaaaaemaaadaaaaaa
aaababaaacacacaaabadadaaaaaaabaaabaaaaaaaaaaaaaaaaaaahaaacaaabaa
aaaaaaaaabacppppfbaaaaafadaaapkaaaaaaaeaaaaaialpaaaaiadpaaaaaaaa
fbaaaaafaeaaapkaaaaaaaecaaaaaaaaaaaaaaaaaaaaaaaabpaaaaacaaaaaaia
aaaaaplabpaaaaacaaaaaaiaabaachlabpaaaaacaaaaaaiaacaachlabpaaaaac
aaaaaajaaaaiapkabpaaaaacaaaaaajaabaiapkabpaaaaacaaaaaajaacaiapka
bpaaaaacaaaaaajaadaiapkaaiaaaaadaaaaciiaacaaoelaacaaoelaahaaaaac
aaaacbiaaaaappiaabaaaaacabaaahiaacaaoelaaeaaaaaeaaaachiaabaaoeia
aaaaaaiaabaaoelaceaaaaacabaachiaaaaaoeiaabaaaaacaaaaadiaaaaaolla
ecaaaaadacaacpiaaaaaoelaacaioekaecaaaaadaaaacpiaaaaaoeiaaaaioeka
aeaaaaaeaaaacdiaacaaohiaadaaaakaadaaffkaaeaaaaaeabaaciiaaaaaaaia
aaaaaaibadaakkkaaeaaaaaeabaaciiaaaaaffiaaaaaffibabaappiaahaaaaac
abaaciiaabaappiaagaaaaacaaaaceiaabaappiaaiaaaaadabaacbiaaaaaoeia
abaaoeiaaiaaaaadaaaacbiaaaaaoeiaabaaoelaalaaaaadabaacciaaaaaaaia
adaappkaalaaaaadaaaaabiaabaaaaiaadaappkaabaaaaacabaaabiaacaaaaka
afaaaaadaaaaaciaabaaaaiaaeaaaakacaaaaaadabaaabiaaaaaaaiaaaaaffia
ecaaaaadacaacpiaaaaaoelaabaioekaecaaaaadadaacpiaaaaaoelaadaioeka
afaaaaadaaaachiaacaappiaadaaoeiaafaaaaadacaachiaacaaoeiaabaaoeka
afaaaaadacaachiaacaaoeiaaaaaoekaafaaaaadaaaachiaaaaaoeiaacaaffka
afaaaaadaaaachiaaaaaoeiaabaaaaiaafaaaaadaaaachiaaaaaoeiaaaaaoeka
aeaaaaaeaaaachiaacaaoeiaabaaffiaaaaaoeiaacaaaaadaaaaciiaaaaappia
aaaappiaafaaaaadaaaachiaaaaappiaaaaaoeiaabaaaaacaaaaciiaadaappka
abaaaaacaaaicpiaaaaaoeiappppaaaafdeieefcimaeaaaaeaaaaaaacdabaaaa
fjaaaaaeegiocaaaaaaaaaaaajaaaaaafkaaaaadaagabaaaaaaaaaaafkaaaaad
aagabaaaabaaaaaafkaaaaadaagabaaaacaaaaaafkaaaaadaagabaaaadaaaaaa
fibiaaaeaahabaaaaaaaaaaaffffaaaafibiaaaeaahabaaaabaaaaaaffffaaaa
fibiaaaeaahabaaaacaaaaaaffffaaaafibiaaaeaahabaaaadaaaaaaffffaaaa
gcbaaaaddcbabaaaabaaaaaagcbaaaadmcbabaaaabaaaaaagcbaaaadhcbabaaa
acaaaaaagcbaaaadhcbabaaaadaaaaaagfaaaaadpccabaaaaaaaaaaagiaaaaac
adaaaaaabaaaaaahbcaabaaaaaaaaaaaegbcbaaaadaaaaaaegbcbaaaadaaaaaa
eeaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaadcaaaaajhcaabaaaaaaaaaaa
egbcbaaaadaaaaaaagaabaaaaaaaaaaaegbcbaaaacaaaaaabaaaaaahicaabaaa
aaaaaaaaegacbaaaaaaaaaaaegacbaaaaaaaaaaaeeaaaaaficaabaaaaaaaaaaa
dkaabaaaaaaaaaaadiaaaaahhcaabaaaaaaaaaaapgapbaaaaaaaaaaaegacbaaa
aaaaaaaaefaaaaajpcaabaaaabaaaaaaegbabaaaabaaaaaaeghobaaaacaaaaaa
aagabaaaacaaaaaadcaaaaapdcaabaaaabaaaaaahgapbaaaabaaaaaaaceaaaaa
aaaaaaeaaaaaaaeaaaaaaaaaaaaaaaaaaceaaaaaaaaaialpaaaaialpaaaaaaaa
aaaaaaaadcaaaaakicaabaaaaaaaaaaaakaabaiaebaaaaaaabaaaaaaakaabaaa
abaaaaaaabeaaaaaaaaaiadpdcaaaaakicaabaaaaaaaaaaabkaabaiaebaaaaaa
abaaaaaabkaabaaaabaaaaaadkaabaaaaaaaaaaaelaaaaafecaabaaaabaaaaaa
dkaabaaaaaaaaaaabaaaaaahbcaabaaaaaaaaaaaegacbaaaabaaaaaaegacbaaa
aaaaaaaabaaaaaahccaabaaaaaaaaaaaegacbaaaabaaaaaaegbcbaaaacaaaaaa
deaaaaakdcaabaaaaaaaaaaaegaabaaaaaaaaaaaaceaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaacpaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaadiaaaaai
ecaabaaaaaaaaaaaakiacaaaaaaaaaaaaiaaaaaaabeaaaaaaaaaaaecdiaaaaah
bcaabaaaaaaaaaaaakaabaaaaaaaaaaackaabaaaaaaaaaaabjaaaaafbcaabaaa
aaaaaaaaakaabaaaaaaaaaaaefaaaaajpcaabaaaabaaaaaaegbabaaaabaaaaaa
eghobaaaabaaaaaaaagabaaaadaaaaaaefaaaaajpcaabaaaacaaaaaaegbabaaa
abaaaaaaeghobaaaaaaaaaaaaagabaaaabaaaaaadiaaaaahhcaabaaaabaaaaaa
egacbaaaabaaaaaapgapbaaaacaaaaaadiaaaaaihcaabaaaacaaaaaaegacbaaa
acaaaaaaegiccaaaaaaaaaaaahaaaaaadiaaaaaihcaabaaaacaaaaaaegacbaaa
acaaaaaaegiccaaaaaaaaaaaabaaaaaadiaaaaaihcaabaaaabaaaaaaegacbaaa
abaaaaaafgifcaaaaaaaaaaaaiaaaaaadiaaaaahncaabaaaaaaaaaaaagaabaaa
aaaaaaaaagajbaaaabaaaaaadiaaaaaincaabaaaaaaaaaaaagaobaaaaaaaaaaa
agijcaaaaaaaaaaaabaaaaaadcaaaaajhcaabaaaaaaaaaaaegacbaaaacaaaaaa
fgafbaaaaaaaaaaaigadbaaaaaaaaaaaefaaaaajpcaabaaaabaaaaaaogbkbaaa
abaaaaaaeghobaaaadaaaaaaaagabaaaaaaaaaaaaaaaaaahicaabaaaaaaaaaaa
dkaabaaaabaaaaaadkaabaaaabaaaaaadiaaaaahhccabaaaaaaaaaaapgapbaaa
aaaaaaaaegacbaaaaaaaaaaadgaaaaaficcabaaaaaaaaaaaabeaaaaaaaaaaaaa
doaaaaabejfdeheojiaaaaaaafaaaaaaaiaaaaaaiaaaaaaaaaaaaaaaabaaaaaa
adaaaaaaaaaaaaaaapaaaaaaimaaaaaaaaaaaaaaaaaaaaaaadaaaaaaabaaaaaa
adadaaaaimaaaaaaadaaaaaaaaaaaaaaadaaaaaaabaaaaaaamamaaaaimaaaaaa
abaaaaaaaaaaaaaaadaaaaaaacaaaaaaahahaaaaimaaaaaaacaaaaaaaaaaaaaa
adaaaaaaadaaaaaaahahaaaafdfgfpfaepfdejfeejepeoaafeeffiedepepfcee
aaklklklepfdeheocmaaaaaaabaaaaaaaiaaaaaacaaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaaaaaaaaaapaaaaaafdfgfpfegbhcghgfheaaklkl"
}

}
	}
	Pass {
		Name "PREPASS"
		Tags { "LightMode" = "PrePassBase" }
		Fog {Mode Off}
Program "vp" {
// Vertex combos: 1
//   opengl - ALU: 21 to 21
//   d3d9 - ALU: 22 to 22
//   d3d11 - ALU: 15 to 15, TEX: 0 to 0, FLOW: 1 to 1
//   d3d11_9x - ALU: 15 to 15, TEX: 0 to 0, FLOW: 1 to 1
SubProgram "opengl " {
Keywords { }
Bind "vertex" Vertex
Bind "tangent" ATTR14
Bind "normal" Normal
Bind "texcoord" TexCoord0
Matrix 5 [_Object2World]
Vector 9 [unity_Scale]
Vector 10 [_MainTex_ST]
"!!ARBvp1.0
# 21 ALU
PARAM c[11] = { program.local[0],
		state.matrix.mvp,
		program.local[5..10] };
TEMP R0;
TEMP R1;
MOV R0.xyz, vertex.attrib[14];
MUL R1.xyz, vertex.normal.zxyw, R0.yzxw;
MAD R0.xyz, vertex.normal.yzxw, R0.zxyw, -R1;
MUL R1.xyz, R0, vertex.attrib[14].w;
DP3 R0.y, R1, c[5];
DP3 R0.x, vertex.attrib[14], c[5];
DP3 R0.z, vertex.normal, c[5];
MUL result.texcoord[1].xyz, R0, c[9].w;
DP3 R0.y, R1, c[6];
DP3 R0.x, vertex.attrib[14], c[6];
DP3 R0.z, vertex.normal, c[6];
MUL result.texcoord[2].xyz, R0, c[9].w;
DP3 R0.y, R1, c[7];
DP3 R0.x, vertex.attrib[14], c[7];
DP3 R0.z, vertex.normal, c[7];
MUL result.texcoord[3].xyz, R0, c[9].w;
MAD result.texcoord[0].xy, vertex.texcoord[0], c[10], c[10].zwzw;
DP4 result.position.w, vertex.position, c[4];
DP4 result.position.z, vertex.position, c[3];
DP4 result.position.y, vertex.position, c[2];
DP4 result.position.x, vertex.position, c[1];
END
# 21 instructions, 2 R-regs
"
}

SubProgram "d3d9 " {
Keywords { }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Matrix 0 [glstate_matrix_mvp]
Matrix 4 [_Object2World]
Vector 8 [unity_Scale]
Vector 9 [_MainTex_ST]
"vs_2_0
; 22 ALU
dcl_position0 v0
dcl_tangent0 v1
dcl_normal0 v2
dcl_texcoord0 v3
mov r0.xyz, v1
mul r1.xyz, v2.zxyw, r0.yzxw
mov r0.xyz, v1
mad r0.xyz, v2.yzxw, r0.zxyw, -r1
mul r1.xyz, r0, v1.w
dp3 r0.y, r1, c4
dp3 r0.x, v1, c4
dp3 r0.z, v2, c4
mul oT1.xyz, r0, c8.w
dp3 r0.y, r1, c5
dp3 r0.x, v1, c5
dp3 r0.z, v2, c5
mul oT2.xyz, r0, c8.w
dp3 r0.y, r1, c6
dp3 r0.x, v1, c6
dp3 r0.z, v2, c6
mul oT3.xyz, r0, c8.w
mad oT0.xy, v3, c9, c9.zwzw
dp4 oPos.w, v0, c3
dp4 oPos.z, v0, c2
dp4 oPos.y, v0, c1
dp4 oPos.x, v0, c0
"
}

SubProgram "xbox360 " {
Keywords { }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 8 [_MainTex_ST]
Matrix 4 [_Object2World] 3
Matrix 0 [glstate_matrix_mvp] 4
Vector 7 [unity_Scale]
// Shader Timing Estimate, in Cycles/64 vertex vector:
// ALU: 33.33 (25 instructions), vertex: 32, texture: 0,
//   sequencer: 16,  9 GPRs, 21 threads,
// Performance (if enough threads): ~33 cycles per vector
// * Vertex cycle estimates are assuming 3 vfetch_minis for every vfetch_full,
//     with <= 32 bytes per vfetch_full group.

"vs_360
backbbabaaaaabfmaaaaabjiaaaaaaaaaaaaaaceaaaaaaaaaaaaabaeaaaaaaaa
aaaaaaaaaaaaaanmaaaaaabmaaaaaamopppoadaaaaaaaaaeaaaaaabmaaaaaaaa
aaaaaamhaaaaaagmaaacaaaiaaabaaaaaaaaaahiaaaaaaaaaaaaaaiiaaacaaae
aaadaaaaaaaaaajiaaaaaaaaaaaaaakiaaacaaaaaaaeaaaaaaaaaajiaaaaaaaa
aaaaaallaaacaaahaaabaaaaaaaaaahiaaaaaaaafpengbgjgofegfhifpfdfeaa
aaabaaadaaabaaaeaaabaaaaaaaaaaaafpepgcgkgfgdhedcfhgphcgmgeaaklkl
aaadaaadaaaeaaaeaaabaaaaaaaaaaaaghgmhdhegbhegffpgngbhehcgjhifpgn
hghaaahfgogjhehjfpfdgdgbgmgfaahghdfpddfpdaaadccodacodcdadddfddco
daaaklklaaaaaaaaaaaaabjiaadbaaaiaaaaaaaaaaaaaaaaaaaacmieaaaaaaab
aaaaaaaeaaaaaaaeaaaaacjaaabaaaaeaaaagaafaaaadaagaacafaahaaaadafa
aaabhbfbaaachcfcaaadhdfdaaaabablaaaababkaaaababpaaaabacapaffeaae
aaaabcaamcaaaaaaaaaaeaaiaaaabcaameaaaaaaaaaagaamgabcbcaabcaaaaaa
aaaagabidabobcaaccaaaaaaafpicaaaaaaaagiiaaaaaaaaafpihaaaaaaaagii
aaaaaaaaafpiaaaaaaaaacihaaaaaaaaafpidaaaaaaaacdpaaaaaaaakmbpagab
aabliiehkbacadaemiapaaabaamgiiaaklacacabmiapaaabaalbdejeklacabab
miapiadoaagmaadeklacaaabbeceabagaablgmblkbahagahkmchagacaamgloeb
ibaaagafkibhabaeaamgmamdibahagafkichabaiaalbgcidibahafafkiehabaf
aabcgfedmbaaahafmiahaaafablhlomaolaaahafmialaaabaalbloleklaaaeab
miahaaaaaagmloleklahaeaibeaeaaabaagmgmgmoaabacaaaebbabacaamglbmg
oaaaaeaebeaeaaaeaalblblboaabacaaaebhaeaaaaloblgmobafahaekiicacae
aalolomanaagafagkiidaaadaamglciaibaaafagmiahiaabaamablaakbaeahaa
miadiaaaaabklabkiladaiaimiadaaaaaalblclaklaaaeadbeacaaabaagmblbl
oaaaacabaeecacacaalbblmgoaaaaaacmiahiaacaamablaakbacahaamiahiaad
aamablaakbabahaaaaaaaaaaaaaaaaaaaaaaaaaa"
}

SubProgram "ps3 " {
Keywords { }
Matrix 256 [glstate_matrix_mvp]
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Matrix 260 [_Object2World]
Vector 467 [unity_Scale]
Vector 466 [_MainTex_ST]
"sce_vp_rsx // 21 instructions using 5 registers
[Configuration]
8
0000001541050500
[Microcode]
336
00019c6c00400e0c0106c0836041dffc401f9c6c011d2808010400d740619f9c
401f9c6c01d0300d8106c0c360403f80401f9c6c01d0200d8106c0c360405f80
401f9c6c01d0100d8106c0c360409f80401f9c6c01d0000d8106c0c360411f80
00001c6c0150620c010600c360405ffc00001c6c01506e0c010600c360411ffc
00009c6c0150520c010600c360405ffc00009c6c01505e0c010600c360411ffc
00011c6c0150420c010600c360405ffc00021c6c00800243011843436041dffc
00019c6c01000230812183630221dffc00011c6c01504e0c010600c360411ffc
00019c6c00800e0c06bfc0836041dffc00001c6c0150600c068600c360409ffc
00011c6c0150400c068600c360409ffc00009c6c0150500c068600c360409ffc
401f9c6c009d300c04bfc0c36041dfa0401f9c6c009d300c02bfc0c36041dfa4
401f9c6c009d300c00bfc0c36041dfa9
"
}

SubProgram "d3d11 " {
Keywords { }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 96 // 96 used size, 7 vars
Vector 80 [_MainTex_ST] 4
ConstBuffer "UnityPerDraw" 336 // 336 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
Matrix 192 [_Object2World] 4
Vector 320 [unity_Scale] 4
BindCB "$Globals" 0
BindCB "UnityPerDraw" 1
// 30 instructions, 3 temp regs, 0 temp arrays:
// ALU 15 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0
eefieceddofdaliokfballcidnldemeeghikookjabaaaaaajiafaaaaadaaaaaa
cmaaaaaapeaaaaaajeabaaaaejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaa
abaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaljaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfc
enebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheojiaaaaaaafaaaaaa
aiaaaaaaiaaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaabaaaaaaadamaaaaimaaaaaaabaaaaaaaaaaaaaa
adaaaaaaacaaaaaaahaiaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaa
ahaiaaaaimaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaahaiaaaafdfgfpfa
epfdejfeejepeoaafeeffiedepepfceeaaklklklfdeieefcpmadaaaaeaaaabaa
ppaaaaaafjaaaaaeegiocaaaaaaaaaaaagaaaaaafjaaaaaeegiocaaaabaaaaaa
bfaaaaaafpaaaaadpcbabaaaaaaaaaaafpaaaaadpcbabaaaabaaaaaafpaaaaad
hcbabaaaacaaaaaafpaaaaaddcbabaaaadaaaaaaghaaaaaepccabaaaaaaaaaaa
abaaaaaagfaaaaaddccabaaaabaaaaaagfaaaaadhccabaaaacaaaaaagfaaaaad
hccabaaaadaaaaaagfaaaaadhccabaaaaeaaaaaagiaaaaacadaaaaaadiaaaaai
pcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaabaaaaaaabaaaaaadcaaaaak
pcaabaaaaaaaaaaaegiocaaaabaaaaaaaaaaaaaaagbabaaaaaaaaaaaegaobaaa
aaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaabaaaaaaacaaaaaakgbkbaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaakpccabaaaaaaaaaaaegiocaaaabaaaaaa
adaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaaldccabaaaabaaaaaa
egbabaaaadaaaaaaegiacaaaaaaaaaaaafaaaaaaogikcaaaaaaaaaaaafaaaaaa
diaaaaahhcaabaaaaaaaaaaajgbebaaaabaaaaaacgbjbaaaacaaaaaadcaaaaak
hcaabaaaaaaaaaaajgbebaaaacaaaaaacgbjbaaaabaaaaaaegacbaiaebaaaaaa
aaaaaaaadiaaaaahhcaabaaaaaaaaaaaegacbaaaaaaaaaaapgbpbaaaabaaaaaa
dgaaaaagbcaabaaaabaaaaaaakiacaaaabaaaaaaamaaaaaadgaaaaagccaabaaa
abaaaaaaakiacaaaabaaaaaaanaaaaaadgaaaaagecaabaaaabaaaaaaakiacaaa
abaaaaaaaoaaaaaabaaaaaahccaabaaaacaaaaaaegacbaaaaaaaaaaaegacbaaa
abaaaaaabaaaaaahbcaabaaaacaaaaaaegbcbaaaabaaaaaaegacbaaaabaaaaaa
baaaaaahecaabaaaacaaaaaaegbcbaaaacaaaaaaegacbaaaabaaaaaadiaaaaai
hccabaaaacaaaaaaegacbaaaacaaaaaapgipcaaaabaaaaaabeaaaaaadgaaaaag
bcaabaaaabaaaaaabkiacaaaabaaaaaaamaaaaaadgaaaaagccaabaaaabaaaaaa
bkiacaaaabaaaaaaanaaaaaadgaaaaagecaabaaaabaaaaaabkiacaaaabaaaaaa
aoaaaaaabaaaaaahccaabaaaacaaaaaaegacbaaaaaaaaaaaegacbaaaabaaaaaa
baaaaaahbcaabaaaacaaaaaaegbcbaaaabaaaaaaegacbaaaabaaaaaabaaaaaah
ecaabaaaacaaaaaaegbcbaaaacaaaaaaegacbaaaabaaaaaadiaaaaaihccabaaa
adaaaaaaegacbaaaacaaaaaapgipcaaaabaaaaaabeaaaaaadgaaaaagbcaabaaa
abaaaaaackiacaaaabaaaaaaamaaaaaadgaaaaagccaabaaaabaaaaaackiacaaa
abaaaaaaanaaaaaadgaaaaagecaabaaaabaaaaaackiacaaaabaaaaaaaoaaaaaa
baaaaaahccaabaaaaaaaaaaaegacbaaaaaaaaaaaegacbaaaabaaaaaabaaaaaah
bcaabaaaaaaaaaaaegbcbaaaabaaaaaaegacbaaaabaaaaaabaaaaaahecaabaaa
aaaaaaaaegbcbaaaacaaaaaaegacbaaaabaaaaaadiaaaaaihccabaaaaeaaaaaa
egacbaaaaaaaaaaapgipcaaaabaaaaaabeaaaaaadoaaaaab"
}

SubProgram "gles " {
Keywords { }
"!!GLES
#define SHADER_API_GLES 1
#define tex2D texture2D


#ifdef VERTEX
#define gl_ModelViewProjectionMatrix glstate_matrix_mvp
uniform mat4 glstate_matrix_mvp;

varying highp vec3 xlv_TEXCOORD3;
varying highp vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 unity_Scale;
uniform highp mat4 _Object2World;

attribute vec4 _glesTANGENT;
attribute vec4 _glesMultiTexCoord0;
attribute vec3 _glesNormal;
attribute vec4 _glesVertex;
void main ()
{
  vec4 tmpvar_1;
  tmpvar_1.xyz = normalize(_glesTANGENT.xyz);
  tmpvar_1.w = _glesTANGENT.w;
  vec3 tmpvar_2;
  tmpvar_2 = normalize(_glesNormal);
  highp vec3 tmpvar_3;
  highp vec3 tmpvar_4;
  tmpvar_3 = tmpvar_1.xyz;
  tmpvar_4 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_5;
  tmpvar_5[0].x = tmpvar_3.x;
  tmpvar_5[0].y = tmpvar_4.x;
  tmpvar_5[0].z = tmpvar_2.x;
  tmpvar_5[1].x = tmpvar_3.y;
  tmpvar_5[1].y = tmpvar_4.y;
  tmpvar_5[1].z = tmpvar_2.y;
  tmpvar_5[2].x = tmpvar_3.z;
  tmpvar_5[2].y = tmpvar_4.z;
  tmpvar_5[2].z = tmpvar_2.z;
  vec3 v_6;
  v_6.x = _Object2World[0].x;
  v_6.y = _Object2World[1].x;
  v_6.z = _Object2World[2].x;
  vec3 v_7;
  v_7.x = _Object2World[0].y;
  v_7.y = _Object2World[1].y;
  v_7.z = _Object2World[2].y;
  vec3 v_8;
  v_8.x = _Object2World[0].z;
  v_8.y = _Object2World[1].z;
  v_8.z = _Object2World[2].z;
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = ((tmpvar_5 * v_6) * unity_Scale.w);
  xlv_TEXCOORD2 = ((tmpvar_5 * v_7) * unity_Scale.w);
  xlv_TEXCOORD3 = ((tmpvar_5 * v_8) * unity_Scale.w);
}



#endif
#ifdef FRAGMENT

varying highp vec3 xlv_TEXCOORD3;
varying highp vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform mediump float _Shininess;
uniform sampler2D _BumpMap;
void main ()
{
  lowp vec4 res_1;
  lowp vec3 worldN_2;
  mediump vec3 tmpvar_3;
  mediump vec3 tmpvar_4;
  lowp vec3 tmpvar_5;
  tmpvar_5 = ((texture2D (_BumpMap, xlv_TEXCOORD0).xyz * 2.0) - 1.0);
  tmpvar_4 = tmpvar_5;
  highp float tmpvar_6;
  tmpvar_6 = dot (xlv_TEXCOORD1, tmpvar_4);
  worldN_2.x = tmpvar_6;
  highp float tmpvar_7;
  tmpvar_7 = dot (xlv_TEXCOORD2, tmpvar_4);
  worldN_2.y = tmpvar_7;
  highp float tmpvar_8;
  tmpvar_8 = dot (xlv_TEXCOORD3, tmpvar_4);
  worldN_2.z = tmpvar_8;
  tmpvar_3 = worldN_2;
  mediump vec3 tmpvar_9;
  tmpvar_9 = ((tmpvar_3 * 0.5) + 0.5);
  res_1.xyz = tmpvar_9;
  res_1.w = _Shininess;
  gl_FragData[0] = res_1;
}



#endif"
}

SubProgram "glesdesktop " {
Keywords { }
"!!GLES
#define SHADER_API_GLES 1
#define tex2D texture2D


#ifdef VERTEX
#define gl_ModelViewProjectionMatrix glstate_matrix_mvp
uniform mat4 glstate_matrix_mvp;

varying highp vec3 xlv_TEXCOORD3;
varying highp vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 unity_Scale;
uniform highp mat4 _Object2World;

attribute vec4 _glesTANGENT;
attribute vec4 _glesMultiTexCoord0;
attribute vec3 _glesNormal;
attribute vec4 _glesVertex;
void main ()
{
  vec4 tmpvar_1;
  tmpvar_1.xyz = normalize(_glesTANGENT.xyz);
  tmpvar_1.w = _glesTANGENT.w;
  vec3 tmpvar_2;
  tmpvar_2 = normalize(_glesNormal);
  highp vec3 tmpvar_3;
  highp vec3 tmpvar_4;
  tmpvar_3 = tmpvar_1.xyz;
  tmpvar_4 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_5;
  tmpvar_5[0].x = tmpvar_3.x;
  tmpvar_5[0].y = tmpvar_4.x;
  tmpvar_5[0].z = tmpvar_2.x;
  tmpvar_5[1].x = tmpvar_3.y;
  tmpvar_5[1].y = tmpvar_4.y;
  tmpvar_5[1].z = tmpvar_2.y;
  tmpvar_5[2].x = tmpvar_3.z;
  tmpvar_5[2].y = tmpvar_4.z;
  tmpvar_5[2].z = tmpvar_2.z;
  vec3 v_6;
  v_6.x = _Object2World[0].x;
  v_6.y = _Object2World[1].x;
  v_6.z = _Object2World[2].x;
  vec3 v_7;
  v_7.x = _Object2World[0].y;
  v_7.y = _Object2World[1].y;
  v_7.z = _Object2World[2].y;
  vec3 v_8;
  v_8.x = _Object2World[0].z;
  v_8.y = _Object2World[1].z;
  v_8.z = _Object2World[2].z;
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = ((tmpvar_5 * v_6) * unity_Scale.w);
  xlv_TEXCOORD2 = ((tmpvar_5 * v_7) * unity_Scale.w);
  xlv_TEXCOORD3 = ((tmpvar_5 * v_8) * unity_Scale.w);
}



#endif
#ifdef FRAGMENT

varying highp vec3 xlv_TEXCOORD3;
varying highp vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform mediump float _Shininess;
uniform sampler2D _BumpMap;
void main ()
{
  lowp vec4 res_1;
  lowp vec3 worldN_2;
  mediump vec3 tmpvar_3;
  mediump vec3 tmpvar_4;
  lowp vec3 normal_5;
  normal_5.xy = ((texture2D (_BumpMap, xlv_TEXCOORD0).wy * 2.0) - 1.0);
  normal_5.z = sqrt(((1.0 - (normal_5.x * normal_5.x)) - (normal_5.y * normal_5.y)));
  tmpvar_4 = normal_5;
  highp float tmpvar_6;
  tmpvar_6 = dot (xlv_TEXCOORD1, tmpvar_4);
  worldN_2.x = tmpvar_6;
  highp float tmpvar_7;
  tmpvar_7 = dot (xlv_TEXCOORD2, tmpvar_4);
  worldN_2.y = tmpvar_7;
  highp float tmpvar_8;
  tmpvar_8 = dot (xlv_TEXCOORD3, tmpvar_4);
  worldN_2.z = tmpvar_8;
  tmpvar_3 = worldN_2;
  mediump vec3 tmpvar_9;
  tmpvar_9 = ((tmpvar_3 * 0.5) + 0.5);
  res_1.xyz = tmpvar_9;
  res_1.w = _Shininess;
  gl_FragData[0] = res_1;
}



#endif"
}

SubProgram "d3d11_9x " {
Keywords { }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 96 // 96 used size, 7 vars
Vector 80 [_MainTex_ST] 4
ConstBuffer "UnityPerDraw" 336 // 336 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
Matrix 192 [_Object2World] 4
Vector 320 [unity_Scale] 4
BindCB "$Globals" 0
BindCB "UnityPerDraw" 1
// 30 instructions, 3 temp regs, 0 temp arrays:
// ALU 15 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0_level_9_3
eefiecedmjfnjcgbfmlmimjocndekmdmcpapbdfbabaaaaaacaaiaaaaaeaaaaaa
daaaaaaaleacaaaaliagaaaaiaahaaaaebgpgodjhmacaaaahmacaaaaaaacpopp
ceacaaaafiaaaaaaaeaaceaaaaaafeaaaaaafeaaaaaaceaaabaafeaaaaaaafaa
abaaabaaaaaaaaaaabaaaaaaaeaaacaaaaaaaaaaabaaamaaadaaagaaaaaaaaaa
abaabeaaabaaajaaaaaaaaaaaaaaaaaaabacpoppbpaaaaacafaaaaiaaaaaapja
bpaaaaacafaaabiaabaaapjabpaaaaacafaaaciaacaaapjabpaaaaacafaaadia
adaaapjaaeaaaaaeaaaaadoaadaaoejaabaaoekaabaaookaabaaaaacaaaaahia
abaaoejaafaaaaadabaaahiaaaaamjiaacaancjaaeaaaaaeaaaaahiaacaamjja
aaaanciaabaaoeibafaaaaadaaaaahiaaaaaoeiaabaappjaabaaaaacabaaabia
agaaaakaabaaaaacabaaaciaahaaaakaabaaaaacabaaaeiaaiaaaakaaiaaaaad
acaaaciaaaaaoeiaabaaoeiaaiaaaaadacaaabiaabaaoejaabaaoeiaaiaaaaad
acaaaeiaacaaoejaabaaoeiaafaaaaadabaaahoaacaaoeiaajaappkaabaaaaac
abaaabiaagaaffkaabaaaaacabaaaciaahaaffkaabaaaaacabaaaeiaaiaaffka
aiaaaaadacaaaciaaaaaoeiaabaaoeiaaiaaaaadacaaabiaabaaoejaabaaoeia
aiaaaaadacaaaeiaacaaoejaabaaoeiaafaaaaadacaaahoaacaaoeiaajaappka
abaaaaacabaaabiaagaakkkaabaaaaacabaaaciaahaakkkaabaaaaacabaaaeia
aiaakkkaaiaaaaadaaaaaciaaaaaoeiaabaaoeiaaiaaaaadaaaaabiaabaaoeja
abaaoeiaaiaaaaadaaaaaeiaacaaoejaabaaoeiaafaaaaadadaaahoaaaaaoeia
ajaappkaafaaaaadaaaaapiaaaaaffjaadaaoekaaeaaaaaeaaaaapiaacaaoeka
aaaaaajaaaaaoeiaaeaaaaaeaaaaapiaaeaaoekaaaaakkjaaaaaoeiaaeaaaaae
aaaaapiaafaaoekaaaaappjaaaaaoeiaaeaaaaaeaaaaadmaaaaappiaaaaaoeka
aaaaoeiaabaaaaacaaaaammaaaaaoeiappppaaaafdeieefcpmadaaaaeaaaabaa
ppaaaaaafjaaaaaeegiocaaaaaaaaaaaagaaaaaafjaaaaaeegiocaaaabaaaaaa
bfaaaaaafpaaaaadpcbabaaaaaaaaaaafpaaaaadpcbabaaaabaaaaaafpaaaaad
hcbabaaaacaaaaaafpaaaaaddcbabaaaadaaaaaaghaaaaaepccabaaaaaaaaaaa
abaaaaaagfaaaaaddccabaaaabaaaaaagfaaaaadhccabaaaacaaaaaagfaaaaad
hccabaaaadaaaaaagfaaaaadhccabaaaaeaaaaaagiaaaaacadaaaaaadiaaaaai
pcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaabaaaaaaabaaaaaadcaaaaak
pcaabaaaaaaaaaaaegiocaaaabaaaaaaaaaaaaaaagbabaaaaaaaaaaaegaobaaa
aaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaabaaaaaaacaaaaaakgbkbaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaakpccabaaaaaaaaaaaegiocaaaabaaaaaa
adaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaaldccabaaaabaaaaaa
egbabaaaadaaaaaaegiacaaaaaaaaaaaafaaaaaaogikcaaaaaaaaaaaafaaaaaa
diaaaaahhcaabaaaaaaaaaaajgbebaaaabaaaaaacgbjbaaaacaaaaaadcaaaaak
hcaabaaaaaaaaaaajgbebaaaacaaaaaacgbjbaaaabaaaaaaegacbaiaebaaaaaa
aaaaaaaadiaaaaahhcaabaaaaaaaaaaaegacbaaaaaaaaaaapgbpbaaaabaaaaaa
dgaaaaagbcaabaaaabaaaaaaakiacaaaabaaaaaaamaaaaaadgaaaaagccaabaaa
abaaaaaaakiacaaaabaaaaaaanaaaaaadgaaaaagecaabaaaabaaaaaaakiacaaa
abaaaaaaaoaaaaaabaaaaaahccaabaaaacaaaaaaegacbaaaaaaaaaaaegacbaaa
abaaaaaabaaaaaahbcaabaaaacaaaaaaegbcbaaaabaaaaaaegacbaaaabaaaaaa
baaaaaahecaabaaaacaaaaaaegbcbaaaacaaaaaaegacbaaaabaaaaaadiaaaaai
hccabaaaacaaaaaaegacbaaaacaaaaaapgipcaaaabaaaaaabeaaaaaadgaaaaag
bcaabaaaabaaaaaabkiacaaaabaaaaaaamaaaaaadgaaaaagccaabaaaabaaaaaa
bkiacaaaabaaaaaaanaaaaaadgaaaaagecaabaaaabaaaaaabkiacaaaabaaaaaa
aoaaaaaabaaaaaahccaabaaaacaaaaaaegacbaaaaaaaaaaaegacbaaaabaaaaaa
baaaaaahbcaabaaaacaaaaaaegbcbaaaabaaaaaaegacbaaaabaaaaaabaaaaaah
ecaabaaaacaaaaaaegbcbaaaacaaaaaaegacbaaaabaaaaaadiaaaaaihccabaaa
adaaaaaaegacbaaaacaaaaaapgipcaaaabaaaaaabeaaaaaadgaaaaagbcaabaaa
abaaaaaackiacaaaabaaaaaaamaaaaaadgaaaaagccaabaaaabaaaaaackiacaaa
abaaaaaaanaaaaaadgaaaaagecaabaaaabaaaaaackiacaaaabaaaaaaaoaaaaaa
baaaaaahccaabaaaaaaaaaaaegacbaaaaaaaaaaaegacbaaaabaaaaaabaaaaaah
bcaabaaaaaaaaaaaegbcbaaaabaaaaaaegacbaaaabaaaaaabaaaaaahecaabaaa
aaaaaaaaegbcbaaaacaaaaaaegacbaaaabaaaaaadiaaaaaihccabaaaaeaaaaaa
egacbaaaaaaaaaaapgipcaaaabaaaaaabeaaaaaadoaaaaabejfdeheomaaaaaaa
agaaaaaaaiaaaaaajiaaaaaaaaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaa
kbaaaaaaaaaaaaaaaaaaaaaaadaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaacaaaaaaahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaa
adaaaaaaapadaaaalaaaaaaaabaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaa
ljaaaaaaaaaaaaaaaaaaaaaaadaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeo
aafeebeoehefeofeaaeoepfcenebemaafeeffiedepepfceeaaedepemepfcaakl
epfdeheojiaaaaaaafaaaaaaaiaaaaaaiaaaaaaaaaaaaaaaabaaaaaaadaaaaaa
aaaaaaaaapaaaaaaimaaaaaaaaaaaaaaaaaaaaaaadaaaaaaabaaaaaaadamaaaa
imaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaaahaiaaaaimaaaaaaacaaaaaa
aaaaaaaaadaaaaaaadaaaaaaahaiaaaaimaaaaaaadaaaaaaaaaaaaaaadaaaaaa
aeaaaaaaahaiaaaafdfgfpfaepfdejfeejepeoaafeeffiedepepfceeaaklklkl
"
}

}
Program "fp" {
// Fragment combos: 1
//   opengl - ALU: 12 to 12, TEX: 1 to 1
//   d3d9 - ALU: 13 to 13, TEX: 1 to 1
//   d3d11 - ALU: 4 to 4, TEX: 1 to 1, FLOW: 1 to 1
//   d3d11_9x - ALU: 4 to 4, TEX: 1 to 1, FLOW: 1 to 1
SubProgram "opengl " {
Keywords { }
Float 0 [_Shininess]
SetTexture 0 [_BumpMap] 2D
"!!ARBfp1.0
OPTION ARB_precision_hint_fastest;
# 12 ALU, 1 TEX
PARAM c[2] = { program.local[0],
		{ 2, 1, 0.5 } };
TEMP R0;
TEMP R1;
TEX R0.yw, fragment.texcoord[0], texture[0], 2D;
MAD R0.xy, R0.wyzw, c[1].x, -c[1].y;
MUL R0.z, R0.y, R0.y;
MAD R0.z, -R0.x, R0.x, -R0;
ADD R0.z, R0, c[1].y;
RSQ R0.z, R0.z;
RCP R0.z, R0.z;
DP3 R1.z, fragment.texcoord[3], R0;
DP3 R1.x, R0, fragment.texcoord[1];
DP3 R1.y, R0, fragment.texcoord[2];
MAD result.color.xyz, R1, c[1].z, c[1].z;
MOV result.color.w, c[0].x;
END
# 12 instructions, 2 R-regs
"
}

SubProgram "d3d9 " {
Keywords { }
Float 0 [_Shininess]
SetTexture 0 [_BumpMap] 2D
"ps_2_0
; 13 ALU, 1 TEX
dcl_2d s0
def c1, 2.00000000, -1.00000000, 1.00000000, 0.50000000
dcl t0.xy
dcl t1.xyz
dcl t2.xyz
dcl t3.xyz
texld r0, t0, s0
mov r0.x, r0.w
mad_pp r1.xy, r0, c1.x, c1.y
mul_pp r0.x, r1.y, r1.y
mad_pp r0.x, -r1, r1, -r0
add_pp r0.x, r0, c1.z
rsq_pp r0.x, r0.x
rcp_pp r1.z, r0.x
dp3 r0.z, t3, r1
dp3 r0.x, r1, t1
dp3 r0.y, r1, t2
mad_pp r0.xyz, r0, c1.w, c1.w
mov_pp r0.w, c0.x
mov_pp oC0, r0
"
}

SubProgram "xbox360 " {
Keywords { }
Float 0 [_Shininess]
SetTexture 0 [_BumpMap] 2D
// Shader Timing Estimate, in Cycles/64 pixel vector:
// ALU: 10.67 (8 instructions), vertex: 0, texture: 4,
//   sequencer: 8, interpolator: 16;    5 GPRs, 36 threads,
// Performance (if enough threads): ~16 cycles per vector
// * Texture cycle estimates are assuming an 8bit/component texture with no
//     aniso or trilinear filtering.

"ps_360
backbbaaaaaaabbaaaaaaanaaaaaaaaaaaaaaaceaaaaaaliaaaaaaoaaaaaaaaa
aaaaaaaaaaaaaajaaaaaaabmaaaaaaidppppadaaaaaaaaacaaaaaabmaaaaaaaa
aaaaaahmaaaaaaeeaaadaaaaaaabaaaaaaaaaafaaaaaaaaaaaaaaagaaaacaaaa
aaabaaaaaaaaaagmaaaaaaaafpechfgnhaengbhaaaklklklaaaeaaamaaabaaab
aaabaaaaaaaaaaaafpfdgigjgogjgogfhdhdaaklaaaaaaadaaabaaabaaabaaaa
aaaaaaaahahdfpddfpdaaadccodacodcdadddfddcodaaaklaaaaaaaaaaaaaaab
aaaaaaaaaaaaaaaaaaaaaabeabpmaabaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaeaaaaaaajabaaaaeaaaaaaaaaeaaaaaaaaaaaacmieaaapaaapaaaaaaab
aaaadafaaaaahbfbaaaahcfcaaaahdfdaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
eaaaaaaadpaaaaaalpiaaaaadpiaaaaaaaabbaacaaaabcaameaaaaaaaaaagaad
caajbcaaccaaaaaabaaiaaabbpbpppnjaaaaeaaabeiaiaaaaaaaaagmmcaaaaaa
miadaaaeaagngmmgilaappppmiabaaaaaegngnblnbaeaeppkaeaaeaaaaaaaagm
ocaaaaiamiabaaaaaaloloaapaaeabaamiacaaaaaaloloaapaaeacaamiaeaaaa
aaloloaapaaeadaamiahiaaaaamalblbilaappppaaaaaaaaaaaaaaaaaaaaaaaa
"
}

SubProgram "ps3 " {
Keywords { }
Float 0 [_Shininess]
SetTexture 0 [_BumpMap] 2D
"sce_fp_rsx // 15 instructions using 2 registers
[Configuration]
24
ffffffff0003c020000ffff1000000000000840002000000
[Offsets]
1
_Shininess 1 0
00000040
[Microcode]
240
94001700c8011c9dc8000001c8003fe106820440ce001c9daa02000054020001
00000000000040000000bf80000000001080014000021c9cc8000001c8000001
0000000000000000000000000000000002800240ab041c9cab040000c8000001
02800440c9041c9fc9040001c900000302800340c9001c9d00020000c8000001
00003f8000000000000000000000000008823b4001003c9cc9000001c8000001
e8800500c8011c9dc9040001c8003fe1c4800500c9041c9dc8010001c8003fe1
a2800500c9041c9dc8010001c8003fe10e810440c9001c9d0002000000020000
00003f00000000000000000000000000
"
}

SubProgram "d3d11 " {
Keywords { }
ConstBuffer "$Globals" 96 // 68 used size, 7 vars
Float 64 [_Shininess]
BindCB "$Globals" 0
SetTexture 0 [_BumpMap] 2D 0
// 11 instructions, 2 temp regs, 0 temp arrays:
// ALU 4 float, 0 int, 0 uint
// TEX 1 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0
eefiecedookgjbalfkiieadilchkpgpfoaiejbmlabaaaaaapaacaaaaadaaaaaa
cmaaaaaammaaaaaaaaabaaaaejfdeheojiaaaaaaafaaaaaaaiaaaaaaiaaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaadadaaaaimaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaahahaaaaimaaaaaa
adaaaaaaaaaaaaaaadaaaaaaaeaaaaaaahahaaaafdfgfpfaepfdejfeejepeoaa
feeffiedepepfceeaaklklklepfdeheocmaaaaaaabaaaaaaaiaaaaaacaaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaafdfgfpfegbhcghgfheaaklkl
fdeieefcoiabaaaaeaaaaaaahkaaaaaafjaaaaaeegiocaaaaaaaaaaaafaaaaaa
fkaaaaadaagabaaaaaaaaaaafibiaaaeaahabaaaaaaaaaaaffffaaaagcbaaaad
dcbabaaaabaaaaaagcbaaaadhcbabaaaacaaaaaagcbaaaadhcbabaaaadaaaaaa
gcbaaaadhcbabaaaaeaaaaaagfaaaaadpccabaaaaaaaaaaagiaaaaacacaaaaaa
efaaaaajpcaabaaaaaaaaaaaegbabaaaabaaaaaaeghobaaaaaaaaaaaaagabaaa
aaaaaaaadcaaaaapdcaabaaaaaaaaaaahgapbaaaaaaaaaaaaceaaaaaaaaaaaea
aaaaaaeaaaaaaaaaaaaaaaaaaceaaaaaaaaaialpaaaaialpaaaaaaaaaaaaaaaa
dcaaaaakicaabaaaaaaaaaaaakaabaiaebaaaaaaaaaaaaaaakaabaaaaaaaaaaa
abeaaaaaaaaaiadpdcaaaaakicaabaaaaaaaaaaabkaabaiaebaaaaaaaaaaaaaa
bkaabaaaaaaaaaaadkaabaaaaaaaaaaaelaaaaafecaabaaaaaaaaaaadkaabaaa
aaaaaaaabaaaaaahbcaabaaaabaaaaaaegbcbaaaacaaaaaaegacbaaaaaaaaaaa
baaaaaahccaabaaaabaaaaaaegbcbaaaadaaaaaaegacbaaaaaaaaaaabaaaaaah
ecaabaaaabaaaaaaegbcbaaaaeaaaaaaegacbaaaaaaaaaaadcaaaaaphccabaaa
aaaaaaaaegacbaaaabaaaaaaaceaaaaaaaaaaadpaaaaaadpaaaaaadpaaaaaaaa
aceaaaaaaaaaaadpaaaaaadpaaaaaadpaaaaaaaadgaaaaagiccabaaaaaaaaaaa
akiacaaaaaaaaaaaaeaaaaaadoaaaaab"
}

SubProgram "gles " {
Keywords { }
"!!GLES"
}

SubProgram "glesdesktop " {
Keywords { }
"!!GLES"
}

SubProgram "d3d11_9x " {
Keywords { }
ConstBuffer "$Globals" 96 // 68 used size, 7 vars
Float 64 [_Shininess]
BindCB "$Globals" 0
SetTexture 0 [_BumpMap] 2D 0
// 11 instructions, 2 temp regs, 0 temp arrays:
// ALU 4 float, 0 int, 0 uint
// TEX 1 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0_level_9_3
eefiecedbglafpahihkddmjfobikdphjcmhhdffmabaaaaaaemaeaaaaaeaaaaaa
daaaaaaaiiabaaaahiadaaaabiaeaaaaebgpgodjfaabaaaafaabaaaaaaacpppp
bmabaaaadeaaaaaaabaaciaaaaaadeaaaaaadeaaabaaceaaaaaadeaaaaaaaaaa
aaaaaeaaabaaaaaaaaaaaaaaabacppppfbaaaaafabaaapkaaaaaaaeaaaaaialp
aaaaiadpaaaaaadpbpaaaaacaaaaaaiaaaaaadlabpaaaaacaaaaaaiaabaaahla
bpaaaaacaaaaaaiaacaaahlabpaaaaacaaaaaaiaadaaahlabpaaaaacaaaaaaja
aaaiapkaecaaaaadaaaacpiaaaaaoelaaaaioekaaeaaaaaeaaaacdiaaaaaohia
abaaaakaabaaffkaaeaaaaaeaaaaciiaaaaaaaiaaaaaaaibabaakkkaaeaaaaae
aaaaciiaaaaaffiaaaaaffibaaaappiaahaaaaacaaaaciiaaaaappiaagaaaaac
aaaaceiaaaaappiaaiaaaaadabaacbiaabaaoelaaaaaoeiaaiaaaaadabaaccia
acaaoelaaaaaoeiaaiaaaaadabaaceiaadaaoelaaaaaoeiaaeaaaaaeaaaachia
abaaoeiaabaappkaabaappkaabaaaaacaaaaciiaaaaaaakaabaaaaacaaaicpia
aaaaoeiappppaaaafdeieefcoiabaaaaeaaaaaaahkaaaaaafjaaaaaeegiocaaa
aaaaaaaaafaaaaaafkaaaaadaagabaaaaaaaaaaafibiaaaeaahabaaaaaaaaaaa
ffffaaaagcbaaaaddcbabaaaabaaaaaagcbaaaadhcbabaaaacaaaaaagcbaaaad
hcbabaaaadaaaaaagcbaaaadhcbabaaaaeaaaaaagfaaaaadpccabaaaaaaaaaaa
giaaaaacacaaaaaaefaaaaajpcaabaaaaaaaaaaaegbabaaaabaaaaaaeghobaaa
aaaaaaaaaagabaaaaaaaaaaadcaaaaapdcaabaaaaaaaaaaahgapbaaaaaaaaaaa
aceaaaaaaaaaaaeaaaaaaaeaaaaaaaaaaaaaaaaaaceaaaaaaaaaialpaaaaialp
aaaaaaaaaaaaaaaadcaaaaakicaabaaaaaaaaaaaakaabaiaebaaaaaaaaaaaaaa
akaabaaaaaaaaaaaabeaaaaaaaaaiadpdcaaaaakicaabaaaaaaaaaaabkaabaia
ebaaaaaaaaaaaaaabkaabaaaaaaaaaaadkaabaaaaaaaaaaaelaaaaafecaabaaa
aaaaaaaadkaabaaaaaaaaaaabaaaaaahbcaabaaaabaaaaaaegbcbaaaacaaaaaa
egacbaaaaaaaaaaabaaaaaahccaabaaaabaaaaaaegbcbaaaadaaaaaaegacbaaa
aaaaaaaabaaaaaahecaabaaaabaaaaaaegbcbaaaaeaaaaaaegacbaaaaaaaaaaa
dcaaaaaphccabaaaaaaaaaaaegacbaaaabaaaaaaaceaaaaaaaaaaadpaaaaaadp
aaaaaadpaaaaaaaaaceaaaaaaaaaaadpaaaaaadpaaaaaadpaaaaaaaadgaaaaag
iccabaaaaaaaaaaaakiacaaaaaaaaaaaaeaaaaaadoaaaaabejfdeheojiaaaaaa
afaaaaaaaiaaaaaaiaaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaa
imaaaaaaaaaaaaaaaaaaaaaaadaaaaaaabaaaaaaadadaaaaimaaaaaaabaaaaaa
aaaaaaaaadaaaaaaacaaaaaaahahaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaa
adaaaaaaahahaaaaimaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaahahaaaa
fdfgfpfaepfdejfeejepeoaafeeffiedepepfceeaaklklklepfdeheocmaaaaaa
abaaaaaaaiaaaaaacaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaa
fdfgfpfegbhcghgfheaaklkl"
}

}
	}
	Pass {
		Name "PREPASS"
		Tags { "LightMode" = "PrePassFinal" }
		ZWrite Off
Program "vp" {
// Vertex combos: 4
//   opengl - ALU: 20 to 28
//   d3d9 - ALU: 20 to 28
//   d3d11 - ALU: 10 to 15, TEX: 0 to 0, FLOW: 1 to 1
//   d3d11_9x - ALU: 10 to 15, TEX: 0 to 0, FLOW: 1 to 1
SubProgram "opengl " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
Bind "vertex" Vertex
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 9 [_ProjectionParams]
Vector 10 [unity_SHAr]
Vector 11 [unity_SHAg]
Vector 12 [unity_SHAb]
Vector 13 [unity_SHBr]
Vector 14 [unity_SHBg]
Vector 15 [unity_SHBb]
Vector 16 [unity_SHC]
Matrix 5 [_Object2World]
Vector 17 [unity_Scale]
Vector 18 [_MainTex_ST]
"!!ARBvp1.0
# 28 ALU
PARAM c[19] = { { 0.5, 1 },
		state.matrix.mvp,
		program.local[5..18] };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
MUL R1.xyz, vertex.normal, c[17].w;
DP3 R2.w, R1, c[6];
DP3 R0.x, R1, c[5];
DP3 R0.z, R1, c[7];
MOV R0.y, R2.w;
MUL R1, R0.xyzz, R0.yzzx;
MOV R0.w, c[0].y;
DP4 R2.z, R0, c[12];
DP4 R2.y, R0, c[11];
DP4 R2.x, R0, c[10];
MUL R0.y, R2.w, R2.w;
DP4 R3.z, R1, c[15];
DP4 R3.y, R1, c[14];
DP4 R3.x, R1, c[13];
DP4 R1.w, vertex.position, c[4];
DP4 R1.z, vertex.position, c[3];
MAD R0.x, R0, R0, -R0.y;
ADD R3.xyz, R2, R3;
MUL R2.xyz, R0.x, c[16];
DP4 R1.x, vertex.position, c[1];
DP4 R1.y, vertex.position, c[2];
MUL R0.xyz, R1.xyww, c[0].x;
MUL R0.y, R0, c[9].x;
ADD result.texcoord[2].xyz, R3, R2;
ADD result.texcoord[1].xy, R0, R0.z;
MOV result.position, R1;
MOV result.texcoord[1].zw, R1;
MAD result.texcoord[0].xy, vertex.texcoord[0], c[18], c[18].zwzw;
END
# 28 instructions, 4 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
Bind "vertex" Vertex
Bind "normal" Normal
Bind "texcoord" TexCoord0
Matrix 0 [glstate_matrix_mvp]
Vector 8 [_ProjectionParams]
Vector 9 [_ScreenParams]
Vector 10 [unity_SHAr]
Vector 11 [unity_SHAg]
Vector 12 [unity_SHAb]
Vector 13 [unity_SHBr]
Vector 14 [unity_SHBg]
Vector 15 [unity_SHBb]
Vector 16 [unity_SHC]
Matrix 4 [_Object2World]
Vector 17 [unity_Scale]
Vector 18 [_MainTex_ST]
"vs_2_0
; 28 ALU
def c19, 0.50000000, 1.00000000, 0, 0
dcl_position0 v0
dcl_normal0 v1
dcl_texcoord0 v2
mul r1.xyz, v1, c17.w
dp3 r2.w, r1, c5
dp3 r0.x, r1, c4
dp3 r0.z, r1, c6
mov r0.y, r2.w
mul r1, r0.xyzz, r0.yzzx
mov r0.w, c19.y
dp4 r2.z, r0, c12
dp4 r2.y, r0, c11
dp4 r2.x, r0, c10
mul r0.y, r2.w, r2.w
dp4 r3.z, r1, c15
dp4 r3.y, r1, c14
dp4 r3.x, r1, c13
dp4 r1.w, v0, c3
dp4 r1.z, v0, c2
mad r0.x, r0, r0, -r0.y
add r3.xyz, r2, r3
mul r2.xyz, r0.x, c16
dp4 r1.x, v0, c0
dp4 r1.y, v0, c1
mul r0.xyz, r1.xyww, c19.x
mul r0.y, r0, c8.x
add oT2.xyz, r3, r2
mad oT1.xy, r0.z, c9.zwzw, r0
mov oPos, r1
mov oT1.zw, r1
mad oT0.xy, v2, c18, c18.zwzw
"
}

SubProgram "xbox360 " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
Bind "vertex" Vertex
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 17 [_MainTex_ST]
Matrix 13 [_Object2World] 3
Vector 0 [_ProjectionParams]
Vector 1 [_ScreenParams]
Matrix 9 [glstate_matrix_mvp] 4
Vector 4 [unity_SHAb]
Vector 3 [unity_SHAg]
Vector 2 [unity_SHAr]
Vector 7 [unity_SHBb]
Vector 6 [unity_SHBg]
Vector 5 [unity_SHBr]
Vector 8 [unity_SHC]
Vector 16 [unity_Scale]
// Shader Timing Estimate, in Cycles/64 vertex vector:
// ALU: 29.33 (22 instructions), vertex: 32, texture: 0,
//   sequencer: 14,  5 GPRs, 31 threads,
// Performance (if enough threads): ~32 cycles per vector
// * Vertex cycle estimates are assuming 3 vfetch_minis for every vfetch_full,
//     with <= 32 bytes per vfetch_full group.

"vs_360
backbbabaaaaacjmaaaaabkiaaaaaaaaaaaaaaceaaaaacceaaaaacemaaaaaaaa
aaaaaaaaaaaaabpmaaaaaabmaaaaaboopppoadaaaaaaaaanaaaaaabmaaaaaaaa
aaaaabohaaaaabcaaaacaabbaaabaaaaaaaaabcmaaaaaaaaaaaaabdmaaacaaan
aaadaaaaaaaaabemaaaaaaaaaaaaabfmaaacaaaaaaabaaaaaaaaabcmaaaaaaaa
aaaaabgoaaacaaabaaabaaaaaaaaabcmaaaaaaaaaaaaabhmaaacaaajaaaeaaaa
aaaaabemaaaaaaaaaaaaabipaaacaaaeaaabaaaaaaaaabcmaaaaaaaaaaaaabjk
aaacaaadaaabaaaaaaaaabcmaaaaaaaaaaaaabkfaaacaaacaaabaaaaaaaaabcm
aaaaaaaaaaaaablaaaacaaahaaabaaaaaaaaabcmaaaaaaaaaaaaabllaaacaaag
aaabaaaaaaaaabcmaaaaaaaaaaaaabmgaaacaaafaaabaaaaaaaaabcmaaaaaaaa
aaaaabnbaaacaaaiaaabaaaaaaaaabcmaaaaaaaaaaaaabnlaaacaabaaaabaaaa
aaaaabcmaaaaaaaafpengbgjgofegfhifpfdfeaaaaabaaadaaabaaaeaaabaaaa
aaaaaaaafpepgcgkgfgdhedcfhgphcgmgeaaklklaaadaaadaaaeaaaeaaabaaaa
aaaaaaaafpfahcgpgkgfgdhegjgpgofagbhcgbgnhdaafpfdgdhcgfgfgofagbhc
gbgnhdaaghgmhdhegbhegffpgngbhehcgjhifpgnhghaaahfgogjhehjfpfdeieb
gcaahfgogjhehjfpfdeiebghaahfgogjhehjfpfdeiebhcaahfgogjhehjfpfdei
ecgcaahfgogjhehjfpfdeiecghaahfgogjhehjfpfdeiechcaahfgogjhehjfpfd
eiedaahfgogjhehjfpfdgdgbgmgfaahghdfpddfpdaaadccodacodcdadddfddco
daaaklklaaaaaaaaaaaaaaabaaaaaaaaaaaaaaaaaaaaaabeaapmaabaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaeaaaaaabgiaacbaaaeaaaaaaaaaaaaaaaa
aaaacegdaaaaaaabaaaaaaadaaaaaaaeaaaaacjaaabaaaaeaaaadaafaadafaag
aaaadafaaaabpbfbaaadhcfcaaaababcaaaaaabbaaaababkaaaababmaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaadpaaaaaaaaaaaaaaaaaaaaaaaaaaaaaahabfdaae
aaaabcaamcaaaaaaaaaafaahaaaabcaameaaaaaaaaaagaamgabcbcaabcaaaaaa
aaaafabiaaaaccaaaaaaaaaaafpidaaaaaaaagiiaaaaaaaaafpicaaaaaaaaoii
aaaaaaaaafpiaaaaaaaaapmiaaaaaaaamiapaaabaabliiaakbadamaamiapaaab
aamgnapikladalabmiapaaabaalbdepikladakabmiapaaabaagmiipikladajab
miapiadoaaiiiiaaocababaamialaaacaagfblaakbacbaaaceihaeadaalblegm
kbacapiamiahaaacaagmlemaklacaoadmiahaaaeaabllemaklacanacaibhabad
aaligmggkbabppaemiamiaabaaigigaaocababaamiadiaaaaalalabkilaabbbb
aicbabacaadoanmbgpacaeaeaiecabacaadoanlbgpadaeaeaiieabacaadoanlm
gpaeaeaemiabaaaaaakhkhaakpabafaamiacaaaaaakhkhaakpabagaaaibeabaa
aakhkhgmkpabahaeaiciabadaalbgmmgkbadaaaemiadiaabaamgbkbikladabad
geihaaaaaalologboaacaaabmiahiaacaablmagfklaaaiaaaaaaaaaaaaaaaaaa
aaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
Matrix 256 [glstate_matrix_mvp]
Bind "vertex" Vertex
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 467 [_ProjectionParams]
Vector 466 [unity_SHAr]
Vector 465 [unity_SHAg]
Vector 464 [unity_SHAb]
Vector 463 [unity_SHBr]
Vector 462 [unity_SHBg]
Vector 461 [unity_SHBb]
Vector 460 [unity_SHC]
Matrix 260 [_Object2World]
Vector 459 [unity_Scale]
Vector 458 [_MainTex_ST]
"sce_vp_rsx // 27 instructions using 3 registers
[Configuration]
8
0000001b01050300
[Defaults]
1
457 1
3f000000
[Microcode]
432
00009c6c009cb20c013fc0c36041dffc401f9c6c011ca808010400d740619f9c
00001c6c01d0300d8106c0c360403ffc00001c6c01d0200d8106c0c360405ffc
00001c6c01d0100d8106c0c360409ffc00001c6c01d0000d8106c0c360411ffc
00011c6c0150400c028600c360411ffc00011c6c0150600c028600c360405ffc
00009c6c0150500c028600c360411ffc401f9c6c0040000d8086c0836041ff80
401f9c6c004000558086c08360407fa000001c6c009c900e008000c36041dffc
00001c6c009d302a808000c360409ffc00001c6c008000000280014360403ffc
00011c6c004000000286c08360409ffc401f9c6c00c000080086c09540219fa0
00001c6c019d000c0486c0c360405ffc00001c6c019d100c0486c0c360409ffc
00001c6c019d200c0486c0c360411ffc00001c6c010000000480027fe0203ffc
00009c6c0080000d049a02436041fffc00011c6c01dcd00d8286c0c360405ffc
00011c6c01dce00d8286c0c360409ffc00011c6c01dcf00d8286c0c360411ffc
00001c6c00c0000c0086c0830121dffc00009c6c009cc07f808600c36041dffc
401f9c6c00c0000c0286c0830021dfa5
"
}

SubProgram "d3d11 " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
Bind "vertex" Vertex
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 112 // 96 used size, 8 vars
Vector 80 [_MainTex_ST] 4
ConstBuffer "UnityPerCamera" 128 // 96 used size, 8 vars
Vector 80 [_ProjectionParams] 4
ConstBuffer "UnityLighting" 400 // 400 used size, 16 vars
Vector 288 [unity_SHAr] 4
Vector 304 [unity_SHAg] 4
Vector 320 [unity_SHAb] 4
Vector 336 [unity_SHBr] 4
Vector 352 [unity_SHBg] 4
Vector 368 [unity_SHBb] 4
Vector 384 [unity_SHC] 4
ConstBuffer "UnityPerDraw" 336 // 336 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
Matrix 192 [_Object2World] 4
Vector 320 [unity_Scale] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityLighting" 2
BindCB "UnityPerDraw" 3
// 27 instructions, 4 temp regs, 0 temp arrays:
// ALU 15 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0
eefieceddedcifmeeidflkbdcfeldmkmpfimoodaabaaaaaaiiafaaaaadaaaaaa
cmaaaaaapeaaaaaahmabaaaaejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapaaaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaa
abaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaljaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfc
enebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheoiaaaaaaaaeaaaaaa
aiaaaaaagiaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaheaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaabaaaaaaadamaaaaheaaaaaaabaaaaaaaaaaaaaa
adaaaaaaacaaaaaaapaaaaaaheaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaa
ahaiaaaafdfgfpfaepfdejfeejepeoaafeeffiedepepfceeaaklklklfdeieefc
aeaeaaaaeaaaabaaababaaaafjaaaaaeegiocaaaaaaaaaaaagaaaaaafjaaaaae
egiocaaaabaaaaaaagaaaaaafjaaaaaeegiocaaaacaaaaaabjaaaaaafjaaaaae
egiocaaaadaaaaaabfaaaaaafpaaaaadpcbabaaaaaaaaaaafpaaaaadhcbabaaa
acaaaaaafpaaaaaddcbabaaaadaaaaaaghaaaaaepccabaaaaaaaaaaaabaaaaaa
gfaaaaaddccabaaaabaaaaaagfaaaaadpccabaaaacaaaaaagfaaaaadhccabaaa
adaaaaaagiaaaaacaeaaaaaadiaaaaaipcaabaaaaaaaaaaafgbfbaaaaaaaaaaa
egiocaaaadaaaaaaabaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaa
aaaaaaaaagbabaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaa
egiocaaaadaaaaaaacaaaaaakgbkbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaak
pcaabaaaaaaaaaaaegiocaaaadaaaaaaadaaaaaapgbpbaaaaaaaaaaaegaobaaa
aaaaaaaadgaaaaafpccabaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaaldccabaaa
abaaaaaaegbabaaaadaaaaaaegiacaaaaaaaaaaaafaaaaaaogikcaaaaaaaaaaa
afaaaaaadiaaaaaiccaabaaaaaaaaaaabkaabaaaaaaaaaaaakiacaaaabaaaaaa
afaaaaaadiaaaaakncaabaaaabaaaaaaagahbaaaaaaaaaaaaceaaaaaaaaaaadp
aaaaaaaaaaaaaadpaaaaaadpdgaaaaafmccabaaaacaaaaaakgaobaaaaaaaaaaa
aaaaaaahdccabaaaacaaaaaakgakbaaaabaaaaaamgaabaaaabaaaaaadiaaaaai
hcaabaaaaaaaaaaaegbcbaaaacaaaaaapgipcaaaadaaaaaabeaaaaaadiaaaaai
hcaabaaaabaaaaaafgafbaaaaaaaaaaaegiccaaaadaaaaaaanaaaaaadcaaaaak
lcaabaaaaaaaaaaaegiicaaaadaaaaaaamaaaaaaagaabaaaaaaaaaaaegaibaaa
abaaaaaadcaaaaakhcaabaaaaaaaaaaaegiccaaaadaaaaaaaoaaaaaakgakbaaa
aaaaaaaaegadbaaaaaaaaaaadgaaaaaficaabaaaaaaaaaaaabeaaaaaaaaaiadp
bbaaaaaibcaabaaaabaaaaaaegiocaaaacaaaaaabcaaaaaaegaobaaaaaaaaaaa
bbaaaaaiccaabaaaabaaaaaaegiocaaaacaaaaaabdaaaaaaegaobaaaaaaaaaaa
bbaaaaaiecaabaaaabaaaaaaegiocaaaacaaaaaabeaaaaaaegaobaaaaaaaaaaa
diaaaaahpcaabaaaacaaaaaajgacbaaaaaaaaaaaegakbaaaaaaaaaaabbaaaaai
bcaabaaaadaaaaaaegiocaaaacaaaaaabfaaaaaaegaobaaaacaaaaaabbaaaaai
ccaabaaaadaaaaaaegiocaaaacaaaaaabgaaaaaaegaobaaaacaaaaaabbaaaaai
ecaabaaaadaaaaaaegiocaaaacaaaaaabhaaaaaaegaobaaaacaaaaaaaaaaaaah
hcaabaaaabaaaaaaegacbaaaabaaaaaaegacbaaaadaaaaaadiaaaaahccaabaaa
aaaaaaaabkaabaaaaaaaaaaabkaabaaaaaaaaaaadcaaaaakbcaabaaaaaaaaaaa
akaabaaaaaaaaaaaakaabaaaaaaaaaaabkaabaiaebaaaaaaaaaaaaaadcaaaaak
hccabaaaadaaaaaaegiccaaaacaaaaaabiaaaaaaagaabaaaaaaaaaaaegacbaaa
abaaaaaadoaaaaab"
}

SubProgram "gles " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
"!!GLES
#define SHADER_API_GLES 1
#define tex2D texture2D


#ifdef VERTEX
#define gl_ModelViewProjectionMatrix glstate_matrix_mvp
uniform mat4 glstate_matrix_mvp;

varying highp vec3 xlv_TEXCOORD2;
varying highp vec4 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 unity_Scale;
uniform highp mat4 _Object2World;

uniform highp vec4 unity_SHC;
uniform highp vec4 unity_SHBb;
uniform highp vec4 unity_SHBg;
uniform highp vec4 unity_SHBr;
uniform highp vec4 unity_SHAb;
uniform highp vec4 unity_SHAg;
uniform highp vec4 unity_SHAr;
uniform highp vec4 _ProjectionParams;
attribute vec4 _glesMultiTexCoord0;
attribute vec3 _glesNormal;
attribute vec4 _glesVertex;
void main ()
{
  highp vec3 tmpvar_1;
  highp vec4 tmpvar_2;
  tmpvar_2 = (gl_ModelViewProjectionMatrix * _glesVertex);
  highp vec4 o_3;
  highp vec4 tmpvar_4;
  tmpvar_4 = (tmpvar_2 * 0.5);
  highp vec2 tmpvar_5;
  tmpvar_5.x = tmpvar_4.x;
  tmpvar_5.y = (tmpvar_4.y * _ProjectionParams.x);
  o_3.xy = (tmpvar_5 + tmpvar_4.w);
  o_3.zw = tmpvar_2.zw;
  mat3 tmpvar_6;
  tmpvar_6[0] = _Object2World[0].xyz;
  tmpvar_6[1] = _Object2World[1].xyz;
  tmpvar_6[2] = _Object2World[2].xyz;
  highp vec4 tmpvar_7;
  tmpvar_7.w = 1.0;
  tmpvar_7.xyz = (tmpvar_6 * (normalize(_glesNormal) * unity_Scale.w));
  mediump vec3 tmpvar_8;
  mediump vec4 normal_9;
  normal_9 = tmpvar_7;
  highp float vC_10;
  mediump vec3 x3_11;
  mediump vec3 x2_12;
  mediump vec3 x1_13;
  highp float tmpvar_14;
  tmpvar_14 = dot (unity_SHAr, normal_9);
  x1_13.x = tmpvar_14;
  highp float tmpvar_15;
  tmpvar_15 = dot (unity_SHAg, normal_9);
  x1_13.y = tmpvar_15;
  highp float tmpvar_16;
  tmpvar_16 = dot (unity_SHAb, normal_9);
  x1_13.z = tmpvar_16;
  mediump vec4 tmpvar_17;
  tmpvar_17 = (normal_9.xyzz * normal_9.yzzx);
  highp float tmpvar_18;
  tmpvar_18 = dot (unity_SHBr, tmpvar_17);
  x2_12.x = tmpvar_18;
  highp float tmpvar_19;
  tmpvar_19 = dot (unity_SHBg, tmpvar_17);
  x2_12.y = tmpvar_19;
  highp float tmpvar_20;
  tmpvar_20 = dot (unity_SHBb, tmpvar_17);
  x2_12.z = tmpvar_20;
  mediump float tmpvar_21;
  tmpvar_21 = ((normal_9.x * normal_9.x) - (normal_9.y * normal_9.y));
  vC_10 = tmpvar_21;
  highp vec3 tmpvar_22;
  tmpvar_22 = (unity_SHC.xyz * vC_10);
  x3_11 = tmpvar_22;
  tmpvar_8 = ((x1_13 + x2_12) + x3_11);
  tmpvar_1 = tmpvar_8;
  gl_Position = tmpvar_2;
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = o_3;
  xlv_TEXCOORD2 = tmpvar_1;
}



#endif
#ifdef FRAGMENT

varying highp vec3 xlv_TEXCOORD2;
varying highp vec4 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform sampler2D _LightBuffer;
uniform mediump float _Gloss;
uniform lowp vec4 _Color;
uniform sampler2D _SpecMap;
uniform sampler2D _MainTex;
uniform lowp vec4 _SpecColor;
void main ()
{
  lowp vec4 tmpvar_1;
  mediump vec4 light_2;
  mediump vec3 tmpvar_3;
  lowp vec4 tmpvar_4;
  tmpvar_4 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_5;
  tmpvar_5 = (tmpvar_4.xyz * _Color.xyz);
  tmpvar_3 = tmpvar_5;
  lowp vec4 tmpvar_6;
  tmpvar_6 = texture2D (_SpecMap, xlv_TEXCOORD0);
  lowp vec4 tmpvar_7;
  tmpvar_7 = texture2DProj (_LightBuffer, xlv_TEXCOORD1);
  light_2 = tmpvar_7;
  mediump vec4 tmpvar_8;
  tmpvar_8 = -(log2(max (light_2, vec4(0.001, 0.001, 0.001, 0.001))));
  light_2.w = tmpvar_8.w;
  highp vec3 tmpvar_9;
  tmpvar_9 = (tmpvar_8.xyz + xlv_TEXCOORD2);
  light_2.xyz = tmpvar_9;
  mediump vec4 c_10;
  mediump vec3 tmpvar_11;
  tmpvar_11 = (tmpvar_8.w * ((tmpvar_4.w * tmpvar_6.xyz) * _Gloss));
  c_10.xyz = ((tmpvar_3 * light_2.xyz) + (light_2.xyz * tmpvar_11));
  c_10.w = (tmpvar_11 * _SpecColor.w).x;
  tmpvar_1 = c_10;
  gl_FragData[0] = tmpvar_1;
}



#endif"
}

SubProgram "glesdesktop " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
"!!GLES
#define SHADER_API_GLES 1
#define tex2D texture2D


#ifdef VERTEX
#define gl_ModelViewProjectionMatrix glstate_matrix_mvp
uniform mat4 glstate_matrix_mvp;

varying highp vec3 xlv_TEXCOORD2;
varying highp vec4 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 unity_Scale;
uniform highp mat4 _Object2World;

uniform highp vec4 unity_SHC;
uniform highp vec4 unity_SHBb;
uniform highp vec4 unity_SHBg;
uniform highp vec4 unity_SHBr;
uniform highp vec4 unity_SHAb;
uniform highp vec4 unity_SHAg;
uniform highp vec4 unity_SHAr;
uniform highp vec4 _ProjectionParams;
attribute vec4 _glesMultiTexCoord0;
attribute vec3 _glesNormal;
attribute vec4 _glesVertex;
void main ()
{
  highp vec3 tmpvar_1;
  highp vec4 tmpvar_2;
  tmpvar_2 = (gl_ModelViewProjectionMatrix * _glesVertex);
  highp vec4 o_3;
  highp vec4 tmpvar_4;
  tmpvar_4 = (tmpvar_2 * 0.5);
  highp vec2 tmpvar_5;
  tmpvar_5.x = tmpvar_4.x;
  tmpvar_5.y = (tmpvar_4.y * _ProjectionParams.x);
  o_3.xy = (tmpvar_5 + tmpvar_4.w);
  o_3.zw = tmpvar_2.zw;
  mat3 tmpvar_6;
  tmpvar_6[0] = _Object2World[0].xyz;
  tmpvar_6[1] = _Object2World[1].xyz;
  tmpvar_6[2] = _Object2World[2].xyz;
  highp vec4 tmpvar_7;
  tmpvar_7.w = 1.0;
  tmpvar_7.xyz = (tmpvar_6 * (normalize(_glesNormal) * unity_Scale.w));
  mediump vec3 tmpvar_8;
  mediump vec4 normal_9;
  normal_9 = tmpvar_7;
  highp float vC_10;
  mediump vec3 x3_11;
  mediump vec3 x2_12;
  mediump vec3 x1_13;
  highp float tmpvar_14;
  tmpvar_14 = dot (unity_SHAr, normal_9);
  x1_13.x = tmpvar_14;
  highp float tmpvar_15;
  tmpvar_15 = dot (unity_SHAg, normal_9);
  x1_13.y = tmpvar_15;
  highp float tmpvar_16;
  tmpvar_16 = dot (unity_SHAb, normal_9);
  x1_13.z = tmpvar_16;
  mediump vec4 tmpvar_17;
  tmpvar_17 = (normal_9.xyzz * normal_9.yzzx);
  highp float tmpvar_18;
  tmpvar_18 = dot (unity_SHBr, tmpvar_17);
  x2_12.x = tmpvar_18;
  highp float tmpvar_19;
  tmpvar_19 = dot (unity_SHBg, tmpvar_17);
  x2_12.y = tmpvar_19;
  highp float tmpvar_20;
  tmpvar_20 = dot (unity_SHBb, tmpvar_17);
  x2_12.z = tmpvar_20;
  mediump float tmpvar_21;
  tmpvar_21 = ((normal_9.x * normal_9.x) - (normal_9.y * normal_9.y));
  vC_10 = tmpvar_21;
  highp vec3 tmpvar_22;
  tmpvar_22 = (unity_SHC.xyz * vC_10);
  x3_11 = tmpvar_22;
  tmpvar_8 = ((x1_13 + x2_12) + x3_11);
  tmpvar_1 = tmpvar_8;
  gl_Position = tmpvar_2;
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = o_3;
  xlv_TEXCOORD2 = tmpvar_1;
}



#endif
#ifdef FRAGMENT

varying highp vec3 xlv_TEXCOORD2;
varying highp vec4 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform sampler2D _LightBuffer;
uniform mediump float _Gloss;
uniform lowp vec4 _Color;
uniform sampler2D _SpecMap;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform lowp vec4 _SpecColor;
void main ()
{
  lowp vec4 tmpvar_1;
  mediump vec4 light_2;
  mediump vec3 tmpvar_3;
  lowp vec4 tmpvar_4;
  tmpvar_4 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_5;
  tmpvar_5 = (tmpvar_4.xyz * _Color.xyz);
  tmpvar_3 = tmpvar_5;
  lowp vec4 tmpvar_6;
  tmpvar_6 = texture2D (_SpecMap, xlv_TEXCOORD0);
  lowp vec3 normal_7;
  normal_7.xy = ((texture2D (_BumpMap, xlv_TEXCOORD0).wy * 2.0) - 1.0);
  normal_7.z = sqrt(((1.0 - (normal_7.x * normal_7.x)) - (normal_7.y * normal_7.y)));
  lowp vec4 tmpvar_8;
  tmpvar_8 = texture2DProj (_LightBuffer, xlv_TEXCOORD1);
  light_2 = tmpvar_8;
  mediump vec4 tmpvar_9;
  tmpvar_9 = -(log2(max (light_2, vec4(0.001, 0.001, 0.001, 0.001))));
  light_2.w = tmpvar_9.w;
  highp vec3 tmpvar_10;
  tmpvar_10 = (tmpvar_9.xyz + xlv_TEXCOORD2);
  light_2.xyz = tmpvar_10;
  mediump vec4 c_11;
  mediump vec3 tmpvar_12;
  tmpvar_12 = (tmpvar_9.w * ((tmpvar_4.w * tmpvar_6.xyz) * _Gloss));
  c_11.xyz = ((tmpvar_3 * light_2.xyz) + (light_2.xyz * tmpvar_12));
  c_11.w = (tmpvar_12 * _SpecColor.w).x;
  tmpvar_1 = c_11;
  gl_FragData[0] = tmpvar_1;
}



#endif"
}

SubProgram "d3d11_9x " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
Bind "vertex" Vertex
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 112 // 96 used size, 8 vars
Vector 80 [_MainTex_ST] 4
ConstBuffer "UnityPerCamera" 128 // 96 used size, 8 vars
Vector 80 [_ProjectionParams] 4
ConstBuffer "UnityLighting" 400 // 400 used size, 16 vars
Vector 288 [unity_SHAr] 4
Vector 304 [unity_SHAg] 4
Vector 320 [unity_SHAb] 4
Vector 336 [unity_SHBr] 4
Vector 352 [unity_SHBg] 4
Vector 368 [unity_SHBb] 4
Vector 384 [unity_SHC] 4
ConstBuffer "UnityPerDraw" 336 // 336 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
Matrix 192 [_Object2World] 4
Vector 320 [unity_Scale] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityLighting" 2
BindCB "UnityPerDraw" 3
// 27 instructions, 4 temp regs, 0 temp arrays:
// ALU 15 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0_level_9_3
eefiecednopjaebfgepmgbcelannfelhlaghpaebabaaaaaacaaiaaaaaeaaaaaa
daaaaaaameacaaaanaagaaaajiahaaaaebgpgodjimacaaaaimacaaaaaaacpopp
bmacaaaahaaaaaaaagaaceaaaaaagmaaaaaagmaaaaaaceaaabaagmaaaaaaafaa
abaaabaaaaaaaaaaabaaafaaabaaacaaaaaaaaaaacaabcaaahaaadaaaaaaaaaa
adaaaaaaaeaaakaaaaaaaaaaadaaamaaadaaaoaaaaaaaaaaadaabeaaabaabbaa
aaaaaaaaaaaaaaaaabacpoppfbaaaaafbcaaapkaaaaaaadpaaaaiadpaaaaaaaa
aaaaaaaabpaaaaacafaaaaiaaaaaapjabpaaaaacafaaaciaacaaapjabpaaaaac
afaaadiaadaaapjaaeaaaaaeaaaaadoaadaaoejaabaaoekaabaaookaafaaaaad
aaaaapiaaaaaffjaalaaoekaaeaaaaaeaaaaapiaakaaoekaaaaaaajaaaaaoeia
aeaaaaaeaaaaapiaamaaoekaaaaakkjaaaaaoeiaaeaaaaaeaaaaapiaanaaoeka
aaaappjaaaaaoeiaafaaaaadabaaabiaaaaaffiaacaaaakaafaaaaadabaaaiia
abaaaaiabcaaaakaafaaaaadabaaafiaaaaapeiabcaaaakaacaaaaadabaaadoa
abaakkiaabaaomiaafaaaaadabaaahiaacaaoejabbaappkaafaaaaadacaaahia
abaaffiaapaaoekaaeaaaaaeabaaaliaaoaakekaabaaaaiaacaakeiaaeaaaaae
abaaahiabaaaoekaabaakkiaabaapeiaabaaaaacabaaaiiabcaaffkaajaaaaad
acaaabiaadaaoekaabaaoeiaajaaaaadacaaaciaaeaaoekaabaaoeiaajaaaaad
acaaaeiaafaaoekaabaaoeiaafaaaaadadaaapiaabaacjiaabaakeiaajaaaaad
aeaaabiaagaaoekaadaaoeiaajaaaaadaeaaaciaahaaoekaadaaoeiaajaaaaad
aeaaaeiaaiaaoekaadaaoeiaacaaaaadacaaahiaacaaoeiaaeaaoeiaafaaaaad
abaaaciaabaaffiaabaaffiaaeaaaaaeabaaabiaabaaaaiaabaaaaiaabaaffib
aeaaaaaeacaaahoaajaaoekaabaaaaiaacaaoeiaaeaaaaaeaaaaadmaaaaappia
aaaaoekaaaaaoeiaabaaaaacaaaaammaaaaaoeiaabaaaaacabaaamoaaaaaoeia
ppppaaaafdeieefcaeaeaaaaeaaaabaaababaaaafjaaaaaeegiocaaaaaaaaaaa
agaaaaaafjaaaaaeegiocaaaabaaaaaaagaaaaaafjaaaaaeegiocaaaacaaaaaa
bjaaaaaafjaaaaaeegiocaaaadaaaaaabfaaaaaafpaaaaadpcbabaaaaaaaaaaa
fpaaaaadhcbabaaaacaaaaaafpaaaaaddcbabaaaadaaaaaaghaaaaaepccabaaa
aaaaaaaaabaaaaaagfaaaaaddccabaaaabaaaaaagfaaaaadpccabaaaacaaaaaa
gfaaaaadhccabaaaadaaaaaagiaaaaacaeaaaaaadiaaaaaipcaabaaaaaaaaaaa
fgbfbaaaaaaaaaaaegiocaaaadaaaaaaabaaaaaadcaaaaakpcaabaaaaaaaaaaa
egiocaaaadaaaaaaaaaaaaaaagbabaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaak
pcaabaaaaaaaaaaaegiocaaaadaaaaaaacaaaaaakgbkbaaaaaaaaaaaegaobaaa
aaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaadaaaaaapgbpbaaa
aaaaaaaaegaobaaaaaaaaaaadgaaaaafpccabaaaaaaaaaaaegaobaaaaaaaaaaa
dcaaaaaldccabaaaabaaaaaaegbabaaaadaaaaaaegiacaaaaaaaaaaaafaaaaaa
ogikcaaaaaaaaaaaafaaaaaadiaaaaaiccaabaaaaaaaaaaabkaabaaaaaaaaaaa
akiacaaaabaaaaaaafaaaaaadiaaaaakncaabaaaabaaaaaaagahbaaaaaaaaaaa
aceaaaaaaaaaaadpaaaaaaaaaaaaaadpaaaaaadpdgaaaaafmccabaaaacaaaaaa
kgaobaaaaaaaaaaaaaaaaaahdccabaaaacaaaaaakgakbaaaabaaaaaamgaabaaa
abaaaaaadiaaaaaihcaabaaaaaaaaaaaegbcbaaaacaaaaaapgipcaaaadaaaaaa
beaaaaaadiaaaaaihcaabaaaabaaaaaafgafbaaaaaaaaaaaegiccaaaadaaaaaa
anaaaaaadcaaaaaklcaabaaaaaaaaaaaegiicaaaadaaaaaaamaaaaaaagaabaaa
aaaaaaaaegaibaaaabaaaaaadcaaaaakhcaabaaaaaaaaaaaegiccaaaadaaaaaa
aoaaaaaakgakbaaaaaaaaaaaegadbaaaaaaaaaaadgaaaaaficaabaaaaaaaaaaa
abeaaaaaaaaaiadpbbaaaaaibcaabaaaabaaaaaaegiocaaaacaaaaaabcaaaaaa
egaobaaaaaaaaaaabbaaaaaiccaabaaaabaaaaaaegiocaaaacaaaaaabdaaaaaa
egaobaaaaaaaaaaabbaaaaaiecaabaaaabaaaaaaegiocaaaacaaaaaabeaaaaaa
egaobaaaaaaaaaaadiaaaaahpcaabaaaacaaaaaajgacbaaaaaaaaaaaegakbaaa
aaaaaaaabbaaaaaibcaabaaaadaaaaaaegiocaaaacaaaaaabfaaaaaaegaobaaa
acaaaaaabbaaaaaiccaabaaaadaaaaaaegiocaaaacaaaaaabgaaaaaaegaobaaa
acaaaaaabbaaaaaiecaabaaaadaaaaaaegiocaaaacaaaaaabhaaaaaaegaobaaa
acaaaaaaaaaaaaahhcaabaaaabaaaaaaegacbaaaabaaaaaaegacbaaaadaaaaaa
diaaaaahccaabaaaaaaaaaaabkaabaaaaaaaaaaabkaabaaaaaaaaaaadcaaaaak
bcaabaaaaaaaaaaaakaabaaaaaaaaaaaakaabaaaaaaaaaaabkaabaiaebaaaaaa
aaaaaaaadcaaaaakhccabaaaadaaaaaaegiccaaaacaaaaaabiaaaaaaagaabaaa
aaaaaaaaegacbaaaabaaaaaadoaaaaabejfdeheomaaaaaaaagaaaaaaaiaaaaaa
jiaaaaaaaaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaabaaaaaaapaaaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaa
acaaaaaaahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaa
laaaaaaaabaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaljaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofe
aaeoepfcenebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheoiaaaaaaa
aeaaaaaaaiaaaaaagiaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaa
heaaaaaaaaaaaaaaaaaaaaaaadaaaaaaabaaaaaaadamaaaaheaaaaaaabaaaaaa
aaaaaaaaadaaaaaaacaaaaaaapaaaaaaheaaaaaaacaaaaaaaaaaaaaaadaaaaaa
adaaaaaaahaiaaaafdfgfpfaepfdejfeejepeoaafeeffiedepepfceeaaklklkl
"
}

SubProgram "opengl " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
Bind "vertex" Vertex
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Vector 13 [_ProjectionParams]
Vector 14 [unity_ShadowFadeCenterAndType]
Matrix 9 [_Object2World]
Vector 15 [unity_LightmapST]
Vector 16 [_MainTex_ST]
"!!ARBvp1.0
# 20 ALU
PARAM c[17] = { { 0.5, 1 },
		state.matrix.modelview[0],
		state.matrix.mvp,
		program.local[9..16] };
TEMP R0;
TEMP R1;
DP4 R0.w, vertex.position, c[8];
DP4 R0.z, vertex.position, c[7];
DP4 R0.x, vertex.position, c[5];
DP4 R0.y, vertex.position, c[6];
MUL R1.xyz, R0.xyww, c[0].x;
MUL R1.y, R1, c[13].x;
ADD result.texcoord[1].xy, R1, R1.z;
MOV result.position, R0;
MOV R0.x, c[0].y;
ADD R0.y, R0.x, -c[14].w;
DP4 R0.x, vertex.position, c[3];
DP4 R1.z, vertex.position, c[11];
DP4 R1.x, vertex.position, c[9];
DP4 R1.y, vertex.position, c[10];
ADD R1.xyz, R1, -c[14];
MOV result.texcoord[1].zw, R0;
MUL result.texcoord[3].xyz, R1, c[14].w;
MAD result.texcoord[0].xy, vertex.texcoord[0], c[16], c[16].zwzw;
MAD result.texcoord[2].xy, vertex.texcoord[1], c[15], c[15].zwzw;
MUL result.texcoord[3].w, -R0.x, R0.y;
END
# 20 instructions, 2 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
Bind "vertex" Vertex
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Matrix 0 [glstate_matrix_modelview0]
Matrix 4 [glstate_matrix_mvp]
Vector 12 [_ProjectionParams]
Vector 13 [_ScreenParams]
Vector 14 [unity_ShadowFadeCenterAndType]
Matrix 8 [_Object2World]
Vector 15 [unity_LightmapST]
Vector 16 [_MainTex_ST]
"vs_2_0
; 20 ALU
def c17, 0.50000000, 1.00000000, 0, 0
dcl_position0 v0
dcl_texcoord0 v1
dcl_texcoord1 v2
dp4 r0.w, v0, c7
dp4 r0.z, v0, c6
dp4 r0.x, v0, c4
dp4 r0.y, v0, c5
mul r1.xyz, r0.xyww, c17.x
mul r1.y, r1, c12.x
mad oT1.xy, r1.z, c13.zwzw, r1
mov oPos, r0
mov r0.x, c14.w
add r0.y, c17, -r0.x
dp4 r0.x, v0, c2
dp4 r1.z, v0, c10
dp4 r1.x, v0, c8
dp4 r1.y, v0, c9
add r1.xyz, r1, -c14
mov oT1.zw, r0
mul oT3.xyz, r1, c14.w
mad oT0.xy, v1, c16, c16.zwzw
mad oT2.xy, v2, c15, c15.zwzw
mul oT3.w, -r0.x, r0.y
"
}

SubProgram "xbox360 " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
Bind "vertex" Vertex
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Vector 16 [_MainTex_ST]
Matrix 11 [_Object2World] 4
Vector 0 [_ProjectionParams]
Vector 1 [_ScreenParams]
Matrix 7 [glstate_matrix_modelview0] 4
Matrix 3 [glstate_matrix_mvp] 4
Vector 15 [unity_LightmapST]
Vector 2 [unity_ShadowFadeCenterAndType]
// Shader Timing Estimate, in Cycles/64 vertex vector:
// ALU: 28.00 (21 instructions), vertex: 32, texture: 0,
//   sequencer: 14,  7 GPRs, 27 threads,
// Performance (if enough threads): ~32 cycles per vector
// * Vertex cycle estimates are assuming 3 vfetch_minis for every vfetch_full,
//     with <= 32 bytes per vfetch_full group.

"vs_360
backbbabaaaaacdeaaaaabjmaaaaaaaaaaaaaaceaaaaablaaaaaabniaaaaaaaa
aaaaaaaaaaaaabiiaaaaaabmaaaaabhlpppoadaaaaaaaaaiaaaaaabmaaaaaaaa
aaaaabheaaaaaalmaaacaabaaaabaaaaaaaaaamiaaaaaaaaaaaaaaniaaacaaal
aaaeaaaaaaaaaaoiaaaaaaaaaaaaaapiaaacaaaaaaabaaaaaaaaaamiaaaaaaaa
aaaaabakaaacaaabaaabaaaaaaaaaamiaaaaaaaaaaaaabbiaaacaaahaaaeaaaa
aaaaaaoiaaaaaaaaaaaaabdcaaacaaadaaaeaaaaaaaaaaoiaaaaaaaaaaaaabef
aaacaaapaaabaaaaaaaaaamiaaaaaaaaaaaaabfgaaacaaacaaabaaaaaaaaaami
aaaaaaaafpengbgjgofegfhifpfdfeaaaaabaaadaaabaaaeaaabaaaaaaaaaaaa
fpepgcgkgfgdhedcfhgphcgmgeaaklklaaadaaadaaaeaaaeaaabaaaaaaaaaaaa
fpfahcgpgkgfgdhegjgpgofagbhcgbgnhdaafpfdgdhcgfgfgofagbhcgbgnhdaa
ghgmhdhegbhegffpgngbhehcgjhifpgngpgegfgmhggjgfhhdaaaghgmhdhegbhe
gffpgngbhehcgjhifpgnhghaaahfgogjhehjfpemgjghgihegngbhafdfeaahfgo
gjhehjfpfdgigbgegphheggbgegfedgfgohegfhcebgogefehjhagfaahghdfpdd
fpdaaadccodacodcdadddfddcodaaaklaaaaaaaaaaaaaaabaaaaaaaaaaaaaaaa
aaaaaabeaapmaabaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaeaaaaaabfm
aadbaaagaaaaaaaaaaaaaaaaaaaadaieaaaaaaabaaaaaaadaaaaaaagaaaaacja
aabaaaaeaaaafaafaacbfaagaaaadafaaaabpbfbaaaddcfcaaaepdfdaaaababe
aaaaaabdaaaabablaaaababfaaaaaabiaaaababkaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaadpiaaaaadpaaaaaaaaaaaaaaaaaaaaaahabfdaaeaaaabcaamcaaaaaa
aaaafaahaaaabcaameaaaaaaaaaagaamgabcbcaabcaaaaaaaaaaeabiaaaaccaa
aaaaaaaaafpicaaaaaaaagiiaaaaaaaaafpieaaaaaaaapmiaaaaaaaaafpibaaa
aaaaaoehaaaaaaaamiapaaaaaabliiaakbacagaamiapaaaaaamgnapiklacafaa
miapaaaaaalbdepiklacaeaamiapaaafaagmnajeklacadaamiapiadoaananaaa
ocafafaakiibagabaeblgmmacaacppahmiahaaadaablleaakbacaoaabeboaaaa
aamgimlbkbacanacmiaoaaaaaalbimabklacamaamiahaaagaagmlebfklacalaa
kiioadaaaapmlbmaibafppaimiapaaadaadedeaaoaagadaamiamiaabaanlnlaa
ocafafaamiadiaaaaalalabkilaebabamiadiaacaamflabkilabapapmiacaaab
aamgmgblklacajadkibhaaadacmamaeciaadacaamiahiaadaamablaakbadacaa
miacaaabaablmglbklacakabmiaiiaadaelbgmaaobababaamiadiaabaablbkgn
klaaabaaaaaaaaaaaaaaaaaaaaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
Matrix 256 [glstate_matrix_modelview0]
Matrix 260 [glstate_matrix_mvp]
Bind "vertex" Vertex
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Vector 467 [_ProjectionParams]
Vector 466 [unity_ShadowFadeCenterAndType]
Matrix 264 [_Object2World]
Vector 465 [unity_LightmapST]
Vector 464 [_MainTex_ST]
"sce_vp_rsx // 21 instructions using 3 registers
[Configuration]
8
0000001503010300
[Defaults]
1
463 2
3f0000003f800000
[Microcode]
336
401f9c6c011d0808010400d740619f9c401f9c6c011d1908010400d740619fa4
00001c6c01d0200d8106c0c360403ffc00009c6c01d0700d8106c0c360403ffc
00009c6c01d0600d8106c0c360405ffc00009c6c01d0500d8106c0c360409ffc
00009c6c01d0400d8106c0c360411ffc00011c6c005d207f8186c08360411ffc
00001c6c01d0a00d8106c0c360405ffc00001c6c01d0900d8106c0c360409ffc
00001c6c01d0800d8106c0c360411ffc00001c6c00dd208c0186c0830021dffc
00011c6c00dcf02a8186c0a001211ffc401f9c6c0040000d8286c0836041ff80
401f9c6c004000558286c08360407fa000009c6c009cf00e028000c36041dffc
401f9c6c009d200c00bfc0c36041dfa8401f9c6c008000ff8080024360403fa8
00001c6c004000000286c08360411ffc00001c6c009d302a828000c360409ffc
401f9c6c00c000080086c09540a19fa1
"
}

SubProgram "d3d11 " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
Bind "vertex" Vertex
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Bind "color" Color
ConstBuffer "$Globals" 144 // 112 used size, 10 vars
Vector 80 [unity_LightmapST] 4
Vector 96 [_MainTex_ST] 4
ConstBuffer "UnityPerCamera" 128 // 96 used size, 8 vars
Vector 80 [_ProjectionParams] 4
ConstBuffer "UnityShadows" 416 // 416 used size, 8 vars
Vector 400 [unity_ShadowFadeCenterAndType] 4
ConstBuffer "UnityPerDraw" 336 // 256 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
Matrix 64 [glstate_matrix_modelview0] 4
Matrix 192 [_Object2World] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityShadows" 2
BindCB "UnityPerDraw" 3
// 24 instructions, 2 temp regs, 0 temp arrays:
// ALU 10 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0
eefieceddegfopjhbogfokaeladbmdgflgadlgnfabaaaaaaiiafaaaaadaaaaaa
cmaaaaaapeaaaaaajeabaaaaejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapaaaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahaaaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaa
abaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapadaaaaljaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfc
enebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheojiaaaaaaafaaaaaa
aiaaaaaaiaaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaabaaaaaaadamaaaaimaaaaaaacaaaaaaaaaaaaaa
adaaaaaaabaaaaaaamadaaaaimaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaa
apaaaaaaimaaaaaaadaaaaaaaaaaaaaaadaaaaaaadaaaaaaapaaaaaafdfgfpfa
epfdejfeejepeoaafeeffiedepepfceeaaklklklfdeieefcomadaaaaeaaaabaa
plaaaaaafjaaaaaeegiocaaaaaaaaaaaahaaaaaafjaaaaaeegiocaaaabaaaaaa
agaaaaaafjaaaaaeegiocaaaacaaaaaabkaaaaaafjaaaaaeegiocaaaadaaaaaa
baaaaaaafpaaaaadpcbabaaaaaaaaaaafpaaaaaddcbabaaaadaaaaaafpaaaaad
dcbabaaaaeaaaaaaghaaaaaepccabaaaaaaaaaaaabaaaaaagfaaaaaddccabaaa
abaaaaaagfaaaaadmccabaaaabaaaaaagfaaaaadpccabaaaacaaaaaagfaaaaad
pccabaaaadaaaaaagiaaaaacacaaaaaadiaaaaaipcaabaaaaaaaaaaafgbfbaaa
aaaaaaaaegiocaaaadaaaaaaabaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaa
adaaaaaaaaaaaaaaagbabaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaa
aaaaaaaaegiocaaaadaaaaaaacaaaaaakgbkbaaaaaaaaaaaegaobaaaaaaaaaaa
dcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaadaaaaaapgbpbaaaaaaaaaaa
egaobaaaaaaaaaaadgaaaaafpccabaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaal
dccabaaaabaaaaaaegbabaaaadaaaaaaegiacaaaaaaaaaaaagaaaaaaogikcaaa
aaaaaaaaagaaaaaadcaaaaalmccabaaaabaaaaaaagbebaaaaeaaaaaaagiecaaa
aaaaaaaaafaaaaaakgiocaaaaaaaaaaaafaaaaaadiaaaaaiccaabaaaaaaaaaaa
bkaabaaaaaaaaaaaakiacaaaabaaaaaaafaaaaaadiaaaaakncaabaaaabaaaaaa
agahbaaaaaaaaaaaaceaaaaaaaaaaadpaaaaaaaaaaaaaadpaaaaaadpdgaaaaaf
mccabaaaacaaaaaakgaobaaaaaaaaaaaaaaaaaahdccabaaaacaaaaaakgakbaaa
abaaaaaamgaabaaaabaaaaaadiaaaaaihcaabaaaaaaaaaaafgbfbaaaaaaaaaaa
egiccaaaadaaaaaaanaaaaaadcaaaaakhcaabaaaaaaaaaaaegiccaaaadaaaaaa
amaaaaaaagbabaaaaaaaaaaaegacbaaaaaaaaaaadcaaaaakhcaabaaaaaaaaaaa
egiccaaaadaaaaaaaoaaaaaakgbkbaaaaaaaaaaaegacbaaaaaaaaaaadcaaaaak
hcaabaaaaaaaaaaaegiccaaaadaaaaaaapaaaaaapgbpbaaaaaaaaaaaegacbaaa
aaaaaaaaaaaaaaajhcaabaaaaaaaaaaaegacbaaaaaaaaaaaegiccaiaebaaaaaa
acaaaaaabjaaaaaadiaaaaaihccabaaaadaaaaaaegacbaaaaaaaaaaapgipcaaa
acaaaaaabjaaaaaadiaaaaaibcaabaaaaaaaaaaabkbabaaaaaaaaaaackiacaaa
adaaaaaaafaaaaaadcaaaaakbcaabaaaaaaaaaaackiacaaaadaaaaaaaeaaaaaa
akbabaaaaaaaaaaaakaabaaaaaaaaaaadcaaaaakbcaabaaaaaaaaaaackiacaaa
adaaaaaaagaaaaaackbabaaaaaaaaaaaakaabaaaaaaaaaaadcaaaaakbcaabaaa
aaaaaaaackiacaaaadaaaaaaahaaaaaadkbabaaaaaaaaaaaakaabaaaaaaaaaaa
aaaaaaajccaabaaaaaaaaaaadkiacaiaebaaaaaaacaaaaaabjaaaaaaabeaaaaa
aaaaiadpdiaaaaaiiccabaaaadaaaaaabkaabaaaaaaaaaaaakaabaiaebaaaaaa
aaaaaaaadoaaaaab"
}

SubProgram "gles " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
"!!GLES
#define SHADER_API_GLES 1
#define tex2D texture2D


#ifdef VERTEX
#define gl_ModelViewProjectionMatrix glstate_matrix_mvp
uniform mat4 glstate_matrix_mvp;
#define gl_ModelViewMatrix glstate_matrix_modelview0
uniform mat4 glstate_matrix_modelview0;

varying highp vec4 xlv_TEXCOORD3;
varying highp vec2 xlv_TEXCOORD2;
varying highp vec4 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 unity_LightmapST;
uniform highp mat4 _Object2World;


uniform highp vec4 unity_ShadowFadeCenterAndType;
uniform highp vec4 _ProjectionParams;
attribute vec4 _glesMultiTexCoord1;
attribute vec4 _glesMultiTexCoord0;
attribute vec4 _glesVertex;
void main ()
{
  highp vec4 tmpvar_1;
  highp vec4 tmpvar_2;
  tmpvar_2 = (gl_ModelViewProjectionMatrix * _glesVertex);
  highp vec4 o_3;
  highp vec4 tmpvar_4;
  tmpvar_4 = (tmpvar_2 * 0.5);
  highp vec2 tmpvar_5;
  tmpvar_5.x = tmpvar_4.x;
  tmpvar_5.y = (tmpvar_4.y * _ProjectionParams.x);
  o_3.xy = (tmpvar_5 + tmpvar_4.w);
  o_3.zw = tmpvar_2.zw;
  tmpvar_1.xyz = (((_Object2World * _glesVertex).xyz - unity_ShadowFadeCenterAndType.xyz) * unity_ShadowFadeCenterAndType.w);
  tmpvar_1.w = (-((gl_ModelViewMatrix * _glesVertex).z) * (1.0 - unity_ShadowFadeCenterAndType.w));
  gl_Position = tmpvar_2;
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = o_3;
  xlv_TEXCOORD2 = ((_glesMultiTexCoord1.xy * unity_LightmapST.xy) + unity_LightmapST.zw);
  xlv_TEXCOORD3 = tmpvar_1;
}



#endif
#ifdef FRAGMENT

varying highp vec4 xlv_TEXCOORD3;
varying highp vec2 xlv_TEXCOORD2;
varying highp vec4 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp vec4 unity_LightmapFade;
uniform sampler2D unity_LightmapInd;
uniform sampler2D unity_Lightmap;
uniform sampler2D _LightBuffer;
uniform mediump float _Gloss;
uniform lowp vec4 _Color;
uniform sampler2D _SpecMap;
uniform sampler2D _MainTex;
uniform lowp vec4 _SpecColor;
void main ()
{
  lowp vec4 tmpvar_1;
  mediump vec3 lmIndirect_2;
  mediump vec3 lmFull_3;
  mediump vec4 light_4;
  mediump vec3 tmpvar_5;
  lowp vec4 tmpvar_6;
  tmpvar_6 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_7;
  tmpvar_7 = (tmpvar_6.xyz * _Color.xyz);
  tmpvar_5 = tmpvar_7;
  lowp vec4 tmpvar_8;
  tmpvar_8 = texture2D (_SpecMap, xlv_TEXCOORD0);
  lowp vec4 tmpvar_9;
  tmpvar_9 = texture2DProj (_LightBuffer, xlv_TEXCOORD1);
  light_4 = tmpvar_9;
  mediump vec4 tmpvar_10;
  tmpvar_10 = -(log2(max (light_4, vec4(0.001, 0.001, 0.001, 0.001))));
  light_4.w = tmpvar_10.w;
  lowp vec3 tmpvar_11;
  tmpvar_11 = (2.0 * texture2D (unity_Lightmap, xlv_TEXCOORD2).xyz);
  lmFull_3 = tmpvar_11;
  lowp vec3 tmpvar_12;
  tmpvar_12 = (2.0 * texture2D (unity_LightmapInd, xlv_TEXCOORD2).xyz);
  lmIndirect_2 = tmpvar_12;
  highp float tmpvar_13;
  tmpvar_13 = clamp (((sqrt(dot (xlv_TEXCOORD3, xlv_TEXCOORD3)) * unity_LightmapFade.z) + unity_LightmapFade.w), 0.0, 1.0);
  light_4.xyz = (tmpvar_10.xyz + mix (lmIndirect_2, lmFull_3, vec3(tmpvar_13)));
  mediump vec4 c_14;
  mediump vec3 tmpvar_15;
  tmpvar_15 = (tmpvar_10.w * ((tmpvar_6.w * tmpvar_8.xyz) * _Gloss));
  c_14.xyz = ((tmpvar_5 * light_4.xyz) + (light_4.xyz * tmpvar_15));
  c_14.w = (tmpvar_15 * _SpecColor.w).x;
  tmpvar_1 = c_14;
  gl_FragData[0] = tmpvar_1;
}



#endif"
}

SubProgram "glesdesktop " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
"!!GLES
#define SHADER_API_GLES 1
#define tex2D texture2D


#ifdef VERTEX
#define gl_ModelViewProjectionMatrix glstate_matrix_mvp
uniform mat4 glstate_matrix_mvp;
#define gl_ModelViewMatrix glstate_matrix_modelview0
uniform mat4 glstate_matrix_modelview0;

varying highp vec4 xlv_TEXCOORD3;
varying highp vec2 xlv_TEXCOORD2;
varying highp vec4 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 unity_LightmapST;
uniform highp mat4 _Object2World;


uniform highp vec4 unity_ShadowFadeCenterAndType;
uniform highp vec4 _ProjectionParams;
attribute vec4 _glesMultiTexCoord1;
attribute vec4 _glesMultiTexCoord0;
attribute vec4 _glesVertex;
void main ()
{
  highp vec4 tmpvar_1;
  highp vec4 tmpvar_2;
  tmpvar_2 = (gl_ModelViewProjectionMatrix * _glesVertex);
  highp vec4 o_3;
  highp vec4 tmpvar_4;
  tmpvar_4 = (tmpvar_2 * 0.5);
  highp vec2 tmpvar_5;
  tmpvar_5.x = tmpvar_4.x;
  tmpvar_5.y = (tmpvar_4.y * _ProjectionParams.x);
  o_3.xy = (tmpvar_5 + tmpvar_4.w);
  o_3.zw = tmpvar_2.zw;
  tmpvar_1.xyz = (((_Object2World * _glesVertex).xyz - unity_ShadowFadeCenterAndType.xyz) * unity_ShadowFadeCenterAndType.w);
  tmpvar_1.w = (-((gl_ModelViewMatrix * _glesVertex).z) * (1.0 - unity_ShadowFadeCenterAndType.w));
  gl_Position = tmpvar_2;
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = o_3;
  xlv_TEXCOORD2 = ((_glesMultiTexCoord1.xy * unity_LightmapST.xy) + unity_LightmapST.zw);
  xlv_TEXCOORD3 = tmpvar_1;
}



#endif
#ifdef FRAGMENT

varying highp vec4 xlv_TEXCOORD3;
varying highp vec2 xlv_TEXCOORD2;
varying highp vec4 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp vec4 unity_LightmapFade;
uniform sampler2D unity_LightmapInd;
uniform sampler2D unity_Lightmap;
uniform sampler2D _LightBuffer;
uniform mediump float _Gloss;
uniform lowp vec4 _Color;
uniform sampler2D _SpecMap;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform lowp vec4 _SpecColor;
void main ()
{
  lowp vec4 tmpvar_1;
  mediump vec3 lmIndirect_2;
  mediump vec3 lmFull_3;
  mediump vec4 light_4;
  mediump vec3 tmpvar_5;
  lowp vec4 tmpvar_6;
  tmpvar_6 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_7;
  tmpvar_7 = (tmpvar_6.xyz * _Color.xyz);
  tmpvar_5 = tmpvar_7;
  lowp vec4 tmpvar_8;
  tmpvar_8 = texture2D (_SpecMap, xlv_TEXCOORD0);
  lowp vec3 normal_9;
  normal_9.xy = ((texture2D (_BumpMap, xlv_TEXCOORD0).wy * 2.0) - 1.0);
  normal_9.z = sqrt(((1.0 - (normal_9.x * normal_9.x)) - (normal_9.y * normal_9.y)));
  lowp vec4 tmpvar_10;
  tmpvar_10 = texture2DProj (_LightBuffer, xlv_TEXCOORD1);
  light_4 = tmpvar_10;
  mediump vec4 tmpvar_11;
  tmpvar_11 = -(log2(max (light_4, vec4(0.001, 0.001, 0.001, 0.001))));
  light_4.w = tmpvar_11.w;
  lowp vec4 tmpvar_12;
  tmpvar_12 = texture2D (unity_Lightmap, xlv_TEXCOORD2);
  lowp vec3 tmpvar_13;
  tmpvar_13 = ((8.0 * tmpvar_12.w) * tmpvar_12.xyz);
  lmFull_3 = tmpvar_13;
  lowp vec4 tmpvar_14;
  tmpvar_14 = texture2D (unity_LightmapInd, xlv_TEXCOORD2);
  lowp vec3 tmpvar_15;
  tmpvar_15 = ((8.0 * tmpvar_14.w) * tmpvar_14.xyz);
  lmIndirect_2 = tmpvar_15;
  highp float tmpvar_16;
  tmpvar_16 = clamp (((sqrt(dot (xlv_TEXCOORD3, xlv_TEXCOORD3)) * unity_LightmapFade.z) + unity_LightmapFade.w), 0.0, 1.0);
  light_4.xyz = (tmpvar_11.xyz + mix (lmIndirect_2, lmFull_3, vec3(tmpvar_16)));
  mediump vec4 c_17;
  mediump vec3 tmpvar_18;
  tmpvar_18 = (tmpvar_11.w * ((tmpvar_6.w * tmpvar_8.xyz) * _Gloss));
  c_17.xyz = ((tmpvar_5 * light_4.xyz) + (light_4.xyz * tmpvar_18));
  c_17.w = (tmpvar_18 * _SpecColor.w).x;
  tmpvar_1 = c_17;
  gl_FragData[0] = tmpvar_1;
}



#endif"
}

SubProgram "d3d11_9x " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
Bind "vertex" Vertex
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Bind "color" Color
ConstBuffer "$Globals" 144 // 112 used size, 10 vars
Vector 80 [unity_LightmapST] 4
Vector 96 [_MainTex_ST] 4
ConstBuffer "UnityPerCamera" 128 // 96 used size, 8 vars
Vector 80 [_ProjectionParams] 4
ConstBuffer "UnityShadows" 416 // 416 used size, 8 vars
Vector 400 [unity_ShadowFadeCenterAndType] 4
ConstBuffer "UnityPerDraw" 336 // 256 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
Matrix 64 [glstate_matrix_modelview0] 4
Matrix 192 [_Object2World] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityShadows" 2
BindCB "UnityPerDraw" 3
// 24 instructions, 2 temp regs, 0 temp arrays:
// ALU 10 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0_level_9_3
eefiecedjopiaimhnihcggeiloincidnceocjfibabaaaaaaaaaiaaaaaeaaaaaa
daaaaaaakeacaaaajiagaaaagaahaaaaebgpgodjgmacaaaagmacaaaaaaacpopp
aiacaaaageaaaaaaafaaceaaaaaagaaaaaaagaaaaaaaceaaabaagaaaaaaaafaa
acaaabaaaaaaaaaaabaaafaaabaaadaaaaaaaaaaacaabjaaabaaaeaaaaaaaaaa
adaaaaaaaiaaafaaaaaaaaaaadaaamaaaeaaanaaaaaaaaaaaaaaaaaaabacpopp
fbaaaaafbbaaapkaaaaaaadpaaaaiadpaaaaaaaaaaaaaaaabpaaaaacafaaaaia
aaaaapjabpaaaaacafaaadiaadaaapjabpaaaaacafaaaeiaaeaaapjaaeaaaaae
aaaaadoaadaaoejaacaaoekaacaaookaafaaaaadaaaaapiaaaaaffjaagaaoeka
aeaaaaaeaaaaapiaafaaoekaaaaaaajaaaaaoeiaaeaaaaaeaaaaapiaahaaoeka
aaaakkjaaaaaoeiaaeaaaaaeaaaaapiaaiaaoekaaaaappjaaaaaoeiaafaaaaad
abaaabiaaaaaffiaadaaaakaafaaaaadabaaaiiaabaaaaiabbaaaakaafaaaaad
abaaafiaaaaapeiabbaaaakaacaaaaadabaaadoaabaakkiaabaaomiaaeaaaaae
aaaaamoaaeaabejaabaabekaabaalekaafaaaaadabaaahiaaaaaffjaaoaaoeka
aeaaaaaeabaaahiaanaaoekaaaaaaajaabaaoeiaaeaaaaaeabaaahiaapaaoeka
aaaakkjaabaaoeiaaeaaaaaeabaaahiabaaaoekaaaaappjaabaaoeiaacaaaaad
abaaahiaabaaoeiaaeaaoekbafaaaaadacaaahoaabaaoeiaaeaappkaafaaaaad
abaaabiaaaaaffjaakaakkkaaeaaaaaeabaaabiaajaakkkaaaaaaajaabaaaaia
aeaaaaaeabaaabiaalaakkkaaaaakkjaabaaaaiaaeaaaaaeabaaabiaamaakkka
aaaappjaabaaaaiaabaaaaacabaaaiiaaeaappkaacaaaaadabaaaciaabaappib
bbaaffkaafaaaaadacaaaioaabaaffiaabaaaaibaeaaaaaeaaaaadmaaaaappia
aaaaoekaaaaaoeiaabaaaaacaaaaammaaaaaoeiaabaaaaacabaaamoaaaaaoeia
ppppaaaafdeieefcomadaaaaeaaaabaaplaaaaaafjaaaaaeegiocaaaaaaaaaaa
ahaaaaaafjaaaaaeegiocaaaabaaaaaaagaaaaaafjaaaaaeegiocaaaacaaaaaa
bkaaaaaafjaaaaaeegiocaaaadaaaaaabaaaaaaafpaaaaadpcbabaaaaaaaaaaa
fpaaaaaddcbabaaaadaaaaaafpaaaaaddcbabaaaaeaaaaaaghaaaaaepccabaaa
aaaaaaaaabaaaaaagfaaaaaddccabaaaabaaaaaagfaaaaadmccabaaaabaaaaaa
gfaaaaadpccabaaaacaaaaaagfaaaaadpccabaaaadaaaaaagiaaaaacacaaaaaa
diaaaaaipcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaadaaaaaaabaaaaaa
dcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaaaaaaaaaagbabaaaaaaaaaaa
egaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaacaaaaaa
kgbkbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaa
adaaaaaaadaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadgaaaaafpccabaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaaldccabaaaabaaaaaaegbabaaaadaaaaaa
egiacaaaaaaaaaaaagaaaaaaogikcaaaaaaaaaaaagaaaaaadcaaaaalmccabaaa
abaaaaaaagbebaaaaeaaaaaaagiecaaaaaaaaaaaafaaaaaakgiocaaaaaaaaaaa
afaaaaaadiaaaaaiccaabaaaaaaaaaaabkaabaaaaaaaaaaaakiacaaaabaaaaaa
afaaaaaadiaaaaakncaabaaaabaaaaaaagahbaaaaaaaaaaaaceaaaaaaaaaaadp
aaaaaaaaaaaaaadpaaaaaadpdgaaaaafmccabaaaacaaaaaakgaobaaaaaaaaaaa
aaaaaaahdccabaaaacaaaaaakgakbaaaabaaaaaamgaabaaaabaaaaaadiaaaaai
hcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiccaaaadaaaaaaanaaaaaadcaaaaak
hcaabaaaaaaaaaaaegiccaaaadaaaaaaamaaaaaaagbabaaaaaaaaaaaegacbaaa
aaaaaaaadcaaaaakhcaabaaaaaaaaaaaegiccaaaadaaaaaaaoaaaaaakgbkbaaa
aaaaaaaaegacbaaaaaaaaaaadcaaaaakhcaabaaaaaaaaaaaegiccaaaadaaaaaa
apaaaaaapgbpbaaaaaaaaaaaegacbaaaaaaaaaaaaaaaaaajhcaabaaaaaaaaaaa
egacbaaaaaaaaaaaegiccaiaebaaaaaaacaaaaaabjaaaaaadiaaaaaihccabaaa
adaaaaaaegacbaaaaaaaaaaapgipcaaaacaaaaaabjaaaaaadiaaaaaibcaabaaa
aaaaaaaabkbabaaaaaaaaaaackiacaaaadaaaaaaafaaaaaadcaaaaakbcaabaaa
aaaaaaaackiacaaaadaaaaaaaeaaaaaaakbabaaaaaaaaaaaakaabaaaaaaaaaaa
dcaaaaakbcaabaaaaaaaaaaackiacaaaadaaaaaaagaaaaaackbabaaaaaaaaaaa
akaabaaaaaaaaaaadcaaaaakbcaabaaaaaaaaaaackiacaaaadaaaaaaahaaaaaa
dkbabaaaaaaaaaaaakaabaaaaaaaaaaaaaaaaaajccaabaaaaaaaaaaadkiacaia
ebaaaaaaacaaaaaabjaaaaaaabeaaaaaaaaaiadpdiaaaaaiiccabaaaadaaaaaa
bkaabaaaaaaaaaaaakaabaiaebaaaaaaaaaaaaaadoaaaaabejfdeheomaaaaaaa
agaaaaaaaiaaaaaajiaaaaaaaaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaa
kbaaaaaaaaaaaaaaaaaaaaaaadaaaaaaabaaaaaaapaaaaaakjaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaacaaaaaaahaaaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaa
adaaaaaaapadaaaalaaaaaaaabaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapadaaaa
ljaaaaaaaaaaaaaaaaaaaaaaadaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeo
aafeebeoehefeofeaaeoepfcenebemaafeeffiedepepfceeaaedepemepfcaakl
epfdeheojiaaaaaaafaaaaaaaiaaaaaaiaaaaaaaaaaaaaaaabaaaaaaadaaaaaa
aaaaaaaaapaaaaaaimaaaaaaaaaaaaaaaaaaaaaaadaaaaaaabaaaaaaadamaaaa
imaaaaaaacaaaaaaaaaaaaaaadaaaaaaabaaaaaaamadaaaaimaaaaaaabaaaaaa
aaaaaaaaadaaaaaaacaaaaaaapaaaaaaimaaaaaaadaaaaaaaaaaaaaaadaaaaaa
adaaaaaaapaaaaaafdfgfpfaepfdejfeejepeoaafeeffiedepepfceeaaklklkl
"
}

SubProgram "opengl " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
Bind "vertex" Vertex
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 9 [_ProjectionParams]
Vector 10 [unity_SHAr]
Vector 11 [unity_SHAg]
Vector 12 [unity_SHAb]
Vector 13 [unity_SHBr]
Vector 14 [unity_SHBg]
Vector 15 [unity_SHBb]
Vector 16 [unity_SHC]
Matrix 5 [_Object2World]
Vector 17 [unity_Scale]
Vector 18 [_MainTex_ST]
"!!ARBvp1.0
# 28 ALU
PARAM c[19] = { { 0.5, 1 },
		state.matrix.mvp,
		program.local[5..18] };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
MUL R1.xyz, vertex.normal, c[17].w;
DP3 R2.w, R1, c[6];
DP3 R0.x, R1, c[5];
DP3 R0.z, R1, c[7];
MOV R0.y, R2.w;
MUL R1, R0.xyzz, R0.yzzx;
MOV R0.w, c[0].y;
DP4 R2.z, R0, c[12];
DP4 R2.y, R0, c[11];
DP4 R2.x, R0, c[10];
MUL R0.y, R2.w, R2.w;
DP4 R3.z, R1, c[15];
DP4 R3.y, R1, c[14];
DP4 R3.x, R1, c[13];
DP4 R1.w, vertex.position, c[4];
DP4 R1.z, vertex.position, c[3];
MAD R0.x, R0, R0, -R0.y;
ADD R3.xyz, R2, R3;
MUL R2.xyz, R0.x, c[16];
DP4 R1.x, vertex.position, c[1];
DP4 R1.y, vertex.position, c[2];
MUL R0.xyz, R1.xyww, c[0].x;
MUL R0.y, R0, c[9].x;
ADD result.texcoord[2].xyz, R3, R2;
ADD result.texcoord[1].xy, R0, R0.z;
MOV result.position, R1;
MOV result.texcoord[1].zw, R1;
MAD result.texcoord[0].xy, vertex.texcoord[0], c[18], c[18].zwzw;
END
# 28 instructions, 4 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
Bind "vertex" Vertex
Bind "normal" Normal
Bind "texcoord" TexCoord0
Matrix 0 [glstate_matrix_mvp]
Vector 8 [_ProjectionParams]
Vector 9 [_ScreenParams]
Vector 10 [unity_SHAr]
Vector 11 [unity_SHAg]
Vector 12 [unity_SHAb]
Vector 13 [unity_SHBr]
Vector 14 [unity_SHBg]
Vector 15 [unity_SHBb]
Vector 16 [unity_SHC]
Matrix 4 [_Object2World]
Vector 17 [unity_Scale]
Vector 18 [_MainTex_ST]
"vs_2_0
; 28 ALU
def c19, 0.50000000, 1.00000000, 0, 0
dcl_position0 v0
dcl_normal0 v1
dcl_texcoord0 v2
mul r1.xyz, v1, c17.w
dp3 r2.w, r1, c5
dp3 r0.x, r1, c4
dp3 r0.z, r1, c6
mov r0.y, r2.w
mul r1, r0.xyzz, r0.yzzx
mov r0.w, c19.y
dp4 r2.z, r0, c12
dp4 r2.y, r0, c11
dp4 r2.x, r0, c10
mul r0.y, r2.w, r2.w
dp4 r3.z, r1, c15
dp4 r3.y, r1, c14
dp4 r3.x, r1, c13
dp4 r1.w, v0, c3
dp4 r1.z, v0, c2
mad r0.x, r0, r0, -r0.y
add r3.xyz, r2, r3
mul r2.xyz, r0.x, c16
dp4 r1.x, v0, c0
dp4 r1.y, v0, c1
mul r0.xyz, r1.xyww, c19.x
mul r0.y, r0, c8.x
add oT2.xyz, r3, r2
mad oT1.xy, r0.z, c9.zwzw, r0
mov oPos, r1
mov oT1.zw, r1
mad oT0.xy, v2, c18, c18.zwzw
"
}

SubProgram "xbox360 " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
Bind "vertex" Vertex
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 17 [_MainTex_ST]
Matrix 13 [_Object2World] 3
Vector 0 [_ProjectionParams]
Vector 1 [_ScreenParams]
Matrix 9 [glstate_matrix_mvp] 4
Vector 4 [unity_SHAb]
Vector 3 [unity_SHAg]
Vector 2 [unity_SHAr]
Vector 7 [unity_SHBb]
Vector 6 [unity_SHBg]
Vector 5 [unity_SHBr]
Vector 8 [unity_SHC]
Vector 16 [unity_Scale]
// Shader Timing Estimate, in Cycles/64 vertex vector:
// ALU: 29.33 (22 instructions), vertex: 32, texture: 0,
//   sequencer: 14,  5 GPRs, 31 threads,
// Performance (if enough threads): ~32 cycles per vector
// * Vertex cycle estimates are assuming 3 vfetch_minis for every vfetch_full,
//     with <= 32 bytes per vfetch_full group.

"vs_360
backbbabaaaaacjmaaaaabkiaaaaaaaaaaaaaaceaaaaacceaaaaacemaaaaaaaa
aaaaaaaaaaaaabpmaaaaaabmaaaaaboopppoadaaaaaaaaanaaaaaabmaaaaaaaa
aaaaabohaaaaabcaaaacaabbaaabaaaaaaaaabcmaaaaaaaaaaaaabdmaaacaaan
aaadaaaaaaaaabemaaaaaaaaaaaaabfmaaacaaaaaaabaaaaaaaaabcmaaaaaaaa
aaaaabgoaaacaaabaaabaaaaaaaaabcmaaaaaaaaaaaaabhmaaacaaajaaaeaaaa
aaaaabemaaaaaaaaaaaaabipaaacaaaeaaabaaaaaaaaabcmaaaaaaaaaaaaabjk
aaacaaadaaabaaaaaaaaabcmaaaaaaaaaaaaabkfaaacaaacaaabaaaaaaaaabcm
aaaaaaaaaaaaablaaaacaaahaaabaaaaaaaaabcmaaaaaaaaaaaaabllaaacaaag
aaabaaaaaaaaabcmaaaaaaaaaaaaabmgaaacaaafaaabaaaaaaaaabcmaaaaaaaa
aaaaabnbaaacaaaiaaabaaaaaaaaabcmaaaaaaaaaaaaabnlaaacaabaaaabaaaa
aaaaabcmaaaaaaaafpengbgjgofegfhifpfdfeaaaaabaaadaaabaaaeaaabaaaa
aaaaaaaafpepgcgkgfgdhedcfhgphcgmgeaaklklaaadaaadaaaeaaaeaaabaaaa
aaaaaaaafpfahcgpgkgfgdhegjgpgofagbhcgbgnhdaafpfdgdhcgfgfgofagbhc
gbgnhdaaghgmhdhegbhegffpgngbhehcgjhifpgnhghaaahfgogjhehjfpfdeieb
gcaahfgogjhehjfpfdeiebghaahfgogjhehjfpfdeiebhcaahfgogjhehjfpfdei
ecgcaahfgogjhehjfpfdeiecghaahfgogjhehjfpfdeiechcaahfgogjhehjfpfd
eiedaahfgogjhehjfpfdgdgbgmgfaahghdfpddfpdaaadccodacodcdadddfddco
daaaklklaaaaaaaaaaaaaaabaaaaaaaaaaaaaaaaaaaaaabeaapmaabaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaeaaaaaabgiaacbaaaeaaaaaaaaaaaaaaaa
aaaacegdaaaaaaabaaaaaaadaaaaaaaeaaaaacjaaabaaaaeaaaadaafaadafaag
aaaadafaaaabpbfbaaadhcfcaaaababcaaaaaabbaaaababkaaaababmaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaadpaaaaaaaaaaaaaaaaaaaaaaaaaaaaaahabfdaae
aaaabcaamcaaaaaaaaaafaahaaaabcaameaaaaaaaaaagaamgabcbcaabcaaaaaa
aaaafabiaaaaccaaaaaaaaaaafpidaaaaaaaagiiaaaaaaaaafpicaaaaaaaaoii
aaaaaaaaafpiaaaaaaaaapmiaaaaaaaamiapaaabaabliiaakbadamaamiapaaab
aamgnapikladalabmiapaaabaalbdepikladakabmiapaaabaagmiipikladajab
miapiadoaaiiiiaaocababaamialaaacaagfblaakbacbaaaceihaeadaalblegm
kbacapiamiahaaacaagmlemaklacaoadmiahaaaeaabllemaklacanacaibhabad
aaligmggkbabppaemiamiaabaaigigaaocababaamiadiaaaaalalabkilaabbbb
aicbabacaadoanmbgpacaeaeaiecabacaadoanlbgpadaeaeaiieabacaadoanlm
gpaeaeaemiabaaaaaakhkhaakpabafaamiacaaaaaakhkhaakpabagaaaibeabaa
aakhkhgmkpabahaeaiciabadaalbgmmgkbadaaaemiadiaabaamgbkbikladabad
geihaaaaaalologboaacaaabmiahiaacaablmagfklaaaiaaaaaaaaaaaaaaaaaa
aaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
Matrix 256 [glstate_matrix_mvp]
Bind "vertex" Vertex
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 467 [_ProjectionParams]
Vector 466 [unity_SHAr]
Vector 465 [unity_SHAg]
Vector 464 [unity_SHAb]
Vector 463 [unity_SHBr]
Vector 462 [unity_SHBg]
Vector 461 [unity_SHBb]
Vector 460 [unity_SHC]
Matrix 260 [_Object2World]
Vector 459 [unity_Scale]
Vector 458 [_MainTex_ST]
"sce_vp_rsx // 27 instructions using 3 registers
[Configuration]
8
0000001b01050300
[Defaults]
1
457 1
3f000000
[Microcode]
432
00009c6c009cb20c013fc0c36041dffc401f9c6c011ca808010400d740619f9c
00001c6c01d0300d8106c0c360403ffc00001c6c01d0200d8106c0c360405ffc
00001c6c01d0100d8106c0c360409ffc00001c6c01d0000d8106c0c360411ffc
00011c6c0150400c028600c360411ffc00011c6c0150600c028600c360405ffc
00009c6c0150500c028600c360411ffc401f9c6c0040000d8086c0836041ff80
401f9c6c004000558086c08360407fa000001c6c009c900e008000c36041dffc
00001c6c009d302a808000c360409ffc00001c6c008000000280014360403ffc
00011c6c004000000286c08360409ffc401f9c6c00c000080086c09540219fa0
00001c6c019d000c0486c0c360405ffc00001c6c019d100c0486c0c360409ffc
00001c6c019d200c0486c0c360411ffc00001c6c010000000480027fe0203ffc
00009c6c0080000d049a02436041fffc00011c6c01dcd00d8286c0c360405ffc
00011c6c01dce00d8286c0c360409ffc00011c6c01dcf00d8286c0c360411ffc
00001c6c00c0000c0086c0830121dffc00009c6c009cc07f808600c36041dffc
401f9c6c00c0000c0286c0830021dfa5
"
}

SubProgram "d3d11 " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
Bind "vertex" Vertex
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 112 // 96 used size, 8 vars
Vector 80 [_MainTex_ST] 4
ConstBuffer "UnityPerCamera" 128 // 96 used size, 8 vars
Vector 80 [_ProjectionParams] 4
ConstBuffer "UnityLighting" 400 // 400 used size, 16 vars
Vector 288 [unity_SHAr] 4
Vector 304 [unity_SHAg] 4
Vector 320 [unity_SHAb] 4
Vector 336 [unity_SHBr] 4
Vector 352 [unity_SHBg] 4
Vector 368 [unity_SHBb] 4
Vector 384 [unity_SHC] 4
ConstBuffer "UnityPerDraw" 336 // 336 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
Matrix 192 [_Object2World] 4
Vector 320 [unity_Scale] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityLighting" 2
BindCB "UnityPerDraw" 3
// 27 instructions, 4 temp regs, 0 temp arrays:
// ALU 15 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0
eefieceddedcifmeeidflkbdcfeldmkmpfimoodaabaaaaaaiiafaaaaadaaaaaa
cmaaaaaapeaaaaaahmabaaaaejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapaaaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaa
abaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaljaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfc
enebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheoiaaaaaaaaeaaaaaa
aiaaaaaagiaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaheaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaabaaaaaaadamaaaaheaaaaaaabaaaaaaaaaaaaaa
adaaaaaaacaaaaaaapaaaaaaheaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaa
ahaiaaaafdfgfpfaepfdejfeejepeoaafeeffiedepepfceeaaklklklfdeieefc
aeaeaaaaeaaaabaaababaaaafjaaaaaeegiocaaaaaaaaaaaagaaaaaafjaaaaae
egiocaaaabaaaaaaagaaaaaafjaaaaaeegiocaaaacaaaaaabjaaaaaafjaaaaae
egiocaaaadaaaaaabfaaaaaafpaaaaadpcbabaaaaaaaaaaafpaaaaadhcbabaaa
acaaaaaafpaaaaaddcbabaaaadaaaaaaghaaaaaepccabaaaaaaaaaaaabaaaaaa
gfaaaaaddccabaaaabaaaaaagfaaaaadpccabaaaacaaaaaagfaaaaadhccabaaa
adaaaaaagiaaaaacaeaaaaaadiaaaaaipcaabaaaaaaaaaaafgbfbaaaaaaaaaaa
egiocaaaadaaaaaaabaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaa
aaaaaaaaagbabaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaa
egiocaaaadaaaaaaacaaaaaakgbkbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaak
pcaabaaaaaaaaaaaegiocaaaadaaaaaaadaaaaaapgbpbaaaaaaaaaaaegaobaaa
aaaaaaaadgaaaaafpccabaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaaldccabaaa
abaaaaaaegbabaaaadaaaaaaegiacaaaaaaaaaaaafaaaaaaogikcaaaaaaaaaaa
afaaaaaadiaaaaaiccaabaaaaaaaaaaabkaabaaaaaaaaaaaakiacaaaabaaaaaa
afaaaaaadiaaaaakncaabaaaabaaaaaaagahbaaaaaaaaaaaaceaaaaaaaaaaadp
aaaaaaaaaaaaaadpaaaaaadpdgaaaaafmccabaaaacaaaaaakgaobaaaaaaaaaaa
aaaaaaahdccabaaaacaaaaaakgakbaaaabaaaaaamgaabaaaabaaaaaadiaaaaai
hcaabaaaaaaaaaaaegbcbaaaacaaaaaapgipcaaaadaaaaaabeaaaaaadiaaaaai
hcaabaaaabaaaaaafgafbaaaaaaaaaaaegiccaaaadaaaaaaanaaaaaadcaaaaak
lcaabaaaaaaaaaaaegiicaaaadaaaaaaamaaaaaaagaabaaaaaaaaaaaegaibaaa
abaaaaaadcaaaaakhcaabaaaaaaaaaaaegiccaaaadaaaaaaaoaaaaaakgakbaaa
aaaaaaaaegadbaaaaaaaaaaadgaaaaaficaabaaaaaaaaaaaabeaaaaaaaaaiadp
bbaaaaaibcaabaaaabaaaaaaegiocaaaacaaaaaabcaaaaaaegaobaaaaaaaaaaa
bbaaaaaiccaabaaaabaaaaaaegiocaaaacaaaaaabdaaaaaaegaobaaaaaaaaaaa
bbaaaaaiecaabaaaabaaaaaaegiocaaaacaaaaaabeaaaaaaegaobaaaaaaaaaaa
diaaaaahpcaabaaaacaaaaaajgacbaaaaaaaaaaaegakbaaaaaaaaaaabbaaaaai
bcaabaaaadaaaaaaegiocaaaacaaaaaabfaaaaaaegaobaaaacaaaaaabbaaaaai
ccaabaaaadaaaaaaegiocaaaacaaaaaabgaaaaaaegaobaaaacaaaaaabbaaaaai
ecaabaaaadaaaaaaegiocaaaacaaaaaabhaaaaaaegaobaaaacaaaaaaaaaaaaah
hcaabaaaabaaaaaaegacbaaaabaaaaaaegacbaaaadaaaaaadiaaaaahccaabaaa
aaaaaaaabkaabaaaaaaaaaaabkaabaaaaaaaaaaadcaaaaakbcaabaaaaaaaaaaa
akaabaaaaaaaaaaaakaabaaaaaaaaaaabkaabaiaebaaaaaaaaaaaaaadcaaaaak
hccabaaaadaaaaaaegiccaaaacaaaaaabiaaaaaaagaabaaaaaaaaaaaegacbaaa
abaaaaaadoaaaaab"
}

SubProgram "gles " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
"!!GLES
#define SHADER_API_GLES 1
#define tex2D texture2D


#ifdef VERTEX
#define gl_ModelViewProjectionMatrix glstate_matrix_mvp
uniform mat4 glstate_matrix_mvp;

varying highp vec3 xlv_TEXCOORD2;
varying highp vec4 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 unity_Scale;
uniform highp mat4 _Object2World;

uniform highp vec4 unity_SHC;
uniform highp vec4 unity_SHBb;
uniform highp vec4 unity_SHBg;
uniform highp vec4 unity_SHBr;
uniform highp vec4 unity_SHAb;
uniform highp vec4 unity_SHAg;
uniform highp vec4 unity_SHAr;
uniform highp vec4 _ProjectionParams;
attribute vec4 _glesMultiTexCoord0;
attribute vec3 _glesNormal;
attribute vec4 _glesVertex;
void main ()
{
  highp vec3 tmpvar_1;
  highp vec4 tmpvar_2;
  tmpvar_2 = (gl_ModelViewProjectionMatrix * _glesVertex);
  highp vec4 o_3;
  highp vec4 tmpvar_4;
  tmpvar_4 = (tmpvar_2 * 0.5);
  highp vec2 tmpvar_5;
  tmpvar_5.x = tmpvar_4.x;
  tmpvar_5.y = (tmpvar_4.y * _ProjectionParams.x);
  o_3.xy = (tmpvar_5 + tmpvar_4.w);
  o_3.zw = tmpvar_2.zw;
  mat3 tmpvar_6;
  tmpvar_6[0] = _Object2World[0].xyz;
  tmpvar_6[1] = _Object2World[1].xyz;
  tmpvar_6[2] = _Object2World[2].xyz;
  highp vec4 tmpvar_7;
  tmpvar_7.w = 1.0;
  tmpvar_7.xyz = (tmpvar_6 * (normalize(_glesNormal) * unity_Scale.w));
  mediump vec3 tmpvar_8;
  mediump vec4 normal_9;
  normal_9 = tmpvar_7;
  highp float vC_10;
  mediump vec3 x3_11;
  mediump vec3 x2_12;
  mediump vec3 x1_13;
  highp float tmpvar_14;
  tmpvar_14 = dot (unity_SHAr, normal_9);
  x1_13.x = tmpvar_14;
  highp float tmpvar_15;
  tmpvar_15 = dot (unity_SHAg, normal_9);
  x1_13.y = tmpvar_15;
  highp float tmpvar_16;
  tmpvar_16 = dot (unity_SHAb, normal_9);
  x1_13.z = tmpvar_16;
  mediump vec4 tmpvar_17;
  tmpvar_17 = (normal_9.xyzz * normal_9.yzzx);
  highp float tmpvar_18;
  tmpvar_18 = dot (unity_SHBr, tmpvar_17);
  x2_12.x = tmpvar_18;
  highp float tmpvar_19;
  tmpvar_19 = dot (unity_SHBg, tmpvar_17);
  x2_12.y = tmpvar_19;
  highp float tmpvar_20;
  tmpvar_20 = dot (unity_SHBb, tmpvar_17);
  x2_12.z = tmpvar_20;
  mediump float tmpvar_21;
  tmpvar_21 = ((normal_9.x * normal_9.x) - (normal_9.y * normal_9.y));
  vC_10 = tmpvar_21;
  highp vec3 tmpvar_22;
  tmpvar_22 = (unity_SHC.xyz * vC_10);
  x3_11 = tmpvar_22;
  tmpvar_8 = ((x1_13 + x2_12) + x3_11);
  tmpvar_1 = tmpvar_8;
  gl_Position = tmpvar_2;
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = o_3;
  xlv_TEXCOORD2 = tmpvar_1;
}



#endif
#ifdef FRAGMENT

varying highp vec3 xlv_TEXCOORD2;
varying highp vec4 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform sampler2D _LightBuffer;
uniform mediump float _Gloss;
uniform lowp vec4 _Color;
uniform sampler2D _SpecMap;
uniform sampler2D _MainTex;
uniform lowp vec4 _SpecColor;
void main ()
{
  lowp vec4 tmpvar_1;
  mediump vec4 light_2;
  mediump vec3 tmpvar_3;
  lowp vec4 tmpvar_4;
  tmpvar_4 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_5;
  tmpvar_5 = (tmpvar_4.xyz * _Color.xyz);
  tmpvar_3 = tmpvar_5;
  lowp vec4 tmpvar_6;
  tmpvar_6 = texture2D (_SpecMap, xlv_TEXCOORD0);
  lowp vec4 tmpvar_7;
  tmpvar_7 = texture2DProj (_LightBuffer, xlv_TEXCOORD1);
  light_2 = tmpvar_7;
  mediump vec4 tmpvar_8;
  tmpvar_8 = max (light_2, vec4(0.001, 0.001, 0.001, 0.001));
  light_2.w = tmpvar_8.w;
  highp vec3 tmpvar_9;
  tmpvar_9 = (tmpvar_8.xyz + xlv_TEXCOORD2);
  light_2.xyz = tmpvar_9;
  mediump vec4 c_10;
  mediump vec3 tmpvar_11;
  tmpvar_11 = (tmpvar_8.w * ((tmpvar_4.w * tmpvar_6.xyz) * _Gloss));
  c_10.xyz = ((tmpvar_3 * light_2.xyz) + (light_2.xyz * tmpvar_11));
  c_10.w = (tmpvar_11 * _SpecColor.w).x;
  tmpvar_1 = c_10;
  gl_FragData[0] = tmpvar_1;
}



#endif"
}

SubProgram "glesdesktop " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
"!!GLES
#define SHADER_API_GLES 1
#define tex2D texture2D


#ifdef VERTEX
#define gl_ModelViewProjectionMatrix glstate_matrix_mvp
uniform mat4 glstate_matrix_mvp;

varying highp vec3 xlv_TEXCOORD2;
varying highp vec4 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 unity_Scale;
uniform highp mat4 _Object2World;

uniform highp vec4 unity_SHC;
uniform highp vec4 unity_SHBb;
uniform highp vec4 unity_SHBg;
uniform highp vec4 unity_SHBr;
uniform highp vec4 unity_SHAb;
uniform highp vec4 unity_SHAg;
uniform highp vec4 unity_SHAr;
uniform highp vec4 _ProjectionParams;
attribute vec4 _glesMultiTexCoord0;
attribute vec3 _glesNormal;
attribute vec4 _glesVertex;
void main ()
{
  highp vec3 tmpvar_1;
  highp vec4 tmpvar_2;
  tmpvar_2 = (gl_ModelViewProjectionMatrix * _glesVertex);
  highp vec4 o_3;
  highp vec4 tmpvar_4;
  tmpvar_4 = (tmpvar_2 * 0.5);
  highp vec2 tmpvar_5;
  tmpvar_5.x = tmpvar_4.x;
  tmpvar_5.y = (tmpvar_4.y * _ProjectionParams.x);
  o_3.xy = (tmpvar_5 + tmpvar_4.w);
  o_3.zw = tmpvar_2.zw;
  mat3 tmpvar_6;
  tmpvar_6[0] = _Object2World[0].xyz;
  tmpvar_6[1] = _Object2World[1].xyz;
  tmpvar_6[2] = _Object2World[2].xyz;
  highp vec4 tmpvar_7;
  tmpvar_7.w = 1.0;
  tmpvar_7.xyz = (tmpvar_6 * (normalize(_glesNormal) * unity_Scale.w));
  mediump vec3 tmpvar_8;
  mediump vec4 normal_9;
  normal_9 = tmpvar_7;
  highp float vC_10;
  mediump vec3 x3_11;
  mediump vec3 x2_12;
  mediump vec3 x1_13;
  highp float tmpvar_14;
  tmpvar_14 = dot (unity_SHAr, normal_9);
  x1_13.x = tmpvar_14;
  highp float tmpvar_15;
  tmpvar_15 = dot (unity_SHAg, normal_9);
  x1_13.y = tmpvar_15;
  highp float tmpvar_16;
  tmpvar_16 = dot (unity_SHAb, normal_9);
  x1_13.z = tmpvar_16;
  mediump vec4 tmpvar_17;
  tmpvar_17 = (normal_9.xyzz * normal_9.yzzx);
  highp float tmpvar_18;
  tmpvar_18 = dot (unity_SHBr, tmpvar_17);
  x2_12.x = tmpvar_18;
  highp float tmpvar_19;
  tmpvar_19 = dot (unity_SHBg, tmpvar_17);
  x2_12.y = tmpvar_19;
  highp float tmpvar_20;
  tmpvar_20 = dot (unity_SHBb, tmpvar_17);
  x2_12.z = tmpvar_20;
  mediump float tmpvar_21;
  tmpvar_21 = ((normal_9.x * normal_9.x) - (normal_9.y * normal_9.y));
  vC_10 = tmpvar_21;
  highp vec3 tmpvar_22;
  tmpvar_22 = (unity_SHC.xyz * vC_10);
  x3_11 = tmpvar_22;
  tmpvar_8 = ((x1_13 + x2_12) + x3_11);
  tmpvar_1 = tmpvar_8;
  gl_Position = tmpvar_2;
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = o_3;
  xlv_TEXCOORD2 = tmpvar_1;
}



#endif
#ifdef FRAGMENT

varying highp vec3 xlv_TEXCOORD2;
varying highp vec4 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform sampler2D _LightBuffer;
uniform mediump float _Gloss;
uniform lowp vec4 _Color;
uniform sampler2D _SpecMap;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform lowp vec4 _SpecColor;
void main ()
{
  lowp vec4 tmpvar_1;
  mediump vec4 light_2;
  mediump vec3 tmpvar_3;
  lowp vec4 tmpvar_4;
  tmpvar_4 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_5;
  tmpvar_5 = (tmpvar_4.xyz * _Color.xyz);
  tmpvar_3 = tmpvar_5;
  lowp vec4 tmpvar_6;
  tmpvar_6 = texture2D (_SpecMap, xlv_TEXCOORD0);
  lowp vec3 normal_7;
  normal_7.xy = ((texture2D (_BumpMap, xlv_TEXCOORD0).wy * 2.0) - 1.0);
  normal_7.z = sqrt(((1.0 - (normal_7.x * normal_7.x)) - (normal_7.y * normal_7.y)));
  lowp vec4 tmpvar_8;
  tmpvar_8 = texture2DProj (_LightBuffer, xlv_TEXCOORD1);
  light_2 = tmpvar_8;
  mediump vec4 tmpvar_9;
  tmpvar_9 = max (light_2, vec4(0.001, 0.001, 0.001, 0.001));
  light_2.w = tmpvar_9.w;
  highp vec3 tmpvar_10;
  tmpvar_10 = (tmpvar_9.xyz + xlv_TEXCOORD2);
  light_2.xyz = tmpvar_10;
  mediump vec4 c_11;
  mediump vec3 tmpvar_12;
  tmpvar_12 = (tmpvar_9.w * ((tmpvar_4.w * tmpvar_6.xyz) * _Gloss));
  c_11.xyz = ((tmpvar_3 * light_2.xyz) + (light_2.xyz * tmpvar_12));
  c_11.w = (tmpvar_12 * _SpecColor.w).x;
  tmpvar_1 = c_11;
  gl_FragData[0] = tmpvar_1;
}



#endif"
}

SubProgram "d3d11_9x " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
Bind "vertex" Vertex
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 112 // 96 used size, 8 vars
Vector 80 [_MainTex_ST] 4
ConstBuffer "UnityPerCamera" 128 // 96 used size, 8 vars
Vector 80 [_ProjectionParams] 4
ConstBuffer "UnityLighting" 400 // 400 used size, 16 vars
Vector 288 [unity_SHAr] 4
Vector 304 [unity_SHAg] 4
Vector 320 [unity_SHAb] 4
Vector 336 [unity_SHBr] 4
Vector 352 [unity_SHBg] 4
Vector 368 [unity_SHBb] 4
Vector 384 [unity_SHC] 4
ConstBuffer "UnityPerDraw" 336 // 336 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
Matrix 192 [_Object2World] 4
Vector 320 [unity_Scale] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityLighting" 2
BindCB "UnityPerDraw" 3
// 27 instructions, 4 temp regs, 0 temp arrays:
// ALU 15 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0_level_9_3
eefiecednopjaebfgepmgbcelannfelhlaghpaebabaaaaaacaaiaaaaaeaaaaaa
daaaaaaameacaaaanaagaaaajiahaaaaebgpgodjimacaaaaimacaaaaaaacpopp
bmacaaaahaaaaaaaagaaceaaaaaagmaaaaaagmaaaaaaceaaabaagmaaaaaaafaa
abaaabaaaaaaaaaaabaaafaaabaaacaaaaaaaaaaacaabcaaahaaadaaaaaaaaaa
adaaaaaaaeaaakaaaaaaaaaaadaaamaaadaaaoaaaaaaaaaaadaabeaaabaabbaa
aaaaaaaaaaaaaaaaabacpoppfbaaaaafbcaaapkaaaaaaadpaaaaiadpaaaaaaaa
aaaaaaaabpaaaaacafaaaaiaaaaaapjabpaaaaacafaaaciaacaaapjabpaaaaac
afaaadiaadaaapjaaeaaaaaeaaaaadoaadaaoejaabaaoekaabaaookaafaaaaad
aaaaapiaaaaaffjaalaaoekaaeaaaaaeaaaaapiaakaaoekaaaaaaajaaaaaoeia
aeaaaaaeaaaaapiaamaaoekaaaaakkjaaaaaoeiaaeaaaaaeaaaaapiaanaaoeka
aaaappjaaaaaoeiaafaaaaadabaaabiaaaaaffiaacaaaakaafaaaaadabaaaiia
abaaaaiabcaaaakaafaaaaadabaaafiaaaaapeiabcaaaakaacaaaaadabaaadoa
abaakkiaabaaomiaafaaaaadabaaahiaacaaoejabbaappkaafaaaaadacaaahia
abaaffiaapaaoekaaeaaaaaeabaaaliaaoaakekaabaaaaiaacaakeiaaeaaaaae
abaaahiabaaaoekaabaakkiaabaapeiaabaaaaacabaaaiiabcaaffkaajaaaaad
acaaabiaadaaoekaabaaoeiaajaaaaadacaaaciaaeaaoekaabaaoeiaajaaaaad
acaaaeiaafaaoekaabaaoeiaafaaaaadadaaapiaabaacjiaabaakeiaajaaaaad
aeaaabiaagaaoekaadaaoeiaajaaaaadaeaaaciaahaaoekaadaaoeiaajaaaaad
aeaaaeiaaiaaoekaadaaoeiaacaaaaadacaaahiaacaaoeiaaeaaoeiaafaaaaad
abaaaciaabaaffiaabaaffiaaeaaaaaeabaaabiaabaaaaiaabaaaaiaabaaffib
aeaaaaaeacaaahoaajaaoekaabaaaaiaacaaoeiaaeaaaaaeaaaaadmaaaaappia
aaaaoekaaaaaoeiaabaaaaacaaaaammaaaaaoeiaabaaaaacabaaamoaaaaaoeia
ppppaaaafdeieefcaeaeaaaaeaaaabaaababaaaafjaaaaaeegiocaaaaaaaaaaa
agaaaaaafjaaaaaeegiocaaaabaaaaaaagaaaaaafjaaaaaeegiocaaaacaaaaaa
bjaaaaaafjaaaaaeegiocaaaadaaaaaabfaaaaaafpaaaaadpcbabaaaaaaaaaaa
fpaaaaadhcbabaaaacaaaaaafpaaaaaddcbabaaaadaaaaaaghaaaaaepccabaaa
aaaaaaaaabaaaaaagfaaaaaddccabaaaabaaaaaagfaaaaadpccabaaaacaaaaaa
gfaaaaadhccabaaaadaaaaaagiaaaaacaeaaaaaadiaaaaaipcaabaaaaaaaaaaa
fgbfbaaaaaaaaaaaegiocaaaadaaaaaaabaaaaaadcaaaaakpcaabaaaaaaaaaaa
egiocaaaadaaaaaaaaaaaaaaagbabaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaak
pcaabaaaaaaaaaaaegiocaaaadaaaaaaacaaaaaakgbkbaaaaaaaaaaaegaobaaa
aaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaadaaaaaapgbpbaaa
aaaaaaaaegaobaaaaaaaaaaadgaaaaafpccabaaaaaaaaaaaegaobaaaaaaaaaaa
dcaaaaaldccabaaaabaaaaaaegbabaaaadaaaaaaegiacaaaaaaaaaaaafaaaaaa
ogikcaaaaaaaaaaaafaaaaaadiaaaaaiccaabaaaaaaaaaaabkaabaaaaaaaaaaa
akiacaaaabaaaaaaafaaaaaadiaaaaakncaabaaaabaaaaaaagahbaaaaaaaaaaa
aceaaaaaaaaaaadpaaaaaaaaaaaaaadpaaaaaadpdgaaaaafmccabaaaacaaaaaa
kgaobaaaaaaaaaaaaaaaaaahdccabaaaacaaaaaakgakbaaaabaaaaaamgaabaaa
abaaaaaadiaaaaaihcaabaaaaaaaaaaaegbcbaaaacaaaaaapgipcaaaadaaaaaa
beaaaaaadiaaaaaihcaabaaaabaaaaaafgafbaaaaaaaaaaaegiccaaaadaaaaaa
anaaaaaadcaaaaaklcaabaaaaaaaaaaaegiicaaaadaaaaaaamaaaaaaagaabaaa
aaaaaaaaegaibaaaabaaaaaadcaaaaakhcaabaaaaaaaaaaaegiccaaaadaaaaaa
aoaaaaaakgakbaaaaaaaaaaaegadbaaaaaaaaaaadgaaaaaficaabaaaaaaaaaaa
abeaaaaaaaaaiadpbbaaaaaibcaabaaaabaaaaaaegiocaaaacaaaaaabcaaaaaa
egaobaaaaaaaaaaabbaaaaaiccaabaaaabaaaaaaegiocaaaacaaaaaabdaaaaaa
egaobaaaaaaaaaaabbaaaaaiecaabaaaabaaaaaaegiocaaaacaaaaaabeaaaaaa
egaobaaaaaaaaaaadiaaaaahpcaabaaaacaaaaaajgacbaaaaaaaaaaaegakbaaa
aaaaaaaabbaaaaaibcaabaaaadaaaaaaegiocaaaacaaaaaabfaaaaaaegaobaaa
acaaaaaabbaaaaaiccaabaaaadaaaaaaegiocaaaacaaaaaabgaaaaaaegaobaaa
acaaaaaabbaaaaaiecaabaaaadaaaaaaegiocaaaacaaaaaabhaaaaaaegaobaaa
acaaaaaaaaaaaaahhcaabaaaabaaaaaaegacbaaaabaaaaaaegacbaaaadaaaaaa
diaaaaahccaabaaaaaaaaaaabkaabaaaaaaaaaaabkaabaaaaaaaaaaadcaaaaak
bcaabaaaaaaaaaaaakaabaaaaaaaaaaaakaabaaaaaaaaaaabkaabaiaebaaaaaa
aaaaaaaadcaaaaakhccabaaaadaaaaaaegiccaaaacaaaaaabiaaaaaaagaabaaa
aaaaaaaaegacbaaaabaaaaaadoaaaaabejfdeheomaaaaaaaagaaaaaaaiaaaaaa
jiaaaaaaaaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaabaaaaaaapaaaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaa
acaaaaaaahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaa
laaaaaaaabaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaljaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofe
aaeoepfcenebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheoiaaaaaaa
aeaaaaaaaiaaaaaagiaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaa
heaaaaaaaaaaaaaaaaaaaaaaadaaaaaaabaaaaaaadamaaaaheaaaaaaabaaaaaa
aaaaaaaaadaaaaaaacaaaaaaapaaaaaaheaaaaaaacaaaaaaaaaaaaaaadaaaaaa
adaaaaaaahaiaaaafdfgfpfaepfdejfeejepeoaafeeffiedepepfceeaaklklkl
"
}

SubProgram "opengl " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
Bind "vertex" Vertex
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Vector 13 [_ProjectionParams]
Vector 14 [unity_ShadowFadeCenterAndType]
Matrix 9 [_Object2World]
Vector 15 [unity_LightmapST]
Vector 16 [_MainTex_ST]
"!!ARBvp1.0
# 20 ALU
PARAM c[17] = { { 0.5, 1 },
		state.matrix.modelview[0],
		state.matrix.mvp,
		program.local[9..16] };
TEMP R0;
TEMP R1;
DP4 R0.w, vertex.position, c[8];
DP4 R0.z, vertex.position, c[7];
DP4 R0.x, vertex.position, c[5];
DP4 R0.y, vertex.position, c[6];
MUL R1.xyz, R0.xyww, c[0].x;
MUL R1.y, R1, c[13].x;
ADD result.texcoord[1].xy, R1, R1.z;
MOV result.position, R0;
MOV R0.x, c[0].y;
ADD R0.y, R0.x, -c[14].w;
DP4 R0.x, vertex.position, c[3];
DP4 R1.z, vertex.position, c[11];
DP4 R1.x, vertex.position, c[9];
DP4 R1.y, vertex.position, c[10];
ADD R1.xyz, R1, -c[14];
MOV result.texcoord[1].zw, R0;
MUL result.texcoord[3].xyz, R1, c[14].w;
MAD result.texcoord[0].xy, vertex.texcoord[0], c[16], c[16].zwzw;
MAD result.texcoord[2].xy, vertex.texcoord[1], c[15], c[15].zwzw;
MUL result.texcoord[3].w, -R0.x, R0.y;
END
# 20 instructions, 2 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
Bind "vertex" Vertex
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Matrix 0 [glstate_matrix_modelview0]
Matrix 4 [glstate_matrix_mvp]
Vector 12 [_ProjectionParams]
Vector 13 [_ScreenParams]
Vector 14 [unity_ShadowFadeCenterAndType]
Matrix 8 [_Object2World]
Vector 15 [unity_LightmapST]
Vector 16 [_MainTex_ST]
"vs_2_0
; 20 ALU
def c17, 0.50000000, 1.00000000, 0, 0
dcl_position0 v0
dcl_texcoord0 v1
dcl_texcoord1 v2
dp4 r0.w, v0, c7
dp4 r0.z, v0, c6
dp4 r0.x, v0, c4
dp4 r0.y, v0, c5
mul r1.xyz, r0.xyww, c17.x
mul r1.y, r1, c12.x
mad oT1.xy, r1.z, c13.zwzw, r1
mov oPos, r0
mov r0.x, c14.w
add r0.y, c17, -r0.x
dp4 r0.x, v0, c2
dp4 r1.z, v0, c10
dp4 r1.x, v0, c8
dp4 r1.y, v0, c9
add r1.xyz, r1, -c14
mov oT1.zw, r0
mul oT3.xyz, r1, c14.w
mad oT0.xy, v1, c16, c16.zwzw
mad oT2.xy, v2, c15, c15.zwzw
mul oT3.w, -r0.x, r0.y
"
}

SubProgram "xbox360 " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
Bind "vertex" Vertex
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Vector 16 [_MainTex_ST]
Matrix 11 [_Object2World] 4
Vector 0 [_ProjectionParams]
Vector 1 [_ScreenParams]
Matrix 7 [glstate_matrix_modelview0] 4
Matrix 3 [glstate_matrix_mvp] 4
Vector 15 [unity_LightmapST]
Vector 2 [unity_ShadowFadeCenterAndType]
// Shader Timing Estimate, in Cycles/64 vertex vector:
// ALU: 28.00 (21 instructions), vertex: 32, texture: 0,
//   sequencer: 14,  7 GPRs, 27 threads,
// Performance (if enough threads): ~32 cycles per vector
// * Vertex cycle estimates are assuming 3 vfetch_minis for every vfetch_full,
//     with <= 32 bytes per vfetch_full group.

"vs_360
backbbabaaaaacdeaaaaabjmaaaaaaaaaaaaaaceaaaaablaaaaaabniaaaaaaaa
aaaaaaaaaaaaabiiaaaaaabmaaaaabhlpppoadaaaaaaaaaiaaaaaabmaaaaaaaa
aaaaabheaaaaaalmaaacaabaaaabaaaaaaaaaamiaaaaaaaaaaaaaaniaaacaaal
aaaeaaaaaaaaaaoiaaaaaaaaaaaaaapiaaacaaaaaaabaaaaaaaaaamiaaaaaaaa
aaaaabakaaacaaabaaabaaaaaaaaaamiaaaaaaaaaaaaabbiaaacaaahaaaeaaaa
aaaaaaoiaaaaaaaaaaaaabdcaaacaaadaaaeaaaaaaaaaaoiaaaaaaaaaaaaabef
aaacaaapaaabaaaaaaaaaamiaaaaaaaaaaaaabfgaaacaaacaaabaaaaaaaaaami
aaaaaaaafpengbgjgofegfhifpfdfeaaaaabaaadaaabaaaeaaabaaaaaaaaaaaa
fpepgcgkgfgdhedcfhgphcgmgeaaklklaaadaaadaaaeaaaeaaabaaaaaaaaaaaa
fpfahcgpgkgfgdhegjgpgofagbhcgbgnhdaafpfdgdhcgfgfgofagbhcgbgnhdaa
ghgmhdhegbhegffpgngbhehcgjhifpgngpgegfgmhggjgfhhdaaaghgmhdhegbhe
gffpgngbhehcgjhifpgnhghaaahfgogjhehjfpemgjghgihegngbhafdfeaahfgo
gjhehjfpfdgigbgegphheggbgegfedgfgohegfhcebgogefehjhagfaahghdfpdd
fpdaaadccodacodcdadddfddcodaaaklaaaaaaaaaaaaaaabaaaaaaaaaaaaaaaa
aaaaaabeaapmaabaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaeaaaaaabfm
aadbaaagaaaaaaaaaaaaaaaaaaaadaieaaaaaaabaaaaaaadaaaaaaagaaaaacja
aabaaaaeaaaafaafaacbfaagaaaadafaaaabpbfbaaaddcfcaaaepdfdaaaababe
aaaaaabdaaaabablaaaababfaaaaaabiaaaababkaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaadpiaaaaadpaaaaaaaaaaaaaaaaaaaaaahabfdaaeaaaabcaamcaaaaaa
aaaafaahaaaabcaameaaaaaaaaaagaamgabcbcaabcaaaaaaaaaaeabiaaaaccaa
aaaaaaaaafpicaaaaaaaagiiaaaaaaaaafpieaaaaaaaapmiaaaaaaaaafpibaaa
aaaaaoehaaaaaaaamiapaaaaaabliiaakbacagaamiapaaaaaamgnapiklacafaa
miapaaaaaalbdepiklacaeaamiapaaafaagmnajeklacadaamiapiadoaananaaa
ocafafaakiibagabaeblgmmacaacppahmiahaaadaablleaakbacaoaabeboaaaa
aamgimlbkbacanacmiaoaaaaaalbimabklacamaamiahaaagaagmlebfklacalaa
kiioadaaaapmlbmaibafppaimiapaaadaadedeaaoaagadaamiamiaabaanlnlaa
ocafafaamiadiaaaaalalabkilaebabamiadiaacaamflabkilabapapmiacaaab
aamgmgblklacajadkibhaaadacmamaeciaadacaamiahiaadaamablaakbadacaa
miacaaabaablmglbklacakabmiaiiaadaelbgmaaobababaamiadiaabaablbkgn
klaaabaaaaaaaaaaaaaaaaaaaaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
Matrix 256 [glstate_matrix_modelview0]
Matrix 260 [glstate_matrix_mvp]
Bind "vertex" Vertex
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Vector 467 [_ProjectionParams]
Vector 466 [unity_ShadowFadeCenterAndType]
Matrix 264 [_Object2World]
Vector 465 [unity_LightmapST]
Vector 464 [_MainTex_ST]
"sce_vp_rsx // 21 instructions using 3 registers
[Configuration]
8
0000001503010300
[Defaults]
1
463 2
3f0000003f800000
[Microcode]
336
401f9c6c011d0808010400d740619f9c401f9c6c011d1908010400d740619fa4
00001c6c01d0200d8106c0c360403ffc00009c6c01d0700d8106c0c360403ffc
00009c6c01d0600d8106c0c360405ffc00009c6c01d0500d8106c0c360409ffc
00009c6c01d0400d8106c0c360411ffc00011c6c005d207f8186c08360411ffc
00001c6c01d0a00d8106c0c360405ffc00001c6c01d0900d8106c0c360409ffc
00001c6c01d0800d8106c0c360411ffc00001c6c00dd208c0186c0830021dffc
00011c6c00dcf02a8186c0a001211ffc401f9c6c0040000d8286c0836041ff80
401f9c6c004000558286c08360407fa000009c6c009cf00e028000c36041dffc
401f9c6c009d200c00bfc0c36041dfa8401f9c6c008000ff8080024360403fa8
00001c6c004000000286c08360411ffc00001c6c009d302a828000c360409ffc
401f9c6c00c000080086c09540a19fa1
"
}

SubProgram "d3d11 " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
Bind "vertex" Vertex
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Bind "color" Color
ConstBuffer "$Globals" 144 // 112 used size, 10 vars
Vector 80 [unity_LightmapST] 4
Vector 96 [_MainTex_ST] 4
ConstBuffer "UnityPerCamera" 128 // 96 used size, 8 vars
Vector 80 [_ProjectionParams] 4
ConstBuffer "UnityShadows" 416 // 416 used size, 8 vars
Vector 400 [unity_ShadowFadeCenterAndType] 4
ConstBuffer "UnityPerDraw" 336 // 256 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
Matrix 64 [glstate_matrix_modelview0] 4
Matrix 192 [_Object2World] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityShadows" 2
BindCB "UnityPerDraw" 3
// 24 instructions, 2 temp regs, 0 temp arrays:
// ALU 10 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0
eefieceddegfopjhbogfokaeladbmdgflgadlgnfabaaaaaaiiafaaaaadaaaaaa
cmaaaaaapeaaaaaajeabaaaaejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapaaaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahaaaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaa
abaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapadaaaaljaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfc
enebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheojiaaaaaaafaaaaaa
aiaaaaaaiaaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaabaaaaaaadamaaaaimaaaaaaacaaaaaaaaaaaaaa
adaaaaaaabaaaaaaamadaaaaimaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaa
apaaaaaaimaaaaaaadaaaaaaaaaaaaaaadaaaaaaadaaaaaaapaaaaaafdfgfpfa
epfdejfeejepeoaafeeffiedepepfceeaaklklklfdeieefcomadaaaaeaaaabaa
plaaaaaafjaaaaaeegiocaaaaaaaaaaaahaaaaaafjaaaaaeegiocaaaabaaaaaa
agaaaaaafjaaaaaeegiocaaaacaaaaaabkaaaaaafjaaaaaeegiocaaaadaaaaaa
baaaaaaafpaaaaadpcbabaaaaaaaaaaafpaaaaaddcbabaaaadaaaaaafpaaaaad
dcbabaaaaeaaaaaaghaaaaaepccabaaaaaaaaaaaabaaaaaagfaaaaaddccabaaa
abaaaaaagfaaaaadmccabaaaabaaaaaagfaaaaadpccabaaaacaaaaaagfaaaaad
pccabaaaadaaaaaagiaaaaacacaaaaaadiaaaaaipcaabaaaaaaaaaaafgbfbaaa
aaaaaaaaegiocaaaadaaaaaaabaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaa
adaaaaaaaaaaaaaaagbabaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaa
aaaaaaaaegiocaaaadaaaaaaacaaaaaakgbkbaaaaaaaaaaaegaobaaaaaaaaaaa
dcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaadaaaaaapgbpbaaaaaaaaaaa
egaobaaaaaaaaaaadgaaaaafpccabaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaal
dccabaaaabaaaaaaegbabaaaadaaaaaaegiacaaaaaaaaaaaagaaaaaaogikcaaa
aaaaaaaaagaaaaaadcaaaaalmccabaaaabaaaaaaagbebaaaaeaaaaaaagiecaaa
aaaaaaaaafaaaaaakgiocaaaaaaaaaaaafaaaaaadiaaaaaiccaabaaaaaaaaaaa
bkaabaaaaaaaaaaaakiacaaaabaaaaaaafaaaaaadiaaaaakncaabaaaabaaaaaa
agahbaaaaaaaaaaaaceaaaaaaaaaaadpaaaaaaaaaaaaaadpaaaaaadpdgaaaaaf
mccabaaaacaaaaaakgaobaaaaaaaaaaaaaaaaaahdccabaaaacaaaaaakgakbaaa
abaaaaaamgaabaaaabaaaaaadiaaaaaihcaabaaaaaaaaaaafgbfbaaaaaaaaaaa
egiccaaaadaaaaaaanaaaaaadcaaaaakhcaabaaaaaaaaaaaegiccaaaadaaaaaa
amaaaaaaagbabaaaaaaaaaaaegacbaaaaaaaaaaadcaaaaakhcaabaaaaaaaaaaa
egiccaaaadaaaaaaaoaaaaaakgbkbaaaaaaaaaaaegacbaaaaaaaaaaadcaaaaak
hcaabaaaaaaaaaaaegiccaaaadaaaaaaapaaaaaapgbpbaaaaaaaaaaaegacbaaa
aaaaaaaaaaaaaaajhcaabaaaaaaaaaaaegacbaaaaaaaaaaaegiccaiaebaaaaaa
acaaaaaabjaaaaaadiaaaaaihccabaaaadaaaaaaegacbaaaaaaaaaaapgipcaaa
acaaaaaabjaaaaaadiaaaaaibcaabaaaaaaaaaaabkbabaaaaaaaaaaackiacaaa
adaaaaaaafaaaaaadcaaaaakbcaabaaaaaaaaaaackiacaaaadaaaaaaaeaaaaaa
akbabaaaaaaaaaaaakaabaaaaaaaaaaadcaaaaakbcaabaaaaaaaaaaackiacaaa
adaaaaaaagaaaaaackbabaaaaaaaaaaaakaabaaaaaaaaaaadcaaaaakbcaabaaa
aaaaaaaackiacaaaadaaaaaaahaaaaaadkbabaaaaaaaaaaaakaabaaaaaaaaaaa
aaaaaaajccaabaaaaaaaaaaadkiacaiaebaaaaaaacaaaaaabjaaaaaaabeaaaaa
aaaaiadpdiaaaaaiiccabaaaadaaaaaabkaabaaaaaaaaaaaakaabaiaebaaaaaa
aaaaaaaadoaaaaab"
}

SubProgram "gles " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
"!!GLES
#define SHADER_API_GLES 1
#define tex2D texture2D


#ifdef VERTEX
#define gl_ModelViewProjectionMatrix glstate_matrix_mvp
uniform mat4 glstate_matrix_mvp;
#define gl_ModelViewMatrix glstate_matrix_modelview0
uniform mat4 glstate_matrix_modelview0;

varying highp vec4 xlv_TEXCOORD3;
varying highp vec2 xlv_TEXCOORD2;
varying highp vec4 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 unity_LightmapST;
uniform highp mat4 _Object2World;


uniform highp vec4 unity_ShadowFadeCenterAndType;
uniform highp vec4 _ProjectionParams;
attribute vec4 _glesMultiTexCoord1;
attribute vec4 _glesMultiTexCoord0;
attribute vec4 _glesVertex;
void main ()
{
  highp vec4 tmpvar_1;
  highp vec4 tmpvar_2;
  tmpvar_2 = (gl_ModelViewProjectionMatrix * _glesVertex);
  highp vec4 o_3;
  highp vec4 tmpvar_4;
  tmpvar_4 = (tmpvar_2 * 0.5);
  highp vec2 tmpvar_5;
  tmpvar_5.x = tmpvar_4.x;
  tmpvar_5.y = (tmpvar_4.y * _ProjectionParams.x);
  o_3.xy = (tmpvar_5 + tmpvar_4.w);
  o_3.zw = tmpvar_2.zw;
  tmpvar_1.xyz = (((_Object2World * _glesVertex).xyz - unity_ShadowFadeCenterAndType.xyz) * unity_ShadowFadeCenterAndType.w);
  tmpvar_1.w = (-((gl_ModelViewMatrix * _glesVertex).z) * (1.0 - unity_ShadowFadeCenterAndType.w));
  gl_Position = tmpvar_2;
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = o_3;
  xlv_TEXCOORD2 = ((_glesMultiTexCoord1.xy * unity_LightmapST.xy) + unity_LightmapST.zw);
  xlv_TEXCOORD3 = tmpvar_1;
}



#endif
#ifdef FRAGMENT

varying highp vec4 xlv_TEXCOORD3;
varying highp vec2 xlv_TEXCOORD2;
varying highp vec4 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp vec4 unity_LightmapFade;
uniform sampler2D unity_LightmapInd;
uniform sampler2D unity_Lightmap;
uniform sampler2D _LightBuffer;
uniform mediump float _Gloss;
uniform lowp vec4 _Color;
uniform sampler2D _SpecMap;
uniform sampler2D _MainTex;
uniform lowp vec4 _SpecColor;
void main ()
{
  lowp vec4 tmpvar_1;
  mediump vec3 lmIndirect_2;
  mediump vec3 lmFull_3;
  mediump vec4 light_4;
  mediump vec3 tmpvar_5;
  lowp vec4 tmpvar_6;
  tmpvar_6 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_7;
  tmpvar_7 = (tmpvar_6.xyz * _Color.xyz);
  tmpvar_5 = tmpvar_7;
  lowp vec4 tmpvar_8;
  tmpvar_8 = texture2D (_SpecMap, xlv_TEXCOORD0);
  lowp vec4 tmpvar_9;
  tmpvar_9 = texture2DProj (_LightBuffer, xlv_TEXCOORD1);
  light_4 = tmpvar_9;
  mediump vec4 tmpvar_10;
  tmpvar_10 = max (light_4, vec4(0.001, 0.001, 0.001, 0.001));
  light_4.w = tmpvar_10.w;
  lowp vec3 tmpvar_11;
  tmpvar_11 = (2.0 * texture2D (unity_Lightmap, xlv_TEXCOORD2).xyz);
  lmFull_3 = tmpvar_11;
  lowp vec3 tmpvar_12;
  tmpvar_12 = (2.0 * texture2D (unity_LightmapInd, xlv_TEXCOORD2).xyz);
  lmIndirect_2 = tmpvar_12;
  highp float tmpvar_13;
  tmpvar_13 = clamp (((sqrt(dot (xlv_TEXCOORD3, xlv_TEXCOORD3)) * unity_LightmapFade.z) + unity_LightmapFade.w), 0.0, 1.0);
  light_4.xyz = (tmpvar_10.xyz + mix (lmIndirect_2, lmFull_3, vec3(tmpvar_13)));
  mediump vec4 c_14;
  mediump vec3 tmpvar_15;
  tmpvar_15 = (tmpvar_10.w * ((tmpvar_6.w * tmpvar_8.xyz) * _Gloss));
  c_14.xyz = ((tmpvar_5 * light_4.xyz) + (light_4.xyz * tmpvar_15));
  c_14.w = (tmpvar_15 * _SpecColor.w).x;
  tmpvar_1 = c_14;
  gl_FragData[0] = tmpvar_1;
}



#endif"
}

SubProgram "glesdesktop " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
"!!GLES
#define SHADER_API_GLES 1
#define tex2D texture2D


#ifdef VERTEX
#define gl_ModelViewProjectionMatrix glstate_matrix_mvp
uniform mat4 glstate_matrix_mvp;
#define gl_ModelViewMatrix glstate_matrix_modelview0
uniform mat4 glstate_matrix_modelview0;

varying highp vec4 xlv_TEXCOORD3;
varying highp vec2 xlv_TEXCOORD2;
varying highp vec4 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 unity_LightmapST;
uniform highp mat4 _Object2World;


uniform highp vec4 unity_ShadowFadeCenterAndType;
uniform highp vec4 _ProjectionParams;
attribute vec4 _glesMultiTexCoord1;
attribute vec4 _glesMultiTexCoord0;
attribute vec4 _glesVertex;
void main ()
{
  highp vec4 tmpvar_1;
  highp vec4 tmpvar_2;
  tmpvar_2 = (gl_ModelViewProjectionMatrix * _glesVertex);
  highp vec4 o_3;
  highp vec4 tmpvar_4;
  tmpvar_4 = (tmpvar_2 * 0.5);
  highp vec2 tmpvar_5;
  tmpvar_5.x = tmpvar_4.x;
  tmpvar_5.y = (tmpvar_4.y * _ProjectionParams.x);
  o_3.xy = (tmpvar_5 + tmpvar_4.w);
  o_3.zw = tmpvar_2.zw;
  tmpvar_1.xyz = (((_Object2World * _glesVertex).xyz - unity_ShadowFadeCenterAndType.xyz) * unity_ShadowFadeCenterAndType.w);
  tmpvar_1.w = (-((gl_ModelViewMatrix * _glesVertex).z) * (1.0 - unity_ShadowFadeCenterAndType.w));
  gl_Position = tmpvar_2;
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = o_3;
  xlv_TEXCOORD2 = ((_glesMultiTexCoord1.xy * unity_LightmapST.xy) + unity_LightmapST.zw);
  xlv_TEXCOORD3 = tmpvar_1;
}



#endif
#ifdef FRAGMENT

varying highp vec4 xlv_TEXCOORD3;
varying highp vec2 xlv_TEXCOORD2;
varying highp vec4 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp vec4 unity_LightmapFade;
uniform sampler2D unity_LightmapInd;
uniform sampler2D unity_Lightmap;
uniform sampler2D _LightBuffer;
uniform mediump float _Gloss;
uniform lowp vec4 _Color;
uniform sampler2D _SpecMap;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform lowp vec4 _SpecColor;
void main ()
{
  lowp vec4 tmpvar_1;
  mediump vec3 lmIndirect_2;
  mediump vec3 lmFull_3;
  mediump vec4 light_4;
  mediump vec3 tmpvar_5;
  lowp vec4 tmpvar_6;
  tmpvar_6 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_7;
  tmpvar_7 = (tmpvar_6.xyz * _Color.xyz);
  tmpvar_5 = tmpvar_7;
  lowp vec4 tmpvar_8;
  tmpvar_8 = texture2D (_SpecMap, xlv_TEXCOORD0);
  lowp vec3 normal_9;
  normal_9.xy = ((texture2D (_BumpMap, xlv_TEXCOORD0).wy * 2.0) - 1.0);
  normal_9.z = sqrt(((1.0 - (normal_9.x * normal_9.x)) - (normal_9.y * normal_9.y)));
  lowp vec4 tmpvar_10;
  tmpvar_10 = texture2DProj (_LightBuffer, xlv_TEXCOORD1);
  light_4 = tmpvar_10;
  mediump vec4 tmpvar_11;
  tmpvar_11 = max (light_4, vec4(0.001, 0.001, 0.001, 0.001));
  light_4.w = tmpvar_11.w;
  lowp vec4 tmpvar_12;
  tmpvar_12 = texture2D (unity_Lightmap, xlv_TEXCOORD2);
  lowp vec3 tmpvar_13;
  tmpvar_13 = ((8.0 * tmpvar_12.w) * tmpvar_12.xyz);
  lmFull_3 = tmpvar_13;
  lowp vec4 tmpvar_14;
  tmpvar_14 = texture2D (unity_LightmapInd, xlv_TEXCOORD2);
  lowp vec3 tmpvar_15;
  tmpvar_15 = ((8.0 * tmpvar_14.w) * tmpvar_14.xyz);
  lmIndirect_2 = tmpvar_15;
  highp float tmpvar_16;
  tmpvar_16 = clamp (((sqrt(dot (xlv_TEXCOORD3, xlv_TEXCOORD3)) * unity_LightmapFade.z) + unity_LightmapFade.w), 0.0, 1.0);
  light_4.xyz = (tmpvar_11.xyz + mix (lmIndirect_2, lmFull_3, vec3(tmpvar_16)));
  mediump vec4 c_17;
  mediump vec3 tmpvar_18;
  tmpvar_18 = (tmpvar_11.w * ((tmpvar_6.w * tmpvar_8.xyz) * _Gloss));
  c_17.xyz = ((tmpvar_5 * light_4.xyz) + (light_4.xyz * tmpvar_18));
  c_17.w = (tmpvar_18 * _SpecColor.w).x;
  tmpvar_1 = c_17;
  gl_FragData[0] = tmpvar_1;
}



#endif"
}

SubProgram "d3d11_9x " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
Bind "vertex" Vertex
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Bind "color" Color
ConstBuffer "$Globals" 144 // 112 used size, 10 vars
Vector 80 [unity_LightmapST] 4
Vector 96 [_MainTex_ST] 4
ConstBuffer "UnityPerCamera" 128 // 96 used size, 8 vars
Vector 80 [_ProjectionParams] 4
ConstBuffer "UnityShadows" 416 // 416 used size, 8 vars
Vector 400 [unity_ShadowFadeCenterAndType] 4
ConstBuffer "UnityPerDraw" 336 // 256 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
Matrix 64 [glstate_matrix_modelview0] 4
Matrix 192 [_Object2World] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityShadows" 2
BindCB "UnityPerDraw" 3
// 24 instructions, 2 temp regs, 0 temp arrays:
// ALU 10 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0_level_9_3
eefiecedjopiaimhnihcggeiloincidnceocjfibabaaaaaaaaaiaaaaaeaaaaaa
daaaaaaakeacaaaajiagaaaagaahaaaaebgpgodjgmacaaaagmacaaaaaaacpopp
aiacaaaageaaaaaaafaaceaaaaaagaaaaaaagaaaaaaaceaaabaagaaaaaaaafaa
acaaabaaaaaaaaaaabaaafaaabaaadaaaaaaaaaaacaabjaaabaaaeaaaaaaaaaa
adaaaaaaaiaaafaaaaaaaaaaadaaamaaaeaaanaaaaaaaaaaaaaaaaaaabacpopp
fbaaaaafbbaaapkaaaaaaadpaaaaiadpaaaaaaaaaaaaaaaabpaaaaacafaaaaia
aaaaapjabpaaaaacafaaadiaadaaapjabpaaaaacafaaaeiaaeaaapjaaeaaaaae
aaaaadoaadaaoejaacaaoekaacaaookaafaaaaadaaaaapiaaaaaffjaagaaoeka
aeaaaaaeaaaaapiaafaaoekaaaaaaajaaaaaoeiaaeaaaaaeaaaaapiaahaaoeka
aaaakkjaaaaaoeiaaeaaaaaeaaaaapiaaiaaoekaaaaappjaaaaaoeiaafaaaaad
abaaabiaaaaaffiaadaaaakaafaaaaadabaaaiiaabaaaaiabbaaaakaafaaaaad
abaaafiaaaaapeiabbaaaakaacaaaaadabaaadoaabaakkiaabaaomiaaeaaaaae
aaaaamoaaeaabejaabaabekaabaalekaafaaaaadabaaahiaaaaaffjaaoaaoeka
aeaaaaaeabaaahiaanaaoekaaaaaaajaabaaoeiaaeaaaaaeabaaahiaapaaoeka
aaaakkjaabaaoeiaaeaaaaaeabaaahiabaaaoekaaaaappjaabaaoeiaacaaaaad
abaaahiaabaaoeiaaeaaoekbafaaaaadacaaahoaabaaoeiaaeaappkaafaaaaad
abaaabiaaaaaffjaakaakkkaaeaaaaaeabaaabiaajaakkkaaaaaaajaabaaaaia
aeaaaaaeabaaabiaalaakkkaaaaakkjaabaaaaiaaeaaaaaeabaaabiaamaakkka
aaaappjaabaaaaiaabaaaaacabaaaiiaaeaappkaacaaaaadabaaaciaabaappib
bbaaffkaafaaaaadacaaaioaabaaffiaabaaaaibaeaaaaaeaaaaadmaaaaappia
aaaaoekaaaaaoeiaabaaaaacaaaaammaaaaaoeiaabaaaaacabaaamoaaaaaoeia
ppppaaaafdeieefcomadaaaaeaaaabaaplaaaaaafjaaaaaeegiocaaaaaaaaaaa
ahaaaaaafjaaaaaeegiocaaaabaaaaaaagaaaaaafjaaaaaeegiocaaaacaaaaaa
bkaaaaaafjaaaaaeegiocaaaadaaaaaabaaaaaaafpaaaaadpcbabaaaaaaaaaaa
fpaaaaaddcbabaaaadaaaaaafpaaaaaddcbabaaaaeaaaaaaghaaaaaepccabaaa
aaaaaaaaabaaaaaagfaaaaaddccabaaaabaaaaaagfaaaaadmccabaaaabaaaaaa
gfaaaaadpccabaaaacaaaaaagfaaaaadpccabaaaadaaaaaagiaaaaacacaaaaaa
diaaaaaipcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaadaaaaaaabaaaaaa
dcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaaaaaaaaaagbabaaaaaaaaaaa
egaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaacaaaaaa
kgbkbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaa
adaaaaaaadaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadgaaaaafpccabaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaaldccabaaaabaaaaaaegbabaaaadaaaaaa
egiacaaaaaaaaaaaagaaaaaaogikcaaaaaaaaaaaagaaaaaadcaaaaalmccabaaa
abaaaaaaagbebaaaaeaaaaaaagiecaaaaaaaaaaaafaaaaaakgiocaaaaaaaaaaa
afaaaaaadiaaaaaiccaabaaaaaaaaaaabkaabaaaaaaaaaaaakiacaaaabaaaaaa
afaaaaaadiaaaaakncaabaaaabaaaaaaagahbaaaaaaaaaaaaceaaaaaaaaaaadp
aaaaaaaaaaaaaadpaaaaaadpdgaaaaafmccabaaaacaaaaaakgaobaaaaaaaaaaa
aaaaaaahdccabaaaacaaaaaakgakbaaaabaaaaaamgaabaaaabaaaaaadiaaaaai
hcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiccaaaadaaaaaaanaaaaaadcaaaaak
hcaabaaaaaaaaaaaegiccaaaadaaaaaaamaaaaaaagbabaaaaaaaaaaaegacbaaa
aaaaaaaadcaaaaakhcaabaaaaaaaaaaaegiccaaaadaaaaaaaoaaaaaakgbkbaaa
aaaaaaaaegacbaaaaaaaaaaadcaaaaakhcaabaaaaaaaaaaaegiccaaaadaaaaaa
apaaaaaapgbpbaaaaaaaaaaaegacbaaaaaaaaaaaaaaaaaajhcaabaaaaaaaaaaa
egacbaaaaaaaaaaaegiccaiaebaaaaaaacaaaaaabjaaaaaadiaaaaaihccabaaa
adaaaaaaegacbaaaaaaaaaaapgipcaaaacaaaaaabjaaaaaadiaaaaaibcaabaaa
aaaaaaaabkbabaaaaaaaaaaackiacaaaadaaaaaaafaaaaaadcaaaaakbcaabaaa
aaaaaaaackiacaaaadaaaaaaaeaaaaaaakbabaaaaaaaaaaaakaabaaaaaaaaaaa
dcaaaaakbcaabaaaaaaaaaaackiacaaaadaaaaaaagaaaaaackbabaaaaaaaaaaa
akaabaaaaaaaaaaadcaaaaakbcaabaaaaaaaaaaackiacaaaadaaaaaaahaaaaaa
dkbabaaaaaaaaaaaakaabaaaaaaaaaaaaaaaaaajccaabaaaaaaaaaaadkiacaia
ebaaaaaaacaaaaaabjaaaaaaabeaaaaaaaaaiadpdiaaaaaiiccabaaaadaaaaaa
bkaabaaaaaaaaaaaakaabaiaebaaaaaaaaaaaaaadoaaaaabejfdeheomaaaaaaa
agaaaaaaaiaaaaaajiaaaaaaaaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaa
kbaaaaaaaaaaaaaaaaaaaaaaadaaaaaaabaaaaaaapaaaaaakjaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaacaaaaaaahaaaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaa
adaaaaaaapadaaaalaaaaaaaabaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapadaaaa
ljaaaaaaaaaaaaaaaaaaaaaaadaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeo
aafeebeoehefeofeaaeoepfcenebemaafeeffiedepepfceeaaedepemepfcaakl
epfdeheojiaaaaaaafaaaaaaaiaaaaaaiaaaaaaaaaaaaaaaabaaaaaaadaaaaaa
aaaaaaaaapaaaaaaimaaaaaaaaaaaaaaaaaaaaaaadaaaaaaabaaaaaaadamaaaa
imaaaaaaacaaaaaaaaaaaaaaadaaaaaaabaaaaaaamadaaaaimaaaaaaabaaaaaa
aaaaaaaaadaaaaaaacaaaaaaapaaaaaaimaaaaaaadaaaaaaaaaaaaaaadaaaaaa
adaaaaaaapaaaaaafdfgfpfaepfdejfeejepeoaafeeffiedepepfceeaaklklkl
"
}

}
Program "fp" {
// Fragment combos: 4
//   opengl - ALU: 11 to 26, TEX: 3 to 5
//   d3d9 - ALU: 9 to 22, TEX: 3 to 5
//   d3d11 - ALU: 8 to 14, TEX: 3 to 5, FLOW: 1 to 1
//   d3d11_9x - ALU: 8 to 14, TEX: 3 to 5, FLOW: 1 to 1
SubProgram "opengl " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
Vector 0 [_SpecColor]
Vector 1 [_Color]
Float 2 [_Gloss]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 3 [_LightBuffer] 2D
"!!ARBfp1.0
OPTION ARB_precision_hint_fastest;
# 15 ALU, 3 TEX
PARAM c[3] = { program.local[0..2] };
TEMP R0;
TEMP R1;
TEMP R2;
TXP R0, fragment.texcoord[1], texture[3], 2D;
TEX R1, fragment.texcoord[0], texture[0], 2D;
TEX R2.xyz, fragment.texcoord[0], texture[1], 2D;
MUL R2.xyz, R1.w, R2;
LG2 R0.x, R0.x;
LG2 R0.z, R0.z;
LG2 R0.y, R0.y;
ADD R0.xyz, -R0, fragment.texcoord[2];
MUL R2.xyz, R2, c[2].x;
LG2 R0.w, R0.w;
MUL R2.xyz, -R0.w, R2;
MUL R2.yzw, R2.xxyz, R0.xxyz;
MUL R1.xyz, R1, c[1];
MAD result.color.xyz, R1, R0, R2.yzww;
MUL result.color.w, R2.x, c[0];
END
# 15 instructions, 3 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
Vector 0 [_SpecColor]
Vector 1 [_Color]
Float 2 [_Gloss]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 3 [_LightBuffer] 2D
"ps_2_0
; 13 ALU, 3 TEX
dcl_2d s0
dcl_2d s1
dcl_2d s3
dcl t0.xy
dcl t1
dcl t2.xyz
texld r2, t0, s0
texldp r0, t1, s3
texld r1, t0, s1
mul_pp r1.xyz, r2.w, r1
log_pp r3.x, r0.w
mul_pp r1.xyz, r1, c2.x
mul_pp r1.xyz, -r3.x, r1
log_pp r0.x, r0.x
log_pp r0.z, r0.z
log_pp r0.y, r0.y
add_pp r0.xyz, -r0, t2
mul_pp r3.xyz, r1, r0
mul_pp r2.xyz, r2, c1
mad_pp r0.xyz, r2, r0, r3
mul_pp r0.w, r1.x, c0
mov_pp oC0, r0
"
}

SubProgram "xbox360 " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
Vector 1 [_Color]
Float 2 [_Gloss]
Vector 0 [_SpecColor]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_LightBuffer] 2D
// Shader Timing Estimate, in Cycles/64 pixel vector:
// ALU: 13.33 (10 instructions), vertex: 0, texture: 12,
//   sequencer: 8, interpolator: 12;    5 GPRs, 36 threads,
// Performance (if enough threads): ~13 cycles per vector
// * Texture cycle estimates are assuming an 8bit/component texture with no
//     aniso or trilinear filtering.

"ps_360
backbbaaaaaaabgiaaaaaamaaaaaaaaaaaaaaaceaaaaaaaaaaaaabdmaaaaaaaa
aaaaaaaaaaaaabbeaaaaaabmaaaaabaippppadaaaaaaaaagaaaaaabmaaaaaaaa
aaaaababaaaaaajeaaacaaabaaabaaaaaaaaaajmaaaaaaaaaaaaaakmaaacaaac
aaabaaaaaaaaaaleaaaaaaaaaaaaaameaaadaaacaaabaaaaaaaaaaneaaaaaaaa
aaaaaaoeaaadaaaaaaabaaaaaaaaaaneaaaaaaaaaaaaaaonaaacaaaaaaabaaaa
aaaaaajmaaaaaaaaaaaaaapiaaadaaabaaabaaaaaaaaaaneaaaaaaaafpedgpgm
gphcaaklaaabaaadaaabaaaeaaabaaaaaaaaaaaafpehgmgphdhdaaklaaaaaaad
aaabaaabaaabaaaaaaaaaaaafpemgjghgiheechfgggggfhcaaklklklaaaeaaam
aaabaaabaaabaaaaaaaaaaaafpengbgjgofegfhiaafpfdhagfgdedgpgmgphcaa
fpfdhagfgdengbhaaahahdfpddfpdaaadccodacodcdadddfddcodaaaaaaaaaaa
aaaaaamabaaaaeaaaaaaaaaeaaaaaaaaaaaacegdaaahaaahaaaaaaabaaaadafa
aaaapbfbaaaahcfcabfafaacaaaabcaameaaaaaaaaaagaahcaanbcaaccaaaaaa
emeaaaaaaaaaaablocaaaaabmiamaaaaaamgkmaaobaaabaababieaabbpbppeeh
aaaaeaaalicibaabbpbppgiiaaaaeaaabaaiaaabbpbppgiiaaaaeaaaeabhaead
aamamagmkbaaabibeachaeaaaablbflbobaaaeibeaeoaeaaaapmgmmgkbaaacib
eabhaaabaemamabloaaeacibmiahaaaaacbfgmaaobaaaaaakiiaiaaaaaaaaaaa
mcaaaaaamiahaaaaaamamaaaobaaabaamiahiaaaaamamamaoladabaaaaaaaaaa
aaaaaaaaaaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
Vector 0 [_SpecColor]
Vector 1 [_Color]
Float 2 [_Gloss]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 3 [_LightBuffer] 2D
"sce_fp_rsx // 18 instructions using 3 registers
[Configuration]
24
ffffffff0001c0200007fff9000000000000840003000000
[Offsets]
3
_SpecColor 1 0
00000100
_Color 1 0
00000050
_Gloss 1 0
00000070
[Microcode]
288
be001806c8011c9dc8000001c8003fe102801d40c8001c9dc8000001c8000001
9e021700c8011c9dc8000001c8003fe104801d40aa001c9cc8000001c8000001
0e840240c8041c9dc8020001c800000100000000000000000000000000000000
08860240fe041c9d00020000c800000100000000000000000000000000000000
08801d4054001c9dc8000001c80000018e041702c8011c9dc8000001c8003fe1
10801d40fe001c9dc8000001c80000010e820240550c1c9dc8080001c8000001
ce800300c8011c9dc9000003c8003fe10e820240ff001c9fc9040001c8000001
0e860240c9041c9dc9000001c80000011080024001041c9cc8020001c8000001
000000000000000000000000000000000e810440c9081c9dc9000001c90c0001
"
}

SubProgram "d3d11 " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
ConstBuffer "$Globals" 112 // 72 used size, 8 vars
Vector 32 [_SpecColor] 4
Vector 48 [_Color] 4
Float 68 [_Gloss]
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 0
SetTexture 1 [_SpecMap] 2D 1
SetTexture 2 [_LightBuffer] 2D 2
// 14 instructions, 3 temp regs, 0 temp arrays:
// ALU 9 float, 0 int, 0 uint
// TEX 3 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0
eefiecedcikkcdfmcainmnkeapaocodaplbnlbpgabaaaaaadaadaaaaadaaaaaa
cmaaaaaaleaaaaaaoiaaaaaaejfdeheoiaaaaaaaaeaaaaaaaiaaaaaagiaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaheaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaadadaaaaheaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaa
apalaaaaheaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaahahaaaafdfgfpfa
epfdejfeejepeoaafeeffiedepepfceeaaklklklepfdeheocmaaaaaaabaaaaaa
aiaaaaaacaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaafdfgfpfe
gbhcghgfheaaklklfdeieefceaacaaaaeaaaaaaajaaaaaaafjaaaaaeegiocaaa
aaaaaaaaafaaaaaafkaaaaadaagabaaaaaaaaaaafkaaaaadaagabaaaabaaaaaa
fkaaaaadaagabaaaacaaaaaafibiaaaeaahabaaaaaaaaaaaffffaaaafibiaaae
aahabaaaabaaaaaaffffaaaafibiaaaeaahabaaaacaaaaaaffffaaaagcbaaaad
dcbabaaaabaaaaaagcbaaaadlcbabaaaacaaaaaagcbaaaadhcbabaaaadaaaaaa
gfaaaaadpccabaaaaaaaaaaagiaaaaacadaaaaaaefaaaaajpcaabaaaaaaaaaaa
egbabaaaabaaaaaaeghobaaaabaaaaaaaagabaaaabaaaaaaefaaaaajpcaabaaa
abaaaaaaegbabaaaabaaaaaaeghobaaaaaaaaaaaaagabaaaaaaaaaaadiaaaaah
hcaabaaaaaaaaaaaegacbaaaaaaaaaaapgapbaaaabaaaaaadiaaaaaihcaabaaa
abaaaaaaegacbaaaabaaaaaaegiccaaaaaaaaaaaadaaaaaadiaaaaaihcaabaaa
aaaaaaaaegacbaaaaaaaaaaafgifcaaaaaaaaaaaaeaaaaaaaoaaaaahdcaabaaa
acaaaaaaegbabaaaacaaaaaapgbpbaaaacaaaaaaefaaaaajpcaabaaaacaaaaaa
egaabaaaacaaaaaaeghobaaaacaaaaaaaagabaaaacaaaaaacpaaaaafpcaabaaa
acaaaaaaegaobaaaacaaaaaadiaaaaaihcaabaaaaaaaaaaaegacbaaaaaaaaaaa
pgapbaiaebaaaaaaacaaaaaaaaaaaaaihcaabaaaacaaaaaaegacbaiaebaaaaaa
acaaaaaaegbcbaaaadaaaaaadiaaaaahocaabaaaaaaaaaaaagajbaaaaaaaaaaa
agajbaaaacaaaaaadiaaaaaiiccabaaaaaaaaaaaakaabaaaaaaaaaaadkiacaaa
aaaaaaaaacaaaaaadcaaaaajhccabaaaaaaaaaaaegacbaaaabaaaaaaegacbaaa
acaaaaaajgahbaaaaaaaaaaadoaaaaab"
}

SubProgram "gles " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
"!!GLES"
}

SubProgram "glesdesktop " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
"!!GLES"
}

SubProgram "d3d11_9x " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
ConstBuffer "$Globals" 112 // 72 used size, 8 vars
Vector 32 [_SpecColor] 4
Vector 48 [_Color] 4
Float 68 [_Gloss]
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 0
SetTexture 1 [_SpecMap] 2D 1
SetTexture 2 [_LightBuffer] 2D 2
// 14 instructions, 3 temp regs, 0 temp arrays:
// ALU 9 float, 0 int, 0 uint
// TEX 3 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0_level_9_3
eefiecedfjbmhlekbcjjncpbhhjndklbmdognlaiabaaaaaaneaeaaaaaeaaaaaa
daaaaaaanaabaaaabiaeaaaakaaeaaaaebgpgodjjiabaaaajiabaaaaaaacpppp
fmabaaaadmaaaaaaabaadaaaaaaadmaaaaaadmaaadaaceaaaaaadmaaaaaaaaaa
abababaaacacacaaaaaaacaaadaaaaaaaaaaaaaaabacppppbpaaaaacaaaaaaia
aaaaadlabpaaaaacaaaaaaiaabaaaplabpaaaaacaaaaaaiaacaaahlabpaaaaac
aaaaaajaaaaiapkabpaaaaacaaaaaajaabaiapkabpaaaaacaaaaaajaacaiapka
agaaaaacaaaaaiiaabaapplaafaaaaadaaaaadiaaaaappiaabaaoelaecaaaaad
aaaacpiaaaaaoeiaacaioekaapaaaaacabaacbiaaaaaaaiaapaaaaacabaaccia
aaaaffiaapaaaaacabaaceiaaaaakkiaapaaaaacabaaciiaaaaappiaacaaaaad
aaaachiaabaaoeibacaaoelaecaaaaadacaacpiaaaaaoelaaaaioekaecaaaaad
adaacpiaaaaaoelaabaioekaafaaaaadabaachiaacaappiaadaaoeiaafaaaaad
acaachiaacaaoeiaabaaoekaafaaaaadabaachiaabaaoeiaacaaffkaafaaaaad
abaachiaabaaoeiaabaappibafaaaaadabaacoiaaaaajaiaabaajaiaaeaaaaae
aaaachiaacaaoeiaaaaaoeiaabaapjiaafaaaaadaaaaciiaabaaaaiaaaaappka
abaaaaacaaaicpiaaaaaoeiappppaaaafdeieefceaacaaaaeaaaaaaajaaaaaaa
fjaaaaaeegiocaaaaaaaaaaaafaaaaaafkaaaaadaagabaaaaaaaaaaafkaaaaad
aagabaaaabaaaaaafkaaaaadaagabaaaacaaaaaafibiaaaeaahabaaaaaaaaaaa
ffffaaaafibiaaaeaahabaaaabaaaaaaffffaaaafibiaaaeaahabaaaacaaaaaa
ffffaaaagcbaaaaddcbabaaaabaaaaaagcbaaaadlcbabaaaacaaaaaagcbaaaad
hcbabaaaadaaaaaagfaaaaadpccabaaaaaaaaaaagiaaaaacadaaaaaaefaaaaaj
pcaabaaaaaaaaaaaegbabaaaabaaaaaaeghobaaaabaaaaaaaagabaaaabaaaaaa
efaaaaajpcaabaaaabaaaaaaegbabaaaabaaaaaaeghobaaaaaaaaaaaaagabaaa
aaaaaaaadiaaaaahhcaabaaaaaaaaaaaegacbaaaaaaaaaaapgapbaaaabaaaaaa
diaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaaegiccaaaaaaaaaaaadaaaaaa
diaaaaaihcaabaaaaaaaaaaaegacbaaaaaaaaaaafgifcaaaaaaaaaaaaeaaaaaa
aoaaaaahdcaabaaaacaaaaaaegbabaaaacaaaaaapgbpbaaaacaaaaaaefaaaaaj
pcaabaaaacaaaaaaegaabaaaacaaaaaaeghobaaaacaaaaaaaagabaaaacaaaaaa
cpaaaaafpcaabaaaacaaaaaaegaobaaaacaaaaaadiaaaaaihcaabaaaaaaaaaaa
egacbaaaaaaaaaaapgapbaiaebaaaaaaacaaaaaaaaaaaaaihcaabaaaacaaaaaa
egacbaiaebaaaaaaacaaaaaaegbcbaaaadaaaaaadiaaaaahocaabaaaaaaaaaaa
agajbaaaaaaaaaaaagajbaaaacaaaaaadiaaaaaiiccabaaaaaaaaaaaakaabaaa
aaaaaaaadkiacaaaaaaaaaaaacaaaaaadcaaaaajhccabaaaaaaaaaaaegacbaaa
abaaaaaaegacbaaaacaaaaaajgahbaaaaaaaaaaadoaaaaabejfdeheoiaaaaaaa
aeaaaaaaaiaaaaaagiaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaa
heaaaaaaaaaaaaaaaaaaaaaaadaaaaaaabaaaaaaadadaaaaheaaaaaaabaaaaaa
aaaaaaaaadaaaaaaacaaaaaaapalaaaaheaaaaaaacaaaaaaaaaaaaaaadaaaaaa
adaaaaaaahahaaaafdfgfpfaepfdejfeejepeoaafeeffiedepepfceeaaklklkl
epfdeheocmaaaaaaabaaaaaaaiaaaaaacaaaaaaaaaaaaaaaaaaaaaaaadaaaaaa
aaaaaaaaapaaaaaafdfgfpfegbhcghgfheaaklkl"
}

SubProgram "opengl " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
Vector 0 [_SpecColor]
Vector 1 [_Color]
Float 2 [_Gloss]
Vector 3 [unity_LightmapFade]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 3 [_LightBuffer] 2D
SetTexture 4 [unity_Lightmap] 2D
SetTexture 5 [unity_LightmapInd] 2D
"!!ARBfp1.0
OPTION ARB_precision_hint_fastest;
# 26 ALU, 5 TEX
PARAM c[5] = { program.local[0..3],
		{ 8 } };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
TEMP R4;
TXP R0, fragment.texcoord[1], texture[3], 2D;
TEX R3, fragment.texcoord[0], texture[0], 2D;
TEX R1, fragment.texcoord[2], texture[5], 2D;
TEX R2, fragment.texcoord[2], texture[4], 2D;
TEX R4.xyz, fragment.texcoord[0], texture[1], 2D;
MUL R2.xyz, R2.w, R2;
MUL R1.xyz, R1.w, R1;
MUL R1.xyz, R1, c[4].x;
DP4 R2.w, fragment.texcoord[3], fragment.texcoord[3];
RSQ R1.w, R2.w;
RCP R1.w, R1.w;
MAD R2.xyz, R2, c[4].x, -R1;
MAD_SAT R1.w, R1, c[3].z, c[3];
MAD R1.xyz, R1.w, R2, R1;
MUL R2.xyz, R3.w, R4;
LG2 R0.x, R0.x;
LG2 R0.y, R0.y;
LG2 R0.z, R0.z;
ADD R0.xyz, -R0, R1;
MUL R1.xyz, R2, c[2].x;
LG2 R0.w, R0.w;
MUL R1.xyz, -R0.w, R1;
MUL R2.xyz, R1, R0;
MUL R3.xyz, R3, c[1];
MAD result.color.xyz, R3, R0, R2;
MUL result.color.w, R1.x, c[0];
END
# 26 instructions, 5 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
Vector 0 [_SpecColor]
Vector 1 [_Color]
Float 2 [_Gloss]
Vector 3 [unity_LightmapFade]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 3 [_LightBuffer] 2D
SetTexture 4 [unity_Lightmap] 2D
SetTexture 5 [unity_LightmapInd] 2D
"ps_2_0
; 22 ALU, 5 TEX
dcl_2d s0
dcl_2d s1
dcl_2d s3
dcl_2d s4
dcl_2d s5
def c4, 8.00000000, 0, 0, 0
dcl t0.xy
dcl t1
dcl t2.xy
dcl t3
texld r2, t0, s0
texld r3, t0, s1
texldp r1, t1, s3
texld r0, t2, s4
texld r4, t2, s5
mul_pp r5.xyz, r0.w, r0
dp4 r0.x, t3, t3
mul_pp r4.xyz, r4.w, r4
mul_pp r4.xyz, r4, c4.x
rsq r0.x, r0.x
rcp r0.x, r0.x
mul_pp r3.xyz, r2.w, r3
mul_pp r3.xyz, r3, c2.x
mad_pp r5.xyz, r5, c4.x, -r4
mad_sat r0.x, r0, c3.z, c3.w
mad_pp r0.xyz, r0.x, r5, r4
log_pp r1.x, r1.x
log_pp r1.y, r1.y
log_pp r1.z, r1.z
add_pp r1.xyz, -r1, r0
log_pp r0.x, r1.w
mul_pp r0.xyz, -r0.x, r3
mul_pp r3.xyz, r0, r1
mul_pp r2.xyz, r2, c1
mad_pp r1.xyz, r2, r1, r3
mul_pp r1.w, r0.x, c0
mov_pp oC0, r1
"
}

SubProgram "xbox360 " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
Vector 1 [_Color]
Float 2 [_Gloss]
Vector 0 [_SpecColor]
Vector 3 [unity_LightmapFade]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_LightBuffer] 2D
SetTexture 3 [unity_Lightmap] 2D
SetTexture 4 [unity_LightmapInd] 2D
// Shader Timing Estimate, in Cycles/64 pixel vector:
// ALU: 21.33 (16 instructions), vertex: 0, texture: 20,
//   sequencer: 12, interpolator: 16;    8 GPRs, 24 threads,
// Performance (if enough threads): ~21 cycles per vector
// * Texture cycle estimates are assuming an 8bit/component texture with no
//     aniso or trilinear filtering.

"ps_360
backbbaaaaaaacaeaaaaabgmaaaaaaaaaaaaaaceaaaaabkmaaaaabneaaaaaaaa
aaaaaaaaaaaaabieaaaaaabmaaaaabhippppadaaaaaaaaajaaaaaabmaaaaaaaa
aaaaabhbaaaaaanaaaacaaabaaabaaaaaaaaaaniaaaaaaaaaaaaaaoiaaacaaac
aaabaaaaaaaaaapaaaaaaaaaaaaaabaaaaadaaacaaabaaaaaaaaabbaaaaaaaaa
aaaaabcaaaadaaaaaaabaaaaaaaaabbaaaaaaaaaaaaaabcjaaacaaaaaaabaaaa
aaaaaaniaaaaaaaaaaaaabdeaaadaaabaaabaaaaaaaaabbaaaaaaaaaaaaaabdn
aaadaaadaaabaaaaaaaaabbaaaaaaaaaaaaaabemaaacaaadaaabaaaaaaaaaani
aaaaaaaaaaaaabfpaaadaaaeaaabaaaaaaaaabbaaaaaaaaafpedgpgmgphcaakl
aaabaaadaaabaaaeaaabaaaaaaaaaaaafpehgmgphdhdaaklaaaaaaadaaabaaab
aaabaaaaaaaaaaaafpemgjghgiheechfgggggfhcaaklklklaaaeaaamaaabaaab
aaabaaaaaaaaaaaafpengbgjgofegfhiaafpfdhagfgdedgpgmgphcaafpfdhagf
gdengbhaaahfgogjhehjfpemgjghgihegngbhaaahfgogjhehjfpemgjghgihegn
gbhaeggbgegfaahfgogjhehjfpemgjghgihegngbhaejgogeaahahdfpddfpdaaa
dccodacodcdadddfddcodaaaaaaaaaaaaaaaaaabaaaaaaaaaaaaaaaaaaaaaabe
abpmaabaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaeaaaaaabcmbaaaahaa
aaaaaaaeaaaaaaaaaaaadaieaaapaaapaaaaaaabaaaadafaaaaapbfbaaaadcfc
aaaapdfdaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaebaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaffagaadbaajbcaabcaaaaabaaaaaaaagaakmeaabcaaaaaaaaaagaba
cabgbcaaccaaaaaaemeaaaaaaaaaaablocaaaaabmiamaaaaaamgkmaaobaaabaa
babifaabbpbppoiiaaaaeaaalicieaabbpbppemiaaaaeaaabaeihaebbpbppgii
aaaaeaaabadigaebbpbppgiiaaaaeaaabaaiaaabbpbppgecaaaaeaaabeceaaab
aakhkhlbopadadaakibcadabaablgmebibagppabeabbacabaablgmgmkbahppie
kaeoabacaablpmmgobaaafibmjaiaaabaamgmgblilabadadeachacafaabfgmlb
kbacacieeaboabadaagmpmmgobabahiemiahaaaeablbmabfolabagadeaehacab
acmagmblobafabiekmiaiaaaaaaaaaaamcaaaaaamiaoaaadaapmblabolaeabad
kichadacacbfmaicmaadacabkiehadabaamamamambabacabmiahiaaaaamamama
oladacabaaaaaaaaaaaaaaaaaaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
Vector 0 [_SpecColor]
Vector 1 [_Color]
Float 2 [_Gloss]
Vector 3 [unity_LightmapFade]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 3 [_LightBuffer] 2D
SetTexture 4 [unity_Lightmap] 2D
SetTexture 5 [unity_LightmapInd] 2D
"sce_fp_rsx // 31 instructions using 4 registers
[Configuration]
24
ffffffff0003c020000ffff5000000000000840004000000
[Offsets]
4
_SpecColor 1 0
000001d0
_Color 1 0
00000130
_Gloss 1 0
00000170
unity_LightmapFade 2 0
000000d0000000b0
[Microcode]
496
de021708c8011c9dc8000001c8003fe1108c0140c8041c9dc8003001c8000001
fe040100c8011c9dc8000001c8003fe110020600c8081c9dc8080001c8000001
de00170ac8011c9dc8000001c8003fe10e8c0240fe001c9dc8003001c8000001
be041806c8011c9dc8000001c8003fe1088e1d4054081c9dc8000001c8000001
02001b00fe041c9dc8000001c8000001028e1d40c8081c9dc8000001c8000001
10023a0054021c9dc8000001c800000100000000000000000000000000000000
10028300c8041c9dc8020001c800000100000000000000000000000000000000
0e020400ff181c9dc8040001c91800030e8c0400fe041c9dc8040001c9180001
028a1d40fe081c9dc8000001c80000019e001700c8011c9dc8000001c8003fe1
0e800240c8001c9dc8020001c800000100000000000000000000000000000000
048e1d40aa081c9cc8000001c80000010e880340c91c1c9fc9180001c8000001
10860240c8001c9d00020000c800000100000000000000000000000000000000
8e021702c8011c9dc8000001c8003fe10e820240ff0c1c9dc8040001c8000001
0e82024001141c9ec9040001c80000010e840240c9041c9dc9100001c8000001
1080024001041c9cc8020001c800000100000000000000000000000000000000
0e810440c9001c9dc9100001c9080001
"
}

SubProgram "d3d11 " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
ConstBuffer "$Globals" 144 // 128 used size, 10 vars
Vector 32 [_SpecColor] 4
Vector 48 [_Color] 4
Float 68 [_Gloss]
Vector 112 [unity_LightmapFade] 4
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 0
SetTexture 1 [_SpecMap] 2D 1
SetTexture 2 [_LightBuffer] 2D 2
SetTexture 3 [unity_Lightmap] 2D 3
SetTexture 4 [unity_LightmapInd] 2D 4
// 24 instructions, 4 temp regs, 0 temp arrays:
// ALU 14 float, 0 int, 0 uint
// TEX 5 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0
eefiecedpdfiaghleilnlennnhlocfljdhlofdnoabaaaaaanaaeaaaaadaaaaaa
cmaaaaaammaaaaaaaaabaaaaejfdeheojiaaaaaaafaaaaaaaiaaaaaaiaaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaadadaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaabaaaaaa
amamaaaaimaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaaapalaaaaimaaaaaa
adaaaaaaaaaaaaaaadaaaaaaadaaaaaaapapaaaafdfgfpfaepfdejfeejepeoaa
feeffiedepepfceeaaklklklepfdeheocmaaaaaaabaaaaaaaiaaaaaacaaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaafdfgfpfegbhcghgfheaaklkl
fdeieefcmiadaaaaeaaaaaaapcaaaaaafjaaaaaeegiocaaaaaaaaaaaaiaaaaaa
fkaaaaadaagabaaaaaaaaaaafkaaaaadaagabaaaabaaaaaafkaaaaadaagabaaa
acaaaaaafkaaaaadaagabaaaadaaaaaafkaaaaadaagabaaaaeaaaaaafibiaaae
aahabaaaaaaaaaaaffffaaaafibiaaaeaahabaaaabaaaaaaffffaaaafibiaaae
aahabaaaacaaaaaaffffaaaafibiaaaeaahabaaaadaaaaaaffffaaaafibiaaae
aahabaaaaeaaaaaaffffaaaagcbaaaaddcbabaaaabaaaaaagcbaaaadmcbabaaa
abaaaaaagcbaaaadlcbabaaaacaaaaaagcbaaaadpcbabaaaadaaaaaagfaaaaad
pccabaaaaaaaaaaagiaaaaacaeaaaaaabbaaaaahbcaabaaaaaaaaaaaegbobaaa
adaaaaaaegbobaaaadaaaaaaelaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaa
dccaaaalbcaabaaaaaaaaaaaakaabaaaaaaaaaaackiacaaaaaaaaaaaahaaaaaa
dkiacaaaaaaaaaaaahaaaaaaefaaaaajpcaabaaaabaaaaaaogbkbaaaabaaaaaa
eghobaaaaeaaaaaaaagabaaaaeaaaaaadiaaaaahccaabaaaaaaaaaaadkaabaaa
abaaaaaaabeaaaaaaaaaaaebdiaaaaahocaabaaaaaaaaaaaagajbaaaabaaaaaa
fgafbaaaaaaaaaaaefaaaaajpcaabaaaabaaaaaaogbkbaaaabaaaaaaeghobaaa
adaaaaaaaagabaaaadaaaaaadiaaaaahicaabaaaabaaaaaadkaabaaaabaaaaaa
abeaaaaaaaaaaaebdcaaaaakhcaabaaaabaaaaaapgapbaaaabaaaaaaegacbaaa
abaaaaaajgahbaiaebaaaaaaaaaaaaaadcaaaaajhcaabaaaaaaaaaaaagaabaaa
aaaaaaaaegacbaaaabaaaaaajgahbaaaaaaaaaaaaoaaaaahdcaabaaaabaaaaaa
egbabaaaacaaaaaapgbpbaaaacaaaaaaefaaaaajpcaabaaaabaaaaaaegaabaaa
abaaaaaaeghobaaaacaaaaaaaagabaaaacaaaaaacpaaaaafpcaabaaaabaaaaaa
egaobaaaabaaaaaaaaaaaaaihcaabaaaaaaaaaaaegacbaaaaaaaaaaaegacbaia
ebaaaaaaabaaaaaaefaaaaajpcaabaaaacaaaaaaegbabaaaabaaaaaaeghobaaa
abaaaaaaaagabaaaabaaaaaaefaaaaajpcaabaaaadaaaaaaegbabaaaabaaaaaa
eghobaaaaaaaaaaaaagabaaaaaaaaaaadiaaaaahhcaabaaaabaaaaaaegacbaaa
acaaaaaapgapbaaaadaaaaaadiaaaaaihcaabaaaacaaaaaaegacbaaaadaaaaaa
egiccaaaaaaaaaaaadaaaaaadiaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaa
fgifcaaaaaaaaaaaaeaaaaaadiaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaa
pgapbaiaebaaaaaaabaaaaaadiaaaaahocaabaaaabaaaaaaagajbaaaaaaaaaaa
agajbaaaabaaaaaadcaaaaajhccabaaaaaaaaaaaegacbaaaacaaaaaaegacbaaa
aaaaaaaajgahbaaaabaaaaaadiaaaaaiiccabaaaaaaaaaaaakaabaaaabaaaaaa
dkiacaaaaaaaaaaaacaaaaaadoaaaaab"
}

SubProgram "gles " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
"!!GLES"
}

SubProgram "glesdesktop " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
"!!GLES"
}

SubProgram "d3d11_9x " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
ConstBuffer "$Globals" 144 // 128 used size, 10 vars
Vector 32 [_SpecColor] 4
Vector 48 [_Color] 4
Float 68 [_Gloss]
Vector 112 [unity_LightmapFade] 4
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 0
SetTexture 1 [_SpecMap] 2D 1
SetTexture 2 [_LightBuffer] 2D 2
SetTexture 3 [unity_Lightmap] 2D 3
SetTexture 4 [unity_LightmapInd] 2D 4
// 24 instructions, 4 temp regs, 0 temp arrays:
// ALU 14 float, 0 int, 0 uint
// TEX 5 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0_level_9_3
eefiecedddipmiangdedfjdephillfpgbomnhnhjabaaaaaahiahaaaaaeaaaaaa
daaaaaaaneacaaaakeagaaaaeeahaaaaebgpgodjjmacaaaajmacaaaaaaacpppp
emacaaaafaaaaaaaacaadiaaaaaafaaaaaaafaaaafaaceaaaaaafaaaaaaaaaaa
abababaaacacacaaadadadaaaeaeaeaaaaaaacaaadaaaaaaaaaaaaaaaaaaahaa
abaaadaaaaaaaaaaabacppppfbaaaaafaeaaapkaaaaaaaebaaaaaaaaaaaaaaaa
aaaaaaaabpaaaaacaaaaaaiaaaaaaplabpaaaaacaaaaaaiaabaaaplabpaaaaac
aaaaaaiaacaaaplabpaaaaacaaaaaajaaaaiapkabpaaaaacaaaaaajaabaiapka
bpaaaaacaaaaaajaacaiapkabpaaaaacaaaaaajaadaiapkabpaaaaacaaaaaaja
aeaiapkaajaaaaadaaaaaiiaacaaoelaacaaoelaahaaaaacaaaaabiaaaaappia
agaaaaacaaaaabiaaaaaaaiaaeaaaaaeaaaabbiaaaaaaaiaadaakkkaadaappka
abaaaaacabaaadiaaaaaollaecaaaaadacaacpiaabaaoeiaadaioekaecaaaaad
abaacpiaabaaoeiaaeaioekaafaaaaadabaaciiaabaappiaaeaaaakaafaaaaad
aaaacoiaabaajaiaabaappiaafaaaaadacaaciiaacaappiaaeaaaakaaeaaaaae
abaaahiaacaappiaacaaoeiaaaaapjibaeaaaaaeaaaachiaaaaaaaiaabaaoeia
aaaapjiaagaaaaacaaaaaiiaabaapplaafaaaaadabaaadiaaaaappiaabaaoela
ecaaaaadabaacpiaabaaoeiaacaioekaapaaaaacacaacbiaabaaaaiaapaaaaac
acaacciaabaaffiaapaaaaacacaaceiaabaakkiaapaaaaacaaaaciiaabaappia
acaaaaadaaaachiaaaaaoeiaacaaoeibecaaaaadabaacpiaaaaaoelaaaaioeka
ecaaaaadacaacpiaaaaaoelaabaioekaafaaaaadacaachiaabaappiaacaaoeia
afaaaaadabaachiaabaaoeiaabaaoekaafaaaaadacaachiaacaaoeiaacaaffka
afaaaaadacaachiaaaaappibacaaoeiaafaaaaadacaacoiaaaaajaiaacaajaia
aeaaaaaeaaaachiaabaaoeiaaaaaoeiaacaapjiaafaaaaadaaaaciiaacaaaaia
aaaappkaabaaaaacaaaicpiaaaaaoeiappppaaaafdeieefcmiadaaaaeaaaaaaa
pcaaaaaafjaaaaaeegiocaaaaaaaaaaaaiaaaaaafkaaaaadaagabaaaaaaaaaaa
fkaaaaadaagabaaaabaaaaaafkaaaaadaagabaaaacaaaaaafkaaaaadaagabaaa
adaaaaaafkaaaaadaagabaaaaeaaaaaafibiaaaeaahabaaaaaaaaaaaffffaaaa
fibiaaaeaahabaaaabaaaaaaffffaaaafibiaaaeaahabaaaacaaaaaaffffaaaa
fibiaaaeaahabaaaadaaaaaaffffaaaafibiaaaeaahabaaaaeaaaaaaffffaaaa
gcbaaaaddcbabaaaabaaaaaagcbaaaadmcbabaaaabaaaaaagcbaaaadlcbabaaa
acaaaaaagcbaaaadpcbabaaaadaaaaaagfaaaaadpccabaaaaaaaaaaagiaaaaac
aeaaaaaabbaaaaahbcaabaaaaaaaaaaaegbobaaaadaaaaaaegbobaaaadaaaaaa
elaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaadccaaaalbcaabaaaaaaaaaaa
akaabaaaaaaaaaaackiacaaaaaaaaaaaahaaaaaadkiacaaaaaaaaaaaahaaaaaa
efaaaaajpcaabaaaabaaaaaaogbkbaaaabaaaaaaeghobaaaaeaaaaaaaagabaaa
aeaaaaaadiaaaaahccaabaaaaaaaaaaadkaabaaaabaaaaaaabeaaaaaaaaaaaeb
diaaaaahocaabaaaaaaaaaaaagajbaaaabaaaaaafgafbaaaaaaaaaaaefaaaaaj
pcaabaaaabaaaaaaogbkbaaaabaaaaaaeghobaaaadaaaaaaaagabaaaadaaaaaa
diaaaaahicaabaaaabaaaaaadkaabaaaabaaaaaaabeaaaaaaaaaaaebdcaaaaak
hcaabaaaabaaaaaapgapbaaaabaaaaaaegacbaaaabaaaaaajgahbaiaebaaaaaa
aaaaaaaadcaaaaajhcaabaaaaaaaaaaaagaabaaaaaaaaaaaegacbaaaabaaaaaa
jgahbaaaaaaaaaaaaoaaaaahdcaabaaaabaaaaaaegbabaaaacaaaaaapgbpbaaa
acaaaaaaefaaaaajpcaabaaaabaaaaaaegaabaaaabaaaaaaeghobaaaacaaaaaa
aagabaaaacaaaaaacpaaaaafpcaabaaaabaaaaaaegaobaaaabaaaaaaaaaaaaai
hcaabaaaaaaaaaaaegacbaaaaaaaaaaaegacbaiaebaaaaaaabaaaaaaefaaaaaj
pcaabaaaacaaaaaaegbabaaaabaaaaaaeghobaaaabaaaaaaaagabaaaabaaaaaa
efaaaaajpcaabaaaadaaaaaaegbabaaaabaaaaaaeghobaaaaaaaaaaaaagabaaa
aaaaaaaadiaaaaahhcaabaaaabaaaaaaegacbaaaacaaaaaapgapbaaaadaaaaaa
diaaaaaihcaabaaaacaaaaaaegacbaaaadaaaaaaegiccaaaaaaaaaaaadaaaaaa
diaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaafgifcaaaaaaaaaaaaeaaaaaa
diaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaapgapbaiaebaaaaaaabaaaaaa
diaaaaahocaabaaaabaaaaaaagajbaaaaaaaaaaaagajbaaaabaaaaaadcaaaaaj
hccabaaaaaaaaaaaegacbaaaacaaaaaaegacbaaaaaaaaaaajgahbaaaabaaaaaa
diaaaaaiiccabaaaaaaaaaaaakaabaaaabaaaaaadkiacaaaaaaaaaaaacaaaaaa
doaaaaabejfdeheojiaaaaaaafaaaaaaaiaaaaaaiaaaaaaaaaaaaaaaabaaaaaa
adaaaaaaaaaaaaaaapaaaaaaimaaaaaaaaaaaaaaaaaaaaaaadaaaaaaabaaaaaa
adadaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaabaaaaaaamamaaaaimaaaaaa
abaaaaaaaaaaaaaaadaaaaaaacaaaaaaapalaaaaimaaaaaaadaaaaaaaaaaaaaa
adaaaaaaadaaaaaaapapaaaafdfgfpfaepfdejfeejepeoaafeeffiedepepfcee
aaklklklepfdeheocmaaaaaaabaaaaaaaiaaaaaacaaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaaaaaaaaaapaaaaaafdfgfpfegbhcghgfheaaklkl"
}

SubProgram "opengl " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
Vector 0 [_SpecColor]
Vector 1 [_Color]
Float 2 [_Gloss]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 3 [_LightBuffer] 2D
"!!ARBfp1.0
OPTION ARB_precision_hint_fastest;
# 11 ALU, 3 TEX
PARAM c[3] = { program.local[0..2] };
TEMP R0;
TEMP R1;
TEMP R2;
TXP R0, fragment.texcoord[1], texture[3], 2D;
TEX R1, fragment.texcoord[0], texture[0], 2D;
TEX R2.xyz, fragment.texcoord[0], texture[1], 2D;
MUL R2.xyz, R1.w, R2;
MUL R2.xyz, R2, c[2].x;
MUL R2.xyz, R0.w, R2;
ADD R0.xyz, R0, fragment.texcoord[2];
MUL R2.yzw, R2.xxyz, R0.xxyz;
MUL R1.xyz, R1, c[1];
MAD result.color.xyz, R1, R0, R2.yzww;
MUL result.color.w, R2.x, c[0];
END
# 11 instructions, 3 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
Vector 0 [_SpecColor]
Vector 1 [_Color]
Float 2 [_Gloss]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 3 [_LightBuffer] 2D
"ps_2_0
; 9 ALU, 3 TEX
dcl_2d s0
dcl_2d s1
dcl_2d s3
dcl t0.xy
dcl t1
dcl t2.xyz
texldp r0, t1, s3
texld r1, t0, s1
texld r2, t0, s0
mul_pp r1.xyz, r2.w, r1
mul_pp r1.xyz, r1, c2.x
mul_pp r1.xyz, r0.w, r1
add_pp r0.xyz, r0, t2
mul_pp r3.xyz, r1, r0
mul_pp r2.xyz, r2, c1
mad_pp r0.xyz, r2, r0, r3
mul_pp r0.w, r1.x, c0
mov_pp oC0, r0
"
}

SubProgram "xbox360 " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
Vector 1 [_Color]
Float 2 [_Gloss]
Vector 0 [_SpecColor]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_LightBuffer] 2D
SetTexture 3 [_LightSpecBuffer] 2D
// Shader Timing Estimate, in Cycles/64 pixel vector:
// ALU: 12.00 (9 instructions), vertex: 0, texture: 16,
//   sequencer: 8, interpolator: 12;    4 GPRs, 48 threads,
// Performance (if enough threads): ~16 cycles per vector
// * Texture cycle estimates are assuming an 8bit/component texture with no
//     aniso or trilinear filtering.

"ps_360
backbbaaaaaaabjaaaaaaamaaaaaaaaaaaaaaaceaaaaaaaaaaaaabgeaaaaaaaa
aaaaaaaaaaaaabdmaaaaaabmaaaaabcnppppadaaaaaaaaahaaaaaabmaaaaaaaa
aaaaabcgaaaaaakiaaacaaabaaabaaaaaaaaaalaaaaaaaaaaaaaaamaaaacaaac
aaabaaaaaaaaaamiaaaaaaaaaaaaaaniaaadaaacaaabaaaaaaaaaaoiaaaaaaaa
aaaaaapiaaadaaadaaabaaaaaaaaaaoiaaaaaaaaaaaaabajaaadaaaaaaabaaaa
aaaaaaoiaaaaaaaaaaaaabbcaaacaaaaaaabaaaaaaaaaalaaaaaaaaaaaaaabbn
aaadaaabaaabaaaaaaaaaaoiaaaaaaaafpedgpgmgphcaaklaaabaaadaaabaaae
aaabaaaaaaaaaaaafpehgmgphdhdaaklaaaaaaadaaabaaabaaabaaaaaaaaaaaa
fpemgjghgiheechfgggggfhcaaklklklaaaeaaamaaabaaabaaabaaaaaaaaaaaa
fpemgjghgihefdhagfgdechfgggggfhcaafpengbgjgofegfhiaafpfdhagfgded
gpgmgphcaafpfdhagfgdengbhaaahahdfpddfpdaaadccodacodcdadddfddcoda
aaklklklaaaaaaaaaaaaaamabaaaadaaaaaaaaaeaaaaaaaaaaaacegdaaahaaah
aaaaaaabaaaadafaaaaapbfbaaaahcfcaffagaacaaaabcaameaaaaaaaaaagaai
baaobcaaccaaaaaaemeaaaaaaaaaaablocaaaaabmiamaaaaaamgkmaaobaaabaa
lidibaabbpbppppiaaaaeaaababibaabbpbppeehaaaaeaaalicidaabbpbppoii
aaaaeaaabaaiaaabbpbppgecaaaaeaaamiahaaacaamamaaaoaadacaabecoaaab
aablablbobaaabaakiboadabaaabgmebibabacabkichadabaabfgmicmbababab
kmiaiaaaaaaaaaaamcaaaaaakiehadabaamamamambabacabmiahiaaaaamamama
oladacabaaaaaaaaaaaaaaaaaaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
Vector 0 [_SpecColor]
Vector 1 [_Color]
Float 2 [_Gloss]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 3 [_LightBuffer] 2D
"sce_fp_rsx // 14 instructions using 3 registers
[Configuration]
24
ffffffff0001c0200007fff9000000000000840003000000
[Offsets]
3
_SpecColor 1 0
000000c0
_Color 1 0
00000020
_Gloss 1 0
00000050
[Microcode]
224
9e001700c8011c9dc8000001c8003fe10e800240c8001c9dc8020001c8000001
00000000000000000000000000000000be021806c8011c9dc8000001c8003fe1
10800240c8001c9d00020000c800000100000000000000000000000000000000
10800240c8041c9dc9000001c8000001ce820300c8011c9dc8040001c8003fe1
8e041702c8011c9dc8000001c8003fe10e880240ff001c9dc8080001c8000001
0e840240c9101c9dc9040001c80000011080024001101c9cc8020001c8000001
000000000000000000000000000000000e810440c9001c9dc9040001c9080001
"
}

SubProgram "d3d11 " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
ConstBuffer "$Globals" 112 // 72 used size, 8 vars
Vector 32 [_SpecColor] 4
Vector 48 [_Color] 4
Float 68 [_Gloss]
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 0
SetTexture 1 [_SpecMap] 2D 1
SetTexture 2 [_LightBuffer] 2D 2
// 13 instructions, 3 temp regs, 0 temp arrays:
// ALU 8 float, 0 int, 0 uint
// TEX 3 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0
eefiecedhdpdkkfmodaajphnndlmfcfaelclpoopabaaaaaabeadaaaaadaaaaaa
cmaaaaaaleaaaaaaoiaaaaaaejfdeheoiaaaaaaaaeaaaaaaaiaaaaaagiaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaheaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaadadaaaaheaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaa
apalaaaaheaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaahahaaaafdfgfpfa
epfdejfeejepeoaafeeffiedepepfceeaaklklklepfdeheocmaaaaaaabaaaaaa
aiaaaaaacaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaafdfgfpfe
gbhcghgfheaaklklfdeieefcceacaaaaeaaaaaaaijaaaaaafjaaaaaeegiocaaa
aaaaaaaaafaaaaaafkaaaaadaagabaaaaaaaaaaafkaaaaadaagabaaaabaaaaaa
fkaaaaadaagabaaaacaaaaaafibiaaaeaahabaaaaaaaaaaaffffaaaafibiaaae
aahabaaaabaaaaaaffffaaaafibiaaaeaahabaaaacaaaaaaffffaaaagcbaaaad
dcbabaaaabaaaaaagcbaaaadlcbabaaaacaaaaaagcbaaaadhcbabaaaadaaaaaa
gfaaaaadpccabaaaaaaaaaaagiaaaaacadaaaaaaefaaaaajpcaabaaaaaaaaaaa
egbabaaaabaaaaaaeghobaaaabaaaaaaaagabaaaabaaaaaaefaaaaajpcaabaaa
abaaaaaaegbabaaaabaaaaaaeghobaaaaaaaaaaaaagabaaaaaaaaaaadiaaaaah
hcaabaaaaaaaaaaaegacbaaaaaaaaaaapgapbaaaabaaaaaadiaaaaaihcaabaaa
abaaaaaaegacbaaaabaaaaaaegiccaaaaaaaaaaaadaaaaaadiaaaaaihcaabaaa
aaaaaaaaegacbaaaaaaaaaaafgifcaaaaaaaaaaaaeaaaaaaaoaaaaahdcaabaaa
acaaaaaaegbabaaaacaaaaaapgbpbaaaacaaaaaaefaaaaajpcaabaaaacaaaaaa
egaabaaaacaaaaaaeghobaaaacaaaaaaaagabaaaacaaaaaadiaaaaahhcaabaaa
aaaaaaaaegacbaaaaaaaaaaapgapbaaaacaaaaaaaaaaaaahhcaabaaaacaaaaaa
egacbaaaacaaaaaaegbcbaaaadaaaaaadiaaaaahocaabaaaaaaaaaaaagajbaaa
aaaaaaaaagajbaaaacaaaaaadiaaaaaiiccabaaaaaaaaaaaakaabaaaaaaaaaaa
dkiacaaaaaaaaaaaacaaaaaadcaaaaajhccabaaaaaaaaaaaegacbaaaabaaaaaa
egacbaaaacaaaaaajgahbaaaaaaaaaaadoaaaaab"
}

SubProgram "gles " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
"!!GLES"
}

SubProgram "glesdesktop " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
"!!GLES"
}

SubProgram "d3d11_9x " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
ConstBuffer "$Globals" 112 // 72 used size, 8 vars
Vector 32 [_SpecColor] 4
Vector 48 [_Color] 4
Float 68 [_Gloss]
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 0
SetTexture 1 [_SpecMap] 2D 1
SetTexture 2 [_LightBuffer] 2D 2
// 13 instructions, 3 temp regs, 0 temp arrays:
// ALU 8 float, 0 int, 0 uint
// TEX 3 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0_level_9_3
eefiecedkobbmccinccoidjbdbahpgbbolecclababaaaaaaiiaeaaaaaeaaaaaa
daaaaaaakaabaaaammadaaaafeaeaaaaebgpgodjgiabaaaagiabaaaaaaacpppp
cmabaaaadmaaaaaaabaadaaaaaaadmaaaaaadmaaadaaceaaaaaadmaaaaaaaaaa
abababaaacacacaaaaaaacaaadaaaaaaaaaaaaaaabacppppbpaaaaacaaaaaaia
aaaaadlabpaaaaacaaaaaaiaabaaaplabpaaaaacaaaaaaiaacaaahlabpaaaaac
aaaaaajaaaaiapkabpaaaaacaaaaaajaabaiapkabpaaaaacaaaaaajaacaiapka
ecaaaaadaaaacpiaaaaaoelaaaaioekaecaaaaadabaacpiaaaaaoelaabaioeka
afaaaaadabaachiaaaaappiaabaaoeiaafaaaaadaaaachiaaaaaoeiaabaaoeka
afaaaaadabaachiaabaaoeiaacaaffkaagaaaaacaaaaaiiaabaapplaafaaaaad
acaaadiaaaaappiaabaaoelaecaaaaadacaacpiaacaaoeiaacaioekaafaaaaad
abaachiaabaaoeiaacaappiaacaaaaadacaachiaacaaoeiaacaaoelaafaaaaad
abaacoiaabaajaiaacaajaiaafaaaaadadaaciiaabaaaaiaaaaappkaaeaaaaae
adaachiaaaaaoeiaacaaoeiaabaapjiaabaaaaacaaaicpiaadaaoeiappppaaaa
fdeieefcceacaaaaeaaaaaaaijaaaaaafjaaaaaeegiocaaaaaaaaaaaafaaaaaa
fkaaaaadaagabaaaaaaaaaaafkaaaaadaagabaaaabaaaaaafkaaaaadaagabaaa
acaaaaaafibiaaaeaahabaaaaaaaaaaaffffaaaafibiaaaeaahabaaaabaaaaaa
ffffaaaafibiaaaeaahabaaaacaaaaaaffffaaaagcbaaaaddcbabaaaabaaaaaa
gcbaaaadlcbabaaaacaaaaaagcbaaaadhcbabaaaadaaaaaagfaaaaadpccabaaa
aaaaaaaagiaaaaacadaaaaaaefaaaaajpcaabaaaaaaaaaaaegbabaaaabaaaaaa
eghobaaaabaaaaaaaagabaaaabaaaaaaefaaaaajpcaabaaaabaaaaaaegbabaaa
abaaaaaaeghobaaaaaaaaaaaaagabaaaaaaaaaaadiaaaaahhcaabaaaaaaaaaaa
egacbaaaaaaaaaaapgapbaaaabaaaaaadiaaaaaihcaabaaaabaaaaaaegacbaaa
abaaaaaaegiccaaaaaaaaaaaadaaaaaadiaaaaaihcaabaaaaaaaaaaaegacbaaa
aaaaaaaafgifcaaaaaaaaaaaaeaaaaaaaoaaaaahdcaabaaaacaaaaaaegbabaaa
acaaaaaapgbpbaaaacaaaaaaefaaaaajpcaabaaaacaaaaaaegaabaaaacaaaaaa
eghobaaaacaaaaaaaagabaaaacaaaaaadiaaaaahhcaabaaaaaaaaaaaegacbaaa
aaaaaaaapgapbaaaacaaaaaaaaaaaaahhcaabaaaacaaaaaaegacbaaaacaaaaaa
egbcbaaaadaaaaaadiaaaaahocaabaaaaaaaaaaaagajbaaaaaaaaaaaagajbaaa
acaaaaaadiaaaaaiiccabaaaaaaaaaaaakaabaaaaaaaaaaadkiacaaaaaaaaaaa
acaaaaaadcaaaaajhccabaaaaaaaaaaaegacbaaaabaaaaaaegacbaaaacaaaaaa
jgahbaaaaaaaaaaadoaaaaabejfdeheoiaaaaaaaaeaaaaaaaiaaaaaagiaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaheaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaadadaaaaheaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaa
apalaaaaheaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaahahaaaafdfgfpfa
epfdejfeejepeoaafeeffiedepepfceeaaklklklepfdeheocmaaaaaaabaaaaaa
aiaaaaaacaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaafdfgfpfe
gbhcghgfheaaklkl"
}

SubProgram "opengl " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
Vector 0 [_SpecColor]
Vector 1 [_Color]
Float 2 [_Gloss]
Vector 3 [unity_LightmapFade]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 3 [_LightBuffer] 2D
SetTexture 4 [unity_Lightmap] 2D
SetTexture 5 [unity_LightmapInd] 2D
"!!ARBfp1.0
OPTION ARB_precision_hint_fastest;
# 22 ALU, 5 TEX
PARAM c[5] = { program.local[0..3],
		{ 8 } };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
TEMP R4;
TEX R3, fragment.texcoord[0], texture[0], 2D;
TEX R1, fragment.texcoord[2], texture[5], 2D;
TEX R2, fragment.texcoord[2], texture[4], 2D;
TEX R4.xyz, fragment.texcoord[0], texture[1], 2D;
TXP R0, fragment.texcoord[1], texture[3], 2D;
MUL R1.xyz, R1.w, R1;
DP4 R1.w, fragment.texcoord[3], fragment.texcoord[3];
RSQ R1.w, R1.w;
RCP R1.w, R1.w;
MUL R1.xyz, R1, c[4].x;
MUL R2.xyz, R2.w, R2;
MAD R2.xyz, R2, c[4].x, -R1;
MAD_SAT R1.w, R1, c[3].z, c[3];
MAD R1.xyz, R1.w, R2, R1;
MUL R4.xyz, R3.w, R4;
ADD R0.xyz, R0, R1;
MUL R2.xyz, R4, c[2].x;
MUL R1.xyz, R0.w, R2;
MUL R2.xyz, R1, R0;
MUL R3.xyz, R3, c[1];
MAD result.color.xyz, R3, R0, R2;
MUL result.color.w, R1.x, c[0];
END
# 22 instructions, 5 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
Vector 0 [_SpecColor]
Vector 1 [_Color]
Float 2 [_Gloss]
Vector 3 [unity_LightmapFade]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 3 [_LightBuffer] 2D
SetTexture 4 [unity_Lightmap] 2D
SetTexture 5 [unity_LightmapInd] 2D
"ps_2_0
; 18 ALU, 5 TEX
dcl_2d s0
dcl_2d s1
dcl_2d s3
dcl_2d s4
dcl_2d s5
def c4, 8.00000000, 0, 0, 0
dcl t0.xy
dcl t1
dcl t2.xy
dcl t3
texldp r1, t1, s3
texld r4, t0, s1
texld r2, t0, s0
texld r3, t2, s5
texld r0, t2, s4
mul_pp r3.xyz, r3.w, r3
mul_pp r0.xyz, r0.w, r0
mul_pp r3.xyz, r3, c4.x
mad_pp r5.xyz, r0, c4.x, -r3
dp4 r0.x, t3, t3
rsq r0.x, r0.x
rcp r0.x, r0.x
mad_sat r0.x, r0, c3.z, c3.w
mad_pp r0.xyz, r0.x, r5, r3
mul_pp r4.xyz, r2.w, r4
add_pp r0.xyz, r1, r0
mul_pp r3.xyz, r4, c2.x
mul_pp r1.xyz, r1.w, r3
mul_pp r3.xyz, r1, r0
mul_pp r2.xyz, r2, c1
mad_pp r0.xyz, r2, r0, r3
mul_pp r0.w, r1.x, c0
mov_pp oC0, r0
"
}

SubProgram "xbox360 " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
Vector 1 [_Color]
Float 2 [_Gloss]
Vector 0 [_SpecColor]
Vector 3 [unity_LightmapFade]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_LightBuffer] 2D
SetTexture 3 [_LightSpecBuffer] 2D
SetTexture 4 [unity_Lightmap] 2D
SetTexture 5 [unity_LightmapInd] 2D
// Shader Timing Estimate, in Cycles/64 pixel vector:
// ALU: 20.00 (15 instructions), vertex: 0, texture: 24,
//   sequencer: 12, interpolator: 16;    8 GPRs, 24 threads,
// Performance (if enough threads): ~24 cycles per vector
// * Texture cycle estimates are assuming an 8bit/component texture with no
//     aniso or trilinear filtering.

"ps_360
backbbaaaaaaaccmaaaaabgmaaaaaaaaaaaaaaceaaaaabneaaaaabpmaaaaaaaa
aaaaaaaaaaaaabkmaaaaaabmaaaaabjnppppadaaaaaaaaakaaaaaabmaaaaaaaa
aaaaabjgaaaaaaoeaaacaaabaaabaaaaaaaaaaomaaaaaaaaaaaaaapmaaacaaac
aaabaaaaaaaaabaeaaaaaaaaaaaaabbeaaadaaacaaabaaaaaaaaabceaaaaaaaa
aaaaabdeaaadaaadaaabaaaaaaaaabceaaaaaaaaaaaaabefaaadaaaaaaabaaaa
aaaaabceaaaaaaaaaaaaabeoaaacaaaaaaabaaaaaaaaaaomaaaaaaaaaaaaabfj
aaadaaabaaabaaaaaaaaabceaaaaaaaaaaaaabgcaaadaaaeaaabaaaaaaaaabce
aaaaaaaaaaaaabhbaaacaaadaaabaaaaaaaaaaomaaaaaaaaaaaaabieaaadaaaf
aaabaaaaaaaaabceaaaaaaaafpedgpgmgphcaaklaaabaaadaaabaaaeaaabaaaa
aaaaaaaafpehgmgphdhdaaklaaaaaaadaaabaaabaaabaaaaaaaaaaaafpemgjgh
giheechfgggggfhcaaklklklaaaeaaamaaabaaabaaabaaaaaaaaaaaafpemgjgh
gihefdhagfgdechfgggggfhcaafpengbgjgofegfhiaafpfdhagfgdedgpgmgphc
aafpfdhagfgdengbhaaahfgogjhehjfpemgjghgihegngbhaaahfgogjhehjfpem
gjghgihegngbhaeggbgegfaahfgogjhehjfpemgjghgihegngbhaejgogeaahahd
fpddfpdaaadccodacodcdadddfddcodaaaklklklaaaaaaaaaaaaaaabaaaaaaaa
aaaaaaaaaaaaaabeabpmaabaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaea
aaaaabcmbaaaahaaaaaaaaaeaaaaaaaaaaaadaieaaapaaapaaaaaaabaaaadafa
aaaapbfbaaaadcfcaaaapdfdaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaebaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaffagaadcaajbcaabcaaaaafaaaaaaaagaalmeaa
bcaaaaaaaaaagabbbabhbcaaccaaaaaaemeaaaaaaaaaaablocaaaaabmiamaaaa
aamgkmaaobaaabaalicieaabbpbppoiiaaaaeaaalidiaaabbpbppodpaaaaeaaa
babifaabbpbppoiiaaaaeaaabaaibaabbpbppgbbaaaaeaaabafihaebbpbppgii
aaaaeaaabaeigaebbpbppgiiaaaaeaaakibaaaaaaaaaaaehocaaaappbeciaaaa
aakhkhblopadadahkaihaaacaablmablobabafiamjaiaaacaablmgblilaaadad
kichaaafaamagmebibacacppbechaaacaalbmamgobaaahabmiaoaaadabgmpmpm
olaaagackibnadaaaapamgebmbafaaabkiiaiaaaaaaaaaaamcaaaaaamiahaaac
aabfblmaoladacackmchadacaamamaiamaacaeabkmehadaaaabemambmbaaacab
miahiaaaaamamamaoladacaaaaaaaaaaaaaaaaaaaaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
Vector 0 [_SpecColor]
Vector 1 [_Color]
Float 2 [_Gloss]
Vector 3 [unity_LightmapFade]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 3 [_LightBuffer] 2D
SetTexture 4 [unity_Lightmap] 2D
SetTexture 5 [unity_LightmapInd] 2D
"sce_fp_rsx // 28 instructions using 3 registers
[Configuration]
24
ffffffff0003c020000ffff5000000000000840003000000
[Offsets]
4
_SpecColor 1 0
000001a0
_Color 1 0
000000d0
_Gloss 1 0
00000120
unity_LightmapFade 2 0
0000008000000060
[Microcode]
448
fe000100c8011c9dc8000001c8003fe102000600c8001c9dc8000001c8000001
de021708c8011c9dc8000001c8003fe110880140c8041c9dc8003001c8000001
02001b00c8001c9dc8000001c800000110023a0054021c9dc8000001c8000001
0000000000000000000000000000000010028300c8041c9dc8020001c8000001
00000000000000000000000000000000de00170ac8011c9dc8000001c8003fe1
0e880240fe001c9dc8003001c80000019e001700c8011c9dc8000001c8003fe1
0e800240c8001c9dc8020001c800000100000000000000000000000000000000
0e020400ff101c9dc8040001c910000310020100c8041c9dc8000001c8000001
0e880400fe041c9dc8040001c910000110880240c8001c9d00020000c8000001
00000000000000000000000000000000be021806c8011c9dc8000001c8003fe1
0e820340c8041c9dc9100001c800000110800240c8041c9dc9100001c8000001
8e021702c8011c9dc8000001c8003fe10e840240ff001c9dc8040001c8000001
0e860240c9081c9dc9040001c80000011080024001081c9cc8020001c8000001
000000000000000000000000000000000e810440c9001c9dc9040001c90c0001
"
}

SubProgram "d3d11 " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
ConstBuffer "$Globals" 144 // 128 used size, 10 vars
Vector 32 [_SpecColor] 4
Vector 48 [_Color] 4
Float 68 [_Gloss]
Vector 112 [unity_LightmapFade] 4
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 0
SetTexture 1 [_SpecMap] 2D 1
SetTexture 2 [_LightBuffer] 2D 2
SetTexture 3 [unity_Lightmap] 2D 3
SetTexture 4 [unity_LightmapInd] 2D 4
// 23 instructions, 4 temp regs, 0 temp arrays:
// ALU 13 float, 0 int, 0 uint
// TEX 5 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0
eefiecedgankdnabdjiopjobdanehhjeapfdgnkjabaaaaaaleaeaaaaadaaaaaa
cmaaaaaammaaaaaaaaabaaaaejfdeheojiaaaaaaafaaaaaaaiaaaaaaiaaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaadadaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaabaaaaaa
amamaaaaimaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaaapalaaaaimaaaaaa
adaaaaaaaaaaaaaaadaaaaaaadaaaaaaapapaaaafdfgfpfaepfdejfeejepeoaa
feeffiedepepfceeaaklklklepfdeheocmaaaaaaabaaaaaaaiaaaaaacaaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaafdfgfpfegbhcghgfheaaklkl
fdeieefckmadaaaaeaaaaaaaolaaaaaafjaaaaaeegiocaaaaaaaaaaaaiaaaaaa
fkaaaaadaagabaaaaaaaaaaafkaaaaadaagabaaaabaaaaaafkaaaaadaagabaaa
acaaaaaafkaaaaadaagabaaaadaaaaaafkaaaaadaagabaaaaeaaaaaafibiaaae
aahabaaaaaaaaaaaffffaaaafibiaaaeaahabaaaabaaaaaaffffaaaafibiaaae
aahabaaaacaaaaaaffffaaaafibiaaaeaahabaaaadaaaaaaffffaaaafibiaaae
aahabaaaaeaaaaaaffffaaaagcbaaaaddcbabaaaabaaaaaagcbaaaadmcbabaaa
abaaaaaagcbaaaadlcbabaaaacaaaaaagcbaaaadpcbabaaaadaaaaaagfaaaaad
pccabaaaaaaaaaaagiaaaaacaeaaaaaabbaaaaahbcaabaaaaaaaaaaaegbobaaa
adaaaaaaegbobaaaadaaaaaaelaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaa
dccaaaalbcaabaaaaaaaaaaaakaabaaaaaaaaaaackiacaaaaaaaaaaaahaaaaaa
dkiacaaaaaaaaaaaahaaaaaaefaaaaajpcaabaaaabaaaaaaogbkbaaaabaaaaaa
eghobaaaaeaaaaaaaagabaaaaeaaaaaadiaaaaahccaabaaaaaaaaaaadkaabaaa
abaaaaaaabeaaaaaaaaaaaebdiaaaaahocaabaaaaaaaaaaaagajbaaaabaaaaaa
fgafbaaaaaaaaaaaefaaaaajpcaabaaaabaaaaaaogbkbaaaabaaaaaaeghobaaa
adaaaaaaaagabaaaadaaaaaadiaaaaahicaabaaaabaaaaaadkaabaaaabaaaaaa
abeaaaaaaaaaaaebdcaaaaakhcaabaaaabaaaaaapgapbaaaabaaaaaaegacbaaa
abaaaaaajgahbaiaebaaaaaaaaaaaaaadcaaaaajhcaabaaaaaaaaaaaagaabaaa
aaaaaaaaegacbaaaabaaaaaajgahbaaaaaaaaaaaaoaaaaahdcaabaaaabaaaaaa
egbabaaaacaaaaaapgbpbaaaacaaaaaaefaaaaajpcaabaaaabaaaaaaegaabaaa
abaaaaaaeghobaaaacaaaaaaaagabaaaacaaaaaaaaaaaaahhcaabaaaaaaaaaaa
egacbaaaaaaaaaaaegacbaaaabaaaaaaefaaaaajpcaabaaaacaaaaaaegbabaaa
abaaaaaaeghobaaaabaaaaaaaagabaaaabaaaaaaefaaaaajpcaabaaaadaaaaaa
egbabaaaabaaaaaaeghobaaaaaaaaaaaaagabaaaaaaaaaaadiaaaaahhcaabaaa
abaaaaaaegacbaaaacaaaaaapgapbaaaadaaaaaadiaaaaaihcaabaaaacaaaaaa
egacbaaaadaaaaaaegiccaaaaaaaaaaaadaaaaaadiaaaaaihcaabaaaabaaaaaa
egacbaaaabaaaaaafgifcaaaaaaaaaaaaeaaaaaadiaaaaahhcaabaaaabaaaaaa
egacbaaaabaaaaaapgapbaaaabaaaaaadiaaaaahocaabaaaabaaaaaaagajbaaa
aaaaaaaaagajbaaaabaaaaaadcaaaaajhccabaaaaaaaaaaaegacbaaaacaaaaaa
egacbaaaaaaaaaaajgahbaaaabaaaaaadiaaaaaiiccabaaaaaaaaaaaakaabaaa
abaaaaaadkiacaaaaaaaaaaaacaaaaaadoaaaaab"
}

SubProgram "gles " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
"!!GLES"
}

SubProgram "glesdesktop " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
"!!GLES"
}

SubProgram "d3d11_9x " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
ConstBuffer "$Globals" 144 // 128 used size, 10 vars
Vector 32 [_SpecColor] 4
Vector 48 [_Color] 4
Float 68 [_Gloss]
Vector 112 [unity_LightmapFade] 4
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 0
SetTexture 1 [_SpecMap] 2D 1
SetTexture 2 [_LightBuffer] 2D 2
SetTexture 3 [unity_Lightmap] 2D 3
SetTexture 4 [unity_LightmapInd] 2D 4
// 23 instructions, 4 temp regs, 0 temp arrays:
// ALU 13 float, 0 int, 0 uint
// TEX 5 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0_level_9_3
eefiecedkhmggpijfbkapffppjbjhmmjmegplanoabaaaaaacmahaaaaaeaaaaaa
daaaaaaakeacaaaafiagaaaapiagaaaaebgpgodjgmacaaaagmacaaaaaaacpppp
bmacaaaafaaaaaaaacaadiaaaaaafaaaaaaafaaaafaaceaaaaaafaaaaaaaaaaa
abababaaacacacaaadadadaaaeaeaeaaaaaaacaaadaaaaaaaaaaaaaaaaaaahaa
abaaadaaaaaaaaaaabacppppfbaaaaafaeaaapkaaaaaaaebaaaaaaaaaaaaaaaa
aaaaaaaabpaaaaacaaaaaaiaaaaaaplabpaaaaacaaaaaaiaabaaaplabpaaaaac
aaaaaaiaacaaaplabpaaaaacaaaaaajaaaaiapkabpaaaaacaaaaaajaabaiapka
bpaaaaacaaaaaajaacaiapkabpaaaaacaaaaaajaadaiapkabpaaaaacaaaaaaja
aeaiapkaajaaaaadaaaaaiiaacaaoelaacaaoelaahaaaaacaaaaabiaaaaappia
agaaaaacaaaaabiaaaaaaaiaaeaaaaaeaaaabbiaaaaaaaiaadaakkkaadaappka
abaaaaacabaaadiaaaaaollaecaaaaadacaacpiaabaaoeiaadaioekaecaaaaad
abaacpiaabaaoeiaaeaioekaafaaaaadabaaciiaabaappiaaeaaaakaafaaaaad
aaaacoiaabaajaiaabaappiaafaaaaadacaaciiaacaappiaaeaaaakaaeaaaaae
abaaahiaacaappiaacaaoeiaaaaapjibaeaaaaaeaaaachiaaaaaaaiaabaaoeia
aaaapjiaagaaaaacaaaaaiiaabaapplaafaaaaadabaaadiaaaaappiaabaaoela
ecaaaaadabaacpiaabaaoeiaacaioekaacaaaaadaaaachiaaaaaoeiaabaaoeia
ecaaaaadacaacpiaaaaaoelaaaaioekaecaaaaadadaacpiaaaaaoelaabaioeka
afaaaaadabaachiaacaappiaadaaoeiaafaaaaadacaachiaacaaoeiaabaaoeka
afaaaaadabaachiaabaaoeiaacaaffkaafaaaaadabaachiaabaaoeiaabaappia
afaaaaadabaacoiaaaaajaiaabaajaiaaeaaaaaeaaaachiaacaaoeiaaaaaoeia
abaapjiaafaaaaadaaaaciiaabaaaaiaaaaappkaabaaaaacaaaicpiaaaaaoeia
ppppaaaafdeieefckmadaaaaeaaaaaaaolaaaaaafjaaaaaeegiocaaaaaaaaaaa
aiaaaaaafkaaaaadaagabaaaaaaaaaaafkaaaaadaagabaaaabaaaaaafkaaaaad
aagabaaaacaaaaaafkaaaaadaagabaaaadaaaaaafkaaaaadaagabaaaaeaaaaaa
fibiaaaeaahabaaaaaaaaaaaffffaaaafibiaaaeaahabaaaabaaaaaaffffaaaa
fibiaaaeaahabaaaacaaaaaaffffaaaafibiaaaeaahabaaaadaaaaaaffffaaaa
fibiaaaeaahabaaaaeaaaaaaffffaaaagcbaaaaddcbabaaaabaaaaaagcbaaaad
mcbabaaaabaaaaaagcbaaaadlcbabaaaacaaaaaagcbaaaadpcbabaaaadaaaaaa
gfaaaaadpccabaaaaaaaaaaagiaaaaacaeaaaaaabbaaaaahbcaabaaaaaaaaaaa
egbobaaaadaaaaaaegbobaaaadaaaaaaelaaaaafbcaabaaaaaaaaaaaakaabaaa
aaaaaaaadccaaaalbcaabaaaaaaaaaaaakaabaaaaaaaaaaackiacaaaaaaaaaaa
ahaaaaaadkiacaaaaaaaaaaaahaaaaaaefaaaaajpcaabaaaabaaaaaaogbkbaaa
abaaaaaaeghobaaaaeaaaaaaaagabaaaaeaaaaaadiaaaaahccaabaaaaaaaaaaa
dkaabaaaabaaaaaaabeaaaaaaaaaaaebdiaaaaahocaabaaaaaaaaaaaagajbaaa
abaaaaaafgafbaaaaaaaaaaaefaaaaajpcaabaaaabaaaaaaogbkbaaaabaaaaaa
eghobaaaadaaaaaaaagabaaaadaaaaaadiaaaaahicaabaaaabaaaaaadkaabaaa
abaaaaaaabeaaaaaaaaaaaebdcaaaaakhcaabaaaabaaaaaapgapbaaaabaaaaaa
egacbaaaabaaaaaajgahbaiaebaaaaaaaaaaaaaadcaaaaajhcaabaaaaaaaaaaa
agaabaaaaaaaaaaaegacbaaaabaaaaaajgahbaaaaaaaaaaaaoaaaaahdcaabaaa
abaaaaaaegbabaaaacaaaaaapgbpbaaaacaaaaaaefaaaaajpcaabaaaabaaaaaa
egaabaaaabaaaaaaeghobaaaacaaaaaaaagabaaaacaaaaaaaaaaaaahhcaabaaa
aaaaaaaaegacbaaaaaaaaaaaegacbaaaabaaaaaaefaaaaajpcaabaaaacaaaaaa
egbabaaaabaaaaaaeghobaaaabaaaaaaaagabaaaabaaaaaaefaaaaajpcaabaaa
adaaaaaaegbabaaaabaaaaaaeghobaaaaaaaaaaaaagabaaaaaaaaaaadiaaaaah
hcaabaaaabaaaaaaegacbaaaacaaaaaapgapbaaaadaaaaaadiaaaaaihcaabaaa
acaaaaaaegacbaaaadaaaaaaegiccaaaaaaaaaaaadaaaaaadiaaaaaihcaabaaa
abaaaaaaegacbaaaabaaaaaafgifcaaaaaaaaaaaaeaaaaaadiaaaaahhcaabaaa
abaaaaaaegacbaaaabaaaaaapgapbaaaabaaaaaadiaaaaahocaabaaaabaaaaaa
agajbaaaaaaaaaaaagajbaaaabaaaaaadcaaaaajhccabaaaaaaaaaaaegacbaaa
acaaaaaaegacbaaaaaaaaaaajgahbaaaabaaaaaadiaaaaaiiccabaaaaaaaaaaa
akaabaaaabaaaaaadkiacaaaaaaaaaaaacaaaaaadoaaaaabejfdeheojiaaaaaa
afaaaaaaaiaaaaaaiaaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaa
imaaaaaaaaaaaaaaaaaaaaaaadaaaaaaabaaaaaaadadaaaaimaaaaaaacaaaaaa
aaaaaaaaadaaaaaaabaaaaaaamamaaaaimaaaaaaabaaaaaaaaaaaaaaadaaaaaa
acaaaaaaapalaaaaimaaaaaaadaaaaaaaaaaaaaaadaaaaaaadaaaaaaapapaaaa
fdfgfpfaepfdejfeejepeoaafeeffiedepepfceeaaklklklepfdeheocmaaaaaa
abaaaaaaaiaaaaaacaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaa
fdfgfpfegbhcghgfheaaklkl"
}

}
	}

#LINE 73

	}
	Fallback "Specular"
}
