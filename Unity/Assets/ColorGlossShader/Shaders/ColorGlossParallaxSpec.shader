Shader "ColorGloss/Parallax Specular" {
	Properties {
		_Color ("Main Color", Color) = (1,1,1,1)
		_Shininess ("Shininess", Range (0.03, 1)) = 0.078125
		_Gloss ("Gloss Power", Float) = 1
		_Parallax ("Height", Range (0.005, 0.08)) = 0.02
		_MainTex ("Base (RGB) Gloss (A)", 2D) = "white" {}
		_BumpMap ("Normalmap", 2D) = "bump" {}
		_SpecMap ("SpecMap", 2D) = "white" {}
		_ParallaxMap ("Heightmap (A)", 2D) = "black" {}
	}
	SubShader {
		Tags { "RenderType" = "Opaque" }
		LOD 400

			
	Pass {
		Name "FORWARD"
		Tags { "LightMode" = "ForwardBase" }
Program "vp" {
// Vertex combos: 9
//   opengl - ALU: 20 to 80
//   d3d9 - ALU: 21 to 83
//   d3d11 - ALU: 8 to 39, TEX: 0 to 0, FLOW: 1 to 1
//   d3d11_9x - ALU: 8 to 36, TEX: 0 to 0, FLOW: 1 to 1
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
Vector 24 [_BumpMap_ST]
"!!ARBvp1.0
# 44 ALU
PARAM c[25] = { { 1 },
		state.matrix.mvp,
		program.local[5..24] };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
MUL R1.xyz, vertex.normal, c[22].w;
DP3 R2.w, R1, c[6];
DP3 R0.x, R1, c[5];
DP3 R0.z, R1, c[7];
MOV R0.y, R2.w;
MUL R1, R0.xyzz, R0.yzzx;
MOV R0.w, c[0].x;
DP4 R2.z, R0, c[17];
DP4 R2.y, R0, c[16];
DP4 R2.x, R0, c[15];
MUL R0.y, R2.w, R2.w;
DP4 R3.z, R1, c[20];
DP4 R3.y, R1, c[19];
DP4 R3.x, R1, c[18];
ADD R2.xyz, R2, R3;
MAD R0.x, R0, R0, -R0.y;
MUL R3.xyz, R0.x, c[21];
MOV R1.xyz, vertex.attrib[14];
MUL R0.xyz, vertex.normal.zxyw, R1.yzxw;
MAD R1.xyz, vertex.normal.yzxw, R1.zxyw, -R0;
ADD result.texcoord[3].xyz, R2, R3;
MOV R0.xyz, c[13];
MOV R0.w, c[0].x;
DP4 R2.z, R0, c[11];
DP4 R2.x, R0, c[9];
DP4 R2.y, R0, c[10];
MAD R0.xyz, R2, c[22].w, -vertex.position;
MUL R2.xyz, R1, vertex.attrib[14].w;
MOV R1, c[14];
DP4 R3.z, R1, c[11];
DP4 R3.x, R1, c[9];
DP4 R3.y, R1, c[10];
DP3 result.texcoord[1].y, R0, R2;
DP3 result.texcoord[2].y, R2, R3;
DP3 result.texcoord[1].z, vertex.normal, R0;
DP3 result.texcoord[1].x, R0, vertex.attrib[14];
DP3 result.texcoord[2].z, vertex.normal, R3;
DP3 result.texcoord[2].x, vertex.attrib[14], R3;
MAD result.texcoord[0].zw, vertex.texcoord[0].xyxy, c[24].xyxy, c[24];
MAD result.texcoord[0].xy, vertex.texcoord[0], c[23], c[23].zwzw;
DP4 result.position.w, vertex.position, c[4];
DP4 result.position.z, vertex.position, c[3];
DP4 result.position.y, vertex.position, c[2];
DP4 result.position.x, vertex.position, c[1];
END
# 44 instructions, 4 R-regs
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
Vector 23 [_BumpMap_ST]
"vs_2_0
; 47 ALU
def c24, 1.00000000, 0, 0, 0
dcl_position0 v0
dcl_tangent0 v1
dcl_normal0 v2
dcl_texcoord0 v3
mul r1.xyz, v2, c21.w
dp3 r2.w, r1, c5
dp3 r0.x, r1, c4
dp3 r0.z, r1, c6
mov r0.y, r2.w
mul r1, r0.xyzz, r0.yzzx
mov r0.w, c24.x
dp4 r2.z, r0, c16
dp4 r2.y, r0, c15
dp4 r2.x, r0, c14
mul r0.y, r2.w, r2.w
dp4 r3.z, r1, c19
dp4 r3.y, r1, c18
dp4 r3.x, r1, c17
add r1.xyz, r2, r3
mad r0.x, r0, r0, -r0.y
mul r2.xyz, r0.x, c20
add oT3.xyz, r1, r2
mov r0.xyz, v1
mul r1.xyz, v2.zxyw, r0.yzxw
mov r0.xyz, v1
mad r0.xyz, v2.yzxw, r0.zxyw, -r1
mul r3.xyz, r0, v1.w
mov r0, c10
dp4 r4.z, c13, r0
mov r0, c9
mov r1.w, c24.x
mov r1.xyz, c12
dp4 r4.y, c13, r0
dp4 r2.z, r1, c10
dp4 r2.x, r1, c8
dp4 r2.y, r1, c9
mad r2.xyz, r2, c21.w, -v0
mov r1, c8
dp4 r4.x, c13, r1
dp3 oT1.y, r2, r3
dp3 oT2.y, r3, r4
dp3 oT1.z, v2, r2
dp3 oT1.x, r2, v1
dp3 oT2.z, v2, r4
dp3 oT2.x, v1, r4
mad oT0.zw, v3.xyxy, c23.xyxy, c23
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
Vector 22 [_BumpMap_ST]
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
// ALU: 48.00 (36 instructions), vertex: 32, texture: 0,
//   sequencer: 20,  9 GPRs, 21 threads,
// Performance (if enough threads): ~48 cycles per vector
// * Vertex cycle estimates are assuming 3 vfetch_minis for every vfetch_full,
//     with <= 32 bytes per vfetch_full group.

"vs_360
backbbabaaaaacomaaaaacciaaaaaaaaaaaaaaceaaaaaaaaaaaaaciaaaaaaaaa
aaaaaaaaaaaaacfiaaaaaabmaaaaacelpppoadaaaaaaaaapaaaaaabmaaaaaaaa
aaaaaceeaaaaabeiaaacaabgaaabaaaaaaaaabfeaaaaaaaaaaaaabgeaaacaabf
aaabaaaaaaaaabfeaaaaaaaaaaaaabhaaaacaaanaaadaaaaaaaaabiaaaaaaaaa
aaaaabjaaaacaabaaaaeaaaaaaaaabiaaaaaaaaaaaaaabjoaaacaaaaaaabaaaa
aaaaableaaaaaaaaaaaaabmeaaacaaabaaabaaaaaaaaabfeaaaaaaaaaaaaabnj
aaacaaajaaaeaaaaaaaaabiaaaaaaaaaaaaaabomaaacaaaeaaabaaaaaaaaabfe
aaaaaaaaaaaaabphaaacaaadaaabaaaaaaaaabfeaaaaaaaaaaaaacacaaacaaac
aaabaaaaaaaaabfeaaaaaaaaaaaaacanaaacaaahaaabaaaaaaaaabfeaaaaaaaa
aaaaacbiaaacaaagaaabaaaaaaaaabfeaaaaaaaaaaaaaccdaaacaaafaaabaaaa
aaaaabfeaaaaaaaaaaaaaccoaaacaaaiaaabaaaaaaaaabfeaaaaaaaaaaaaacdi
aaacaabeaaabaaaaaaaaabfeaaaaaaaafpechfgnhaengbhafpfdfeaaaaabaaad
aaabaaaeaaabaaaaaaaaaaaafpengbgjgofegfhifpfdfeaafpepgcgkgfgdhedc
fhgphcgmgeaaklklaaadaaadaaaeaaaeaaabaaaaaaaaaaaafpfhgphcgmgedcep
gcgkgfgdheaafpfhgphcgmgefdhagbgdgfedgbgngfhcgbfagphdaaklaaabaaad
aaabaaadaaabaaaaaaaaaaaafpfhgphcgmgefdhagbgdgfemgjghgihefagphdda
aaghgmhdhegbhegffpgngbhehcgjhifpgnhghaaahfgogjhehjfpfdeiebgcaahf
gogjhehjfpfdeiebghaahfgogjhehjfpfdeiebhcaahfgogjhehjfpfdeiecgcaa
hfgogjhehjfpfdeiecghaahfgogjhehjfpfdeiechcaahfgogjhehjfpfdeiedaa
hfgogjhehjfpfdgdgbgmgfaahghdfpddfpdaaadccodacodcdadddfddcodaaakl
aaaaaaaaaaaaacciaadbaaaiaaaaaaaaaaaaaaaaaaaadeieaaaaaaabaaaaaaae
aaaaaaajaaaaacjaaabaaaafaaaagaagaaaadaahaadafaaiaaaapafaaaachbfb
aaafhcfcaaaihdfdaaaaaaccaaaabacdaaaaaabmaaaaaabnaaaababoaaaaaabp
aaaaaacaaaaabacbaaaabacmpaffeaafaaaabcaamcaaaaaaaaaaeaajaaaabcaa
meaaaaaaaaaagaangabdbcaabcaaaaaaaaaagabjgabpbcaabcaaaaaaaaaagacf
caclbcaaccaaaaaaafpigaaaaaaaagiiaaaaaaaaafpifaaaaaaaagiiaaaaaaaa
afpibaaaaaaaaoiiaaaaaaaaafpiaaaaaaaaapmiaaaaaaaamiapaaacaabliiaa
kbagamaamiapaaacaamgiiaaklagalacmiapaaacaalbdejeklagakacmiapiado
aagmaadeklagajacmiahaaadaamamgmaalbcaabdmiahaaacaaleblaacbbdabaa
miahaaacaamamgleclbcabacmiahaaaeaalogfaaobabafaamiahaaaiaalelble
clbbaaadmialaaadaagfblaakbabbeaamiahaaahaalbleaakbadapaamiahaaai
aamagmleclbaaaaimiahaaaeabgflomaolabafaemiahaaacaalelbleclbbabac
miahaaacaamagmleclbaabacmiahaaaeaamablaaobaeafaamiahaaagabmablma
klaibeagmiahaaadaagmlemakladaoahmiahaaadaabllemakladanadmiabiaab
aaloloaapaagafaamiaciaabaaloloaapaaeagaamiaeiaabaaloloaapaagabaa
miabiaacaaloloaapaacafaamiaciaacaaloloaapaaeacaamiaeiaacaaloloaa
paacabaamiadiaaaaalalabkilaabfbfmiamiaaaaakmkmagilaabgbgceipadae
aalehcgmobadadiamiabaaacaadoanaagpacadaamiacaaacaadoanaagpadadaa
miaeaaacaadoanaagpaeadaamiabaaaaaakhkhaakpaeafaaaibcabaaaakhkhgm
kpaeagadaiceabaaaakhkhmgkpaeahadgeihaaaaaalologboaacaaabmiahiaad
aablmagfklaaaiaaaaaaaaaaaaaaaaaaaaaaaaaa"
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
Vector 456 [_BumpMap_ST]
"sce_vp_rsx // 42 instructions using 5 registers
[Configuration]
8
0000002a41050500
[Microcode]
672
00009c6c005d200d8186c0836041fffc00019c6c00400e0c0106c0836041dffc
00001c6c005d300c0186c0836041dffc00021c6c009ca20c013fc0c36041dffc
401f9c6c011c8800810040d560607f9c401f9c6c011c9808010400d740619f9c
401f9c6c01d0300d8106c0c360403f80401f9c6c01d0200d8106c0c360405f80
401f9c6c01d0100d8106c0c360409f80401f9c6c01d0000d8106c0c360411f80
00011c6c01d0a00d8286c0c360405ffc00011c6c01d0900d8286c0c360409ffc
00011c6c01d0800d8286c0c360411ffc00009c6c0150400c088600c360411ffc
00009c6c0150600c088600c360405ffc00001c6c0150500c088600c360403ffc
00021c6c0190a00c0086c0c360405ffc00021c6c0190900c0086c0c360409ffc
00021c6c0190800c0086c0c360411ffc00001c6c00800243011843436041dffc
00001c6c01000230812183630021dffc00019c6c011ca00c08bfc0e30041dffc
401f9c6c0140020c0106024360405fa4401f9c6c01400e0c0106024360411fa4
00009c6c0080007f80bfc04360403ffc00009c6c0040007f8086c08360409ffc
00021c6c00800e0c00bfc0836041dffc401f9c6c0140020c0106034360405fa0
401f9c6c01400e0c0686008360411fa000001c6c019cf00c0286c0c360405ffc
00001c6c019d000c0286c0c360409ffc00001c6c019d100c0286c0c360411ffc
00001c6c010000000280017fe0a03ffc00009c6c0080000d029a01436041fffc
401f9c6c0140000c0886024360409fa4401f9c6c0140000c0686044360409fa0
00011c6c01dcc00d8286c0c360405ffc00011c6c01dcd00d8286c0c360409ffc
00011c6c01dce00d8286c0c360411ffc00001c6c00c0000c0086c0830121dffc
00009c6c009cb07f808600c36041dffc401f9c6c00c0000c0286c0830021dfa9
"
}

SubProgram "d3d11 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 112 // 112 used size, 9 vars
Vector 80 [_MainTex_ST] 4
Vector 96 [_BumpMap_ST] 4
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
// 41 instructions, 4 temp regs, 0 temp arrays:
// ALU 23 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0
eefiecedonjjcopgmfffiaoiiibpbbahhnklgaooabaaaaaanaahaaaaadaaaaaa
cmaaaaaapeaaaaaajeabaaaaejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaa
abaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaljaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfc
enebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheojiaaaaaaafaaaaaa
aiaaaaaaiaaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaabaaaaaaapaaaaaaimaaaaaaabaaaaaaaaaaaaaa
adaaaaaaacaaaaaaahaiaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaa
ahaiaaaaimaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaahaiaaaafdfgfpfa
epfdejfeejepeoaafeeffiedepepfceeaaklklklfdeieefcdeagaaaaeaaaabaa
inabaaaafjaaaaaeegiocaaaaaaaaaaaahaaaaaafjaaaaaeegiocaaaabaaaaaa
afaaaaaafjaaaaaeegiocaaaacaaaaaabjaaaaaafjaaaaaeegiocaaaadaaaaaa
bfaaaaaafpaaaaadpcbabaaaaaaaaaaafpaaaaadpcbabaaaabaaaaaafpaaaaad
hcbabaaaacaaaaaafpaaaaaddcbabaaaadaaaaaaghaaaaaepccabaaaaaaaaaaa
abaaaaaagfaaaaadpccabaaaabaaaaaagfaaaaadhccabaaaacaaaaaagfaaaaad
hccabaaaadaaaaaagfaaaaadhccabaaaaeaaaaaagiaaaaacaeaaaaaadiaaaaai
pcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaadaaaaaaabaaaaaadcaaaaak
pcaabaaaaaaaaaaaegiocaaaadaaaaaaaaaaaaaaagbabaaaaaaaaaaaegaobaaa
aaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaacaaaaaakgbkbaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaakpccabaaaaaaaaaaaegiocaaaadaaaaaa
adaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaaldccabaaaabaaaaaa
egbabaaaadaaaaaaegiacaaaaaaaaaaaafaaaaaaogikcaaaaaaaaaaaafaaaaaa
dcaaaaalmccabaaaabaaaaaaagbebaaaadaaaaaaagiecaaaaaaaaaaaagaaaaaa
kgiocaaaaaaaaaaaagaaaaaadiaaaaahhcaabaaaaaaaaaaajgbebaaaabaaaaaa
cgbjbaaaacaaaaaadcaaaaakhcaabaaaaaaaaaaajgbebaaaacaaaaaacgbjbaaa
abaaaaaaegacbaiaebaaaaaaaaaaaaaadiaaaaahhcaabaaaaaaaaaaaegacbaaa
aaaaaaaapgbpbaaaabaaaaaadiaaaaajhcaabaaaabaaaaaafgifcaaaabaaaaaa
aeaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaa
adaaaaaabaaaaaaaagiacaaaabaaaaaaaeaaaaaaegacbaaaabaaaaaadcaaaaal
hcaabaaaabaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaaabaaaaaaaeaaaaaa
egacbaaaabaaaaaaaaaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaaegiccaaa
adaaaaaabdaaaaaadcaaaaalhcaabaaaabaaaaaaegacbaaaabaaaaaapgipcaaa
adaaaaaabeaaaaaaegbcbaiaebaaaaaaaaaaaaaabaaaaaahcccabaaaacaaaaaa
egacbaaaaaaaaaaaegacbaaaabaaaaaabaaaaaahbccabaaaacaaaaaaegbcbaaa
abaaaaaaegacbaaaabaaaaaabaaaaaaheccabaaaacaaaaaaegbcbaaaacaaaaaa
egacbaaaabaaaaaadiaaaaajhcaabaaaabaaaaaafgifcaaaacaaaaaaaaaaaaaa
egiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaa
baaaaaaaagiacaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaa
abaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaaacaaaaaaaaaaaaaaegacbaaa
abaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabdaaaaaapgipcaaa
acaaaaaaaaaaaaaaegacbaaaabaaaaaabaaaaaahcccabaaaadaaaaaaegacbaaa
aaaaaaaaegacbaaaabaaaaaabaaaaaahbccabaaaadaaaaaaegbcbaaaabaaaaaa
egacbaaaabaaaaaabaaaaaaheccabaaaadaaaaaaegbcbaaaacaaaaaaegacbaaa
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
aaaaaaaadcaaaaakhccabaaaaeaaaaaaegiccaaaacaaaaaabiaaaaaaagaabaaa
aaaaaaaaegacbaaaabaaaaaadoaaaaab"
}

SubProgram "gles " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
"!!GLES
#define SHADER_API_GLES 1
#define tex2D texture2D


#ifdef VERTEX
#define gl_ModelViewProjectionMatrix glstate_matrix_mvp
uniform mat4 glstate_matrix_mvp;

varying lowp vec3 xlv_TEXCOORD3;
varying lowp vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 _BumpMap_ST;
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
  highp vec4 tmpvar_4;
  lowp vec3 tmpvar_5;
  lowp vec3 tmpvar_6;
  tmpvar_4.xy = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  tmpvar_4.zw = ((_glesMultiTexCoord0.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
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
  tmpvar_5 = tmpvar_11;
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
  tmpvar_6 = shlight_3;
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = tmpvar_4;
  xlv_TEXCOORD1 = (tmpvar_10 * (((_World2Object * tmpvar_12).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = tmpvar_5;
  xlv_TEXCOORD3 = tmpvar_6;
}



#endif
#ifdef FRAGMENT

varying lowp vec3 xlv_TEXCOORD3;
varying lowp vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp float _Parallax;
uniform mediump float _Gloss;
uniform mediump float _Shininess;
uniform lowp vec4 _Color;
uniform sampler2D _ParallaxMap;
uniform sampler2D _SpecMap;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform lowp vec4 _LightColor0;
void main ()
{
  lowp vec4 c_1;
  mediump vec3 tmpvar_2;
  mediump vec3 tmpvar_3;
  mediump vec4 spec_4;
  mediump float h_5;
  lowp float tmpvar_6;
  tmpvar_6 = texture2D (_ParallaxMap, xlv_TEXCOORD0.zw).w;
  h_5 = tmpvar_6;
  highp vec2 tmpvar_7;
  mediump float height_8;
  height_8 = _Parallax;
  mediump vec3 viewDir_9;
  viewDir_9 = xlv_TEXCOORD1;
  highp vec3 v_10;
  mediump float tmpvar_11;
  tmpvar_11 = ((h_5 * height_8) - (height_8 / 2.0));
  mediump vec3 tmpvar_12;
  tmpvar_12 = normalize(viewDir_9);
  v_10 = tmpvar_12;
  v_10.z = (v_10.z + 0.42);
  tmpvar_7 = (tmpvar_11 * (v_10.xy / v_10.z));
  highp vec2 tmpvar_13;
  tmpvar_13 = (xlv_TEXCOORD0.xy + tmpvar_7);
  highp vec2 tmpvar_14;
  tmpvar_14 = (xlv_TEXCOORD0.zw + tmpvar_7);
  lowp vec4 tmpvar_15;
  tmpvar_15 = texture2D (_MainTex, tmpvar_13);
  lowp vec3 tmpvar_16;
  tmpvar_16 = (tmpvar_15.xyz * _Color.xyz);
  tmpvar_2 = tmpvar_16;
  lowp vec4 tmpvar_17;
  tmpvar_17 = texture2D (_SpecMap, tmpvar_13);
  spec_4 = tmpvar_17;
  mediump vec3 tmpvar_18;
  tmpvar_18 = ((tmpvar_15.w * spec_4.xyz) * _Gloss);
  lowp vec3 tmpvar_19;
  tmpvar_19 = ((texture2D (_BumpMap, tmpvar_14).xyz * 2.0) - 1.0);
  tmpvar_3 = tmpvar_19;
  highp vec3 tmpvar_20;
  tmpvar_20 = normalize(xlv_TEXCOORD1);
  mediump vec3 lightDir_21;
  lightDir_21 = xlv_TEXCOORD2;
  mediump vec3 viewDir_22;
  viewDir_22 = tmpvar_20;
  mediump vec4 c_23;
  mediump vec3 specCol_24;
  highp float nh_25;
  mediump float tmpvar_26;
  tmpvar_26 = max (0.0, dot (tmpvar_3, normalize((lightDir_21 + viewDir_22))));
  nh_25 = tmpvar_26;
  mediump float arg1_27;
  arg1_27 = (32.0 * _Shininess);
  highp vec3 tmpvar_28;
  tmpvar_28 = (pow (nh_25, arg1_27) * tmpvar_18);
  specCol_24 = tmpvar_28;
  c_23.xyz = ((((tmpvar_2 * _LightColor0.xyz) * max (0.0, dot (tmpvar_3, lightDir_21))) + (_LightColor0.xyz * specCol_24)) * 2.0);
  c_23.w = 0.0;
  c_1 = c_23;
  mediump vec3 tmpvar_29;
  tmpvar_29 = (c_1.xyz + (tmpvar_2 * xlv_TEXCOORD3));
  c_1.xyz = tmpvar_29;
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

varying lowp vec3 xlv_TEXCOORD3;
varying lowp vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 _BumpMap_ST;
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
  highp vec4 tmpvar_4;
  lowp vec3 tmpvar_5;
  lowp vec3 tmpvar_6;
  tmpvar_4.xy = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  tmpvar_4.zw = ((_glesMultiTexCoord0.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
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
  tmpvar_5 = tmpvar_11;
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
  tmpvar_6 = shlight_3;
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = tmpvar_4;
  xlv_TEXCOORD1 = (tmpvar_10 * (((_World2Object * tmpvar_12).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = tmpvar_5;
  xlv_TEXCOORD3 = tmpvar_6;
}



#endif
#ifdef FRAGMENT

varying lowp vec3 xlv_TEXCOORD3;
varying lowp vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp float _Parallax;
uniform mediump float _Gloss;
uniform mediump float _Shininess;
uniform lowp vec4 _Color;
uniform sampler2D _ParallaxMap;
uniform sampler2D _SpecMap;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform lowp vec4 _LightColor0;
void main ()
{
  lowp vec4 c_1;
  mediump vec3 tmpvar_2;
  mediump vec3 tmpvar_3;
  mediump vec4 spec_4;
  mediump float h_5;
  lowp float tmpvar_6;
  tmpvar_6 = texture2D (_ParallaxMap, xlv_TEXCOORD0.zw).w;
  h_5 = tmpvar_6;
  highp vec2 tmpvar_7;
  mediump float height_8;
  height_8 = _Parallax;
  mediump vec3 viewDir_9;
  viewDir_9 = xlv_TEXCOORD1;
  highp vec3 v_10;
  mediump float tmpvar_11;
  tmpvar_11 = ((h_5 * height_8) - (height_8 / 2.0));
  mediump vec3 tmpvar_12;
  tmpvar_12 = normalize(viewDir_9);
  v_10 = tmpvar_12;
  v_10.z = (v_10.z + 0.42);
  tmpvar_7 = (tmpvar_11 * (v_10.xy / v_10.z));
  highp vec2 tmpvar_13;
  tmpvar_13 = (xlv_TEXCOORD0.xy + tmpvar_7);
  highp vec2 tmpvar_14;
  tmpvar_14 = (xlv_TEXCOORD0.zw + tmpvar_7);
  lowp vec4 tmpvar_15;
  tmpvar_15 = texture2D (_MainTex, tmpvar_13);
  lowp vec3 tmpvar_16;
  tmpvar_16 = (tmpvar_15.xyz * _Color.xyz);
  tmpvar_2 = tmpvar_16;
  lowp vec4 tmpvar_17;
  tmpvar_17 = texture2D (_SpecMap, tmpvar_13);
  spec_4 = tmpvar_17;
  mediump vec3 tmpvar_18;
  tmpvar_18 = ((tmpvar_15.w * spec_4.xyz) * _Gloss);
  lowp vec3 normal_19;
  normal_19.xy = ((texture2D (_BumpMap, tmpvar_14).wy * 2.0) - 1.0);
  normal_19.z = sqrt(((1.0 - (normal_19.x * normal_19.x)) - (normal_19.y * normal_19.y)));
  tmpvar_3 = normal_19;
  highp vec3 tmpvar_20;
  tmpvar_20 = normalize(xlv_TEXCOORD1);
  mediump vec3 lightDir_21;
  lightDir_21 = xlv_TEXCOORD2;
  mediump vec3 viewDir_22;
  viewDir_22 = tmpvar_20;
  mediump vec4 c_23;
  mediump vec3 specCol_24;
  highp float nh_25;
  mediump float tmpvar_26;
  tmpvar_26 = max (0.0, dot (tmpvar_3, normalize((lightDir_21 + viewDir_22))));
  nh_25 = tmpvar_26;
  mediump float arg1_27;
  arg1_27 = (32.0 * _Shininess);
  highp vec3 tmpvar_28;
  tmpvar_28 = (pow (nh_25, arg1_27) * tmpvar_18);
  specCol_24 = tmpvar_28;
  c_23.xyz = ((((tmpvar_2 * _LightColor0.xyz) * max (0.0, dot (tmpvar_3, lightDir_21))) + (_LightColor0.xyz * specCol_24)) * 2.0);
  c_23.w = 0.0;
  c_1 = c_23;
  mediump vec3 tmpvar_29;
  tmpvar_29 = (c_1.xyz + (tmpvar_2 * xlv_TEXCOORD3));
  c_1.xyz = tmpvar_29;
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
Vector 23 [_BumpMap_ST]
"agal_vs
c24 1.0 0.0 0.0 0.0
[bc]
adaaaaaaabaaahacabaaaaoeaaaaaaaabfaaaappabaaaaaa mul r1.xyz, a1, c21.w
bcaaaaaaacaaaiacabaaaakeacaaaaaaafaaaaoeabaaaaaa dp3 r2.w, r1.xyzz, c5
bcaaaaaaaaaaabacabaaaakeacaaaaaaaeaaaaoeabaaaaaa dp3 r0.x, r1.xyzz, c4
bcaaaaaaaaaaaeacabaaaakeacaaaaaaagaaaaoeabaaaaaa dp3 r0.z, r1.xyzz, c6
aaaaaaaaaaaaacacacaaaappacaaaaaaaaaaaaaaaaaaaaaa mov r0.y, r2.w
adaaaaaaabaaapacaaaaaakeacaaaaaaaaaaaacjacaaaaaa mul r1, r0.xyzz, r0.yzzx
aaaaaaaaaaaaaiacbiaaaaaaabaaaaaaaaaaaaaaaaaaaaaa mov r0.w, c24.x
bdaaaaaaacaaaeacaaaaaaoeacaaaaaabaaaaaoeabaaaaaa dp4 r2.z, r0, c16
bdaaaaaaacaaacacaaaaaaoeacaaaaaaapaaaaoeabaaaaaa dp4 r2.y, r0, c15
bdaaaaaaacaaabacaaaaaaoeacaaaaaaaoaaaaoeabaaaaaa dp4 r2.x, r0, c14
adaaaaaaaaaaacacacaaaappacaaaaaaacaaaappacaaaaaa mul r0.y, r2.w, r2.w
bdaaaaaaadaaaeacabaaaaoeacaaaaaabdaaaaoeabaaaaaa dp4 r3.z, r1, c19
bdaaaaaaadaaacacabaaaaoeacaaaaaabcaaaaoeabaaaaaa dp4 r3.y, r1, c18
bdaaaaaaadaaabacabaaaaoeacaaaaaabbaaaaoeabaaaaaa dp4 r3.x, r1, c17
abaaaaaaabaaahacacaaaakeacaaaaaaadaaaakeacaaaaaa add r1.xyz, r2.xyzz, r3.xyzz
adaaaaaaadaaaiacaaaaaaaaacaaaaaaaaaaaaaaacaaaaaa mul r3.w, r0.x, r0.x
acaaaaaaaaaaabacadaaaappacaaaaaaaaaaaaffacaaaaaa sub r0.x, r3.w, r0.y
adaaaaaaacaaahacaaaaaaaaacaaaaaabeaaaaoeabaaaaaa mul r2.xyz, r0.x, c20
abaaaaaaadaaahaeabaaaakeacaaaaaaacaaaakeacaaaaaa add v3.xyz, r1.xyzz, r2.xyzz
aaaaaaaaaaaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, a5
adaaaaaaabaaahacabaaaancaaaaaaaaaaaaaaajacaaaaaa mul r1.xyz, a1.zxyw, r0.yzxx
aaaaaaaaaaaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, a5
adaaaaaaaeaaahacabaaaamjaaaaaaaaaaaaaafcacaaaaaa mul r4.xyz, a1.yzxw, r0.zxyy
acaaaaaaaaaaahacaeaaaakeacaaaaaaabaaaakeacaaaaaa sub r0.xyz, r4.xyzz, r1.xyzz
adaaaaaaadaaahacaaaaaakeacaaaaaaafaaaappaaaaaaaa mul r3.xyz, r0.xyzz, a5.w
aaaaaaaaaaaaapacakaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0, c10
bdaaaaaaaeaaaeacanaaaaoeabaaaaaaaaaaaaoeacaaaaaa dp4 r4.z, c13, r0
aaaaaaaaaaaaapacajaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0, c9
aaaaaaaaabaaaiacbiaaaaaaabaaaaaaaaaaaaaaaaaaaaaa mov r1.w, c24.x
aaaaaaaaabaaahacamaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r1.xyz, c12
bdaaaaaaaeaaacacanaaaaoeabaaaaaaaaaaaaoeacaaaaaa dp4 r4.y, c13, r0
bdaaaaaaacaaaeacabaaaaoeacaaaaaaakaaaaoeabaaaaaa dp4 r2.z, r1, c10
bdaaaaaaacaaabacabaaaaoeacaaaaaaaiaaaaoeabaaaaaa dp4 r2.x, r1, c8
bdaaaaaaacaaacacabaaaaoeacaaaaaaajaaaaoeabaaaaaa dp4 r2.y, r1, c9
adaaaaaaaaaaahacacaaaakeacaaaaaabfaaaappabaaaaaa mul r0.xyz, r2.xyzz, c21.w
acaaaaaaacaaahacaaaaaakeacaaaaaaaaaaaaoeaaaaaaaa sub r2.xyz, r0.xyzz, a0
aaaaaaaaabaaapacaiaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r1, c8
bdaaaaaaaeaaabacanaaaaoeabaaaaaaabaaaaoeacaaaaaa dp4 r4.x, c13, r1
bcaaaaaaabaaacaeacaaaakeacaaaaaaadaaaakeacaaaaaa dp3 v1.y, r2.xyzz, r3.xyzz
bcaaaaaaacaaacaeadaaaakeacaaaaaaaeaaaakeacaaaaaa dp3 v2.y, r3.xyzz, r4.xyzz
bcaaaaaaabaaaeaeabaaaaoeaaaaaaaaacaaaakeacaaaaaa dp3 v1.z, a1, r2.xyzz
bcaaaaaaabaaabaeacaaaakeacaaaaaaafaaaaoeaaaaaaaa dp3 v1.x, r2.xyzz, a5
bcaaaaaaacaaaeaeabaaaaoeaaaaaaaaaeaaaakeacaaaaaa dp3 v2.z, a1, r4.xyzz
bcaaaaaaacaaabaeafaaaaoeaaaaaaaaaeaaaakeacaaaaaa dp3 v2.x, a5, r4.xyzz
adaaaaaaaaaaamacadaaaaeeaaaaaaaabhaaaaeeabaaaaaa mul r0.zw, a3.xyxy, c23.xyxy
abaaaaaaaaaaamaeaaaaaaopacaaaaaabhaaaaoeabaaaaaa add v0.zw, r0.wwzw, c23
adaaaaaaaaaaadacadaaaaoeaaaaaaaabgaaaaoeabaaaaaa mul r0.xy, a3, c22
abaaaaaaaaaaadaeaaaaaafeacaaaaaabgaaaaooabaaaaaa add v0.xy, r0.xyyy, c22.zwzw
bdaaaaaaaaaaaiadaaaaaaoeaaaaaaaaadaaaaoeabaaaaaa dp4 o0.w, a0, c3
bdaaaaaaaaaaaeadaaaaaaoeaaaaaaaaacaaaaoeabaaaaaa dp4 o0.z, a0, c2
bdaaaaaaaaaaacadaaaaaaoeaaaaaaaaabaaaaoeabaaaaaa dp4 o0.y, a0, c1
bdaaaaaaaaaaabadaaaaaaoeaaaaaaaaaaaaaaoeabaaaaaa dp4 o0.x, a0, c0
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
ConstBuffer "$Globals" 112 // 112 used size, 9 vars
Vector 80 [_MainTex_ST] 4
Vector 96 [_BumpMap_ST] 4
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
// 41 instructions, 4 temp regs, 0 temp arrays:
// ALU 23 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0_level_9_3
eefieceddemfgfgdhagloelbpjnamhbomelbdbfnabaaaaaakialaaaaaeaaaaaa
daaaaaaaaeaeaaaaeaakaaaaaialaaaaebgpgodjmmadaaaammadaaaaaaacpopp
faadaaaahmaaaaaaahaaceaaaaaahiaaaaaahiaaaaaaceaaabaahiaaaaaaafaa
acaaabaaaaaaaaaaabaaaeaaabaaadaaaaaaaaaaacaaaaaaabaaaeaaaaaaaaaa
acaabcaaahaaafaaaaaaaaaaadaaaaaaaeaaamaaaaaaaaaaadaaamaaadaabaaa
aaaaaaaaadaabaaaafaabdaaaaaaaaaaaaaaaaaaabacpoppfbaaaaafbiaaapka
aaaaiadpaaaaaaaaaaaaaaaaaaaaaaaabpaaaaacafaaaaiaaaaaapjabpaaaaac
afaaabiaabaaapjabpaaaaacafaaaciaacaaapjabpaaaaacafaaadiaadaaapja
aeaaaaaeaaaaadoaadaaoejaabaaoekaabaaookaaeaaaaaeaaaaamoaadaaeeja
acaaeekaacaaoekaabaaaaacaaaaapiaaeaaoekaafaaaaadabaaahiaaaaaffia
beaaoekaaeaaaaaeabaaahiabdaaoekaaaaaaaiaabaaoeiaaeaaaaaeaaaaahia
bfaaoekaaaaakkiaabaaoeiaaeaaaaaeaaaaahiabgaaoekaaaaappiaaaaaoeia
aiaaaaadacaaaboaabaaoejaaaaaoeiaabaaaaacabaaahiaacaaoejaafaaaaad
acaaahiaabaanciaabaamjjaaeaaaaaeabaaahiaabaamjiaabaancjaacaaoeib
afaaaaadabaaahiaabaaoeiaabaappjaaiaaaaadacaaacoaabaaoeiaaaaaoeia
aiaaaaadacaaaeoaacaaoejaaaaaoeiaabaaaaacaaaaahiaadaaoekaafaaaaad
acaaahiaaaaaffiabeaaoekaaeaaaaaeaaaaaliabdaakekaaaaaaaiaacaakeia
aeaaaaaeaaaaahiabfaaoekaaaaakkiaaaaapeiaacaaaaadaaaaahiaaaaaoeia
bgaaoekaaeaaaaaeaaaaahiaaaaaoeiabhaappkaaaaaoejbaiaaaaadabaaaboa
abaaoejaaaaaoeiaaiaaaaadabaaacoaabaaoeiaaaaaoeiaaiaaaaadabaaaeoa
acaaoejaaaaaoeiaafaaaaadaaaaahiaacaaoejabhaappkaafaaaaadabaaahia
aaaaffiabbaaoekaaeaaaaaeaaaaaliabaaakekaaaaaaaiaabaakeiaaeaaaaae
aaaaahiabcaaoekaaaaakkiaaaaapeiaabaaaaacaaaaaiiabiaaaakaajaaaaad
abaaabiaafaaoekaaaaaoeiaajaaaaadabaaaciaagaaoekaaaaaoeiaajaaaaad
abaaaeiaahaaoekaaaaaoeiaafaaaaadacaaapiaaaaacjiaaaaakeiaajaaaaad
adaaabiaaiaaoekaacaaoeiaajaaaaadadaaaciaajaaoekaacaaoeiaajaaaaad
adaaaeiaakaaoekaacaaoeiaacaaaaadabaaahiaabaaoeiaadaaoeiaafaaaaad
aaaaaciaaaaaffiaaaaaffiaaeaaaaaeaaaaabiaaaaaaaiaaaaaaaiaaaaaffib
aeaaaaaeadaaahoaalaaoekaaaaaaaiaabaaoeiaafaaaaadaaaaapiaaaaaffja
anaaoekaaeaaaaaeaaaaapiaamaaoekaaaaaaajaaaaaoeiaaeaaaaaeaaaaapia
aoaaoekaaaaakkjaaaaaoeiaaeaaaaaeaaaaapiaapaaoekaaaaappjaaaaaoeia
aeaaaaaeaaaaadmaaaaappiaaaaaoekaaaaaoeiaabaaaaacaaaaammaaaaaoeia
ppppaaaafdeieefcdeagaaaaeaaaabaainabaaaafjaaaaaeegiocaaaaaaaaaaa
ahaaaaaafjaaaaaeegiocaaaabaaaaaaafaaaaaafjaaaaaeegiocaaaacaaaaaa
bjaaaaaafjaaaaaeegiocaaaadaaaaaabfaaaaaafpaaaaadpcbabaaaaaaaaaaa
fpaaaaadpcbabaaaabaaaaaafpaaaaadhcbabaaaacaaaaaafpaaaaaddcbabaaa
adaaaaaaghaaaaaepccabaaaaaaaaaaaabaaaaaagfaaaaadpccabaaaabaaaaaa
gfaaaaadhccabaaaacaaaaaagfaaaaadhccabaaaadaaaaaagfaaaaadhccabaaa
aeaaaaaagiaaaaacaeaaaaaadiaaaaaipcaabaaaaaaaaaaafgbfbaaaaaaaaaaa
egiocaaaadaaaaaaabaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaa
aaaaaaaaagbabaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaa
egiocaaaadaaaaaaacaaaaaakgbkbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaak
pccabaaaaaaaaaaaegiocaaaadaaaaaaadaaaaaapgbpbaaaaaaaaaaaegaobaaa
aaaaaaaadcaaaaaldccabaaaabaaaaaaegbabaaaadaaaaaaegiacaaaaaaaaaaa
afaaaaaaogikcaaaaaaaaaaaafaaaaaadcaaaaalmccabaaaabaaaaaaagbebaaa
adaaaaaaagiecaaaaaaaaaaaagaaaaaakgiocaaaaaaaaaaaagaaaaaadiaaaaah
hcaabaaaaaaaaaaajgbebaaaabaaaaaacgbjbaaaacaaaaaadcaaaaakhcaabaaa
aaaaaaaajgbebaaaacaaaaaacgbjbaaaabaaaaaaegacbaiaebaaaaaaaaaaaaaa
diaaaaahhcaabaaaaaaaaaaaegacbaaaaaaaaaaapgbpbaaaabaaaaaadiaaaaaj
hcaabaaaabaaaaaafgifcaaaabaaaaaaaeaaaaaaegiccaaaadaaaaaabbaaaaaa
dcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaaabaaaaaa
aeaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaa
bcaaaaaakgikcaaaabaaaaaaaeaaaaaaegacbaaaabaaaaaaaaaaaaaihcaabaaa
abaaaaaaegacbaaaabaaaaaaegiccaaaadaaaaaabdaaaaaadcaaaaalhcaabaaa
abaaaaaaegacbaaaabaaaaaapgipcaaaadaaaaaabeaaaaaaegbcbaiaebaaaaaa
aaaaaaaabaaaaaahcccabaaaacaaaaaaegacbaaaaaaaaaaaegacbaaaabaaaaaa
baaaaaahbccabaaaacaaaaaaegbcbaaaabaaaaaaegacbaaaabaaaaaabaaaaaah
eccabaaaacaaaaaaegbcbaaaacaaaaaaegacbaaaabaaaaaadiaaaaajhcaabaaa
abaaaaaafgifcaaaacaaaaaaaaaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaal
hcaabaaaabaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaaacaaaaaaaaaaaaaa
egacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabcaaaaaa
kgikcaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaa
egiccaaaadaaaaaabdaaaaaapgipcaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaa
baaaaaahcccabaaaadaaaaaaegacbaaaaaaaaaaaegacbaaaabaaaaaabaaaaaah
bccabaaaadaaaaaaegbcbaaaabaaaaaaegacbaaaabaaaaaabaaaaaaheccabaaa
adaaaaaaegbcbaaaacaaaaaaegacbaaaabaaaaaadiaaaaaihcaabaaaaaaaaaaa
egbcbaaaacaaaaaapgipcaaaadaaaaaabeaaaaaadiaaaaaihcaabaaaabaaaaaa
fgafbaaaaaaaaaaaegiccaaaadaaaaaaanaaaaaadcaaaaaklcaabaaaaaaaaaaa
egiicaaaadaaaaaaamaaaaaaagaabaaaaaaaaaaaegaibaaaabaaaaaadcaaaaak
hcaabaaaaaaaaaaaegiccaaaadaaaaaaaoaaaaaakgakbaaaaaaaaaaaegadbaaa
aaaaaaaadgaaaaaficaabaaaaaaaaaaaabeaaaaaaaaaiadpbbaaaaaibcaabaaa
abaaaaaaegiocaaaacaaaaaabcaaaaaaegaobaaaaaaaaaaabbaaaaaiccaabaaa
abaaaaaaegiocaaaacaaaaaabdaaaaaaegaobaaaaaaaaaaabbaaaaaiecaabaaa
abaaaaaaegiocaaaacaaaaaabeaaaaaaegaobaaaaaaaaaaadiaaaaahpcaabaaa
acaaaaaajgacbaaaaaaaaaaaegakbaaaaaaaaaaabbaaaaaibcaabaaaadaaaaaa
egiocaaaacaaaaaabfaaaaaaegaobaaaacaaaaaabbaaaaaiccaabaaaadaaaaaa
egiocaaaacaaaaaabgaaaaaaegaobaaaacaaaaaabbaaaaaiecaabaaaadaaaaaa
egiocaaaacaaaaaabhaaaaaaegaobaaaacaaaaaaaaaaaaahhcaabaaaabaaaaaa
egacbaaaabaaaaaaegacbaaaadaaaaaadiaaaaahccaabaaaaaaaaaaabkaabaaa
aaaaaaaabkaabaaaaaaaaaaadcaaaaakbcaabaaaaaaaaaaaakaabaaaaaaaaaaa
akaabaaaaaaaaaaabkaabaiaebaaaaaaaaaaaaaadcaaaaakhccabaaaaeaaaaaa
egiccaaaacaaaaaabiaaaaaaagaabaaaaaaaaaaaegacbaaaabaaaaaadoaaaaab
ejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaaaaaaaaaaaaaaaaaaadaaaaaa
aaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaaadaaaaaaabaaaaaaapapaaaa
kjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaaahahaaaalaaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaaabaaaaaaaaaaaaaaadaaaaaa
aeaaaaaaapaaaaaaljaaaaaaaaaaaaaaaaaaaaaaadaaaaaaafaaaaaaapaaaaaa
faepfdejfeejepeoaafeebeoehefeofeaaeoepfcenebemaafeeffiedepepfcee
aaedepemepfcaaklepfdeheojiaaaaaaafaaaaaaaiaaaaaaiaaaaaaaaaaaaaaa
abaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaaaaaaaaaaaaaaaaaaadaaaaaa
abaaaaaaapaaaaaaimaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaaahaiaaaa
imaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaahaiaaaaimaaaaaaadaaaaaa
aaaaaaaaadaaaaaaaeaaaaaaahaiaaaafdfgfpfaepfdejfeejepeoaafeeffied
epepfceeaaklklkl"
}

SubProgram "opengl " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Bind "vertex" Vertex
Bind "tangent" ATTR14
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Vector 13 [_WorldSpaceCameraPos]
Matrix 9 [_World2Object]
Vector 15 [unity_Scale]
Vector 16 [unity_LightmapST]
Vector 17 [_MainTex_ST]
Vector 18 [_BumpMap_ST]
"!!ARBvp1.0
# 20 ALU
PARAM c[19] = { { 1 },
		state.matrix.mvp,
		program.local[5..18] };
TEMP R0;
TEMP R1;
TEMP R2;
MOV R0.xyz, vertex.attrib[14];
MUL R1.xyz, vertex.normal.zxyw, R0.yzxw;
MAD R0.xyz, vertex.normal.yzxw, R0.zxyw, -R1;
MUL R1.xyz, R0, vertex.attrib[14].w;
MOV R0.xyz, c[13];
MOV R0.w, c[0].x;
DP4 R2.z, R0, c[11];
DP4 R2.x, R0, c[9];
DP4 R2.y, R0, c[10];
MAD R0.xyz, R2, c[15].w, -vertex.position;
DP3 result.texcoord[1].y, R0, R1;
DP3 result.texcoord[1].z, vertex.normal, R0;
DP3 result.texcoord[1].x, R0, vertex.attrib[14];
MAD result.texcoord[0].zw, vertex.texcoord[0].xyxy, c[18].xyxy, c[18];
MAD result.texcoord[0].xy, vertex.texcoord[0], c[17], c[17].zwzw;
MAD result.texcoord[2].xy, vertex.texcoord[1], c[16], c[16].zwzw;
DP4 result.position.w, vertex.position, c[4];
DP4 result.position.z, vertex.position, c[3];
DP4 result.position.y, vertex.position, c[2];
DP4 result.position.x, vertex.position, c[1];
END
# 20 instructions, 3 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Matrix 0 [glstate_matrix_mvp]
Vector 12 [_WorldSpaceCameraPos]
Matrix 8 [_World2Object]
Vector 13 [unity_Scale]
Vector 14 [unity_LightmapST]
Vector 15 [_MainTex_ST]
Vector 16 [_BumpMap_ST]
"vs_2_0
; 21 ALU
def c17, 1.00000000, 0, 0, 0
dcl_position0 v0
dcl_tangent0 v1
dcl_normal0 v2
dcl_texcoord0 v3
dcl_texcoord1 v4
mov r0.xyz, v1
mul r1.xyz, v2.zxyw, r0.yzxw
mov r0.xyz, v1
mad r0.xyz, v2.yzxw, r0.zxyw, -r1
mul r1.xyz, r0, v1.w
mov r0.xyz, c12
mov r0.w, c17.x
dp4 r2.z, r0, c10
dp4 r2.x, r0, c8
dp4 r2.y, r0, c9
mad r0.xyz, r2, c13.w, -v0
dp3 oT1.y, r0, r1
dp3 oT1.z, v2, r0
dp3 oT1.x, r0, v1
mad oT0.zw, v3.xyxy, c16.xyxy, c16
mad oT0.xy, v3, c15, c15.zwzw
mad oT2.xy, v4, c14, c14.zwzw
dp4 oPos.w, v0, c3
dp4 oPos.z, v0, c2
dp4 oPos.y, v0, c1
dp4 oPos.x, v0, c0
"
}

SubProgram "xbox360 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Vector 12 [_BumpMap_ST]
Vector 11 [_MainTex_ST]
Matrix 5 [_World2Object] 4
Vector 0 [_WorldSpaceCameraPos]
Matrix 1 [glstate_matrix_mvp] 4
Vector 10 [unity_LightmapST]
Vector 9 [unity_Scale]
// Shader Timing Estimate, in Cycles/64 vertex vector:
// ALU: 22.67 (17 instructions), vertex: 64, texture: 0,
//   sequencer: 14,  6 GPRs, 30 threads,
// Performance (if enough threads): ~64 cycles per vector
// * Vertex cycle estimates are assuming 3 vfetch_minis for every vfetch_full,
//     with <= 32 bytes per vfetch_full group.

"vs_360
backbbabaaaaaboeaaaaabeeaaaaaaaaaaaaaaceaaaaaaaaaaaaabieaaaaaaaa
aaaaaaaaaaaaabfmaaaaaabmaaaaabeppppoadaaaaaaaaahaaaaaabmaaaaaaaa
aaaaabeiaaaaaakiaaacaaamaaabaaaaaaaaaaleaaaaaaaaaaaaaameaaacaaal
aaabaaaaaaaaaaleaaaaaaaaaaaaaanaaaacaaafaaaeaaaaaaaaaaoaaaaaaaaa
aaaaaapaaaacaaaaaaabaaaaaaaaabaiaaaaaaaaaaaaabbiaaacaaabaaaeaaaa
aaaaaaoaaaaaaaaaaaaaabclaaacaaakaaabaaaaaaaaaaleaaaaaaaaaaaaabdm
aaacaaajaaabaaaaaaaaaaleaaaaaaaafpechfgnhaengbhafpfdfeaaaaabaaad
aaabaaaeaaabaaaaaaaaaaaafpengbgjgofegfhifpfdfeaafpfhgphcgmgedcep
gcgkgfgdheaaklklaaadaaadaaaeaaaeaaabaaaaaaaaaaaafpfhgphcgmgefdha
gbgdgfedgbgngfhcgbfagphdaaklklklaaabaaadaaabaaadaaabaaaaaaaaaaaa
ghgmhdhegbhegffpgngbhehcgjhifpgnhghaaahfgogjhehjfpemgjghgihegngb
hafdfeaahfgogjhehjfpfdgdgbgmgfaahghdfpddfpdaaadccodacodcdadddfdd
codaaaklaaaaaaaaaaaaabeeaacbaaafaaaaaaaaaaaaaaaaaaaacegdaaaaaaab
aaaaaaafaaaaaaagaaaaacjaaabaaaaeaaaagaafaaaadaagaaaafaahaadbfaai
aaaapafaaaachbfbaaafdcfcaaaaaabiaaaababjaaaaaabeaaaaaabfaaaababg
aaaababhpbfffaaeaaaabcabmcaaaaaaaaaaeaajaaaabcaameaaaaaaaaaagaan
gabdbcaabcaaaaaaaaaababjaaaaccaaaaaaaaaaafpicaaaaaaaagiiaaaaaaaa
afpieaaaaaaaagiiaaaaaaaaafpibaaaaaaaaoiiaaaaaaaaafpiaaaaaaaaaoeh
aaaaaaaaafpiaaaaaaaaadpiaaaaaaaamiapaaadaabliiaakbacaeaamiapaaad
aamgiiaaklacadadmiapaaadaalbdejeklacacadmiapiadoaagmaadeklacabad
miahaaafaamamgmaalahaaaimiahaaadaalogfaaobabaeaamiahaaafaalelble
clagaaafmiahaaafaamagmleclafaaafmiahaaadabgflomaolabaeadmiahaaad
aamablaaobadaeaamiahaaacabmablmaklafajacmiabiaabaaloloaapaacaeaa
miaciaabaaloloaapaadacaamiaeiaabaaloloaapaacabaamiadiaacaabilabk
ilaaakakmiadiaaaaamflabkilaaalalmiamiaaaaapbkmagilaaamamaaaaaaaa
aaaaaaaaaaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Matrix 256 [glstate_matrix_mvp]
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Vector 467 [_WorldSpaceCameraPos]
Matrix 264 [_World2Object]
Vector 466 [unity_Scale]
Vector 465 [unity_LightmapST]
Vector 464 [_MainTex_ST]
Vector 463 [_BumpMap_ST]
"sce_vp_rsx // 19 instructions using 3 registers
[Configuration]
8
0000001343050300
[Microcode]
304
00009c6c00400e0c0106c0836041dffc00011c6c005d300c0186c0836041dffc
401f9c6c011cf800810040d560607f9c401f9c6c011d0808010400d740619f9c
401f9c6c011d1908010400d740619fa4401f9c6c01d0300d8106c0c360403f80
401f9c6c01d0200d8106c0c360405f80401f9c6c01d0100d8106c0c360409f80
401f9c6c01d0000d8106c0c360411f8000001c6c0190a00c0486c0c360405ffc
00001c6c0190900c0486c0c360409ffc00001c6c0190800c0486c0c360411ffc
00011c6c00800243011841436041dffc00009c6c01000230812181630121dffc
00001c6c011d200c00bfc0e30041dffc00009c6c00800e0c02bfc0836041dffc
401f9c6c0140020c0106004360405fa0401f9c6c01400e0c0086008360411fa0
401f9c6c0140000c0086014360409fa1
"
}

SubProgram "d3d11 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Bind "color" Color
ConstBuffer "$Globals" 128 // 128 used size, 10 vars
Vector 80 [unity_LightmapST] 4
Vector 96 [_MainTex_ST] 4
Vector 112 [_BumpMap_ST] 4
ConstBuffer "UnityPerCamera" 128 // 76 used size, 8 vars
Vector 64 [_WorldSpaceCameraPos] 3
ConstBuffer "UnityPerDraw" 336 // 336 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
Matrix 256 [_World2Object] 4
Vector 320 [unity_Scale] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityPerDraw" 2
// 19 instructions, 2 temp regs, 0 temp arrays:
// ALU 8 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0
eefiecedlckajnjphoaajglgklambbhfcphgjhlfabaaaaaanaaeaaaaadaaaaaa
cmaaaaaapeaaaaaahmabaaaaejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaa
abaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapadaaaaljaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfc
enebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheoiaaaaaaaaeaaaaaa
aiaaaaaagiaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaheaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaabaaaaaaapaaaaaaheaaaaaaabaaaaaaaaaaaaaa
adaaaaaaacaaaaaaahaiaaaaheaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaa
adamaaaafdfgfpfaepfdejfeejepeoaafeeffiedepepfceeaaklklklfdeieefc
emadaaaaeaaaabaandaaaaaafjaaaaaeegiocaaaaaaaaaaaaiaaaaaafjaaaaae
egiocaaaabaaaaaaafaaaaaafjaaaaaeegiocaaaacaaaaaabfaaaaaafpaaaaad
pcbabaaaaaaaaaaafpaaaaadpcbabaaaabaaaaaafpaaaaadhcbabaaaacaaaaaa
fpaaaaaddcbabaaaadaaaaaafpaaaaaddcbabaaaaeaaaaaaghaaaaaepccabaaa
aaaaaaaaabaaaaaagfaaaaadpccabaaaabaaaaaagfaaaaadhccabaaaacaaaaaa
gfaaaaaddccabaaaadaaaaaagiaaaaacacaaaaaadiaaaaaipcaabaaaaaaaaaaa
fgbfbaaaaaaaaaaaegiocaaaacaaaaaaabaaaaaadcaaaaakpcaabaaaaaaaaaaa
egiocaaaacaaaaaaaaaaaaaaagbabaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaak
pcaabaaaaaaaaaaaegiocaaaacaaaaaaacaaaaaakgbkbaaaaaaaaaaaegaobaaa
aaaaaaaadcaaaaakpccabaaaaaaaaaaaegiocaaaacaaaaaaadaaaaaapgbpbaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaaldccabaaaabaaaaaaegbabaaaadaaaaaa
egiacaaaaaaaaaaaagaaaaaaogikcaaaaaaaaaaaagaaaaaadcaaaaalmccabaaa
abaaaaaaagbebaaaadaaaaaaagiecaaaaaaaaaaaahaaaaaakgiocaaaaaaaaaaa
ahaaaaaadiaaaaahhcaabaaaaaaaaaaajgbebaaaabaaaaaacgbjbaaaacaaaaaa
dcaaaaakhcaabaaaaaaaaaaajgbebaaaacaaaaaacgbjbaaaabaaaaaaegacbaia
ebaaaaaaaaaaaaaadiaaaaahhcaabaaaaaaaaaaaegacbaaaaaaaaaaapgbpbaaa
abaaaaaadiaaaaajhcaabaaaabaaaaaafgifcaaaabaaaaaaaeaaaaaaegiccaaa
acaaaaaabbaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaacaaaaaabaaaaaaa
agiacaaaabaaaaaaaeaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaa
egiccaaaacaaaaaabcaaaaaakgikcaaaabaaaaaaaeaaaaaaegacbaaaabaaaaaa
aaaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaaegiccaaaacaaaaaabdaaaaaa
dcaaaaalhcaabaaaabaaaaaaegacbaaaabaaaaaapgipcaaaacaaaaaabeaaaaaa
egbcbaiaebaaaaaaaaaaaaaabaaaaaahcccabaaaacaaaaaaegacbaaaaaaaaaaa
egacbaaaabaaaaaabaaaaaahbccabaaaacaaaaaaegbcbaaaabaaaaaaegacbaaa
abaaaaaabaaaaaaheccabaaaacaaaaaaegbcbaaaacaaaaaaegacbaaaabaaaaaa
dcaaaaaldccabaaaadaaaaaaegbabaaaaeaaaaaaegiacaaaaaaaaaaaafaaaaaa
ogikcaaaaaaaaaaaafaaaaaadoaaaaab"
}

SubProgram "gles " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
"!!GLES
#define SHADER_API_GLES 1
#define tex2D texture2D


#ifdef VERTEX
#define gl_ModelViewProjectionMatrix glstate_matrix_mvp
uniform mat4 glstate_matrix_mvp;

varying highp vec2 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 _BumpMap_ST;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 unity_LightmapST;
uniform highp vec4 unity_Scale;
uniform highp mat4 _World2Object;

uniform highp vec3 _WorldSpaceCameraPos;
attribute vec4 _glesTANGENT;
attribute vec4 _glesMultiTexCoord1;
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
  highp vec4 tmpvar_3;
  tmpvar_3.xy = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  tmpvar_3.zw = ((_glesMultiTexCoord0.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
  highp vec3 tmpvar_4;
  highp vec3 tmpvar_5;
  tmpvar_4 = tmpvar_1.xyz;
  tmpvar_5 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_6;
  tmpvar_6[0].x = tmpvar_4.x;
  tmpvar_6[0].y = tmpvar_5.x;
  tmpvar_6[0].z = tmpvar_2.x;
  tmpvar_6[1].x = tmpvar_4.y;
  tmpvar_6[1].y = tmpvar_5.y;
  tmpvar_6[1].z = tmpvar_2.y;
  tmpvar_6[2].x = tmpvar_4.z;
  tmpvar_6[2].y = tmpvar_5.z;
  tmpvar_6[2].z = tmpvar_2.z;
  highp vec4 tmpvar_7;
  tmpvar_7.w = 1.0;
  tmpvar_7.xyz = _WorldSpaceCameraPos;
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = tmpvar_3;
  xlv_TEXCOORD1 = (tmpvar_6 * (((_World2Object * tmpvar_7).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = ((_glesMultiTexCoord1.xy * unity_LightmapST.xy) + unity_LightmapST.zw);
}



#endif
#ifdef FRAGMENT

varying highp vec2 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform sampler2D unity_Lightmap;
uniform highp float _Parallax;
uniform lowp vec4 _Color;
uniform sampler2D _ParallaxMap;
uniform sampler2D _MainTex;
void main ()
{
  lowp vec4 c_1;
  mediump vec3 tmpvar_2;
  mediump float h_3;
  lowp float tmpvar_4;
  tmpvar_4 = texture2D (_ParallaxMap, xlv_TEXCOORD0.zw).w;
  h_3 = tmpvar_4;
  mediump float height_5;
  height_5 = _Parallax;
  mediump vec3 viewDir_6;
  viewDir_6 = xlv_TEXCOORD1;
  highp vec3 v_7;
  mediump float tmpvar_8;
  tmpvar_8 = ((h_3 * height_5) - (height_5 / 2.0));
  mediump vec3 tmpvar_9;
  tmpvar_9 = normalize(viewDir_6);
  v_7 = tmpvar_9;
  v_7.z = (v_7.z + 0.42);
  highp vec2 tmpvar_10;
  tmpvar_10 = (xlv_TEXCOORD0.xy + (tmpvar_8 * (v_7.xy / v_7.z)));
  lowp vec3 tmpvar_11;
  tmpvar_11 = (texture2D (_MainTex, tmpvar_10).xyz * _Color.xyz);
  tmpvar_2 = tmpvar_11;
  lowp vec3 tmpvar_12;
  tmpvar_12 = (2.0 * texture2D (unity_Lightmap, xlv_TEXCOORD2).xyz);
  mediump vec3 tmpvar_13;
  tmpvar_13 = (tmpvar_2 * tmpvar_12);
  c_1.xyz = tmpvar_13;
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

varying highp vec2 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 _BumpMap_ST;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 unity_LightmapST;
uniform highp vec4 unity_Scale;
uniform highp mat4 _World2Object;

uniform highp vec3 _WorldSpaceCameraPos;
attribute vec4 _glesTANGENT;
attribute vec4 _glesMultiTexCoord1;
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
  highp vec4 tmpvar_3;
  tmpvar_3.xy = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  tmpvar_3.zw = ((_glesMultiTexCoord0.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
  highp vec3 tmpvar_4;
  highp vec3 tmpvar_5;
  tmpvar_4 = tmpvar_1.xyz;
  tmpvar_5 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_6;
  tmpvar_6[0].x = tmpvar_4.x;
  tmpvar_6[0].y = tmpvar_5.x;
  tmpvar_6[0].z = tmpvar_2.x;
  tmpvar_6[1].x = tmpvar_4.y;
  tmpvar_6[1].y = tmpvar_5.y;
  tmpvar_6[1].z = tmpvar_2.y;
  tmpvar_6[2].x = tmpvar_4.z;
  tmpvar_6[2].y = tmpvar_5.z;
  tmpvar_6[2].z = tmpvar_2.z;
  highp vec4 tmpvar_7;
  tmpvar_7.w = 1.0;
  tmpvar_7.xyz = _WorldSpaceCameraPos;
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = tmpvar_3;
  xlv_TEXCOORD1 = (tmpvar_6 * (((_World2Object * tmpvar_7).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = ((_glesMultiTexCoord1.xy * unity_LightmapST.xy) + unity_LightmapST.zw);
}



#endif
#ifdef FRAGMENT

varying highp vec2 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform sampler2D unity_Lightmap;
uniform highp float _Parallax;
uniform lowp vec4 _Color;
uniform sampler2D _ParallaxMap;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
void main ()
{
  lowp vec4 c_1;
  mediump vec3 tmpvar_2;
  mediump float h_3;
  lowp float tmpvar_4;
  tmpvar_4 = texture2D (_ParallaxMap, xlv_TEXCOORD0.zw).w;
  h_3 = tmpvar_4;
  highp vec2 tmpvar_5;
  mediump float height_6;
  height_6 = _Parallax;
  mediump vec3 viewDir_7;
  viewDir_7 = xlv_TEXCOORD1;
  highp vec3 v_8;
  mediump float tmpvar_9;
  tmpvar_9 = ((h_3 * height_6) - (height_6 / 2.0));
  mediump vec3 tmpvar_10;
  tmpvar_10 = normalize(viewDir_7);
  v_8 = tmpvar_10;
  v_8.z = (v_8.z + 0.42);
  tmpvar_5 = (tmpvar_9 * (v_8.xy / v_8.z));
  highp vec2 tmpvar_11;
  tmpvar_11 = (xlv_TEXCOORD0.xy + tmpvar_5);
  highp vec2 tmpvar_12;
  tmpvar_12 = (xlv_TEXCOORD0.zw + tmpvar_5);
  lowp vec3 tmpvar_13;
  tmpvar_13 = (texture2D (_MainTex, tmpvar_11).xyz * _Color.xyz);
  tmpvar_2 = tmpvar_13;
  lowp vec3 normal_14;
  normal_14.xy = ((texture2D (_BumpMap, tmpvar_12).wy * 2.0) - 1.0);
  normal_14.z = sqrt(((1.0 - (normal_14.x * normal_14.x)) - (normal_14.y * normal_14.y)));
  lowp vec4 tmpvar_15;
  tmpvar_15 = texture2D (unity_Lightmap, xlv_TEXCOORD2);
  lowp vec3 tmpvar_16;
  tmpvar_16 = ((8.0 * tmpvar_15.w) * tmpvar_15.xyz);
  mediump vec3 tmpvar_17;
  tmpvar_17 = (tmpvar_2 * tmpvar_16);
  c_1.xyz = tmpvar_17;
  c_1.w = 0.0;
  gl_FragData[0] = c_1;
}



#endif"
}

SubProgram "flash " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Matrix 0 [glstate_matrix_mvp]
Vector 12 [_WorldSpaceCameraPos]
Matrix 8 [_World2Object]
Vector 13 [unity_Scale]
Vector 14 [unity_LightmapST]
Vector 15 [_MainTex_ST]
Vector 16 [_BumpMap_ST]
"agal_vs
c17 1.0 0.0 0.0 0.0
[bc]
aaaaaaaaaaaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, a5
adaaaaaaabaaahacabaaaancaaaaaaaaaaaaaaajacaaaaaa mul r1.xyz, a1.zxyw, r0.yzxx
aaaaaaaaaaaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, a5
adaaaaaaacaaahacabaaaamjaaaaaaaaaaaaaafcacaaaaaa mul r2.xyz, a1.yzxw, r0.zxyy
acaaaaaaaaaaahacacaaaakeacaaaaaaabaaaakeacaaaaaa sub r0.xyz, r2.xyzz, r1.xyzz
adaaaaaaabaaahacaaaaaakeacaaaaaaafaaaappaaaaaaaa mul r1.xyz, r0.xyzz, a5.w
aaaaaaaaaaaaahacamaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, c12
aaaaaaaaaaaaaiacbbaaaaaaabaaaaaaaaaaaaaaaaaaaaaa mov r0.w, c17.x
bdaaaaaaacaaaeacaaaaaaoeacaaaaaaakaaaaoeabaaaaaa dp4 r2.z, r0, c10
bdaaaaaaacaaabacaaaaaaoeacaaaaaaaiaaaaoeabaaaaaa dp4 r2.x, r0, c8
bdaaaaaaacaaacacaaaaaaoeacaaaaaaajaaaaoeabaaaaaa dp4 r2.y, r0, c9
adaaaaaaacaaahacacaaaakeacaaaaaaanaaaappabaaaaaa mul r2.xyz, r2.xyzz, c13.w
acaaaaaaaaaaahacacaaaakeacaaaaaaaaaaaaoeaaaaaaaa sub r0.xyz, r2.xyzz, a0
bcaaaaaaabaaacaeaaaaaakeacaaaaaaabaaaakeacaaaaaa dp3 v1.y, r0.xyzz, r1.xyzz
bcaaaaaaabaaaeaeabaaaaoeaaaaaaaaaaaaaakeacaaaaaa dp3 v1.z, a1, r0.xyzz
bcaaaaaaabaaabaeaaaaaakeacaaaaaaafaaaaoeaaaaaaaa dp3 v1.x, r0.xyzz, a5
adaaaaaaaaaaamacadaaaaeeaaaaaaaabaaaaaeeabaaaaaa mul r0.zw, a3.xyxy, c16.xyxy
abaaaaaaaaaaamaeaaaaaaopacaaaaaabaaaaaoeabaaaaaa add v0.zw, r0.wwzw, c16
adaaaaaaaaaaadacadaaaaoeaaaaaaaaapaaaaoeabaaaaaa mul r0.xy, a3, c15
abaaaaaaaaaaadaeaaaaaafeacaaaaaaapaaaaooabaaaaaa add v0.xy, r0.xyyy, c15.zwzw
adaaaaaaaaaaadacaeaaaaoeaaaaaaaaaoaaaaoeabaaaaaa mul r0.xy, a4, c14
abaaaaaaacaaadaeaaaaaafeacaaaaaaaoaaaaooabaaaaaa add v2.xy, r0.xyyy, c14.zwzw
bdaaaaaaaaaaaiadaaaaaaoeaaaaaaaaadaaaaoeabaaaaaa dp4 o0.w, a0, c3
bdaaaaaaaaaaaeadaaaaaaoeaaaaaaaaacaaaaoeabaaaaaa dp4 o0.z, a0, c2
bdaaaaaaaaaaacadaaaaaaoeaaaaaaaaabaaaaoeabaaaaaa dp4 o0.y, a0, c1
bdaaaaaaaaaaabadaaaaaaoeaaaaaaaaaaaaaaoeabaaaaaa dp4 o0.x, a0, c0
aaaaaaaaabaaaiaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v1.w, c0
aaaaaaaaacaaamaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v2.zw, c0
"
}

SubProgram "d3d11_9x " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Bind "color" Color
ConstBuffer "$Globals" 128 // 128 used size, 10 vars
Vector 80 [unity_LightmapST] 4
Vector 96 [_MainTex_ST] 4
Vector 112 [_BumpMap_ST] 4
ConstBuffer "UnityPerCamera" 128 // 76 used size, 8 vars
Vector 64 [_WorldSpaceCameraPos] 3
ConstBuffer "UnityPerDraw" 336 // 336 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
Matrix 256 [_World2Object] 4
Vector 320 [unity_Scale] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityPerDraw" 2
// 19 instructions, 2 temp regs, 0 temp arrays:
// ALU 8 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0_level_9_3
eefiecedkgleieekpnpfkaihkjmkabkgjdigoloaabaaaaaapiagaaaaaeaaaaaa
daaaaaaafeacaaaakiafaaaahaagaaaaebgpgodjbmacaaaabmacaaaaaaacpopp
meabaaaafiaaaaaaaeaaceaaaaaafeaaaaaafeaaaaaaceaaabaafeaaaaaaafaa
adaaabaaaaaaaaaaabaaaeaaabaaaeaaaaaaaaaaacaaaaaaaeaaafaaaaaaaaaa
acaabaaaafaaajaaaaaaaaaaaaaaaaaaabacpoppbpaaaaacafaaaaiaaaaaapja
bpaaaaacafaaabiaabaaapjabpaaaaacafaaaciaacaaapjabpaaaaacafaaadia
adaaapjabpaaaaacafaaaeiaaeaaapjaaeaaaaaeaaaaadoaadaaoejaacaaoeka
acaaookaaeaaaaaeaaaaamoaadaaeejaadaaeekaadaaoekaaeaaaaaeacaaadoa
aeaaoejaabaaoekaabaaookaabaaaaacaaaaahiaaeaaoekaafaaaaadabaaahia
aaaaffiaakaaoekaaeaaaaaeaaaaaliaajaakekaaaaaaaiaabaakeiaaeaaaaae
aaaaahiaalaaoekaaaaakkiaaaaapeiaacaaaaadaaaaahiaaaaaoeiaamaaoeka
aeaaaaaeaaaaahiaaaaaoeiaanaappkaaaaaoejbaiaaaaadabaaaboaabaaoeja
aaaaoeiaabaaaaacabaaahiaabaaoejaafaaaaadacaaahiaabaamjiaacaancja
aeaaaaaeabaaahiaacaamjjaabaanciaacaaoeibafaaaaadabaaahiaabaaoeia
abaappjaaiaaaaadabaaacoaabaaoeiaaaaaoeiaaiaaaaadabaaaeoaacaaoeja
aaaaoeiaafaaaaadaaaaapiaaaaaffjaagaaoekaaeaaaaaeaaaaapiaafaaoeka
aaaaaajaaaaaoeiaaeaaaaaeaaaaapiaahaaoekaaaaakkjaaaaaoeiaaeaaaaae
aaaaapiaaiaaoekaaaaappjaaaaaoeiaaeaaaaaeaaaaadmaaaaappiaaaaaoeka
aaaaoeiaabaaaaacaaaaammaaaaaoeiappppaaaafdeieefcemadaaaaeaaaabaa
ndaaaaaafjaaaaaeegiocaaaaaaaaaaaaiaaaaaafjaaaaaeegiocaaaabaaaaaa
afaaaaaafjaaaaaeegiocaaaacaaaaaabfaaaaaafpaaaaadpcbabaaaaaaaaaaa
fpaaaaadpcbabaaaabaaaaaafpaaaaadhcbabaaaacaaaaaafpaaaaaddcbabaaa
adaaaaaafpaaaaaddcbabaaaaeaaaaaaghaaaaaepccabaaaaaaaaaaaabaaaaaa
gfaaaaadpccabaaaabaaaaaagfaaaaadhccabaaaacaaaaaagfaaaaaddccabaaa
adaaaaaagiaaaaacacaaaaaadiaaaaaipcaabaaaaaaaaaaafgbfbaaaaaaaaaaa
egiocaaaacaaaaaaabaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaacaaaaaa
aaaaaaaaagbabaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaa
egiocaaaacaaaaaaacaaaaaakgbkbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaak
pccabaaaaaaaaaaaegiocaaaacaaaaaaadaaaaaapgbpbaaaaaaaaaaaegaobaaa
aaaaaaaadcaaaaaldccabaaaabaaaaaaegbabaaaadaaaaaaegiacaaaaaaaaaaa
agaaaaaaogikcaaaaaaaaaaaagaaaaaadcaaaaalmccabaaaabaaaaaaagbebaaa
adaaaaaaagiecaaaaaaaaaaaahaaaaaakgiocaaaaaaaaaaaahaaaaaadiaaaaah
hcaabaaaaaaaaaaajgbebaaaabaaaaaacgbjbaaaacaaaaaadcaaaaakhcaabaaa
aaaaaaaajgbebaaaacaaaaaacgbjbaaaabaaaaaaegacbaiaebaaaaaaaaaaaaaa
diaaaaahhcaabaaaaaaaaaaaegacbaaaaaaaaaaapgbpbaaaabaaaaaadiaaaaaj
hcaabaaaabaaaaaafgifcaaaabaaaaaaaeaaaaaaegiccaaaacaaaaaabbaaaaaa
dcaaaaalhcaabaaaabaaaaaaegiccaaaacaaaaaabaaaaaaaagiacaaaabaaaaaa
aeaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaacaaaaaa
bcaaaaaakgikcaaaabaaaaaaaeaaaaaaegacbaaaabaaaaaaaaaaaaaihcaabaaa
abaaaaaaegacbaaaabaaaaaaegiccaaaacaaaaaabdaaaaaadcaaaaalhcaabaaa
abaaaaaaegacbaaaabaaaaaapgipcaaaacaaaaaabeaaaaaaegbcbaiaebaaaaaa
aaaaaaaabaaaaaahcccabaaaacaaaaaaegacbaaaaaaaaaaaegacbaaaabaaaaaa
baaaaaahbccabaaaacaaaaaaegbcbaaaabaaaaaaegacbaaaabaaaaaabaaaaaah
eccabaaaacaaaaaaegbcbaaaacaaaaaaegacbaaaabaaaaaadcaaaaaldccabaaa
adaaaaaaegbabaaaaeaaaaaaegiacaaaaaaaaaaaafaaaaaaogikcaaaaaaaaaaa
afaaaaaadoaaaaabejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaaadaaaaaa
abaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaaahahaaaa
laaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaaabaaaaaa
aaaaaaaaadaaaaaaaeaaaaaaapadaaaaljaaaaaaaaaaaaaaaaaaaaaaadaaaaaa
afaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfcenebemaa
feeffiedepepfceeaaedepemepfcaaklepfdeheoiaaaaaaaaeaaaaaaaiaaaaaa
giaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaheaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaabaaaaaaapaaaaaaheaaaaaaabaaaaaaaaaaaaaaadaaaaaa
acaaaaaaahaiaaaaheaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaadamaaaa
fdfgfpfaepfdejfeejepeoaafeeffiedepepfceeaaklklkl"
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
Vector 25 [_BumpMap_ST]
"!!ARBvp1.0
# 49 ALU
PARAM c[26] = { { 1, 0.5 },
		state.matrix.mvp,
		program.local[5..25] };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
MUL R1.xyz, vertex.normal, c[23].w;
DP3 R2.w, R1, c[6];
DP3 R0.x, R1, c[5];
DP3 R0.z, R1, c[7];
MOV R0.y, R2.w;
MUL R1, R0.xyzz, R0.yzzx;
MOV R0.w, c[0].x;
DP4 R2.z, R0, c[18];
DP4 R2.y, R0, c[17];
DP4 R2.x, R0, c[16];
MUL R0.y, R2.w, R2.w;
DP4 R3.z, R1, c[21];
DP4 R3.y, R1, c[20];
DP4 R3.x, R1, c[19];
ADD R2.xyz, R2, R3;
MAD R0.x, R0, R0, -R0.y;
MUL R3.xyz, R0.x, c[22];
MOV R1.xyz, vertex.attrib[14];
MUL R0.xyz, vertex.normal.zxyw, R1.yzxw;
MAD R1.xyz, vertex.normal.yzxw, R1.zxyw, -R0;
ADD result.texcoord[3].xyz, R2, R3;
MOV R0.w, c[0].x;
MOV R0.xyz, c[13];
DP4 R2.z, R0, c[11];
DP4 R2.x, R0, c[9];
DP4 R2.y, R0, c[10];
MAD R0.xyz, R2, c[23].w, -vertex.position;
MUL R2.xyz, R1, vertex.attrib[14].w;
MOV R1, c[15];
DP4 R3.z, R1, c[11];
DP4 R3.x, R1, c[9];
DP4 R3.y, R1, c[10];
DP3 result.texcoord[1].y, R0, R2;
DP3 result.texcoord[1].z, vertex.normal, R0;
DP3 result.texcoord[1].x, R0, vertex.attrib[14];
DP4 R0.w, vertex.position, c[4];
DP4 R0.z, vertex.position, c[3];
DP4 R0.x, vertex.position, c[1];
DP4 R0.y, vertex.position, c[2];
MUL R1.xyz, R0.xyww, c[0].y;
MUL R1.y, R1, c[14].x;
DP3 result.texcoord[2].y, R2, R3;
DP3 result.texcoord[2].z, vertex.normal, R3;
DP3 result.texcoord[2].x, vertex.attrib[14], R3;
ADD result.texcoord[4].xy, R1, R1.z;
MOV result.position, R0;
MOV result.texcoord[4].zw, R0;
MAD result.texcoord[0].zw, vertex.texcoord[0].xyxy, c[25].xyxy, c[25];
MAD result.texcoord[0].xy, vertex.texcoord[0], c[24], c[24].zwzw;
END
# 49 instructions, 4 R-regs
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
Vector 25 [_BumpMap_ST]
"vs_2_0
; 52 ALU
def c26, 1.00000000, 0.50000000, 0, 0
dcl_position0 v0
dcl_tangent0 v1
dcl_normal0 v2
dcl_texcoord0 v3
mul r1.xyz, v2, c23.w
dp3 r2.w, r1, c5
dp3 r0.x, r1, c4
dp3 r0.z, r1, c6
mov r0.y, r2.w
mul r1, r0.xyzz, r0.yzzx
mov r0.w, c26.x
dp4 r2.z, r0, c18
dp4 r2.y, r0, c17
dp4 r2.x, r0, c16
mul r0.y, r2.w, r2.w
dp4 r3.z, r1, c21
dp4 r3.y, r1, c20
dp4 r3.x, r1, c19
add r1.xyz, r2, r3
mad r0.x, r0, r0, -r0.y
mul r2.xyz, r0.x, c22
add oT3.xyz, r1, r2
mov r0.xyz, v1
mul r1.xyz, v2.zxyw, r0.yzxw
mov r0.xyz, v1
mad r0.xyz, v2.yzxw, r0.zxyw, -r1
mul r3.xyz, r0, v1.w
mov r0, c10
dp4 r4.z, c15, r0
mov r0, c9
dp4 r4.y, c15, r0
mov r1.w, c26.x
mov r1.xyz, c12
dp4 r0.w, v0, c3
dp4 r0.z, v0, c2
dp4 r2.z, r1, c10
dp4 r2.x, r1, c8
dp4 r2.y, r1, c9
mad r2.xyz, r2, c23.w, -v0
mov r1, c8
dp4 r4.x, c15, r1
dp4 r0.x, v0, c0
dp4 r0.y, v0, c1
mul r1.xyz, r0.xyww, c26.y
mul r1.y, r1, c13.x
dp3 oT1.y, r2, r3
dp3 oT2.y, r3, r4
dp3 oT1.z, v2, r2
dp3 oT1.x, r2, v1
dp3 oT2.z, v2, r4
dp3 oT2.x, v1, r4
mad oT4.xy, r1.z, c14.zwzw, r1
mov oPos, r0
mov oT4.zw, r0
mad oT0.zw, v3.xyxy, c25.xyxy, c25
mad oT0.xy, v3, c24, c24.zwzw
"
}

SubProgram "xbox360 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 24 [_BumpMap_ST]
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
// ALU: 53.33 (40 instructions), vertex: 32, texture: 0,
//   sequencer: 20,  10 GPRs, 18 threads,
// Performance (if enough threads): ~53 cycles per vector
// * Vertex cycle estimates are assuming 3 vfetch_minis for every vfetch_full,
//     with <= 32 bytes per vfetch_full group.

"vs_360
backbbabaaaaadgiaaaaacjiaaaaaaaaaaaaaaceaaaaacmiaaaaacpaaaaaaaaa
aaaaaaaaaaaaackaaaaaaabmaaaaacjdpppoadaaaaaaaabbaaaaaabmaaaaaaaa
aaaaacimaaaaabhaaaacaabiaaabaaaaaaaaabhmaaaaaaaaaaaaabimaaacaabh
aaabaaaaaaaaabhmaaaaaaaaaaaaabjiaaacaaapaaadaaaaaaaaabkiaaaaaaaa
aaaaabliaaacaaabaaabaaaaaaaaabhmaaaaaaaaaaaaabmkaaacaaacaaabaaaa
aaaaabhmaaaaaaaaaaaaabniaaacaabcaaaeaaaaaaaaabkiaaaaaaaaaaaaabog
aaacaaaaaaabaaaaaaaaabpmaaaaaaaaaaaaacamaaacaaadaaabaaaaaaaaabhm
aaaaaaaaaaaaaccbaaacaaalaaaeaaaaaaaaabkiaaaaaaaaaaaaacdeaaacaaag
aaabaaaaaaaaabhmaaaaaaaaaaaaacdpaaacaaafaaabaaaaaaaaabhmaaaaaaaa
aaaaacekaaacaaaeaaabaaaaaaaaabhmaaaaaaaaaaaaacffaaacaaajaaabaaaa
aaaaabhmaaaaaaaaaaaaacgaaaacaaaiaaabaaaaaaaaabhmaaaaaaaaaaaaacgl
aaacaaahaaabaaaaaaaaabhmaaaaaaaaaaaaachgaaacaaakaaabaaaaaaaaabhm
aaaaaaaaaaaaaciaaaacaabgaaabaaaaaaaaabhmaaaaaaaafpechfgnhaengbha
fpfdfeaaaaabaaadaaabaaaeaaabaaaaaaaaaaaafpengbgjgofegfhifpfdfeaa
fpepgcgkgfgdhedcfhgphcgmgeaaklklaaadaaadaaaeaaaeaaabaaaaaaaaaaaa
fpfahcgpgkgfgdhegjgpgofagbhcgbgnhdaafpfdgdhcgfgfgofagbhcgbgnhdaa
fpfhgphcgmgedcepgcgkgfgdheaafpfhgphcgmgefdhagbgdgfedgbgngfhcgbfa
gphdaaklaaabaaadaaabaaadaaabaaaaaaaaaaaafpfhgphcgmgefdhagbgdgfem
gjghgihefagphddaaaghgmhdhegbhegffpgngbhehcgjhifpgnhghaaahfgogjhe
hjfpfdeiebgcaahfgogjhehjfpfdeiebghaahfgogjhehjfpfdeiebhcaahfgogj
hehjfpfdeiecgcaahfgogjhehjfpfdeiecghaahfgogjhehjfpfdeiechcaahfgo
gjhehjfpfdeiedaahfgogjhehjfpfdgdgbgmgfaahghdfpddfpdaaadccodacodc
dadddfddcodaaaklaaaaaaaaaaaaaaabaaaaaaaaaaaaaaaaaaaaaabeaapmaaba
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaeaaaaaacfiaaebaaajaaaaaaaa
aaaaaaaaaaaaeekfaaaaaaabaaaaaaaeaaaaaaalaaaaacjaaabaaaafaaaagaag
aaaadaahaadafaaiaaaapafaaaachbfbaaafhcfcaaaihdfdaaajpefeaaaaaacf
aaaabacgaaaaaabpaaaaaacaaaaabacbaaaaaaccaaaaaacdaaaabaceaaaabada
aaaaaaboaaaabacoaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaadpaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaapaffeaafaaaabcaamcaaaaaaaaaafaajaaaabcaameaaaaaa
aaaagaaogabebcaabcaaaaaaaaaagabkgacabcaabcaaaaaaaaaagacgfacmbcaa
ccaaaaaaafpihaaaaaaaagiiaaaaaaaaafpigaaaaaaaagiiaaaaaaaaafpibaaa
aaaaaeehaaaaaaaaafpiaaaaaaaaapmiaaaaaaaamiapaaacaabliiaakbahaoaa
miapaaacaamgnapiklahanacmiapaaacaalbdepiklahamacmiapaaaiaagmnaje
klahalacmiapiadoaananaaaocaiaiaamiahaaadaamamgmaalbeaabfmiahaaac
aaleblaacbbfadaamiahaaacaamamgleclbeadacmiahaaafaamdgfaaobabagaa
miahaaajaalelbleclbdaaadmialaaadaalkblaakbabbgaamiahaaaeaalbleaa
kbadbbaamiahaaajaamagmleclbcaaajmiahaaafablklomaolabagafmiahaaac
aalelbleclbdadacmiahaaacaamagmleclbcadacceihaeafaamablgmobafagia
miahaaahabmablmaklajbgahmiahaaadaagmlemakladbaaemiahaaaeaabllema
kladapadaibhabadaamagmggkbaippaemiamiaaeaanlnlaaocaiaiaamiabiaab
aaloloaapaahagaamiaciaabaaloloaapaafahaamiaeiaabaalomdaapaahabaa
miabiaacaaloloaapaacagaamiaciaacaaloloaapaafacaamiaeiaacaalomdaa
paacabaamiadiaaaaalalabkilaabhbhmiamiaaaaakmkmagilaabibiaicbabac
aadoanmbgpaeaeaeaiecabacaadoanlbgpafaeaeaiieabacaadoanlmgpagaeae
miabaaaaaakhkhaakpabahaamiacaaaaaakhkhaakpabaiaaaibeabaaaakhkhgm
kpabajaeaiciabadaalbgmmgkbadabaemiadiaaeaamgbkbikladacadgeihaaaa
aalologboaacaaabmiahiaadaablmagfklaaakaaaaaaaaaaaaaaaaaaaaaaaaaa
"
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
Vector 455 [_BumpMap_ST]
"sce_vp_rsx // 47 instructions using 7 registers
[Configuration]
8
0000002f41050700
[Defaults]
1
454 1
3f000000
[Microcode]
752
00009c6c005d100d8186c0836041fffc00019c6c00400e0c0106c0836041dffc
00011c6c005d300c0186c0836041dffc00021c6c009c920c013fc0c36041dffc
401f9c6c011c7800810040d560607f9c401f9c6c011c8808010400d740619f9c
00001c6c01d0300d8106c0c360403ffc00001c6c01d0200d8106c0c360405ffc
00001c6c01d0100d8106c0c360409ffc00001c6c01d0000d8106c0c360411ffc
00029c6c01d0a00d8286c0c360405ffc00029c6c01d0900d8286c0c360409ffc
00029c6c01d0800d8286c0c360411ffc00031c6c0150400c088600c360411ffc
00031c6c0150600c088600c360405ffc00009c6c0150500c088600c360403ffc
00009c6c0190a00c0486c0c360405ffc00009c6c0190900c0486c0c360409ffc
00009c6c0190800c0486c0c360411ffc00011c6c00800243011843436041dffc
00011c6c01000230812183630121dffc401f9c6c0040000d8086c0836041ff80
401f9c6c004000558086c08360407fac00009c6c011c900c02bfc0e30041dffc
00001c6c009c600e008000c36041dffc401f9c6c0140020c0106054360405fa4
401f9c6c01400e0c0106054360411fa400001c6c009d202a808000c360409ffc
00001c6c0080007f82bfc14360403ffc00031c6c0040007f8286c08360409ffc
401f9c6c00c000080086c09540219fac00011c6c00800e0c04bfc0836041dffc
401f9c6c0140020c0106014360405fa0401f9c6c01400e0c0286008360411fa0
00001c6c019ce00c0c86c0c360405ffc00001c6c019cf00c0c86c0c360409ffc
00001c6c019d000c0c86c0c360411ffc00001c6c010000000c80067fe0203ffc
00019c6c0080000d0c9a06436041fffc401f9c6c0140000c0486054360409fa4
401f9c6c0140000c0286024360409fa000009c6c01dcb00d8686c0c360405ffc
00009c6c01dcc00d8686c0c360409ffc00009c6c01dcd00d8686c0c360411ffc
00001c6c00c0000c0086c08300a1dffc00009c6c009ca07f808600c36041dffc
401f9c6c00c0000c0286c0830021dfa9
"
}

SubProgram "d3d11 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 176 // 176 used size, 10 vars
Vector 144 [_MainTex_ST] 4
Vector 160 [_BumpMap_ST] 4
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
// 46 instructions, 5 temp regs, 0 temp arrays:
// ALU 26 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0
eefiecedocaebghbcjikdeoniaheacncilginjniabaaaaaaiaaiaaaaadaaaaaa
cmaaaaaapeaaaaaakmabaaaaejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaa
abaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaljaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfc
enebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheolaaaaaaaagaaaaaa
aiaaaaaajiaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaakeaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaabaaaaaaapaaaaaakeaaaaaaabaaaaaaaaaaaaaa
adaaaaaaacaaaaaaahaiaaaakeaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaa
ahaiaaaakeaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaahaiaaaakeaaaaaa
aeaaaaaaaaaaaaaaadaaaaaaafaaaaaaapaaaaaafdfgfpfaepfdejfeejepeoaa
feeffiedepepfceeaaklklklfdeieefcmmagaaaaeaaaabaaldabaaaafjaaaaae
egiocaaaaaaaaaaaalaaaaaafjaaaaaeegiocaaaabaaaaaaagaaaaaafjaaaaae
egiocaaaacaaaaaabjaaaaaafjaaaaaeegiocaaaadaaaaaabfaaaaaafpaaaaad
pcbabaaaaaaaaaaafpaaaaadpcbabaaaabaaaaaafpaaaaadhcbabaaaacaaaaaa
fpaaaaaddcbabaaaadaaaaaaghaaaaaepccabaaaaaaaaaaaabaaaaaagfaaaaad
pccabaaaabaaaaaagfaaaaadhccabaaaacaaaaaagfaaaaadhccabaaaadaaaaaa
gfaaaaadhccabaaaaeaaaaaagfaaaaadpccabaaaafaaaaaagiaaaaacafaaaaaa
diaaaaaipcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaadaaaaaaabaaaaaa
dcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaaaaaaaaaagbabaaaaaaaaaaa
egaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaacaaaaaa
kgbkbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaa
adaaaaaaadaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadgaaaaafpccabaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaaldccabaaaabaaaaaaegbabaaaadaaaaaa
egiacaaaaaaaaaaaajaaaaaaogikcaaaaaaaaaaaajaaaaaadcaaaaalmccabaaa
abaaaaaaagbebaaaadaaaaaaagiecaaaaaaaaaaaakaaaaaakgiocaaaaaaaaaaa
akaaaaaadiaaaaahhcaabaaaabaaaaaajgbebaaaabaaaaaacgbjbaaaacaaaaaa
dcaaaaakhcaabaaaabaaaaaajgbebaaaacaaaaaacgbjbaaaabaaaaaaegacbaia
ebaaaaaaabaaaaaadiaaaaahhcaabaaaabaaaaaaegacbaaaabaaaaaapgbpbaaa
abaaaaaadiaaaaajhcaabaaaacaaaaaafgifcaaaabaaaaaaaeaaaaaaegiccaaa
adaaaaaabbaaaaaadcaaaaalhcaabaaaacaaaaaaegiccaaaadaaaaaabaaaaaaa
agiacaaaabaaaaaaaeaaaaaaegacbaaaacaaaaaadcaaaaalhcaabaaaacaaaaaa
egiccaaaadaaaaaabcaaaaaakgikcaaaabaaaaaaaeaaaaaaegacbaaaacaaaaaa
aaaaaaaihcaabaaaacaaaaaaegacbaaaacaaaaaaegiccaaaadaaaaaabdaaaaaa
dcaaaaalhcaabaaaacaaaaaaegacbaaaacaaaaaapgipcaaaadaaaaaabeaaaaaa
egbcbaiaebaaaaaaaaaaaaaabaaaaaahcccabaaaacaaaaaaegacbaaaabaaaaaa
egacbaaaacaaaaaabaaaaaahbccabaaaacaaaaaaegbcbaaaabaaaaaaegacbaaa
acaaaaaabaaaaaaheccabaaaacaaaaaaegbcbaaaacaaaaaaegacbaaaacaaaaaa
diaaaaajhcaabaaaacaaaaaafgifcaaaacaaaaaaaaaaaaaaegiccaaaadaaaaaa
bbaaaaaadcaaaaalhcaabaaaacaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaa
acaaaaaaaaaaaaaaegacbaaaacaaaaaadcaaaaalhcaabaaaacaaaaaaegiccaaa
adaaaaaabcaaaaaakgikcaaaacaaaaaaaaaaaaaaegacbaaaacaaaaaadcaaaaal
hcaabaaaacaaaaaaegiccaaaadaaaaaabdaaaaaapgipcaaaacaaaaaaaaaaaaaa
egacbaaaacaaaaaabaaaaaahcccabaaaadaaaaaaegacbaaaabaaaaaaegacbaaa
acaaaaaabaaaaaahbccabaaaadaaaaaaegbcbaaaabaaaaaaegacbaaaacaaaaaa
baaaaaaheccabaaaadaaaaaaegbcbaaaacaaaaaaegacbaaaacaaaaaadiaaaaai
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
hcaabaaaacaaaaaaegacbaaaacaaaaaaegacbaaaaeaaaaaadiaaaaahccaabaaa
abaaaaaabkaabaaaabaaaaaabkaabaaaabaaaaaadcaaaaakbcaabaaaabaaaaaa
akaabaaaabaaaaaaakaabaaaabaaaaaabkaabaiaebaaaaaaabaaaaaadcaaaaak
hccabaaaaeaaaaaaegiccaaaacaaaaaabiaaaaaaagaabaaaabaaaaaaegacbaaa
acaaaaaadiaaaaaiccaabaaaaaaaaaaabkaabaaaaaaaaaaaakiacaaaabaaaaaa
afaaaaaadiaaaaakncaabaaaabaaaaaaagahbaaaaaaaaaaaaceaaaaaaaaaaadp
aaaaaaaaaaaaaadpaaaaaadpdgaaaaafmccabaaaafaaaaaakgaobaaaaaaaaaaa
aaaaaaahdccabaaaafaaaaaakgakbaaaabaaaaaamgaabaaaabaaaaaadoaaaaab
"
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
varying lowp vec3 xlv_TEXCOORD3;
varying lowp vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 _BumpMap_ST;
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
  highp vec4 tmpvar_4;
  lowp vec3 tmpvar_5;
  lowp vec3 tmpvar_6;
  tmpvar_4.xy = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  tmpvar_4.zw = ((_glesMultiTexCoord0.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
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
  tmpvar_5 = tmpvar_11;
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
  tmpvar_6 = shlight_3;
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = tmpvar_4;
  xlv_TEXCOORD1 = (tmpvar_10 * (((_World2Object * tmpvar_12).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = tmpvar_5;
  xlv_TEXCOORD3 = tmpvar_6;
  xlv_TEXCOORD4 = (unity_World2Shadow[0] * (_Object2World * _glesVertex));
}



#endif
#ifdef FRAGMENT

varying highp vec4 xlv_TEXCOORD4;
varying lowp vec3 xlv_TEXCOORD3;
varying lowp vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp float _Parallax;
uniform mediump float _Gloss;
uniform mediump float _Shininess;
uniform lowp vec4 _Color;
uniform sampler2D _ParallaxMap;
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
  mediump vec4 spec_4;
  mediump float h_5;
  lowp float tmpvar_6;
  tmpvar_6 = texture2D (_ParallaxMap, xlv_TEXCOORD0.zw).w;
  h_5 = tmpvar_6;
  highp vec2 tmpvar_7;
  mediump float height_8;
  height_8 = _Parallax;
  mediump vec3 viewDir_9;
  viewDir_9 = xlv_TEXCOORD1;
  highp vec3 v_10;
  mediump float tmpvar_11;
  tmpvar_11 = ((h_5 * height_8) - (height_8 / 2.0));
  mediump vec3 tmpvar_12;
  tmpvar_12 = normalize(viewDir_9);
  v_10 = tmpvar_12;
  v_10.z = (v_10.z + 0.42);
  tmpvar_7 = (tmpvar_11 * (v_10.xy / v_10.z));
  highp vec2 tmpvar_13;
  tmpvar_13 = (xlv_TEXCOORD0.xy + tmpvar_7);
  highp vec2 tmpvar_14;
  tmpvar_14 = (xlv_TEXCOORD0.zw + tmpvar_7);
  lowp vec4 tmpvar_15;
  tmpvar_15 = texture2D (_MainTex, tmpvar_13);
  lowp vec3 tmpvar_16;
  tmpvar_16 = (tmpvar_15.xyz * _Color.xyz);
  tmpvar_2 = tmpvar_16;
  lowp vec4 tmpvar_17;
  tmpvar_17 = texture2D (_SpecMap, tmpvar_13);
  spec_4 = tmpvar_17;
  mediump vec3 tmpvar_18;
  tmpvar_18 = ((tmpvar_15.w * spec_4.xyz) * _Gloss);
  lowp vec3 tmpvar_19;
  tmpvar_19 = ((texture2D (_BumpMap, tmpvar_14).xyz * 2.0) - 1.0);
  tmpvar_3 = tmpvar_19;
  lowp float tmpvar_20;
  mediump float lightShadowDataX_21;
  highp float dist_22;
  lowp float tmpvar_23;
  tmpvar_23 = texture2DProj (_ShadowMapTexture, xlv_TEXCOORD4).x;
  dist_22 = tmpvar_23;
  highp float tmpvar_24;
  tmpvar_24 = _LightShadowData.x;
  lightShadowDataX_21 = tmpvar_24;
  highp float tmpvar_25;
  tmpvar_25 = max (float((dist_22 > (xlv_TEXCOORD4.z / xlv_TEXCOORD4.w))), lightShadowDataX_21);
  tmpvar_20 = tmpvar_25;
  highp vec3 tmpvar_26;
  tmpvar_26 = normalize(xlv_TEXCOORD1);
  mediump vec3 lightDir_27;
  lightDir_27 = xlv_TEXCOORD2;
  mediump vec3 viewDir_28;
  viewDir_28 = tmpvar_26;
  mediump float atten_29;
  atten_29 = tmpvar_20;
  mediump vec4 c_30;
  mediump vec3 specCol_31;
  highp float nh_32;
  mediump float tmpvar_33;
  tmpvar_33 = max (0.0, dot (tmpvar_3, normalize((lightDir_27 + viewDir_28))));
  nh_32 = tmpvar_33;
  mediump float arg1_34;
  arg1_34 = (32.0 * _Shininess);
  highp vec3 tmpvar_35;
  tmpvar_35 = (pow (nh_32, arg1_34) * tmpvar_18);
  specCol_31 = tmpvar_35;
  c_30.xyz = ((((tmpvar_2 * _LightColor0.xyz) * max (0.0, dot (tmpvar_3, lightDir_27))) + (_LightColor0.xyz * specCol_31)) * (atten_29 * 2.0));
  c_30.w = 0.0;
  c_1 = c_30;
  mediump vec3 tmpvar_36;
  tmpvar_36 = (c_1.xyz + (tmpvar_2 * xlv_TEXCOORD3));
  c_1.xyz = tmpvar_36;
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
varying lowp vec3 xlv_TEXCOORD3;
varying lowp vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 _BumpMap_ST;
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
  highp vec4 tmpvar_4;
  lowp vec3 tmpvar_5;
  lowp vec3 tmpvar_6;
  highp vec4 tmpvar_7;
  tmpvar_7 = (gl_ModelViewProjectionMatrix * _glesVertex);
  tmpvar_4.xy = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  tmpvar_4.zw = ((_glesMultiTexCoord0.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
  mat3 tmpvar_8;
  tmpvar_8[0] = _Object2World[0].xyz;
  tmpvar_8[1] = _Object2World[1].xyz;
  tmpvar_8[2] = _Object2World[2].xyz;
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
  tmpvar_5 = tmpvar_12;
  highp vec4 tmpvar_13;
  tmpvar_13.w = 1.0;
  tmpvar_13.xyz = _WorldSpaceCameraPos;
  highp vec4 tmpvar_14;
  tmpvar_14.w = 1.0;
  tmpvar_14.xyz = (tmpvar_8 * (tmpvar_2 * unity_Scale.w));
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
  tmpvar_6 = shlight_3;
  highp vec4 o_30;
  highp vec4 tmpvar_31;
  tmpvar_31 = (tmpvar_7 * 0.5);
  highp vec2 tmpvar_32;
  tmpvar_32.x = tmpvar_31.x;
  tmpvar_32.y = (tmpvar_31.y * _ProjectionParams.x);
  o_30.xy = (tmpvar_32 + tmpvar_31.w);
  o_30.zw = tmpvar_7.zw;
  gl_Position = tmpvar_7;
  xlv_TEXCOORD0 = tmpvar_4;
  xlv_TEXCOORD1 = (tmpvar_11 * (((_World2Object * tmpvar_13).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = tmpvar_5;
  xlv_TEXCOORD3 = tmpvar_6;
  xlv_TEXCOORD4 = o_30;
}



#endif
#ifdef FRAGMENT

varying highp vec4 xlv_TEXCOORD4;
varying lowp vec3 xlv_TEXCOORD3;
varying lowp vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp float _Parallax;
uniform mediump float _Gloss;
uniform mediump float _Shininess;
uniform lowp vec4 _Color;
uniform sampler2D _ParallaxMap;
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
  mediump vec4 spec_4;
  mediump float h_5;
  lowp float tmpvar_6;
  tmpvar_6 = texture2D (_ParallaxMap, xlv_TEXCOORD0.zw).w;
  h_5 = tmpvar_6;
  highp vec2 tmpvar_7;
  mediump float height_8;
  height_8 = _Parallax;
  mediump vec3 viewDir_9;
  viewDir_9 = xlv_TEXCOORD1;
  highp vec3 v_10;
  mediump float tmpvar_11;
  tmpvar_11 = ((h_5 * height_8) - (height_8 / 2.0));
  mediump vec3 tmpvar_12;
  tmpvar_12 = normalize(viewDir_9);
  v_10 = tmpvar_12;
  v_10.z = (v_10.z + 0.42);
  tmpvar_7 = (tmpvar_11 * (v_10.xy / v_10.z));
  highp vec2 tmpvar_13;
  tmpvar_13 = (xlv_TEXCOORD0.xy + tmpvar_7);
  highp vec2 tmpvar_14;
  tmpvar_14 = (xlv_TEXCOORD0.zw + tmpvar_7);
  lowp vec4 tmpvar_15;
  tmpvar_15 = texture2D (_MainTex, tmpvar_13);
  lowp vec3 tmpvar_16;
  tmpvar_16 = (tmpvar_15.xyz * _Color.xyz);
  tmpvar_2 = tmpvar_16;
  lowp vec4 tmpvar_17;
  tmpvar_17 = texture2D (_SpecMap, tmpvar_13);
  spec_4 = tmpvar_17;
  mediump vec3 tmpvar_18;
  tmpvar_18 = ((tmpvar_15.w * spec_4.xyz) * _Gloss);
  lowp vec3 normal_19;
  normal_19.xy = ((texture2D (_BumpMap, tmpvar_14).wy * 2.0) - 1.0);
  normal_19.z = sqrt(((1.0 - (normal_19.x * normal_19.x)) - (normal_19.y * normal_19.y)));
  tmpvar_3 = normal_19;
  lowp float tmpvar_20;
  tmpvar_20 = texture2DProj (_ShadowMapTexture, xlv_TEXCOORD4).x;
  highp vec3 tmpvar_21;
  tmpvar_21 = normalize(xlv_TEXCOORD1);
  mediump vec3 lightDir_22;
  lightDir_22 = xlv_TEXCOORD2;
  mediump vec3 viewDir_23;
  viewDir_23 = tmpvar_21;
  mediump float atten_24;
  atten_24 = tmpvar_20;
  mediump vec4 c_25;
  mediump vec3 specCol_26;
  highp float nh_27;
  mediump float tmpvar_28;
  tmpvar_28 = max (0.0, dot (tmpvar_3, normalize((lightDir_22 + viewDir_23))));
  nh_27 = tmpvar_28;
  mediump float arg1_29;
  arg1_29 = (32.0 * _Shininess);
  highp vec3 tmpvar_30;
  tmpvar_30 = (pow (nh_27, arg1_29) * tmpvar_18);
  specCol_26 = tmpvar_30;
  c_25.xyz = ((((tmpvar_2 * _LightColor0.xyz) * max (0.0, dot (tmpvar_3, lightDir_22))) + (_LightColor0.xyz * specCol_26)) * (atten_24 * 2.0));
  c_25.w = 0.0;
  c_1 = c_25;
  mediump vec3 tmpvar_31;
  tmpvar_31 = (c_1.xyz + (tmpvar_2 * xlv_TEXCOORD3));
  c_1.xyz = tmpvar_31;
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
Vector 25 [_BumpMap_ST]
"agal_vs
c26 1.0 0.5 0.0 0.0
[bc]
adaaaaaaabaaahacabaaaaoeaaaaaaaabgaaaappabaaaaaa mul r1.xyz, a1, c22.w
bcaaaaaaacaaaiacabaaaakeacaaaaaaafaaaaoeabaaaaaa dp3 r2.w, r1.xyzz, c5
bcaaaaaaaaaaabacabaaaakeacaaaaaaaeaaaaoeabaaaaaa dp3 r0.x, r1.xyzz, c4
bcaaaaaaaaaaaeacabaaaakeacaaaaaaagaaaaoeabaaaaaa dp3 r0.z, r1.xyzz, c6
aaaaaaaaaaaaacacacaaaappacaaaaaaaaaaaaaaaaaaaaaa mov r0.y, r2.w
adaaaaaaabaaapacaaaaaakeacaaaaaaaaaaaacjacaaaaaa mul r1, r0.xyzz, r0.yzzx
aaaaaaaaaaaaaiacbkaaaaaaabaaaaaaaaaaaaaaaaaaaaaa mov r0.w, c26.x
bdaaaaaaacaaaeacaaaaaaoeacaaaaaabbaaaaoeabaaaaaa dp4 r2.z, r0, c17
bdaaaaaaacaaacacaaaaaaoeacaaaaaabaaaaaoeabaaaaaa dp4 r2.y, r0, c16
bdaaaaaaacaaabacaaaaaaoeacaaaaaaapaaaaoeabaaaaaa dp4 r2.x, r0, c15
adaaaaaaaaaaacacacaaaappacaaaaaaacaaaappacaaaaaa mul r0.y, r2.w, r2.w
bdaaaaaaadaaaeacabaaaaoeacaaaaaabeaaaaoeabaaaaaa dp4 r3.z, r1, c20
bdaaaaaaadaaacacabaaaaoeacaaaaaabdaaaaoeabaaaaaa dp4 r3.y, r1, c19
bdaaaaaaadaaabacabaaaaoeacaaaaaabcaaaaoeabaaaaaa dp4 r3.x, r1, c18
abaaaaaaabaaahacacaaaakeacaaaaaaadaaaakeacaaaaaa add r1.xyz, r2.xyzz, r3.xyzz
adaaaaaaadaaaiacaaaaaaaaacaaaaaaaaaaaaaaacaaaaaa mul r3.w, r0.x, r0.x
acaaaaaaaaaaabacadaaaappacaaaaaaaaaaaaffacaaaaaa sub r0.x, r3.w, r0.y
adaaaaaaacaaahacaaaaaaaaacaaaaaabfaaaaoeabaaaaaa mul r2.xyz, r0.x, c21
abaaaaaaadaaahaeabaaaakeacaaaaaaacaaaakeacaaaaaa add v3.xyz, r1.xyzz, r2.xyzz
aaaaaaaaaaaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, a5
adaaaaaaabaaahacabaaaancaaaaaaaaaaaaaaajacaaaaaa mul r1.xyz, a1.zxyw, r0.yzxx
aaaaaaaaaaaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, a5
adaaaaaaaeaaahacabaaaamjaaaaaaaaaaaaaafcacaaaaaa mul r4.xyz, a1.yzxw, r0.zxyy
acaaaaaaaaaaahacaeaaaakeacaaaaaaabaaaakeacaaaaaa sub r0.xyz, r4.xyzz, r1.xyzz
adaaaaaaadaaahacaaaaaakeacaaaaaaafaaaappaaaaaaaa mul r3.xyz, r0.xyzz, a5.w
aaaaaaaaaaaaapacakaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0, c10
bdaaaaaaaeaaaeacaoaaaaoeabaaaaaaaaaaaaoeacaaaaaa dp4 r4.z, c14, r0
aaaaaaaaaaaaapacajaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0, c9
bdaaaaaaaeaaacacaoaaaaoeabaaaaaaaaaaaaoeacaaaaaa dp4 r4.y, c14, r0
aaaaaaaaabaaaiacbkaaaaaaabaaaaaaaaaaaaaaaaaaaaaa mov r1.w, c26.x
aaaaaaaaabaaahacamaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r1.xyz, c12
bdaaaaaaaaaaaiacaaaaaaoeaaaaaaaaadaaaaoeabaaaaaa dp4 r0.w, a0, c3
bdaaaaaaaaaaaeacaaaaaaoeaaaaaaaaacaaaaoeabaaaaaa dp4 r0.z, a0, c2
bdaaaaaaacaaaeacabaaaaoeacaaaaaaakaaaaoeabaaaaaa dp4 r2.z, r1, c10
bdaaaaaaacaaabacabaaaaoeacaaaaaaaiaaaaoeabaaaaaa dp4 r2.x, r1, c8
bdaaaaaaacaaacacabaaaaoeacaaaaaaajaaaaoeabaaaaaa dp4 r2.y, r1, c9
adaaaaaaafaaahacacaaaakeacaaaaaabgaaaappabaaaaaa mul r5.xyz, r2.xyzz, c22.w
acaaaaaaacaaahacafaaaakeacaaaaaaaaaaaaoeaaaaaaaa sub r2.xyz, r5.xyzz, a0
aaaaaaaaabaaapacaiaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r1, c8
bdaaaaaaaeaaabacaoaaaaoeabaaaaaaabaaaaoeacaaaaaa dp4 r4.x, c14, r1
bdaaaaaaaaaaabacaaaaaaoeaaaaaaaaaaaaaaoeabaaaaaa dp4 r0.x, a0, c0
bdaaaaaaaaaaacacaaaaaaoeaaaaaaaaabaaaaoeabaaaaaa dp4 r0.y, a0, c1
adaaaaaaabaaahacaaaaaapeacaaaaaabkaaaaffabaaaaaa mul r1.xyz, r0.xyww, c26.y
adaaaaaaabaaacacabaaaaffacaaaaaaanaaaaaaabaaaaaa mul r1.y, r1.y, c13.x
abaaaaaaabaaadacabaaaafeacaaaaaaabaaaakkacaaaaaa add r1.xy, r1.xyyy, r1.z
bcaaaaaaabaaacaeacaaaakeacaaaaaaadaaaakeacaaaaaa dp3 v1.y, r2.xyzz, r3.xyzz
bcaaaaaaacaaacaeadaaaakeacaaaaaaaeaaaakeacaaaaaa dp3 v2.y, r3.xyzz, r4.xyzz
bcaaaaaaabaaaeaeabaaaaoeaaaaaaaaacaaaakeacaaaaaa dp3 v1.z, a1, r2.xyzz
bcaaaaaaabaaabaeacaaaakeacaaaaaaafaaaaoeaaaaaaaa dp3 v1.x, r2.xyzz, a5
bcaaaaaaacaaaeaeabaaaaoeaaaaaaaaaeaaaakeacaaaaaa dp3 v2.z, a1, r4.xyzz
bcaaaaaaacaaabaeafaaaaoeaaaaaaaaaeaaaakeacaaaaaa dp3 v2.x, a5, r4.xyzz
adaaaaaaaeaaadaeabaaaafeacaaaaaabhaaaaoeabaaaaaa mul v4.xy, r1.xyyy, c23
aaaaaaaaaaaaapadaaaaaaoeacaaaaaaaaaaaaaaaaaaaaaa mov o0, r0
aaaaaaaaaeaaamaeaaaaaaopacaaaaaaaaaaaaaaaaaaaaaa mov v4.zw, r0.wwzw
adaaaaaaafaaamacadaaaaeeaaaaaaaabjaaaaeeabaaaaaa mul r5.zw, a3.xyxy, c25.xyxy
abaaaaaaaaaaamaeafaaaaopacaaaaaabjaaaaoeabaaaaaa add v0.zw, r5.wwzw, c25
adaaaaaaafaaadacadaaaaoeaaaaaaaabiaaaaoeabaaaaaa mul r5.xy, a3, c24
abaaaaaaaaaaadaeafaaaafeacaaaaaabiaaaaooabaaaaaa add v0.xy, r5.xyyy, c24.zwzw
aaaaaaaaabaaaiaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v1.w, c0
aaaaaaaaacaaaiaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v2.w, c0
aaaaaaaaadaaaiaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v3.w, c0
"
}

SubProgram "opengl " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Bind "vertex" Vertex
Bind "tangent" ATTR14
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Vector 13 [_WorldSpaceCameraPos]
Vector 14 [_ProjectionParams]
Matrix 9 [_World2Object]
Vector 16 [unity_Scale]
Vector 17 [unity_LightmapST]
Vector 18 [_MainTex_ST]
Vector 19 [_BumpMap_ST]
"!!ARBvp1.0
# 25 ALU
PARAM c[20] = { { 1, 0.5 },
		state.matrix.mvp,
		program.local[5..19] };
TEMP R0;
TEMP R1;
TEMP R2;
MOV R0.xyz, vertex.attrib[14];
MUL R1.xyz, vertex.normal.zxyw, R0.yzxw;
MAD R0.xyz, vertex.normal.yzxw, R0.zxyw, -R1;
MUL R0.xyz, R0, vertex.attrib[14].w;
MOV R1.xyz, c[13];
MOV R1.w, c[0].x;
DP4 R0.w, vertex.position, c[4];
DP4 R2.z, R1, c[11];
DP4 R2.x, R1, c[9];
DP4 R2.y, R1, c[10];
MAD R2.xyz, R2, c[16].w, -vertex.position;
DP3 result.texcoord[1].y, R2, R0;
DP4 R0.z, vertex.position, c[3];
DP4 R0.x, vertex.position, c[1];
DP4 R0.y, vertex.position, c[2];
MUL R1.xyz, R0.xyww, c[0].y;
MUL R1.y, R1, c[14].x;
DP3 result.texcoord[1].z, vertex.normal, R2;
DP3 result.texcoord[1].x, R2, vertex.attrib[14];
ADD result.texcoord[3].xy, R1, R1.z;
MOV result.position, R0;
MOV result.texcoord[3].zw, R0;
MAD result.texcoord[0].zw, vertex.texcoord[0].xyxy, c[19].xyxy, c[19];
MAD result.texcoord[0].xy, vertex.texcoord[0], c[18], c[18].zwzw;
MAD result.texcoord[2].xy, vertex.texcoord[1], c[17], c[17].zwzw;
END
# 25 instructions, 3 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Matrix 0 [glstate_matrix_mvp]
Vector 12 [_WorldSpaceCameraPos]
Vector 13 [_ProjectionParams]
Vector 14 [_ScreenParams]
Matrix 8 [_World2Object]
Vector 15 [unity_Scale]
Vector 16 [unity_LightmapST]
Vector 17 [_MainTex_ST]
Vector 18 [_BumpMap_ST]
"vs_2_0
; 26 ALU
def c19, 1.00000000, 0.50000000, 0, 0
dcl_position0 v0
dcl_tangent0 v1
dcl_normal0 v2
dcl_texcoord0 v3
dcl_texcoord1 v4
mov r0.xyz, v1
mul r1.xyz, v2.zxyw, r0.yzxw
mov r0.xyz, v1
mad r0.xyz, v2.yzxw, r0.zxyw, -r1
mul r0.xyz, r0, v1.w
mov r1.xyz, c12
mov r1.w, c19.x
dp4 r0.w, v0, c3
dp4 r2.z, r1, c10
dp4 r2.x, r1, c8
dp4 r2.y, r1, c9
mad r2.xyz, r2, c15.w, -v0
dp3 oT1.y, r2, r0
dp4 r0.z, v0, c2
dp4 r0.x, v0, c0
dp4 r0.y, v0, c1
mul r1.xyz, r0.xyww, c19.y
mul r1.y, r1, c13.x
dp3 oT1.z, v2, r2
dp3 oT1.x, r2, v1
mad oT3.xy, r1.z, c14.zwzw, r1
mov oPos, r0
mov oT3.zw, r0
mad oT0.zw, v3.xyxy, c18.xyxy, c18
mad oT0.xy, v3, c17, c17.zwzw
mad oT2.xy, v4, c16, c16.zwzw
"
}

SubProgram "xbox360 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Vector 14 [_BumpMap_ST]
Vector 13 [_MainTex_ST]
Vector 1 [_ProjectionParams]
Vector 2 [_ScreenParams]
Matrix 7 [_World2Object] 4
Vector 0 [_WorldSpaceCameraPos]
Matrix 3 [glstate_matrix_mvp] 4
Vector 12 [unity_LightmapST]
Vector 11 [unity_Scale]
// Shader Timing Estimate, in Cycles/64 vertex vector:
// ALU: 28.00 (21 instructions), vertex: 64, texture: 0,
//   sequencer: 14,  7 GPRs, 27 threads,
// Performance (if enough threads): ~64 cycles per vector
// * Vertex cycle estimates are assuming 3 vfetch_minis for every vfetch_full,
//     with <= 32 bytes per vfetch_full group.

"vs_360
backbbabaaaaacgaaaaaableaaaaaaaaaaaaaaceaaaaabmmaaaaabpeaaaaaaaa
aaaaaaaaaaaaabkeaaaaaabmaaaaabjhpppoadaaaaaaaaajaaaaaabmaaaaaaaa
aaaaabjaaaaaaanaaaacaaaoaaabaaaaaaaaaanmaaaaaaaaaaaaaaomaaacaaan
aaabaaaaaaaaaanmaaaaaaaaaaaaaapiaaacaaabaaabaaaaaaaaaanmaaaaaaaa
aaaaabakaaacaaacaaabaaaaaaaaaanmaaaaaaaaaaaaabbiaaacaaahaaaeaaaa
aaaaabciaaaaaaaaaaaaabdiaaacaaaaaaabaaaaaaaaabfaaaaaaaaaaaaaabga
aaacaaadaaaeaaaaaaaaabciaaaaaaaaaaaaabhdaaacaaamaaabaaaaaaaaaanm
aaaaaaaaaaaaabieaaacaaalaaabaaaaaaaaaanmaaaaaaaafpechfgnhaengbha
fpfdfeaaaaabaaadaaabaaaeaaabaaaaaaaaaaaafpengbgjgofegfhifpfdfeaa
fpfahcgpgkgfgdhegjgpgofagbhcgbgnhdaafpfdgdhcgfgfgofagbhcgbgnhdaa
fpfhgphcgmgedcepgcgkgfgdheaaklklaaadaaadaaaeaaaeaaabaaaaaaaaaaaa
fpfhgphcgmgefdhagbgdgfedgbgngfhcgbfagphdaaklklklaaabaaadaaabaaad
aaabaaaaaaaaaaaaghgmhdhegbhegffpgngbhehcgjhifpgnhghaaahfgogjhehj
fpemgjghgihegngbhafdfeaahfgogjhehjfpfdgdgbgmgfaahghdfpddfpdaaadc
codacodcdadddfddcodaaaklaaaaaaaaaaaaaaabaaaaaaaaaaaaaaaaaaaaaabe
aapmaabaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaeaaaaaabheaadbaaag
aaaaaaaaaaaaaaaaaaaadeieaaaaaaabaaaaaaafaaaaaaaiaaaaacjaaabaaaae
aaaagaafaaaadaagaaaafaahaacbfaaiaaaapafaaaachbfbaaafdcfcaaagpdfd
aaaaaablaaaababmaaaaaabgaaaaaabiaaaababjaaaababkaaaaaabfaaaababn
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaadpaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
pbfffaaeaaaabcabmcaaaaaaaaaafaajaaaabcaameaaaaaaaaaagaaogabebcaa
bcaaaaaaaaaaeabkaaaaccaaaaaaaaaaafpidaaaaaaaagiiaaaaaaaaafpieaaa
aaaaagiiaaaaaaaaafpicaaaaaaaaoiiaaaaaaaaafpibaaaaaaaapmiaaaaaaaa
afpibaaaaaaaacdpaaaaaaaamiapaaaaaabliiaakbadagaamiapaaaaaamgnapi
kladafaamiapaaaaaalbdepikladaeaamiapaaagaagmnajekladadaamiapiado
aananaaaocagagaamiahaaaaaamamgmaalajaaakmiahaaafaalogfaaobacaeaa
miahaaaaaalelbleclaiaaaamiahaaaaaamagmleclahaaaamiahaaafabgfloma
olacaeafmiahaaadabmablmaklaaaladmiahaaaaaamagmaakbagppaamiamiaad
aanlnlaaocagagaamiabiaabaaloloaapaadaeaakiihaaaeaamablebmbafaeab
miaciaabaaloloaapaaeadaamiaeiaabaaloloaapaadacaamiadiaacaabklabk
ilabamammiadiaaaaalalabkilabananmiamiaaaaakmkmagilabaoaomiadiaad
aamgbkbiklaaacaaaaaaaaaaaaaaaaaaaaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Matrix 256 [glstate_matrix_mvp]
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Vector 467 [_WorldSpaceCameraPos]
Vector 466 [_ProjectionParams]
Matrix 264 [_World2Object]
Vector 465 [unity_Scale]
Vector 464 [unity_LightmapST]
Vector 463 [_MainTex_ST]
Vector 462 [_BumpMap_ST]
"sce_vp_rsx // 24 instructions using 4 registers
[Configuration]
8
0000001843050400
[Defaults]
1
461 1
3f000000
[Microcode]
384
00011c6c00400e0c0106c0836041dffc00019c6c005d300c0186c0836041dffc
401f9c6c011ce800810040d560607f9c401f9c6c011cf808010400d740619f9c
401f9c6c011d0908010400d740619fa400009c6c01d0300d8106c0c360403ffc
00009c6c01d0200d8106c0c360405ffc00009c6c01d0100d8106c0c360409ffc
00009c6c01d0000d8106c0c360411ffc00001c6c0190a00c0686c0c360405ffc
00001c6c0190900c0686c0c360409ffc00001c6c0190800c0686c0c360411ffc
00019c6c00800243011842436041dffc00011c6c010002308121826301a1dffc
401f9c6c0040000d8286c0836041ff80401f9c6c004000558286c08360407fa8
00001c6c011d100c00bfc0e30041dffc00009c6c009cd00e028000c36041dffc
00009c6c009d202a828000c360409ffc401f9c6c00c000080286c09540a19fa8
00009c6c00800e0c04bfc0836041dffc401f9c6c0140020c0106004360405fa0
401f9c6c01400e0c0086008360411fa0401f9c6c0140000c0086014360409fa1
"
}

SubProgram "d3d11 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Bind "color" Color
ConstBuffer "$Globals" 192 // 192 used size, 11 vars
Vector 144 [unity_LightmapST] 4
Vector 160 [_MainTex_ST] 4
Vector 176 [_BumpMap_ST] 4
ConstBuffer "UnityPerCamera" 128 // 96 used size, 8 vars
Vector 64 [_WorldSpaceCameraPos] 3
Vector 80 [_ProjectionParams] 4
ConstBuffer "UnityPerDraw" 336 // 336 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
Matrix 256 [_World2Object] 4
Vector 320 [unity_Scale] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityPerDraw" 2
// 24 instructions, 3 temp regs, 0 temp arrays:
// ALU 11 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0
eefiecedmiicmabohkdefhchpkppmpenbkfeefbeabaaaaaaiaafaaaaadaaaaaa
cmaaaaaapeaaaaaajeabaaaaejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaa
abaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapadaaaaljaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfc
enebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheojiaaaaaaafaaaaaa
aiaaaaaaiaaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaabaaaaaaapaaaaaaimaaaaaaabaaaaaaaaaaaaaa
adaaaaaaacaaaaaaahaiaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaa
adamaaaaimaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaafdfgfpfa
epfdejfeejepeoaafeeffiedepepfceeaaklklklfdeieefcoeadaaaaeaaaabaa
pjaaaaaafjaaaaaeegiocaaaaaaaaaaaamaaaaaafjaaaaaeegiocaaaabaaaaaa
agaaaaaafjaaaaaeegiocaaaacaaaaaabfaaaaaafpaaaaadpcbabaaaaaaaaaaa
fpaaaaadpcbabaaaabaaaaaafpaaaaadhcbabaaaacaaaaaafpaaaaaddcbabaaa
adaaaaaafpaaaaaddcbabaaaaeaaaaaaghaaaaaepccabaaaaaaaaaaaabaaaaaa
gfaaaaadpccabaaaabaaaaaagfaaaaadhccabaaaacaaaaaagfaaaaaddccabaaa
adaaaaaagfaaaaadpccabaaaaeaaaaaagiaaaaacadaaaaaadiaaaaaipcaabaaa
aaaaaaaafgbfbaaaaaaaaaaaegiocaaaacaaaaaaabaaaaaadcaaaaakpcaabaaa
aaaaaaaaegiocaaaacaaaaaaaaaaaaaaagbabaaaaaaaaaaaegaobaaaaaaaaaaa
dcaaaaakpcaabaaaaaaaaaaaegiocaaaacaaaaaaacaaaaaakgbkbaaaaaaaaaaa
egaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaacaaaaaaadaaaaaa
pgbpbaaaaaaaaaaaegaobaaaaaaaaaaadgaaaaafpccabaaaaaaaaaaaegaobaaa
aaaaaaaadcaaaaaldccabaaaabaaaaaaegbabaaaadaaaaaaegiacaaaaaaaaaaa
akaaaaaaogikcaaaaaaaaaaaakaaaaaadcaaaaalmccabaaaabaaaaaaagbebaaa
adaaaaaaagiecaaaaaaaaaaaalaaaaaakgiocaaaaaaaaaaaalaaaaaadiaaaaah
hcaabaaaabaaaaaajgbebaaaabaaaaaacgbjbaaaacaaaaaadcaaaaakhcaabaaa
abaaaaaajgbebaaaacaaaaaacgbjbaaaabaaaaaaegacbaiaebaaaaaaabaaaaaa
diaaaaahhcaabaaaabaaaaaaegacbaaaabaaaaaapgbpbaaaabaaaaaadiaaaaaj
hcaabaaaacaaaaaafgifcaaaabaaaaaaaeaaaaaaegiccaaaacaaaaaabbaaaaaa
dcaaaaalhcaabaaaacaaaaaaegiccaaaacaaaaaabaaaaaaaagiacaaaabaaaaaa
aeaaaaaaegacbaaaacaaaaaadcaaaaalhcaabaaaacaaaaaaegiccaaaacaaaaaa
bcaaaaaakgikcaaaabaaaaaaaeaaaaaaegacbaaaacaaaaaaaaaaaaaihcaabaaa
acaaaaaaegacbaaaacaaaaaaegiccaaaacaaaaaabdaaaaaadcaaaaalhcaabaaa
acaaaaaaegacbaaaacaaaaaapgipcaaaacaaaaaabeaaaaaaegbcbaiaebaaaaaa
aaaaaaaabaaaaaahcccabaaaacaaaaaaegacbaaaabaaaaaaegacbaaaacaaaaaa
baaaaaahbccabaaaacaaaaaaegbcbaaaabaaaaaaegacbaaaacaaaaaabaaaaaah
eccabaaaacaaaaaaegbcbaaaacaaaaaaegacbaaaacaaaaaadcaaaaaldccabaaa
adaaaaaaegbabaaaaeaaaaaaegiacaaaaaaaaaaaajaaaaaaogikcaaaaaaaaaaa
ajaaaaaadiaaaaaiccaabaaaaaaaaaaabkaabaaaaaaaaaaaakiacaaaabaaaaaa
afaaaaaadiaaaaakncaabaaaabaaaaaaagahbaaaaaaaaaaaaceaaaaaaaaaaadp
aaaaaaaaaaaaaadpaaaaaadpdgaaaaafmccabaaaaeaaaaaakgaobaaaaaaaaaaa
aaaaaaahdccabaaaaeaaaaaakgakbaaaabaaaaaamgaabaaaabaaaaaadoaaaaab
"
}

SubProgram "gles " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
"!!GLES
#define SHADER_API_GLES 1
#define tex2D texture2D


#ifdef VERTEX
#define gl_ModelViewProjectionMatrix glstate_matrix_mvp
uniform mat4 glstate_matrix_mvp;

varying highp vec4 xlv_TEXCOORD3;
varying highp vec2 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 _BumpMap_ST;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 unity_LightmapST;
uniform highp vec4 unity_Scale;
uniform highp mat4 _World2Object;
uniform highp mat4 _Object2World;

uniform highp mat4 unity_World2Shadow[4];
uniform highp vec3 _WorldSpaceCameraPos;
attribute vec4 _glesTANGENT;
attribute vec4 _glesMultiTexCoord1;
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
  highp vec4 tmpvar_3;
  tmpvar_3.xy = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  tmpvar_3.zw = ((_glesMultiTexCoord0.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
  highp vec3 tmpvar_4;
  highp vec3 tmpvar_5;
  tmpvar_4 = tmpvar_1.xyz;
  tmpvar_5 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_6;
  tmpvar_6[0].x = tmpvar_4.x;
  tmpvar_6[0].y = tmpvar_5.x;
  tmpvar_6[0].z = tmpvar_2.x;
  tmpvar_6[1].x = tmpvar_4.y;
  tmpvar_6[1].y = tmpvar_5.y;
  tmpvar_6[1].z = tmpvar_2.y;
  tmpvar_6[2].x = tmpvar_4.z;
  tmpvar_6[2].y = tmpvar_5.z;
  tmpvar_6[2].z = tmpvar_2.z;
  highp vec4 tmpvar_7;
  tmpvar_7.w = 1.0;
  tmpvar_7.xyz = _WorldSpaceCameraPos;
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = tmpvar_3;
  xlv_TEXCOORD1 = (tmpvar_6 * (((_World2Object * tmpvar_7).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = ((_glesMultiTexCoord1.xy * unity_LightmapST.xy) + unity_LightmapST.zw);
  xlv_TEXCOORD3 = (unity_World2Shadow[0] * (_Object2World * _glesVertex));
}



#endif
#ifdef FRAGMENT

varying highp vec4 xlv_TEXCOORD3;
varying highp vec2 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform sampler2D unity_Lightmap;
uniform highp float _Parallax;
uniform lowp vec4 _Color;
uniform sampler2D _ParallaxMap;
uniform sampler2D _MainTex;
uniform sampler2D _ShadowMapTexture;
uniform highp vec4 _LightShadowData;
void main ()
{
  lowp vec4 c_1;
  mediump vec3 tmpvar_2;
  mediump float h_3;
  lowp float tmpvar_4;
  tmpvar_4 = texture2D (_ParallaxMap, xlv_TEXCOORD0.zw).w;
  h_3 = tmpvar_4;
  mediump float height_5;
  height_5 = _Parallax;
  mediump vec3 viewDir_6;
  viewDir_6 = xlv_TEXCOORD1;
  highp vec3 v_7;
  mediump float tmpvar_8;
  tmpvar_8 = ((h_3 * height_5) - (height_5 / 2.0));
  mediump vec3 tmpvar_9;
  tmpvar_9 = normalize(viewDir_6);
  v_7 = tmpvar_9;
  v_7.z = (v_7.z + 0.42);
  highp vec2 tmpvar_10;
  tmpvar_10 = (xlv_TEXCOORD0.xy + (tmpvar_8 * (v_7.xy / v_7.z)));
  lowp vec3 tmpvar_11;
  tmpvar_11 = (texture2D (_MainTex, tmpvar_10).xyz * _Color.xyz);
  tmpvar_2 = tmpvar_11;
  lowp float tmpvar_12;
  mediump float lightShadowDataX_13;
  highp float dist_14;
  lowp float tmpvar_15;
  tmpvar_15 = texture2DProj (_ShadowMapTexture, xlv_TEXCOORD3).x;
  dist_14 = tmpvar_15;
  highp float tmpvar_16;
  tmpvar_16 = _LightShadowData.x;
  lightShadowDataX_13 = tmpvar_16;
  highp float tmpvar_17;
  tmpvar_17 = max (float((dist_14 > (xlv_TEXCOORD3.z / xlv_TEXCOORD3.w))), lightShadowDataX_13);
  tmpvar_12 = tmpvar_17;
  lowp vec3 tmpvar_18;
  tmpvar_18 = min ((2.0 * texture2D (unity_Lightmap, xlv_TEXCOORD2).xyz), vec3((tmpvar_12 * 2.0)));
  mediump vec3 tmpvar_19;
  tmpvar_19 = (tmpvar_2 * tmpvar_18);
  c_1.xyz = tmpvar_19;
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

varying highp vec4 xlv_TEXCOORD3;
varying highp vec2 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 _BumpMap_ST;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 unity_LightmapST;
uniform highp vec4 unity_Scale;
uniform highp mat4 _World2Object;

uniform highp vec4 _ProjectionParams;
uniform highp vec3 _WorldSpaceCameraPos;
attribute vec4 _glesTANGENT;
attribute vec4 _glesMultiTexCoord1;
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
  highp vec4 tmpvar_3;
  highp vec4 tmpvar_4;
  tmpvar_4 = (gl_ModelViewProjectionMatrix * _glesVertex);
  tmpvar_3.xy = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  tmpvar_3.zw = ((_glesMultiTexCoord0.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
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
  highp vec4 tmpvar_8;
  tmpvar_8.w = 1.0;
  tmpvar_8.xyz = _WorldSpaceCameraPos;
  highp vec4 o_9;
  highp vec4 tmpvar_10;
  tmpvar_10 = (tmpvar_4 * 0.5);
  highp vec2 tmpvar_11;
  tmpvar_11.x = tmpvar_10.x;
  tmpvar_11.y = (tmpvar_10.y * _ProjectionParams.x);
  o_9.xy = (tmpvar_11 + tmpvar_10.w);
  o_9.zw = tmpvar_4.zw;
  gl_Position = tmpvar_4;
  xlv_TEXCOORD0 = tmpvar_3;
  xlv_TEXCOORD1 = (tmpvar_7 * (((_World2Object * tmpvar_8).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = ((_glesMultiTexCoord1.xy * unity_LightmapST.xy) + unity_LightmapST.zw);
  xlv_TEXCOORD3 = o_9;
}



#endif
#ifdef FRAGMENT

varying highp vec4 xlv_TEXCOORD3;
varying highp vec2 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform sampler2D unity_Lightmap;
uniform highp float _Parallax;
uniform lowp vec4 _Color;
uniform sampler2D _ParallaxMap;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform sampler2D _ShadowMapTexture;
void main ()
{
  lowp vec4 c_1;
  mediump vec3 tmpvar_2;
  mediump float h_3;
  lowp float tmpvar_4;
  tmpvar_4 = texture2D (_ParallaxMap, xlv_TEXCOORD0.zw).w;
  h_3 = tmpvar_4;
  highp vec2 tmpvar_5;
  mediump float height_6;
  height_6 = _Parallax;
  mediump vec3 viewDir_7;
  viewDir_7 = xlv_TEXCOORD1;
  highp vec3 v_8;
  mediump float tmpvar_9;
  tmpvar_9 = ((h_3 * height_6) - (height_6 / 2.0));
  mediump vec3 tmpvar_10;
  tmpvar_10 = normalize(viewDir_7);
  v_8 = tmpvar_10;
  v_8.z = (v_8.z + 0.42);
  tmpvar_5 = (tmpvar_9 * (v_8.xy / v_8.z));
  highp vec2 tmpvar_11;
  tmpvar_11 = (xlv_TEXCOORD0.xy + tmpvar_5);
  highp vec2 tmpvar_12;
  tmpvar_12 = (xlv_TEXCOORD0.zw + tmpvar_5);
  lowp vec3 tmpvar_13;
  tmpvar_13 = (texture2D (_MainTex, tmpvar_11).xyz * _Color.xyz);
  tmpvar_2 = tmpvar_13;
  lowp vec3 normal_14;
  normal_14.xy = ((texture2D (_BumpMap, tmpvar_12).wy * 2.0) - 1.0);
  normal_14.z = sqrt(((1.0 - (normal_14.x * normal_14.x)) - (normal_14.y * normal_14.y)));
  lowp vec4 tmpvar_15;
  tmpvar_15 = texture2DProj (_ShadowMapTexture, xlv_TEXCOORD3);
  lowp vec4 tmpvar_16;
  tmpvar_16 = texture2D (unity_Lightmap, xlv_TEXCOORD2);
  lowp vec3 tmpvar_17;
  tmpvar_17 = ((8.0 * tmpvar_16.w) * tmpvar_16.xyz);
  lowp vec3 tmpvar_18;
  tmpvar_18 = max (min (tmpvar_17, ((tmpvar_15.x * 2.0) * tmpvar_16.xyz)), (tmpvar_17 * tmpvar_15.x));
  mediump vec3 tmpvar_19;
  tmpvar_19 = (tmpvar_2 * tmpvar_18);
  c_1.xyz = tmpvar_19;
  c_1.w = 0.0;
  gl_FragData[0] = c_1;
}



#endif"
}

SubProgram "flash " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Matrix 0 [glstate_matrix_mvp]
Vector 12 [_WorldSpaceCameraPos]
Vector 13 [_ProjectionParams]
Matrix 8 [_World2Object]
Vector 14 [unity_Scale]
Vector 15 [unity_NPOTScale]
Vector 16 [unity_LightmapST]
Vector 17 [_MainTex_ST]
Vector 18 [_BumpMap_ST]
"agal_vs
c19 1.0 0.5 0.0 0.0
[bc]
aaaaaaaaaaaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, a5
adaaaaaaabaaahacabaaaancaaaaaaaaaaaaaaajacaaaaaa mul r1.xyz, a1.zxyw, r0.yzxx
aaaaaaaaaaaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, a5
adaaaaaaacaaahacabaaaamjaaaaaaaaaaaaaafcacaaaaaa mul r2.xyz, a1.yzxw, r0.zxyy
acaaaaaaaaaaahacacaaaakeacaaaaaaabaaaakeacaaaaaa sub r0.xyz, r2.xyzz, r1.xyzz
adaaaaaaaaaaahacaaaaaakeacaaaaaaafaaaappaaaaaaaa mul r0.xyz, r0.xyzz, a5.w
aaaaaaaaabaaahacamaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r1.xyz, c12
aaaaaaaaabaaaiacbdaaaaaaabaaaaaaaaaaaaaaaaaaaaaa mov r1.w, c19.x
bdaaaaaaacaaaeacabaaaaoeacaaaaaaakaaaaoeabaaaaaa dp4 r2.z, r1, c10
bdaaaaaaacaaabacabaaaaoeacaaaaaaaiaaaaoeabaaaaaa dp4 r2.x, r1, c8
bdaaaaaaacaaacacabaaaaoeacaaaaaaajaaaaoeabaaaaaa dp4 r2.y, r1, c9
adaaaaaaadaaahacacaaaakeacaaaaaaaoaaaappabaaaaaa mul r3.xyz, r2.xyzz, c14.w
acaaaaaaabaaahacadaaaakeacaaaaaaaaaaaaoeaaaaaaaa sub r1.xyz, r3.xyzz, a0
bcaaaaaaabaaacaeabaaaakeacaaaaaaaaaaaakeacaaaaaa dp3 v1.y, r1.xyzz, r0.xyzz
bdaaaaaaaaaaaiacaaaaaaoeaaaaaaaaadaaaaoeabaaaaaa dp4 r0.w, a0, c3
bdaaaaaaaaaaaeacaaaaaaoeaaaaaaaaacaaaaoeabaaaaaa dp4 r0.z, a0, c2
bcaaaaaaabaaaeaeabaaaaoeaaaaaaaaabaaaakeacaaaaaa dp3 v1.z, a1, r1.xyzz
bdaaaaaaaaaaabacaaaaaaoeaaaaaaaaaaaaaaoeabaaaaaa dp4 r0.x, a0, c0
bdaaaaaaaaaaacacaaaaaaoeaaaaaaaaabaaaaoeabaaaaaa dp4 r0.y, a0, c1
adaaaaaaacaaahacaaaaaapeacaaaaaabdaaaaffabaaaaaa mul r2.xyz, r0.xyww, c19.y
bcaaaaaaabaaabaeabaaaakeacaaaaaaafaaaaoeaaaaaaaa dp3 v1.x, r1.xyzz, a5
adaaaaaaabaaacacacaaaaffacaaaaaaanaaaaaaabaaaaaa mul r1.y, r2.y, c13.x
aaaaaaaaabaaabacacaaaaaaacaaaaaaaaaaaaaaaaaaaaaa mov r1.x, r2.x
abaaaaaaabaaadacabaaaafeacaaaaaaacaaaakkacaaaaaa add r1.xy, r1.xyyy, r2.z
adaaaaaaadaaadaeabaaaafeacaaaaaaapaaaaoeabaaaaaa mul v3.xy, r1.xyyy, c15
aaaaaaaaaaaaapadaaaaaaoeacaaaaaaaaaaaaaaaaaaaaaa mov o0, r0
aaaaaaaaadaaamaeaaaaaaopacaaaaaaaaaaaaaaaaaaaaaa mov v3.zw, r0.wwzw
adaaaaaaadaaamacadaaaaeeaaaaaaaabcaaaaeeabaaaaaa mul r3.zw, a3.xyxy, c18.xyxy
abaaaaaaaaaaamaeadaaaaopacaaaaaabcaaaaoeabaaaaaa add v0.zw, r3.wwzw, c18
adaaaaaaadaaadacadaaaaoeaaaaaaaabbaaaaoeabaaaaaa mul r3.xy, a3, c17
abaaaaaaaaaaadaeadaaaafeacaaaaaabbaaaaooabaaaaaa add v0.xy, r3.xyyy, c17.zwzw
adaaaaaaadaaadacaeaaaaoeaaaaaaaabaaaaaoeabaaaaaa mul r3.xy, a4, c16
abaaaaaaacaaadaeadaaaafeacaaaaaabaaaaaooabaaaaaa add v2.xy, r3.xyyy, c16.zwzw
aaaaaaaaabaaaiaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v1.w, c0
aaaaaaaaacaaamaeaaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov v2.zw, c0
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
Vector 32 [_BumpMap_ST]
"!!ARBvp1.0
# 75 ALU
PARAM c[33] = { { 1, 0 },
		state.matrix.mvp,
		program.local[5..32] };
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
DP4 R3.z, R0, c[28];
DP4 R3.y, R0, c[27];
DP4 R3.x, R0, c[26];
MUL R1.w, R3, R3;
MAD R0.x, R4, R4, -R1.w;
MOV R0.w, c[0].x;
DP4 R2.z, R4, c[25];
DP4 R2.y, R4, c[24];
DP4 R2.x, R4, c[23];
ADD R2.xyz, R2, R3;
MUL R3.xyz, R0.x, c[29];
ADD R3.xyz, R2, R3;
MOV R0.xyz, vertex.attrib[14];
MUL R2.xyz, vertex.normal.zxyw, R0.yzxw;
ADD result.texcoord[3].xyz, R3, R1;
MAD R1.xyz, vertex.normal.yzxw, R0.zxyw, -R2;
MOV R0.xyz, c[13];
DP4 R2.z, R0, c[11];
DP4 R2.x, R0, c[9];
DP4 R2.y, R0, c[10];
MAD R0.xyz, R2, c[30].w, -vertex.position;
MUL R2.xyz, R1, vertex.attrib[14].w;
MOV R1, c[14];
DP4 R3.z, R1, c[11];
DP4 R3.x, R1, c[9];
DP4 R3.y, R1, c[10];
DP3 result.texcoord[1].y, R0, R2;
DP3 result.texcoord[2].y, R2, R3;
DP3 result.texcoord[1].z, vertex.normal, R0;
DP3 result.texcoord[1].x, R0, vertex.attrib[14];
DP3 result.texcoord[2].z, vertex.normal, R3;
DP3 result.texcoord[2].x, vertex.attrib[14], R3;
MAD result.texcoord[0].zw, vertex.texcoord[0].xyxy, c[32].xyxy, c[32];
MAD result.texcoord[0].xy, vertex.texcoord[0], c[31], c[31].zwzw;
DP4 result.position.w, vertex.position, c[4];
DP4 result.position.z, vertex.position, c[3];
DP4 result.position.y, vertex.position, c[2];
DP4 result.position.x, vertex.position, c[1];
END
# 75 instructions, 5 R-regs
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
Vector 31 [_BumpMap_ST]
"vs_2_0
; 78 ALU
def c32, 1.00000000, 0.00000000, 0, 0
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
mov r4.w, c32.x
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
add r1, r2, c32.x
dp4 r2.z, r4, c24
dp4 r2.y, r4, c23
dp4 r2.x, r4, c22
rcp r1.x, r1.x
rcp r1.y, r1.y
rcp r1.w, r1.w
rcp r1.z, r1.z
max r0, r0, c32.y
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
add r2.xyz, r2, r0
add oT3.xyz, r2, r1
mov r0.xyz, v1
mul r1.xyz, v2.zxyw, r0.yzxw
mov r0.xyz, v1
mad r0.xyz, v2.yzxw, r0.zxyw, -r1
mul r3.xyz, r0, v1.w
mov r0, c10
dp4 r4.z, c13, r0
mov r0, c9
mov r1.w, c32.x
mov r1.xyz, c12
dp4 r4.y, c13, r0
dp4 r2.z, r1, c10
dp4 r2.x, r1, c8
dp4 r2.y, r1, c9
mad r2.xyz, r2, c29.w, -v0
mov r1, c8
dp4 r4.x, c13, r1
dp3 oT1.y, r2, r3
dp3 oT2.y, r3, r4
dp3 oT1.z, v2, r2
dp3 oT1.x, r2, v1
dp3 oT2.z, v2, r4
dp3 oT2.x, v1, r4
mad oT0.zw, v3.xyxy, c31.xyxy, c31
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
Vector 31 [_BumpMap_ST]
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
// ALU: 90.67 (68 instructions), vertex: 32, texture: 0,
//   sequencer: 30,  10 GPRs, 18 threads,
// Performance (if enough threads): ~90 cycles per vector
// * Vertex cycle estimates are assuming 3 vfetch_minis for every vfetch_full,
//     with <= 32 bytes per vfetch_full group.

"vs_360
backbbabaaaaadoeaaaaaeamaaaaaaaaaaaaaaceaaaaadfaaaaaadhiaaaaaaaa
aaaaaaaaaaaaadciaaaaaabmaaaaadblpppoadaaaaaaaabeaaaaaabmaaaaaaaa
aaaaadbeaaaaabkmaaacaabpaaabaaaaaaaaabliaaaaaaaaaaaaabmiaaacaabo
aaabaaaaaaaaabliaaaaaaaaaaaaabneaaacaabfaaaeaaaaaaaaaboeaaaaaaaa
aaaaabpeaaacaabjaaaeaaaaaaaaaboeaaaaaaaaaaaaacacaaacaaaaaaabaaaa
aaaaacbiaaaaaaaaaaaaacciaaacaaabaaabaaaaaaaaabliaaaaaaaaaaaaacdn
aaacaabbaaaeaaaaaaaaaboeaaaaaaaaaaaaacfaaaacaaafaaabaaaaaaaaabli
aaaaaaaaaaaaacgdaaacaaacaaabaaaaaaaaabliaaaaaaaaaaaaachfaaacaaad
aaabaaaaaaaaabliaaaaaaaaaaaaacihaaacaaaeaaabaaaaaaaaabliaaaaaaaa
aaaaacjjaaacaaagaaaeaaaaaaaaackmaaaaaaaaaaaaaclmaaacaaamaaabaaaa
aaaaabliaaaaaaaaaaaaacmhaaacaaalaaabaaaaaaaaabliaaaaaaaaaaaaacnc
aaacaaakaaabaaaaaaaaabliaaaaaaaaaaaaacnnaaacaaapaaabaaaaaaaaabli
aaaaaaaaaaaaacoiaaacaaaoaaabaaaaaaaaabliaaaaaaaaaaaaacpdaaacaaan
aaabaaaaaaaaabliaaaaaaaaaaaaacpoaaacaabaaaabaaaaaaaaabliaaaaaaaa
aaaaadaiaaacaabnaaabaaaaaaaaabliaaaaaaaafpechfgnhaengbhafpfdfeaa
aaabaaadaaabaaaeaaabaaaaaaaaaaaafpengbgjgofegfhifpfdfeaafpepgcgk
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
aaaaaabeaapmaabaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaeaaaaaadmm
aadbaaajaaaaaaaaaaaaaaaaaaaadeieaaaaaaabaaaaaaaeaaaaaaajaaaaacja
aabaaaaiaaaagaajaaaadaakaadafaalaaaapafaaaachbfbaaafhcfcaaaihdfd
aaaaaacjaaaabackaaaaaacdaaaaaaceaaaabacfaaaaaacgaaaaaachaaaabaci
aaaabaepaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaadpiaaaaaaaaaaaaaaaaaaaaa
aaaaaaaapaffeaaiaaaabcaamcaaaaaaaaaaeaamaaaabcaameaaaaaaaaaagaba
gabgbcaabcaaaaaaaaaagabmgaccbcaabcaaaaaaaaaagacigacobcaabcaaaaaa
aaaagadegadkbcaabcaaaaaaaaaagaeagaegbcaabcaaaaaaaaaaeaemaaaaccaa
aaaaaaaaafpiiaaaaaaaaanbaaaaaaaaafpifaaaaaaaagiiaaaaaaaaafpicaaa
aaaaaoiiaaaaaaaaafpiaaaaaaaaapmiaaaaaaaamiapaaabaamgiiaakbaibeaa
miapaaabaalbiiaaklaibdabmiapaaabaagmdejeklaibcabmiapiadoaablaade
klaibbabmiahaaabaamamgmaalblaabmmiahaaadaaleblaacbbmabaamiahaaad
aamamgleclblabadmiahaaaeaalogfaaobacafaamiahaaahaalelbleclbkaaab
miahaaabaagfblaakbacbnaamiahaaagaamgleaakbaibiaamiahaaajaalbmale
klaibhagmiahaaagaalbleaakbabbhaamiahaaahaamagmleclbjaaahmiahaaae
abgflomaolacafaemiahaaadaalelbleclbkabadmiahaaadaamagmleclbjabad
miahaaaeaamablaaobaeafaamiahaaahabmabllpklahbnaimiahaaagaagmlema
klabbgagmiahaaaiaagmleleklaibgajmialaaabaabllemaklaibfaimiahaaag
aamglemaklabbfagmiabiaabaaloloaapaahafaamiaciaabaaloloaapaaeahaa
miaeiaabaaloloaapaahacaamiabiaacaaloloaapaadafaamiaciaacaaloloaa
paaeadaamiaeiaacaaloloaapaadacaamiadiaaaaalalabkilaabobomiamiaaa
aakmkmagilaabpbpceipagaaaalehcgmobagagiaaibpadafaegmaagmkaabacag
aicpadacaelbaamgkaabaeagbeabaaaeabdoanblgpakagabaebcahaeaadoangm
epalagadbeaeaaaeabdoanblgpamagabaecbahabaakhkhlbipaaanadbeacaaab
abkhkhblkpaaaoabaeeeahabaakhkhmgipaaapadbeapaaaaabpipiblobacacab
aeipahacaapilbblmbacagadmiapaaaaaajejepiolahahaamiapaaacaajemgpi
olahagacmiapaaacaajegmaaolafagacmiapaaaaaaaaaapiolafafaageihabab
aalologboaaeabadmiahaaabaabllemnklabbaabmiapaaaeaapipigmilaaafpp
fibaaaaaaaaaaagmocaaaaiaficaaaaaaaaaaalbocaaaaiafieaaaaaaaaaaamg
ocaaaaiafiiaaaaaaaaaaablocaaaaiamiapaaaaaapiaaaaobacaaaaemipaaad
aapilbmgkcaappaeemecacaaaamgblgmobadaaaeemciacacaagmmgblobadacae
embbaaacaabllblbobadacaemiaeaaaaaalbgmaaobadaaaakibhacaeaalmmaec
ibacaiajkiciacaeaamgblicmbaeadajkieoacafaabgpmmaibacagajbeahaaaa
aabbmalbkbaaahafambiafaaaamgmggmobaaadadbeahaaaaaabebamgoaafaaac
amihacaaaamabalboaaaaeadmiahaaaaaamabaaaoaaaacaamiahiaadaalemaaa
oaabaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
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
Vector 448 [_BumpMap_ST]
"sce_vp_rsx // 64 instructions using 9 registers
[Configuration]
8
0000004041050900
[Defaults]
1
447 2
000000003f800000
[Microcode]
1024
00011c6c005d200d8186c0836041fffc00031c6c00400e0c0106c0836041dffc
00001c6c005d300c0186c0836041dffc00009c6c009c220c013fc0c36041dffc
401f9c6c011c0800810040d560607f9c401f9c6c011c1808010400d740619f9c
401f9c6c01d0300d8106c0c360403f80401f9c6c01d0200d8106c0c360405f80
401f9c6c01d0100d8106c0c360409f80401f9c6c01d0000d8106c0c360411f80
00019c6c01d0500d8106c0c360411ffc00009c6c01d0400d8106c0c360403ffc
00001c6c01d0600d8106c0c360403ffc00029c6c01d0a00d8486c0c360405ffc
00029c6c01d0900d8486c0c360409ffc00029c6c01d0800d8486c0c360411ffc
00021c6c0150400c028600c360411ffc00021c6c0150600c028600c360403ffc
00021c6c0150500c028600c360409ffc00011c6c0190a00c0086c0c360405ffc
00011c6c0190900c0086c0c360409ffc00011c6c0190800c0086c0c360411ffc
00001c6c00dcf00d8186c0bfe021fffc00009c6c00dd100d8186c0bfe0a1fffc
00019c6c00dd000d8186c0a001a1fffc00039c6c00800243011846436041dffc
00039c6c010002308121866303a1dffc00031c6c011c200c04bfc0e30041dffc
401f9c6c0140020c0106054360405fa400011c6c0080002a8886c3436041fffc
00019c6c0080000d8686c3436041fffc00029c6c0080002a8895444360403ffc
00021c6c0040007f8886c08360405ffc00011c6c010000000886c1436121fffc
00009c6c0100000d8286c14361a1fffc00041c6c019c700c0886c0c360405ffc
00041c6c019c800c0886c0c360409ffc00041c6c019c900c0886c0c360411ffc
00029c6c010000000880047fe2a03ffc00019c6c0080000d089a04436041fffc
00011c6c0100007f8886c0436121fffc00001c6c0100000d8086c04360a1fffc
00009c6c01dc400d8686c0c360405ffc00009c6c01dc500d8686c0c360409ffc
00009c6c01dc600d8686c0c360411ffc00009c6c00c0000c1086c08300a1dffc
00019c6c009c307f8a8600c36041dffc00019c6c00c0000c0686c08300a1dffc
401f9c6c21400e0c01060540003100a400039c6c20800e0c0ebfc08aa029c0fc
00021c6c209ce00d8086c0d54025e0fc00021c6c00dbf02a8186c0836221fffc
401f9c6c2140020c0106065fe02240a0401f9c6c11400e0c0c86008002310020
401f9c6c1140000c0e86054aa2288024401f9c6c1140000c0c86075542248020
00009c6c1080000d8486c15fe223e07c00009c6c029bf00d828000c36041fffc
00001c6c0080000d8286c0436041fffc00009c6c009cc02a808600c36041dffc
00009c6c011cd000008600c300a1dffc00001c6c011cb055008600c300a1dffc
00001c6c011ca07f808600c30021dffc401f9c6c00c0000c0686c0830021dfa9
"
}

SubProgram "d3d11 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" "VERTEXLIGHT_ON" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 112 // 112 used size, 9 vars
Vector 80 [_MainTex_ST] 4
Vector 96 [_BumpMap_ST] 4
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
// 65 instructions, 6 temp regs, 0 temp arrays:
// ALU 36 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0
eefiecedopkmmpolggpnlecejifmdnekbijegopaabaaaaaacaalaaaaadaaaaaa
cmaaaaaapeaaaaaajeabaaaaejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaa
abaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaljaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfc
enebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheojiaaaaaaafaaaaaa
aiaaaaaaiaaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaabaaaaaaapaaaaaaimaaaaaaabaaaaaaaaaaaaaa
adaaaaaaacaaaaaaahaiaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaa
ahaiaaaaimaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaahaiaaaafdfgfpfa
epfdejfeejepeoaafeeffiedepepfceeaaklklklfdeieefcieajaaaaeaaaabaa
gbacaaaafjaaaaaeegiocaaaaaaaaaaaahaaaaaafjaaaaaeegiocaaaabaaaaaa
afaaaaaafjaaaaaeegiocaaaacaaaaaabjaaaaaafjaaaaaeegiocaaaadaaaaaa
bfaaaaaafpaaaaadpcbabaaaaaaaaaaafpaaaaadpcbabaaaabaaaaaafpaaaaad
hcbabaaaacaaaaaafpaaaaaddcbabaaaadaaaaaaghaaaaaepccabaaaaaaaaaaa
abaaaaaagfaaaaadpccabaaaabaaaaaagfaaaaadhccabaaaacaaaaaagfaaaaad
hccabaaaadaaaaaagfaaaaadhccabaaaaeaaaaaagiaaaaacagaaaaaadiaaaaai
pcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaadaaaaaaabaaaaaadcaaaaak
pcaabaaaaaaaaaaaegiocaaaadaaaaaaaaaaaaaaagbabaaaaaaaaaaaegaobaaa
aaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaacaaaaaakgbkbaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaakpccabaaaaaaaaaaaegiocaaaadaaaaaa
adaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaaldccabaaaabaaaaaa
egbabaaaadaaaaaaegiacaaaaaaaaaaaafaaaaaaogikcaaaaaaaaaaaafaaaaaa
dcaaaaalmccabaaaabaaaaaaagbebaaaadaaaaaaagiecaaaaaaaaaaaagaaaaaa
kgiocaaaaaaaaaaaagaaaaaadiaaaaahhcaabaaaaaaaaaaajgbebaaaabaaaaaa
cgbjbaaaacaaaaaadcaaaaakhcaabaaaaaaaaaaajgbebaaaacaaaaaacgbjbaaa
abaaaaaaegacbaiaebaaaaaaaaaaaaaadiaaaaahhcaabaaaaaaaaaaaegacbaaa
aaaaaaaapgbpbaaaabaaaaaadiaaaaajhcaabaaaabaaaaaafgifcaaaabaaaaaa
aeaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaa
adaaaaaabaaaaaaaagiacaaaabaaaaaaaeaaaaaaegacbaaaabaaaaaadcaaaaal
hcaabaaaabaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaaabaaaaaaaeaaaaaa
egacbaaaabaaaaaaaaaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaaegiccaaa
adaaaaaabdaaaaaadcaaaaalhcaabaaaabaaaaaaegacbaaaabaaaaaapgipcaaa
adaaaaaabeaaaaaaegbcbaiaebaaaaaaaaaaaaaabaaaaaahcccabaaaacaaaaaa
egacbaaaaaaaaaaaegacbaaaabaaaaaabaaaaaahbccabaaaacaaaaaaegbcbaaa
abaaaaaaegacbaaaabaaaaaabaaaaaaheccabaaaacaaaaaaegbcbaaaacaaaaaa
egacbaaaabaaaaaadiaaaaajhcaabaaaabaaaaaafgifcaaaacaaaaaaaaaaaaaa
egiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaa
baaaaaaaagiacaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaa
abaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaaacaaaaaaaaaaaaaaegacbaaa
abaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabdaaaaaapgipcaaa
acaaaaaaaaaaaaaaegacbaaaabaaaaaabaaaaaahcccabaaaadaaaaaaegacbaaa
aaaaaaaaegacbaaaabaaaaaabaaaaaahbccabaaaadaaaaaaegbcbaaaabaaaaaa
egacbaaaabaaaaaabaaaaaaheccabaaaadaaaaaaegbcbaaaacaaaaaaegacbaaa
abaaaaaadgaaaaaficaabaaaaaaaaaaaabeaaaaaaaaaiadpdiaaaaaihcaabaaa
abaaaaaaegbcbaaaacaaaaaapgipcaaaadaaaaaabeaaaaaadiaaaaaihcaabaaa
acaaaaaafgafbaaaabaaaaaaegiccaaaadaaaaaaanaaaaaadcaaaaaklcaabaaa
abaaaaaaegiicaaaadaaaaaaamaaaaaaagaabaaaabaaaaaaegaibaaaacaaaaaa
dcaaaaakhcaabaaaaaaaaaaaegiccaaaadaaaaaaaoaaaaaakgakbaaaabaaaaaa
egadbaaaabaaaaaabbaaaaaibcaabaaaabaaaaaaegiocaaaacaaaaaabcaaaaaa
egaobaaaaaaaaaaabbaaaaaiccaabaaaabaaaaaaegiocaaaacaaaaaabdaaaaaa
egaobaaaaaaaaaaabbaaaaaiecaabaaaabaaaaaaegiocaaaacaaaaaabeaaaaaa
egaobaaaaaaaaaaadiaaaaahpcaabaaaacaaaaaajgacbaaaaaaaaaaaegakbaaa
aaaaaaaabbaaaaaibcaabaaaadaaaaaaegiocaaaacaaaaaabfaaaaaaegaobaaa
acaaaaaabbaaaaaiccaabaaaadaaaaaaegiocaaaacaaaaaabgaaaaaaegaobaaa
acaaaaaabbaaaaaiecaabaaaadaaaaaaegiocaaaacaaaaaabhaaaaaaegaobaaa
acaaaaaaaaaaaaahhcaabaaaabaaaaaaegacbaaaabaaaaaaegacbaaaadaaaaaa
diaaaaahicaabaaaaaaaaaaabkaabaaaaaaaaaaabkaabaaaaaaaaaaadcaaaaak
icaabaaaaaaaaaaaakaabaaaaaaaaaaaakaabaaaaaaaaaaadkaabaiaebaaaaaa
aaaaaaaadcaaaaakhcaabaaaabaaaaaaegiccaaaacaaaaaabiaaaaaapgapbaaa
aaaaaaaaegacbaaaabaaaaaadiaaaaaihcaabaaaacaaaaaafgbfbaaaaaaaaaaa
egiccaaaadaaaaaaanaaaaaadcaaaaakhcaabaaaacaaaaaaegiccaaaadaaaaaa
amaaaaaaagbabaaaaaaaaaaaegacbaaaacaaaaaadcaaaaakhcaabaaaacaaaaaa
egiccaaaadaaaaaaaoaaaaaakgbkbaaaaaaaaaaaegacbaaaacaaaaaadcaaaaak
hcaabaaaacaaaaaaegiccaaaadaaaaaaapaaaaaapgbpbaaaaaaaaaaaegacbaaa
acaaaaaaaaaaaaajpcaabaaaadaaaaaafgafbaiaebaaaaaaacaaaaaaegiocaaa
acaaaaaaadaaaaaadiaaaaahpcaabaaaaeaaaaaafgafbaaaaaaaaaaaegaobaaa
adaaaaaadiaaaaahpcaabaaaadaaaaaaegaobaaaadaaaaaaegaobaaaadaaaaaa
aaaaaaajpcaabaaaafaaaaaaagaabaiaebaaaaaaacaaaaaaegiocaaaacaaaaaa
acaaaaaaaaaaaaajpcaabaaaacaaaaaakgakbaiaebaaaaaaacaaaaaaegiocaaa
acaaaaaaaeaaaaaadcaaaaajpcaabaaaaeaaaaaaegaobaaaafaaaaaaagaabaaa
aaaaaaaaegaobaaaaeaaaaaadcaaaaajpcaabaaaaaaaaaaaegaobaaaacaaaaaa
kgakbaaaaaaaaaaaegaobaaaaeaaaaaadcaaaaajpcaabaaaadaaaaaaegaobaaa
afaaaaaaegaobaaaafaaaaaaegaobaaaadaaaaaadcaaaaajpcaabaaaacaaaaaa
egaobaaaacaaaaaaegaobaaaacaaaaaaegaobaaaadaaaaaaeeaaaaafpcaabaaa
adaaaaaaegaobaaaacaaaaaadcaaaaanpcaabaaaacaaaaaaegaobaaaacaaaaaa
egiocaaaacaaaaaaafaaaaaaaceaaaaaaaaaiadpaaaaiadpaaaaiadpaaaaiadp
aoaaaaakpcaabaaaacaaaaaaaceaaaaaaaaaiadpaaaaiadpaaaaiadpaaaaiadp
egaobaaaacaaaaaadiaaaaahpcaabaaaaaaaaaaaegaobaaaaaaaaaaaegaobaaa
adaaaaaadeaaaaakpcaabaaaaaaaaaaaegaobaaaaaaaaaaaaceaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaadiaaaaahpcaabaaaaaaaaaaaegaobaaaacaaaaaa
egaobaaaaaaaaaaadiaaaaaihcaabaaaacaaaaaafgafbaaaaaaaaaaaegiccaaa
acaaaaaaahaaaaaadcaaaaakhcaabaaaacaaaaaaegiccaaaacaaaaaaagaaaaaa
agaabaaaaaaaaaaaegacbaaaacaaaaaadcaaaaakhcaabaaaaaaaaaaaegiccaaa
acaaaaaaaiaaaaaakgakbaaaaaaaaaaaegacbaaaacaaaaaadcaaaaakhcaabaaa
aaaaaaaaegiccaaaacaaaaaaajaaaaaapgapbaaaaaaaaaaaegacbaaaaaaaaaaa
aaaaaaahhccabaaaaeaaaaaaegacbaaaaaaaaaaaegacbaaaabaaaaaadoaaaaab
"
}

SubProgram "gles " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" "VERTEXLIGHT_ON" }
"!!GLES
#define SHADER_API_GLES 1
#define tex2D texture2D


#ifdef VERTEX
#define gl_ModelViewProjectionMatrix glstate_matrix_mvp
uniform mat4 glstate_matrix_mvp;

varying lowp vec3 xlv_TEXCOORD3;
varying lowp vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 _BumpMap_ST;
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
  highp vec4 tmpvar_4;
  lowp vec3 tmpvar_5;
  lowp vec3 tmpvar_6;
  tmpvar_4.xy = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  tmpvar_4.zw = ((_glesMultiTexCoord0.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
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
  tmpvar_5 = tmpvar_12;
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
  tmpvar_6 = shlight_3;
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
  tmpvar_36 = (tmpvar_6 + ((((unity_LightColor[0].xyz * tmpvar_35.x) + (unity_LightColor[1].xyz * tmpvar_35.y)) + (unity_LightColor[2].xyz * tmpvar_35.z)) + (unity_LightColor[3].xyz * tmpvar_35.w)));
  tmpvar_6 = tmpvar_36;
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = tmpvar_4;
  xlv_TEXCOORD1 = (tmpvar_11 * (((_World2Object * tmpvar_13).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = tmpvar_5;
  xlv_TEXCOORD3 = tmpvar_6;
}



#endif
#ifdef FRAGMENT

varying lowp vec3 xlv_TEXCOORD3;
varying lowp vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp float _Parallax;
uniform mediump float _Gloss;
uniform mediump float _Shininess;
uniform lowp vec4 _Color;
uniform sampler2D _ParallaxMap;
uniform sampler2D _SpecMap;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform lowp vec4 _LightColor0;
void main ()
{
  lowp vec4 c_1;
  mediump vec3 tmpvar_2;
  mediump vec3 tmpvar_3;
  mediump vec4 spec_4;
  mediump float h_5;
  lowp float tmpvar_6;
  tmpvar_6 = texture2D (_ParallaxMap, xlv_TEXCOORD0.zw).w;
  h_5 = tmpvar_6;
  highp vec2 tmpvar_7;
  mediump float height_8;
  height_8 = _Parallax;
  mediump vec3 viewDir_9;
  viewDir_9 = xlv_TEXCOORD1;
  highp vec3 v_10;
  mediump float tmpvar_11;
  tmpvar_11 = ((h_5 * height_8) - (height_8 / 2.0));
  mediump vec3 tmpvar_12;
  tmpvar_12 = normalize(viewDir_9);
  v_10 = tmpvar_12;
  v_10.z = (v_10.z + 0.42);
  tmpvar_7 = (tmpvar_11 * (v_10.xy / v_10.z));
  highp vec2 tmpvar_13;
  tmpvar_13 = (xlv_TEXCOORD0.xy + tmpvar_7);
  highp vec2 tmpvar_14;
  tmpvar_14 = (xlv_TEXCOORD0.zw + tmpvar_7);
  lowp vec4 tmpvar_15;
  tmpvar_15 = texture2D (_MainTex, tmpvar_13);
  lowp vec3 tmpvar_16;
  tmpvar_16 = (tmpvar_15.xyz * _Color.xyz);
  tmpvar_2 = tmpvar_16;
  lowp vec4 tmpvar_17;
  tmpvar_17 = texture2D (_SpecMap, tmpvar_13);
  spec_4 = tmpvar_17;
  mediump vec3 tmpvar_18;
  tmpvar_18 = ((tmpvar_15.w * spec_4.xyz) * _Gloss);
  lowp vec3 tmpvar_19;
  tmpvar_19 = ((texture2D (_BumpMap, tmpvar_14).xyz * 2.0) - 1.0);
  tmpvar_3 = tmpvar_19;
  highp vec3 tmpvar_20;
  tmpvar_20 = normalize(xlv_TEXCOORD1);
  mediump vec3 lightDir_21;
  lightDir_21 = xlv_TEXCOORD2;
  mediump vec3 viewDir_22;
  viewDir_22 = tmpvar_20;
  mediump vec4 c_23;
  mediump vec3 specCol_24;
  highp float nh_25;
  mediump float tmpvar_26;
  tmpvar_26 = max (0.0, dot (tmpvar_3, normalize((lightDir_21 + viewDir_22))));
  nh_25 = tmpvar_26;
  mediump float arg1_27;
  arg1_27 = (32.0 * _Shininess);
  highp vec3 tmpvar_28;
  tmpvar_28 = (pow (nh_25, arg1_27) * tmpvar_18);
  specCol_24 = tmpvar_28;
  c_23.xyz = ((((tmpvar_2 * _LightColor0.xyz) * max (0.0, dot (tmpvar_3, lightDir_21))) + (_LightColor0.xyz * specCol_24)) * 2.0);
  c_23.w = 0.0;
  c_1 = c_23;
  mediump vec3 tmpvar_29;
  tmpvar_29 = (c_1.xyz + (tmpvar_2 * xlv_TEXCOORD3));
  c_1.xyz = tmpvar_29;
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

varying lowp vec3 xlv_TEXCOORD3;
varying lowp vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 _BumpMap_ST;
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
  highp vec4 tmpvar_4;
  lowp vec3 tmpvar_5;
  lowp vec3 tmpvar_6;
  tmpvar_4.xy = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  tmpvar_4.zw = ((_glesMultiTexCoord0.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
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
  tmpvar_5 = tmpvar_12;
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
  tmpvar_6 = shlight_3;
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
  tmpvar_36 = (tmpvar_6 + ((((unity_LightColor[0].xyz * tmpvar_35.x) + (unity_LightColor[1].xyz * tmpvar_35.y)) + (unity_LightColor[2].xyz * tmpvar_35.z)) + (unity_LightColor[3].xyz * tmpvar_35.w)));
  tmpvar_6 = tmpvar_36;
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = tmpvar_4;
  xlv_TEXCOORD1 = (tmpvar_11 * (((_World2Object * tmpvar_13).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = tmpvar_5;
  xlv_TEXCOORD3 = tmpvar_6;
}



#endif
#ifdef FRAGMENT

varying lowp vec3 xlv_TEXCOORD3;
varying lowp vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp float _Parallax;
uniform mediump float _Gloss;
uniform mediump float _Shininess;
uniform lowp vec4 _Color;
uniform sampler2D _ParallaxMap;
uniform sampler2D _SpecMap;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform lowp vec4 _LightColor0;
void main ()
{
  lowp vec4 c_1;
  mediump vec3 tmpvar_2;
  mediump vec3 tmpvar_3;
  mediump vec4 spec_4;
  mediump float h_5;
  lowp float tmpvar_6;
  tmpvar_6 = texture2D (_ParallaxMap, xlv_TEXCOORD0.zw).w;
  h_5 = tmpvar_6;
  highp vec2 tmpvar_7;
  mediump float height_8;
  height_8 = _Parallax;
  mediump vec3 viewDir_9;
  viewDir_9 = xlv_TEXCOORD1;
  highp vec3 v_10;
  mediump float tmpvar_11;
  tmpvar_11 = ((h_5 * height_8) - (height_8 / 2.0));
  mediump vec3 tmpvar_12;
  tmpvar_12 = normalize(viewDir_9);
  v_10 = tmpvar_12;
  v_10.z = (v_10.z + 0.42);
  tmpvar_7 = (tmpvar_11 * (v_10.xy / v_10.z));
  highp vec2 tmpvar_13;
  tmpvar_13 = (xlv_TEXCOORD0.xy + tmpvar_7);
  highp vec2 tmpvar_14;
  tmpvar_14 = (xlv_TEXCOORD0.zw + tmpvar_7);
  lowp vec4 tmpvar_15;
  tmpvar_15 = texture2D (_MainTex, tmpvar_13);
  lowp vec3 tmpvar_16;
  tmpvar_16 = (tmpvar_15.xyz * _Color.xyz);
  tmpvar_2 = tmpvar_16;
  lowp vec4 tmpvar_17;
  tmpvar_17 = texture2D (_SpecMap, tmpvar_13);
  spec_4 = tmpvar_17;
  mediump vec3 tmpvar_18;
  tmpvar_18 = ((tmpvar_15.w * spec_4.xyz) * _Gloss);
  lowp vec3 normal_19;
  normal_19.xy = ((texture2D (_BumpMap, tmpvar_14).wy * 2.0) - 1.0);
  normal_19.z = sqrt(((1.0 - (normal_19.x * normal_19.x)) - (normal_19.y * normal_19.y)));
  tmpvar_3 = normal_19;
  highp vec3 tmpvar_20;
  tmpvar_20 = normalize(xlv_TEXCOORD1);
  mediump vec3 lightDir_21;
  lightDir_21 = xlv_TEXCOORD2;
  mediump vec3 viewDir_22;
  viewDir_22 = tmpvar_20;
  mediump vec4 c_23;
  mediump vec3 specCol_24;
  highp float nh_25;
  mediump float tmpvar_26;
  tmpvar_26 = max (0.0, dot (tmpvar_3, normalize((lightDir_21 + viewDir_22))));
  nh_25 = tmpvar_26;
  mediump float arg1_27;
  arg1_27 = (32.0 * _Shininess);
  highp vec3 tmpvar_28;
  tmpvar_28 = (pow (nh_25, arg1_27) * tmpvar_18);
  specCol_24 = tmpvar_28;
  c_23.xyz = ((((tmpvar_2 * _LightColor0.xyz) * max (0.0, dot (tmpvar_3, lightDir_21))) + (_LightColor0.xyz * specCol_24)) * 2.0);
  c_23.w = 0.0;
  c_1 = c_23;
  mediump vec3 tmpvar_29;
  tmpvar_29 = (c_1.xyz + (tmpvar_2 * xlv_TEXCOORD3));
  c_1.xyz = tmpvar_29;
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
Vector 31 [_BumpMap_ST]
"agal_vs
c32 1.0 0.0 0.0 0.0
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
aaaaaaaaaeaaaiaccaaaaaaaabaaaaaaaaaaaaaaaaaaaaaa mov r4.w, c32.x
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
abaaaaaaabaaapacacaaaaoeacaaaaaacaaaaaaaabaaaaaa add r1, r2, c32.x
bdaaaaaaacaaaeacaeaaaaoeacaaaaaabiaaaaoeabaaaaaa dp4 r2.z, r4, c24
bdaaaaaaacaaacacaeaaaaoeacaaaaaabhaaaaoeabaaaaaa dp4 r2.y, r4, c23
bdaaaaaaacaaabacaeaaaaoeacaaaaaabgaaaaoeabaaaaaa dp4 r2.x, r4, c22
afaaaaaaabaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rcp r1.x, r1.x
afaaaaaaabaaacacabaaaaffacaaaaaaaaaaaaaaaaaaaaaa rcp r1.y, r1.y
afaaaaaaabaaaiacabaaaappacaaaaaaaaaaaaaaaaaaaaaa rcp r1.w, r1.w
afaaaaaaabaaaeacabaaaakkacaaaaaaaaaaaaaaaaaaaaaa rcp r1.z, r1.z
ahaaaaaaaaaaapacaaaaaaoeacaaaaaacaaaaaffabaaaaaa max r0, r0, c32.y
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
abaaaaaaacaaahacacaaaakeacaaaaaaaaaaaakeacaaaaaa add r2.xyz, r2.xyzz, r0.xyzz
abaaaaaaadaaahaeacaaaakeacaaaaaaabaaaakeacaaaaaa add v3.xyz, r2.xyzz, r1.xyzz
aaaaaaaaaaaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, a5
adaaaaaaabaaahacabaaaancaaaaaaaaaaaaaaajacaaaaaa mul r1.xyz, a1.zxyw, r0.yzxx
aaaaaaaaaaaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, a5
adaaaaaaafaaahacabaaaamjaaaaaaaaaaaaaafcacaaaaaa mul r5.xyz, a1.yzxw, r0.zxyy
acaaaaaaaaaaahacafaaaakeacaaaaaaabaaaakeacaaaaaa sub r0.xyz, r5.xyzz, r1.xyzz
adaaaaaaadaaahacaaaaaakeacaaaaaaafaaaappaaaaaaaa mul r3.xyz, r0.xyzz, a5.w
aaaaaaaaaaaaapacakaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0, c10
bdaaaaaaaeaaaeacanaaaaoeabaaaaaaaaaaaaoeacaaaaaa dp4 r4.z, c13, r0
aaaaaaaaaaaaapacajaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0, c9
aaaaaaaaabaaaiaccaaaaaaaabaaaaaaaaaaaaaaaaaaaaaa mov r1.w, c32.x
aaaaaaaaabaaahacamaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r1.xyz, c12
bdaaaaaaaeaaacacanaaaaoeabaaaaaaaaaaaaoeacaaaaaa dp4 r4.y, c13, r0
bdaaaaaaacaaaeacabaaaaoeacaaaaaaakaaaaoeabaaaaaa dp4 r2.z, r1, c10
bdaaaaaaacaaabacabaaaaoeacaaaaaaaiaaaaoeabaaaaaa dp4 r2.x, r1, c8
bdaaaaaaacaaacacabaaaaoeacaaaaaaajaaaaoeabaaaaaa dp4 r2.y, r1, c9
adaaaaaaafaaahacacaaaakeacaaaaaabnaaaappabaaaaaa mul r5.xyz, r2.xyzz, c29.w
acaaaaaaacaaahacafaaaakeacaaaaaaaaaaaaoeaaaaaaaa sub r2.xyz, r5.xyzz, a0
aaaaaaaaabaaapacaiaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r1, c8
bdaaaaaaaeaaabacanaaaaoeabaaaaaaabaaaaoeacaaaaaa dp4 r4.x, c13, r1
bcaaaaaaabaaacaeacaaaakeacaaaaaaadaaaakeacaaaaaa dp3 v1.y, r2.xyzz, r3.xyzz
bcaaaaaaacaaacaeadaaaakeacaaaaaaaeaaaakeacaaaaaa dp3 v2.y, r3.xyzz, r4.xyzz
bcaaaaaaabaaaeaeabaaaaoeaaaaaaaaacaaaakeacaaaaaa dp3 v1.z, a1, r2.xyzz
bcaaaaaaabaaabaeacaaaakeacaaaaaaafaaaaoeaaaaaaaa dp3 v1.x, r2.xyzz, a5
bcaaaaaaacaaaeaeabaaaaoeaaaaaaaaaeaaaakeacaaaaaa dp3 v2.z, a1, r4.xyzz
bcaaaaaaacaaabaeafaaaaoeaaaaaaaaaeaaaakeacaaaaaa dp3 v2.x, a5, r4.xyzz
adaaaaaaafaaamacadaaaaeeaaaaaaaabpaaaaeeabaaaaaa mul r5.zw, a3.xyxy, c31.xyxy
abaaaaaaaaaaamaeafaaaaopacaaaaaabpaaaaoeabaaaaaa add v0.zw, r5.wwzw, c31
adaaaaaaafaaadacadaaaaoeaaaaaaaaboaaaaoeabaaaaaa mul r5.xy, a3, c30
abaaaaaaaaaaadaeafaaaafeacaaaaaaboaaaaooabaaaaaa add v0.xy, r5.xyyy, c30.zwzw
bdaaaaaaaaaaaiadaaaaaaoeaaaaaaaaadaaaaoeabaaaaaa dp4 o0.w, a0, c3
bdaaaaaaaaaaaeadaaaaaaoeaaaaaaaaacaaaaoeabaaaaaa dp4 o0.z, a0, c2
bdaaaaaaaaaaacadaaaaaaoeaaaaaaaaabaaaaoeabaaaaaa dp4 o0.y, a0, c1
bdaaaaaaaaaaabadaaaaaaoeaaaaaaaaaaaaaaoeabaaaaaa dp4 o0.x, a0, c0
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
ConstBuffer "$Globals" 112 // 112 used size, 9 vars
Vector 80 [_MainTex_ST] 4
Vector 96 [_BumpMap_ST] 4
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
// 65 instructions, 6 temp regs, 0 temp arrays:
// ALU 36 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0_level_9_3
eefiecedllmhckkmfacbcmknbgfccfbfcmjempijabaaaaaapabaaaaaaeaaaaaa
daaaaaaapmafaaaaiiapaaaafabaaaaaebgpgodjmeafaaaameafaaaaaaacpopp
eiafaaaahmaaaaaaahaaceaaaaaahiaaaaaahiaaaaaaceaaabaahiaaaaaaafaa
acaaabaaaaaaaaaaabaaaeaaabaaadaaaaaaaaaaacaaaaaaabaaaeaaaaaaaaaa
acaaacaaaiaaafaaaaaaaaaaacaabcaaahaaanaaaaaaaaaaadaaaaaaaeaabeaa
aaaaaaaaadaaamaaajaabiaaaaaaaaaaaaaaaaaaabacpoppfbaaaaafcbaaapka
aaaaiadpaaaaaaaaaaaaaaaaaaaaaaaabpaaaaacafaaaaiaaaaaapjabpaaaaac
afaaabiaabaaapjabpaaaaacafaaaciaacaaapjabpaaaaacafaaadiaadaaapja
aeaaaaaeaaaaadoaadaaoejaabaaoekaabaaookaaeaaaaaeaaaaamoaadaaeeja
acaaeekaacaaoekaabaaaaacaaaaapiaaeaaoekaafaaaaadabaaahiaaaaaffia
bnaaoekaaeaaaaaeabaaahiabmaaoekaaaaaaaiaabaaoeiaaeaaaaaeaaaaahia
boaaoekaaaaakkiaabaaoeiaaeaaaaaeaaaaahiabpaaoekaaaaappiaaaaaoeia
aiaaaaadacaaaboaabaaoejaaaaaoeiaabaaaaacabaaahiaacaaoejaafaaaaad
acaaahiaabaanciaabaamjjaaeaaaaaeabaaahiaabaamjiaabaancjaacaaoeib
afaaaaadabaaahiaabaaoeiaabaappjaaiaaaaadacaaacoaabaaoeiaaaaaoeia
aiaaaaadacaaaeoaacaaoejaaaaaoeiaabaaaaacaaaaahiaadaaoekaafaaaaad
acaaahiaaaaaffiabnaaoekaaeaaaaaeaaaaaliabmaakekaaaaaaaiaacaakeia
aeaaaaaeaaaaahiaboaaoekaaaaakkiaaaaapeiaacaaaaadaaaaahiaaaaaoeia
bpaaoekaaeaaaaaeaaaaahiaaaaaoeiacaaappkaaaaaoejbaiaaaaadabaaaboa
abaaoejaaaaaoeiaaiaaaaadabaaacoaabaaoeiaaaaaoeiaaiaaaaadabaaaeoa
acaaoejaaaaaoeiaafaaaaadaaaaahiaaaaaffjabjaaoekaaeaaaaaeaaaaahia
biaaoekaaaaaaajaaaaaoeiaaeaaaaaeaaaaahiabkaaoekaaaaakkjaaaaaoeia
aeaaaaaeaaaaahiablaaoekaaaaappjaaaaaoeiaacaaaaadabaaapiaaaaakkib
ahaaoekaacaaaaadacaaapiaaaaaaaibafaaoekaacaaaaadaaaaapiaaaaaffib
agaaoekaafaaaaadadaaahiaacaaoejacaaappkaafaaaaadaeaaahiaadaaffia
bjaaoekaaeaaaaaeadaaaliabiaakekaadaaaaiaaeaakeiaaeaaaaaeadaaahia
bkaaoekaadaakkiaadaapeiaafaaaaadaeaaapiaaaaaoeiaadaaffiaafaaaaad
aaaaapiaaaaaoeiaaaaaoeiaaeaaaaaeaaaaapiaacaaoeiaacaaoeiaaaaaoeia
aeaaaaaeacaaapiaacaaoeiaadaaaaiaaeaaoeiaaeaaaaaeacaaapiaabaaoeia
adaakkiaacaaoeiaaeaaaaaeaaaaapiaabaaoeiaabaaoeiaaaaaoeiaahaaaaac
abaaabiaaaaaaaiaahaaaaacabaaaciaaaaaffiaahaaaaacabaaaeiaaaaakkia
ahaaaaacabaaaiiaaaaappiaabaaaaacaeaaabiacbaaaakaaeaaaaaeaaaaapia
aaaaoeiaaiaaoekaaeaaaaiaafaaaaadabaaapiaabaaoeiaacaaoeiaalaaaaad
abaaapiaabaaoeiacbaaffkaagaaaaacacaaabiaaaaaaaiaagaaaaacacaaacia
aaaaffiaagaaaaacacaaaeiaaaaakkiaagaaaaacacaaaiiaaaaappiaafaaaaad
aaaaapiaabaaoeiaacaaoeiaafaaaaadabaaahiaaaaaffiaakaaoekaaeaaaaae
abaaahiaajaaoekaaaaaaaiaabaaoeiaaeaaaaaeaaaaahiaalaaoekaaaaakkia
abaaoeiaaeaaaaaeaaaaahiaamaaoekaaaaappiaaaaaoeiaabaaaaacadaaaiia
cbaaaakaajaaaaadabaaabiaanaaoekaadaaoeiaajaaaaadabaaaciaaoaaoeka
adaaoeiaajaaaaadabaaaeiaapaaoekaadaaoeiaafaaaaadacaaapiaadaacjia
adaakeiaajaaaaadaeaaabiabaaaoekaacaaoeiaajaaaaadaeaaaciabbaaoeka
acaaoeiaajaaaaadaeaaaeiabcaaoekaacaaoeiaacaaaaadabaaahiaabaaoeia
aeaaoeiaafaaaaadaaaaaiiaadaaffiaadaaffiaaeaaaaaeaaaaaiiaadaaaaia
adaaaaiaaaaappibaeaaaaaeabaaahiabdaaoekaaaaappiaabaaoeiaacaaaaad
adaaahoaaaaaoeiaabaaoeiaafaaaaadaaaaapiaaaaaffjabfaaoekaaeaaaaae
aaaaapiabeaaoekaaaaaaajaaaaaoeiaaeaaaaaeaaaaapiabgaaoekaaaaakkja
aaaaoeiaaeaaaaaeaaaaapiabhaaoekaaaaappjaaaaaoeiaaeaaaaaeaaaaadma
aaaappiaaaaaoekaaaaaoeiaabaaaaacaaaaammaaaaaoeiappppaaaafdeieefc
ieajaaaaeaaaabaagbacaaaafjaaaaaeegiocaaaaaaaaaaaahaaaaaafjaaaaae
egiocaaaabaaaaaaafaaaaaafjaaaaaeegiocaaaacaaaaaabjaaaaaafjaaaaae
egiocaaaadaaaaaabfaaaaaafpaaaaadpcbabaaaaaaaaaaafpaaaaadpcbabaaa
abaaaaaafpaaaaadhcbabaaaacaaaaaafpaaaaaddcbabaaaadaaaaaaghaaaaae
pccabaaaaaaaaaaaabaaaaaagfaaaaadpccabaaaabaaaaaagfaaaaadhccabaaa
acaaaaaagfaaaaadhccabaaaadaaaaaagfaaaaadhccabaaaaeaaaaaagiaaaaac
agaaaaaadiaaaaaipcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaadaaaaaa
abaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaaaaaaaaaagbabaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaa
acaaaaaakgbkbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpccabaaaaaaaaaaa
egiocaaaadaaaaaaadaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaal
dccabaaaabaaaaaaegbabaaaadaaaaaaegiacaaaaaaaaaaaafaaaaaaogikcaaa
aaaaaaaaafaaaaaadcaaaaalmccabaaaabaaaaaaagbebaaaadaaaaaaagiecaaa
aaaaaaaaagaaaaaakgiocaaaaaaaaaaaagaaaaaadiaaaaahhcaabaaaaaaaaaaa
jgbebaaaabaaaaaacgbjbaaaacaaaaaadcaaaaakhcaabaaaaaaaaaaajgbebaaa
acaaaaaacgbjbaaaabaaaaaaegacbaiaebaaaaaaaaaaaaaadiaaaaahhcaabaaa
aaaaaaaaegacbaaaaaaaaaaapgbpbaaaabaaaaaadiaaaaajhcaabaaaabaaaaaa
fgifcaaaabaaaaaaaeaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaa
abaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaaabaaaaaaaeaaaaaaegacbaaa
abaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaa
abaaaaaaaeaaaaaaegacbaaaabaaaaaaaaaaaaaihcaabaaaabaaaaaaegacbaaa
abaaaaaaegiccaaaadaaaaaabdaaaaaadcaaaaalhcaabaaaabaaaaaaegacbaaa
abaaaaaapgipcaaaadaaaaaabeaaaaaaegbcbaiaebaaaaaaaaaaaaaabaaaaaah
cccabaaaacaaaaaaegacbaaaaaaaaaaaegacbaaaabaaaaaabaaaaaahbccabaaa
acaaaaaaegbcbaaaabaaaaaaegacbaaaabaaaaaabaaaaaaheccabaaaacaaaaaa
egbcbaaaacaaaaaaegacbaaaabaaaaaadiaaaaajhcaabaaaabaaaaaafgifcaaa
acaaaaaaaaaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaaabaaaaaa
egiccaaaadaaaaaabaaaaaaaagiacaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaa
dcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaaacaaaaaa
aaaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaa
bdaaaaaapgipcaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaabaaaaaahcccabaaa
adaaaaaaegacbaaaaaaaaaaaegacbaaaabaaaaaabaaaaaahbccabaaaadaaaaaa
egbcbaaaabaaaaaaegacbaaaabaaaaaabaaaaaaheccabaaaadaaaaaaegbcbaaa
acaaaaaaegacbaaaabaaaaaadgaaaaaficaabaaaaaaaaaaaabeaaaaaaaaaiadp
diaaaaaihcaabaaaabaaaaaaegbcbaaaacaaaaaapgipcaaaadaaaaaabeaaaaaa
diaaaaaihcaabaaaacaaaaaafgafbaaaabaaaaaaegiccaaaadaaaaaaanaaaaaa
dcaaaaaklcaabaaaabaaaaaaegiicaaaadaaaaaaamaaaaaaagaabaaaabaaaaaa
egaibaaaacaaaaaadcaaaaakhcaabaaaaaaaaaaaegiccaaaadaaaaaaaoaaaaaa
kgakbaaaabaaaaaaegadbaaaabaaaaaabbaaaaaibcaabaaaabaaaaaaegiocaaa
acaaaaaabcaaaaaaegaobaaaaaaaaaaabbaaaaaiccaabaaaabaaaaaaegiocaaa
acaaaaaabdaaaaaaegaobaaaaaaaaaaabbaaaaaiecaabaaaabaaaaaaegiocaaa
acaaaaaabeaaaaaaegaobaaaaaaaaaaadiaaaaahpcaabaaaacaaaaaajgacbaaa
aaaaaaaaegakbaaaaaaaaaaabbaaaaaibcaabaaaadaaaaaaegiocaaaacaaaaaa
bfaaaaaaegaobaaaacaaaaaabbaaaaaiccaabaaaadaaaaaaegiocaaaacaaaaaa
bgaaaaaaegaobaaaacaaaaaabbaaaaaiecaabaaaadaaaaaaegiocaaaacaaaaaa
bhaaaaaaegaobaaaacaaaaaaaaaaaaahhcaabaaaabaaaaaaegacbaaaabaaaaaa
egacbaaaadaaaaaadiaaaaahicaabaaaaaaaaaaabkaabaaaaaaaaaaabkaabaaa
aaaaaaaadcaaaaakicaabaaaaaaaaaaaakaabaaaaaaaaaaaakaabaaaaaaaaaaa
dkaabaiaebaaaaaaaaaaaaaadcaaaaakhcaabaaaabaaaaaaegiccaaaacaaaaaa
biaaaaaapgapbaaaaaaaaaaaegacbaaaabaaaaaadiaaaaaihcaabaaaacaaaaaa
fgbfbaaaaaaaaaaaegiccaaaadaaaaaaanaaaaaadcaaaaakhcaabaaaacaaaaaa
egiccaaaadaaaaaaamaaaaaaagbabaaaaaaaaaaaegacbaaaacaaaaaadcaaaaak
hcaabaaaacaaaaaaegiccaaaadaaaaaaaoaaaaaakgbkbaaaaaaaaaaaegacbaaa
acaaaaaadcaaaaakhcaabaaaacaaaaaaegiccaaaadaaaaaaapaaaaaapgbpbaaa
aaaaaaaaegacbaaaacaaaaaaaaaaaaajpcaabaaaadaaaaaafgafbaiaebaaaaaa
acaaaaaaegiocaaaacaaaaaaadaaaaaadiaaaaahpcaabaaaaeaaaaaafgafbaaa
aaaaaaaaegaobaaaadaaaaaadiaaaaahpcaabaaaadaaaaaaegaobaaaadaaaaaa
egaobaaaadaaaaaaaaaaaaajpcaabaaaafaaaaaaagaabaiaebaaaaaaacaaaaaa
egiocaaaacaaaaaaacaaaaaaaaaaaaajpcaabaaaacaaaaaakgakbaiaebaaaaaa
acaaaaaaegiocaaaacaaaaaaaeaaaaaadcaaaaajpcaabaaaaeaaaaaaegaobaaa
afaaaaaaagaabaaaaaaaaaaaegaobaaaaeaaaaaadcaaaaajpcaabaaaaaaaaaaa
egaobaaaacaaaaaakgakbaaaaaaaaaaaegaobaaaaeaaaaaadcaaaaajpcaabaaa
adaaaaaaegaobaaaafaaaaaaegaobaaaafaaaaaaegaobaaaadaaaaaadcaaaaaj
pcaabaaaacaaaaaaegaobaaaacaaaaaaegaobaaaacaaaaaaegaobaaaadaaaaaa
eeaaaaafpcaabaaaadaaaaaaegaobaaaacaaaaaadcaaaaanpcaabaaaacaaaaaa
egaobaaaacaaaaaaegiocaaaacaaaaaaafaaaaaaaceaaaaaaaaaiadpaaaaiadp
aaaaiadpaaaaiadpaoaaaaakpcaabaaaacaaaaaaaceaaaaaaaaaiadpaaaaiadp
aaaaiadpaaaaiadpegaobaaaacaaaaaadiaaaaahpcaabaaaaaaaaaaaegaobaaa
aaaaaaaaegaobaaaadaaaaaadeaaaaakpcaabaaaaaaaaaaaegaobaaaaaaaaaaa
aceaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaadiaaaaahpcaabaaaaaaaaaaa
egaobaaaacaaaaaaegaobaaaaaaaaaaadiaaaaaihcaabaaaacaaaaaafgafbaaa
aaaaaaaaegiccaaaacaaaaaaahaaaaaadcaaaaakhcaabaaaacaaaaaaegiccaaa
acaaaaaaagaaaaaaagaabaaaaaaaaaaaegacbaaaacaaaaaadcaaaaakhcaabaaa
aaaaaaaaegiccaaaacaaaaaaaiaaaaaakgakbaaaaaaaaaaaegacbaaaacaaaaaa
dcaaaaakhcaabaaaaaaaaaaaegiccaaaacaaaaaaajaaaaaapgapbaaaaaaaaaaa
egacbaaaaaaaaaaaaaaaaaahhccabaaaaeaaaaaaegacbaaaaaaaaaaaegacbaaa
abaaaaaadoaaaaabejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaaadaaaaaa
abaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaaahahaaaa
laaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaaabaaaaaa
aaaaaaaaadaaaaaaaeaaaaaaapaaaaaaljaaaaaaaaaaaaaaaaaaaaaaadaaaaaa
afaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfcenebemaa
feeffiedepepfceeaaedepemepfcaaklepfdeheojiaaaaaaafaaaaaaaiaaaaaa
iaaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaabaaaaaaapaaaaaaimaaaaaaabaaaaaaaaaaaaaaadaaaaaa
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
Vector 33 [_BumpMap_ST]
"!!ARBvp1.0
# 80 ALU
PARAM c[34] = { { 1, 0, 0.5 },
		state.matrix.mvp,
		program.local[5..33] };
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
DP4 R3.z, R0, c[29];
DP4 R3.y, R0, c[28];
DP4 R3.x, R0, c[27];
MUL R1.w, R3, R3;
MOV R0.w, c[0].x;
MAD R0.x, R4, R4, -R1.w;
DP4 R2.z, R4, c[26];
DP4 R2.y, R4, c[25];
DP4 R2.x, R4, c[24];
ADD R2.xyz, R2, R3;
MUL R3.xyz, R0.x, c[30];
ADD R3.xyz, R2, R3;
MOV R0.xyz, vertex.attrib[14];
MUL R2.xyz, vertex.normal.zxyw, R0.yzxw;
ADD result.texcoord[3].xyz, R3, R1;
MAD R1.xyz, vertex.normal.yzxw, R0.zxyw, -R2;
MOV R0.xyz, c[13];
DP4 R2.z, R0, c[11];
DP4 R2.x, R0, c[9];
DP4 R2.y, R0, c[10];
MAD R0.xyz, R2, c[31].w, -vertex.position;
MUL R2.xyz, R1, vertex.attrib[14].w;
MOV R1, c[15];
DP4 R3.z, R1, c[11];
DP4 R3.x, R1, c[9];
DP4 R3.y, R1, c[10];
DP3 result.texcoord[1].y, R0, R2;
DP3 result.texcoord[1].z, vertex.normal, R0;
DP3 result.texcoord[1].x, R0, vertex.attrib[14];
DP4 R0.w, vertex.position, c[4];
DP4 R0.z, vertex.position, c[3];
DP4 R0.x, vertex.position, c[1];
DP4 R0.y, vertex.position, c[2];
MUL R1.xyz, R0.xyww, c[0].z;
MUL R1.y, R1, c[14].x;
DP3 result.texcoord[2].y, R2, R3;
DP3 result.texcoord[2].z, vertex.normal, R3;
DP3 result.texcoord[2].x, vertex.attrib[14], R3;
ADD result.texcoord[4].xy, R1, R1.z;
MOV result.position, R0;
MOV result.texcoord[4].zw, R0;
MAD result.texcoord[0].zw, vertex.texcoord[0].xyxy, c[33].xyxy, c[33];
MAD result.texcoord[0].xy, vertex.texcoord[0], c[32], c[32].zwzw;
END
# 80 instructions, 5 R-regs
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
Vector 33 [_BumpMap_ST]
"vs_2_0
; 83 ALU
def c34, 1.00000000, 0.00000000, 0.50000000, 0
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
mov r4.w, c34.x
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
add r1, r2, c34.x
dp4 r2.z, r4, c26
dp4 r2.y, r4, c25
dp4 r2.x, r4, c24
rcp r1.x, r1.x
rcp r1.y, r1.y
rcp r1.w, r1.w
rcp r1.z, r1.z
max r0, r0, c34.y
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
add r2.xyz, r2, r0
add oT3.xyz, r2, r1
mov r0.xyz, v1
mul r1.xyz, v2.zxyw, r0.yzxw
mov r0.xyz, v1
mad r0.xyz, v2.yzxw, r0.zxyw, -r1
mul r3.xyz, r0, v1.w
mov r0, c10
dp4 r4.z, c15, r0
mov r0, c9
dp4 r4.y, c15, r0
mov r1.w, c34.x
mov r1.xyz, c12
dp4 r0.w, v0, c3
dp4 r0.z, v0, c2
dp4 r2.z, r1, c10
dp4 r2.x, r1, c8
dp4 r2.y, r1, c9
mad r2.xyz, r2, c31.w, -v0
mov r1, c8
dp4 r4.x, c15, r1
dp4 r0.x, v0, c0
dp4 r0.y, v0, c1
mul r1.xyz, r0.xyww, c34.z
mul r1.y, r1, c13.x
dp3 oT1.y, r2, r3
dp3 oT2.y, r3, r4
dp3 oT1.z, v2, r2
dp3 oT1.x, r2, v1
dp3 oT2.z, v2, r4
dp3 oT2.x, v1, r4
mad oT4.xy, r1.z, c14.zwzw, r1
mov oPos, r0
mov oT4.zw, r0
mad oT0.zw, v3.xyxy, c33.xyxy, c33
mad oT0.xy, v3, c32, c32.zwzw
"
}

SubProgram "xbox360 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" "VERTEXLIGHT_ON" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 33 [_BumpMap_ST]
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
// ALU: 96.00 (72 instructions), vertex: 32, texture: 0,
//   sequencer: 32,  13 GPRs, 12 threads,
// Performance (if enough threads): ~96 cycles per vector
// * Vertex cycle estimates are assuming 3 vfetch_minis for every vfetch_full,
//     with <= 32 bytes per vfetch_full group.

"vs_360
backbbabaaaaaediaaaaaedmaaaaaaaaaaaaaaceaaaaadjiaaaaadmaaaaaaaaa
aaaaaaaaaaaaadhaaaaaaabmaaaaadgdpppoadaaaaaaaabgaaaaaabmaaaaaaaa
aaaaadfmaaaaabneaaacaacbaaabaaaaaaaaaboaaaaaaaaaaaaaabpaaaacaaca
aaabaaaaaaaaaboaaaaaaaaaaaaaabpmaaacaabhaaaeaaaaaaaaacamaaaaaaaa
aaaaacbmaaacaaabaaabaaaaaaaaaboaaaaaaaaaaaaaaccoaaacaaacaaabaaaa
aaaaaboaaaaaaaaaaaaaacdmaaacaablaaaeaaaaaaaaacamaaaaaaaaaaaaacek
aaacaaaaaaabaaaaaaaaacgaaaaaaaaaaaaaachaaaacaaadaaabaaaaaaaaaboa
aaaaaaaaaaaaacifaaacaabdaaaeaaaaaaaaacamaaaaaaaaaaaaacjiaaacaaah
aaabaaaaaaaaaboaaaaaaaaaaaaaacklaaacaaaeaaabaaaaaaaaaboaaaaaaaaa
aaaaaclnaaacaaafaaabaaaaaaaaaboaaaaaaaaaaaaaacmpaaacaaagaaabaaaa
aaaaaboaaaaaaaaaaaaaacobaaacaaaiaaaeaaaaaaaaacpeaaaaaaaaaaaaadae
aaacaaaoaaabaaaaaaaaaboaaaaaaaaaaaaaadapaaacaaanaaabaaaaaaaaaboa
aaaaaaaaaaaaadbkaaacaaamaaabaaaaaaaaaboaaaaaaaaaaaaaadcfaaacaabb
aaabaaaaaaaaaboaaaaaaaaaaaaaaddaaaacaabaaaabaaaaaaaaaboaaaaaaaaa
aaaaaddlaaacaaapaaabaaaaaaaaaboaaaaaaaaaaaaaadegaaacaabcaaabaaaa
aaaaaboaaaaaaaaaaaaaadfaaaacaabpaaabaaaaaaaaaboaaaaaaaaafpechfgn
haengbhafpfdfeaaaaabaaadaaabaaaeaaabaaaaaaaaaaaafpengbgjgofegfhi
fpfdfeaafpepgcgkgfgdhedcfhgphcgmgeaaklklaaadaaadaaaeaaaeaaabaaaa
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
aaaaaaeaaaaaadpmaaebaaamaaaaaaaaaaaaaaaaaaaaeekfaaaaaaabaaaaaaae
aaaaaaalaaaaacjaaabaaaaiaaaagaajaaaadaakaacafaalaaaapafaaaachbfb
aaafhcfcaaaihdfdaaajpefeaaaaaacpaaaabadaaaaaaacjaaaaaackaaaabacl
aaaaaacmaaaaaacnaaaabacoaaaabafdaaaaaaciaaaabadlaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaadpaaaaaaaaaaaaaadpiaaaaaaaaaaaaapaffeaaiaaaabcaa
mcaaaaaaaaaafaamaaaabcaameaaaaaaaaaagabbgabhbcaabcaaaaaaaaaagabn
gacdbcaabcaaaaaaaaaagacjgacpbcaabcaaaaaaaaaagadfgadlbcaabcaaaaaa
aaaagaebgaehbcaabcaaaaaaaaaagaenbafdbcaaccaaaaaaafpibaaaaaaaaanb
aaaaaaaaafpikaaaaaaaagiiaaaaaaaaafpieaaaaaaaaoiiaaaaaaaaafpidaaa
aaaaacdpaaaaaaaamiapaaaaaamgiiaakbabbgaamiapaaaaaalbnapiklabbfaa
miapaaaaaagmdepiklabbeaamiapaaamaablnajeklabbdaamiapiadoaananaaa
ocamamaamiahaaaaaamamgmaalbnaabomiahaaacaaleblaacbboadaamiahaaai
aamamgleclbnadacmiahaaahaalogfaaobaeakaamiahaaagaalelbleclbmaaaa
mialaaaaaagfblaakbaebpaamiahaaacaamgleaakbabbkaamiahaaacaalbmale
klabbjacmiahaaafaalbleaakbaabjaamiahaaagaamagmleclblaaagmiahaaah
abgflomaolaeakahmiahaaaiaalelbleclbmadaimiahaaaiaamagmleclbladai
miahaaajaamablaaobahakaamiahaaalabmabllpklagbpabmiahaaaaaagmlema
klaabiafmiahaaabaagmleleklabbiacmialaaabaabllemaklabbhabmiahaaah
aabllemaklaabhaaceihahaaaamagmgmkbamppiaaibpadafaalehcgmobahahah
aicpadagaegmaamgkaabaeahbeapaaacaflbaablkaabagabmiamiaaeaanlnlaa
ocamamaamiabiaabaaloloaapaalakaamiaciaabaaloloaapaajalaamiaeiaab
aaloloaapaalaeaamiabiaacaaloloaapaaiakaamiaciaacaaloloaapaajaiaa
miaeiaacaaloloaapaaiaeaamiadiaaaaabklabkiladcacamiamiaaaaaagkmag
iladcbcbaebbaiaeaadoangmepamahafbeacaaaeabdoanblgpanahabaeceaiae
aadoanlbepaoahafbeabaaababkhkhblkpafapabaeecaiabaakhkhmgipafbaaf
beaeaaababkhkhblkpafbbabaeipaiafaapipiblmbacacafkiipaaacaapilbeb
mbacahabmiapaaafaajejepiolaiaiafmiapaaacaajemgpiolaiahacmiadiaae
aamgbkbiklaaacaamiapaaacaajegmaaolagahacmiapaaaaaaaaaapiolagagaf
geihababaalologboaaeabadmiahaaabaabllemnklabbcabmiapaaaeaapipimg
ilaaahppfibaaaaaaaaaaagmocaaaaiaficaaaaaaaaaaalbocaaaaiafieaaaaa
aaaaaamgocaaaaiafiiaaaaaaaaaaablocaaaaiamiapaaaaaapiaaaaobacaaaa
emipaaadaapilbmgkcaappaeemecacaaaamgblgmobadaaaeemciacacaagmmgbl
obadacaeembbaaacaabllblbobadacaemiaeaaaaaalbgmaaobadaaaakibhacae
aalmmaecibacakalkiciacaeaamgblicmbaeadalkieoacafaabgpmmaibacaial
beahaaaaaabbmalbkbaaajafambiafaaaamgmggmobaaadadbeahaaaaaabebamg
oaafaaacamihacaaaamabalboaaaaeadmiahaaaaaamabaaaoaaaacaamiahiaad
aalemaaaoaabaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
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
Vector 447 [_BumpMap_ST]
"sce_vp_rsx // 69 instructions using 9 registers
[Configuration]
8
0000004541050900
[Defaults]
1
446 3
000000003f8000003f000000
[Microcode]
1104
00009c6c005d100d8186c0836041fffc00039c6c00400e0c0106c0836041dffc
00001c6c005d300c0186c0836041dffc00019c6c009c120c013fc0c36041dffc
401f9c6c011bf800810040d560607f9c401f9c6c011c0808010400d740619f9c
00011c6c01d0300d8106c0c360403ffc00011c6c01d0200d8106c0c360405ffc
00011c6c01d0100d8106c0c360409ffc00011c6c01d0000d8106c0c360411ffc
00019c6c01d0500d8106c0c360403ffc00021c6c01d0400d8106c0c360405ffc
00001c6c01d0600d8106c0c360403ffc00029c6c01d0a00d8286c0c360405ffc
00029c6c01d0900d8286c0c360409ffc00029c6c01d0800d8286c0c360411ffc
00021c6c0150400c068600c360411ffc00021c6c0150600c068600c360403ffc
00021c6c0150500c068600c360409ffc00031c6c0190a00c0086c0c360405ffc
00031c6c0190900c0086c0c360409ffc00031c6c0190800c0086c0c360411ffc
00001c6c00dce00d8186c0bfe021fffc00009c6c00dd000d8186c0b54221fffc
00019c6c00dcf00d8186c0bfe1a1fffc00041c6c00800243011847436041dffc
00039c6c01000230812187630421dffc401f9c6c0040000d8486c0836041ff80
401f9c6c004000558486c08360407fac00031c6c011c100c0cbfc0e30041dffc
00041c6c009be00e04aa80c36041dffc401f9c6c0140020c0106054360405fa4
00011c6c0080002a8886c3436041fffc00041c6c009d202a908000c360409ffc
00019c6c0080000d8686c3436041fffc00029c6c0080002a8895444360403ffc
00021c6c0040007f8886c08360405ffc00011c6c010000000886c1436121fffc
00009c6c0100000d8286c14361a1fffc401f9c6c00c000081086c09544219fac
00041c6c019c600c0886c0c360405ffc00041c6c019c700c0886c0c360409ffc
00041c6c019c800c0886c0c360411ffc00029c6c010000000880047fe2a03ffc
00019c6c0080000d089a04436041fffc00011c6c0100007f8886c0436121fffc
00001c6c0100000d8086c04360a1fffc00009c6c01dc300d8686c0c360405ffc
00009c6c01dc400d8686c0c360409ffc00009c6c01dc500d8686c0c360411ffc
00009c6c00c0000c1086c08300a1dffc00019c6c009c207f8a8600c36041dffc
00019c6c00c0000c0686c08300a1dffc401f9c6c21400e0c01060540003100a4
00039c6c20800e0c0ebfc08aa029c0fc00021c6c209cd00d8086c0d54025e0fc
00021c6c00dbe02a8186c0836221fffc401f9c6c2140020c0106065fe02240a0
401f9c6c11400e0c0c86008002310020401f9c6c1140000c0e86054aa2288024
401f9c6c1140000c0c8607554224802000009c6c1080000d8486c15fe223e07c
00009c6c029be00d828000c36041fffc00001c6c0080000d8286c0436041fffc
00009c6c009cb02a808600c36041dffc00009c6c011cc000008600c300a1dffc
00001c6c011ca055008600c300a1dffc00001c6c011c907f808600c30021dffc
401f9c6c00c0000c0686c0830021dfa9
"
}

SubProgram "d3d11 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" "VERTEXLIGHT_ON" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 176 // 176 used size, 10 vars
Vector 144 [_MainTex_ST] 4
Vector 160 [_BumpMap_ST] 4
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
// 70 instructions, 7 temp regs, 0 temp arrays:
// ALU 39 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0
eefiecedflgnopjkabjbloighnaihjohacpemdibabaaaaaanaalaaaaadaaaaaa
cmaaaaaapeaaaaaakmabaaaaejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaa
abaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaljaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfc
enebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheolaaaaaaaagaaaaaa
aiaaaaaajiaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaakeaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaabaaaaaaapaaaaaakeaaaaaaabaaaaaaaaaaaaaa
adaaaaaaacaaaaaaahaiaaaakeaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaa
ahaiaaaakeaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaahaiaaaakeaaaaaa
aeaaaaaaaaaaaaaaadaaaaaaafaaaaaaapaaaaaafdfgfpfaepfdejfeejepeoaa
feeffiedepepfceeaaklklklfdeieefcbmakaaaaeaaaabaaihacaaaafjaaaaae
egiocaaaaaaaaaaaalaaaaaafjaaaaaeegiocaaaabaaaaaaagaaaaaafjaaaaae
egiocaaaacaaaaaabjaaaaaafjaaaaaeegiocaaaadaaaaaabfaaaaaafpaaaaad
pcbabaaaaaaaaaaafpaaaaadpcbabaaaabaaaaaafpaaaaadhcbabaaaacaaaaaa
fpaaaaaddcbabaaaadaaaaaaghaaaaaepccabaaaaaaaaaaaabaaaaaagfaaaaad
pccabaaaabaaaaaagfaaaaadhccabaaaacaaaaaagfaaaaadhccabaaaadaaaaaa
gfaaaaadhccabaaaaeaaaaaagfaaaaadpccabaaaafaaaaaagiaaaaacahaaaaaa
diaaaaaipcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaadaaaaaaabaaaaaa
dcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaaaaaaaaaagbabaaaaaaaaaaa
egaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaacaaaaaa
kgbkbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaa
adaaaaaaadaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadgaaaaafpccabaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaaldccabaaaabaaaaaaegbabaaaadaaaaaa
egiacaaaaaaaaaaaajaaaaaaogikcaaaaaaaaaaaajaaaaaadcaaaaalmccabaaa
abaaaaaaagbebaaaadaaaaaaagiecaaaaaaaaaaaakaaaaaakgiocaaaaaaaaaaa
akaaaaaadiaaaaahhcaabaaaabaaaaaajgbebaaaabaaaaaacgbjbaaaacaaaaaa
dcaaaaakhcaabaaaabaaaaaajgbebaaaacaaaaaacgbjbaaaabaaaaaaegacbaia
ebaaaaaaabaaaaaadiaaaaahhcaabaaaabaaaaaaegacbaaaabaaaaaapgbpbaaa
abaaaaaadiaaaaajhcaabaaaacaaaaaafgifcaaaabaaaaaaaeaaaaaaegiccaaa
adaaaaaabbaaaaaadcaaaaalhcaabaaaacaaaaaaegiccaaaadaaaaaabaaaaaaa
agiacaaaabaaaaaaaeaaaaaaegacbaaaacaaaaaadcaaaaalhcaabaaaacaaaaaa
egiccaaaadaaaaaabcaaaaaakgikcaaaabaaaaaaaeaaaaaaegacbaaaacaaaaaa
aaaaaaaihcaabaaaacaaaaaaegacbaaaacaaaaaaegiccaaaadaaaaaabdaaaaaa
dcaaaaalhcaabaaaacaaaaaaegacbaaaacaaaaaapgipcaaaadaaaaaabeaaaaaa
egbcbaiaebaaaaaaaaaaaaaabaaaaaahcccabaaaacaaaaaaegacbaaaabaaaaaa
egacbaaaacaaaaaabaaaaaahbccabaaaacaaaaaaegbcbaaaabaaaaaaegacbaaa
acaaaaaabaaaaaaheccabaaaacaaaaaaegbcbaaaacaaaaaaegacbaaaacaaaaaa
diaaaaajhcaabaaaacaaaaaafgifcaaaacaaaaaaaaaaaaaaegiccaaaadaaaaaa
bbaaaaaadcaaaaalhcaabaaaacaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaa
acaaaaaaaaaaaaaaegacbaaaacaaaaaadcaaaaalhcaabaaaacaaaaaaegiccaaa
adaaaaaabcaaaaaakgikcaaaacaaaaaaaaaaaaaaegacbaaaacaaaaaadcaaaaal
hcaabaaaacaaaaaaegiccaaaadaaaaaabdaaaaaapgipcaaaacaaaaaaaaaaaaaa
egacbaaaacaaaaaabaaaaaahcccabaaaadaaaaaaegacbaaaabaaaaaaegacbaaa
acaaaaaabaaaaaahbccabaaaadaaaaaaegbcbaaaabaaaaaaegacbaaaacaaaaaa
baaaaaaheccabaaaadaaaaaaegbcbaaaacaaaaaaegacbaaaacaaaaaadgaaaaaf
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
abaaaaaabkaabaaaabaaaaaabkaabaaaabaaaaaadcaaaaakicaabaaaabaaaaaa
akaabaaaabaaaaaaakaabaaaabaaaaaadkaabaiaebaaaaaaabaaaaaadcaaaaak
hcaabaaaacaaaaaaegiccaaaacaaaaaabiaaaaaapgapbaaaabaaaaaaegacbaaa
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
aeaaaaaaegacbaaaabaaaaaaegacbaaaacaaaaaadiaaaaaiccaabaaaaaaaaaaa
bkaabaaaaaaaaaaaakiacaaaabaaaaaaafaaaaaadiaaaaakncaabaaaabaaaaaa
agahbaaaaaaaaaaaaceaaaaaaaaaaadpaaaaaaaaaaaaaadpaaaaaadpdgaaaaaf
mccabaaaafaaaaaakgaobaaaaaaaaaaaaaaaaaahdccabaaaafaaaaaakgakbaaa
abaaaaaamgaabaaaabaaaaaadoaaaaab"
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
varying lowp vec3 xlv_TEXCOORD3;
varying lowp vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 _BumpMap_ST;
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
  highp vec4 tmpvar_4;
  lowp vec3 tmpvar_5;
  lowp vec3 tmpvar_6;
  tmpvar_4.xy = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  tmpvar_4.zw = ((_glesMultiTexCoord0.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
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
  tmpvar_5 = tmpvar_12;
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
  tmpvar_6 = shlight_3;
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
  tmpvar_36 = (tmpvar_6 + ((((unity_LightColor[0].xyz * tmpvar_35.x) + (unity_LightColor[1].xyz * tmpvar_35.y)) + (unity_LightColor[2].xyz * tmpvar_35.z)) + (unity_LightColor[3].xyz * tmpvar_35.w)));
  tmpvar_6 = tmpvar_36;
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = tmpvar_4;
  xlv_TEXCOORD1 = (tmpvar_11 * (((_World2Object * tmpvar_13).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = tmpvar_5;
  xlv_TEXCOORD3 = tmpvar_6;
  xlv_TEXCOORD4 = (unity_World2Shadow[0] * (_Object2World * _glesVertex));
}



#endif
#ifdef FRAGMENT

varying highp vec4 xlv_TEXCOORD4;
varying lowp vec3 xlv_TEXCOORD3;
varying lowp vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp float _Parallax;
uniform mediump float _Gloss;
uniform mediump float _Shininess;
uniform lowp vec4 _Color;
uniform sampler2D _ParallaxMap;
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
  mediump vec4 spec_4;
  mediump float h_5;
  lowp float tmpvar_6;
  tmpvar_6 = texture2D (_ParallaxMap, xlv_TEXCOORD0.zw).w;
  h_5 = tmpvar_6;
  highp vec2 tmpvar_7;
  mediump float height_8;
  height_8 = _Parallax;
  mediump vec3 viewDir_9;
  viewDir_9 = xlv_TEXCOORD1;
  highp vec3 v_10;
  mediump float tmpvar_11;
  tmpvar_11 = ((h_5 * height_8) - (height_8 / 2.0));
  mediump vec3 tmpvar_12;
  tmpvar_12 = normalize(viewDir_9);
  v_10 = tmpvar_12;
  v_10.z = (v_10.z + 0.42);
  tmpvar_7 = (tmpvar_11 * (v_10.xy / v_10.z));
  highp vec2 tmpvar_13;
  tmpvar_13 = (xlv_TEXCOORD0.xy + tmpvar_7);
  highp vec2 tmpvar_14;
  tmpvar_14 = (xlv_TEXCOORD0.zw + tmpvar_7);
  lowp vec4 tmpvar_15;
  tmpvar_15 = texture2D (_MainTex, tmpvar_13);
  lowp vec3 tmpvar_16;
  tmpvar_16 = (tmpvar_15.xyz * _Color.xyz);
  tmpvar_2 = tmpvar_16;
  lowp vec4 tmpvar_17;
  tmpvar_17 = texture2D (_SpecMap, tmpvar_13);
  spec_4 = tmpvar_17;
  mediump vec3 tmpvar_18;
  tmpvar_18 = ((tmpvar_15.w * spec_4.xyz) * _Gloss);
  lowp vec3 tmpvar_19;
  tmpvar_19 = ((texture2D (_BumpMap, tmpvar_14).xyz * 2.0) - 1.0);
  tmpvar_3 = tmpvar_19;
  lowp float tmpvar_20;
  mediump float lightShadowDataX_21;
  highp float dist_22;
  lowp float tmpvar_23;
  tmpvar_23 = texture2DProj (_ShadowMapTexture, xlv_TEXCOORD4).x;
  dist_22 = tmpvar_23;
  highp float tmpvar_24;
  tmpvar_24 = _LightShadowData.x;
  lightShadowDataX_21 = tmpvar_24;
  highp float tmpvar_25;
  tmpvar_25 = max (float((dist_22 > (xlv_TEXCOORD4.z / xlv_TEXCOORD4.w))), lightShadowDataX_21);
  tmpvar_20 = tmpvar_25;
  highp vec3 tmpvar_26;
  tmpvar_26 = normalize(xlv_TEXCOORD1);
  mediump vec3 lightDir_27;
  lightDir_27 = xlv_TEXCOORD2;
  mediump vec3 viewDir_28;
  viewDir_28 = tmpvar_26;
  mediump float atten_29;
  atten_29 = tmpvar_20;
  mediump vec4 c_30;
  mediump vec3 specCol_31;
  highp float nh_32;
  mediump float tmpvar_33;
  tmpvar_33 = max (0.0, dot (tmpvar_3, normalize((lightDir_27 + viewDir_28))));
  nh_32 = tmpvar_33;
  mediump float arg1_34;
  arg1_34 = (32.0 * _Shininess);
  highp vec3 tmpvar_35;
  tmpvar_35 = (pow (nh_32, arg1_34) * tmpvar_18);
  specCol_31 = tmpvar_35;
  c_30.xyz = ((((tmpvar_2 * _LightColor0.xyz) * max (0.0, dot (tmpvar_3, lightDir_27))) + (_LightColor0.xyz * specCol_31)) * (atten_29 * 2.0));
  c_30.w = 0.0;
  c_1 = c_30;
  mediump vec3 tmpvar_36;
  tmpvar_36 = (c_1.xyz + (tmpvar_2 * xlv_TEXCOORD3));
  c_1.xyz = tmpvar_36;
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
varying lowp vec3 xlv_TEXCOORD3;
varying lowp vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 _BumpMap_ST;
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
  highp vec4 tmpvar_4;
  lowp vec3 tmpvar_5;
  lowp vec3 tmpvar_6;
  highp vec4 tmpvar_7;
  tmpvar_7 = (gl_ModelViewProjectionMatrix * _glesVertex);
  tmpvar_4.xy = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  tmpvar_4.zw = ((_glesMultiTexCoord0.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
  mat3 tmpvar_8;
  tmpvar_8[0] = _Object2World[0].xyz;
  tmpvar_8[1] = _Object2World[1].xyz;
  tmpvar_8[2] = _Object2World[2].xyz;
  highp vec3 tmpvar_9;
  tmpvar_9 = (tmpvar_8 * (tmpvar_2 * unity_Scale.w));
  highp vec3 tmpvar_10;
  highp vec3 tmpvar_11;
  tmpvar_10 = tmpvar_1.xyz;
  tmpvar_11 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_12;
  tmpvar_12[0].x = tmpvar_10.x;
  tmpvar_12[0].y = tmpvar_11.x;
  tmpvar_12[0].z = tmpvar_2.x;
  tmpvar_12[1].x = tmpvar_10.y;
  tmpvar_12[1].y = tmpvar_11.y;
  tmpvar_12[1].z = tmpvar_2.y;
  tmpvar_12[2].x = tmpvar_10.z;
  tmpvar_12[2].y = tmpvar_11.z;
  tmpvar_12[2].z = tmpvar_2.z;
  highp vec3 tmpvar_13;
  tmpvar_13 = (tmpvar_12 * (_World2Object * _WorldSpaceLightPos0).xyz);
  tmpvar_5 = tmpvar_13;
  highp vec4 tmpvar_14;
  tmpvar_14.w = 1.0;
  tmpvar_14.xyz = _WorldSpaceCameraPos;
  highp vec4 tmpvar_15;
  tmpvar_15.w = 1.0;
  tmpvar_15.xyz = tmpvar_9;
  mediump vec3 tmpvar_16;
  mediump vec4 normal_17;
  normal_17 = tmpvar_15;
  highp float vC_18;
  mediump vec3 x3_19;
  mediump vec3 x2_20;
  mediump vec3 x1_21;
  highp float tmpvar_22;
  tmpvar_22 = dot (unity_SHAr, normal_17);
  x1_21.x = tmpvar_22;
  highp float tmpvar_23;
  tmpvar_23 = dot (unity_SHAg, normal_17);
  x1_21.y = tmpvar_23;
  highp float tmpvar_24;
  tmpvar_24 = dot (unity_SHAb, normal_17);
  x1_21.z = tmpvar_24;
  mediump vec4 tmpvar_25;
  tmpvar_25 = (normal_17.xyzz * normal_17.yzzx);
  highp float tmpvar_26;
  tmpvar_26 = dot (unity_SHBr, tmpvar_25);
  x2_20.x = tmpvar_26;
  highp float tmpvar_27;
  tmpvar_27 = dot (unity_SHBg, tmpvar_25);
  x2_20.y = tmpvar_27;
  highp float tmpvar_28;
  tmpvar_28 = dot (unity_SHBb, tmpvar_25);
  x2_20.z = tmpvar_28;
  mediump float tmpvar_29;
  tmpvar_29 = ((normal_17.x * normal_17.x) - (normal_17.y * normal_17.y));
  vC_18 = tmpvar_29;
  highp vec3 tmpvar_30;
  tmpvar_30 = (unity_SHC.xyz * vC_18);
  x3_19 = tmpvar_30;
  tmpvar_16 = ((x1_21 + x2_20) + x3_19);
  shlight_3 = tmpvar_16;
  tmpvar_6 = shlight_3;
  highp vec3 tmpvar_31;
  tmpvar_31 = (_Object2World * _glesVertex).xyz;
  highp vec4 tmpvar_32;
  tmpvar_32 = (unity_4LightPosX0 - tmpvar_31.x);
  highp vec4 tmpvar_33;
  tmpvar_33 = (unity_4LightPosY0 - tmpvar_31.y);
  highp vec4 tmpvar_34;
  tmpvar_34 = (unity_4LightPosZ0 - tmpvar_31.z);
  highp vec4 tmpvar_35;
  tmpvar_35 = (((tmpvar_32 * tmpvar_32) + (tmpvar_33 * tmpvar_33)) + (tmpvar_34 * tmpvar_34));
  highp vec4 tmpvar_36;
  tmpvar_36 = (max (vec4(0.0, 0.0, 0.0, 0.0), ((((tmpvar_32 * tmpvar_9.x) + (tmpvar_33 * tmpvar_9.y)) + (tmpvar_34 * tmpvar_9.z)) * inversesqrt(tmpvar_35))) * (1.0/((1.0 + (tmpvar_35 * unity_4LightAtten0)))));
  highp vec3 tmpvar_37;
  tmpvar_37 = (tmpvar_6 + ((((unity_LightColor[0].xyz * tmpvar_36.x) + (unity_LightColor[1].xyz * tmpvar_36.y)) + (unity_LightColor[2].xyz * tmpvar_36.z)) + (unity_LightColor[3].xyz * tmpvar_36.w)));
  tmpvar_6 = tmpvar_37;
  highp vec4 o_38;
  highp vec4 tmpvar_39;
  tmpvar_39 = (tmpvar_7 * 0.5);
  highp vec2 tmpvar_40;
  tmpvar_40.x = tmpvar_39.x;
  tmpvar_40.y = (tmpvar_39.y * _ProjectionParams.x);
  o_38.xy = (tmpvar_40 + tmpvar_39.w);
  o_38.zw = tmpvar_7.zw;
  gl_Position = tmpvar_7;
  xlv_TEXCOORD0 = tmpvar_4;
  xlv_TEXCOORD1 = (tmpvar_12 * (((_World2Object * tmpvar_14).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = tmpvar_5;
  xlv_TEXCOORD3 = tmpvar_6;
  xlv_TEXCOORD4 = o_38;
}



#endif
#ifdef FRAGMENT

varying highp vec4 xlv_TEXCOORD4;
varying lowp vec3 xlv_TEXCOORD3;
varying lowp vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp float _Parallax;
uniform mediump float _Gloss;
uniform mediump float _Shininess;
uniform lowp vec4 _Color;
uniform sampler2D _ParallaxMap;
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
  mediump vec4 spec_4;
  mediump float h_5;
  lowp float tmpvar_6;
  tmpvar_6 = texture2D (_ParallaxMap, xlv_TEXCOORD0.zw).w;
  h_5 = tmpvar_6;
  highp vec2 tmpvar_7;
  mediump float height_8;
  height_8 = _Parallax;
  mediump vec3 viewDir_9;
  viewDir_9 = xlv_TEXCOORD1;
  highp vec3 v_10;
  mediump float tmpvar_11;
  tmpvar_11 = ((h_5 * height_8) - (height_8 / 2.0));
  mediump vec3 tmpvar_12;
  tmpvar_12 = normalize(viewDir_9);
  v_10 = tmpvar_12;
  v_10.z = (v_10.z + 0.42);
  tmpvar_7 = (tmpvar_11 * (v_10.xy / v_10.z));
  highp vec2 tmpvar_13;
  tmpvar_13 = (xlv_TEXCOORD0.xy + tmpvar_7);
  highp vec2 tmpvar_14;
  tmpvar_14 = (xlv_TEXCOORD0.zw + tmpvar_7);
  lowp vec4 tmpvar_15;
  tmpvar_15 = texture2D (_MainTex, tmpvar_13);
  lowp vec3 tmpvar_16;
  tmpvar_16 = (tmpvar_15.xyz * _Color.xyz);
  tmpvar_2 = tmpvar_16;
  lowp vec4 tmpvar_17;
  tmpvar_17 = texture2D (_SpecMap, tmpvar_13);
  spec_4 = tmpvar_17;
  mediump vec3 tmpvar_18;
  tmpvar_18 = ((tmpvar_15.w * spec_4.xyz) * _Gloss);
  lowp vec3 normal_19;
  normal_19.xy = ((texture2D (_BumpMap, tmpvar_14).wy * 2.0) - 1.0);
  normal_19.z = sqrt(((1.0 - (normal_19.x * normal_19.x)) - (normal_19.y * normal_19.y)));
  tmpvar_3 = normal_19;
  lowp float tmpvar_20;
  tmpvar_20 = texture2DProj (_ShadowMapTexture, xlv_TEXCOORD4).x;
  highp vec3 tmpvar_21;
  tmpvar_21 = normalize(xlv_TEXCOORD1);
  mediump vec3 lightDir_22;
  lightDir_22 = xlv_TEXCOORD2;
  mediump vec3 viewDir_23;
  viewDir_23 = tmpvar_21;
  mediump float atten_24;
  atten_24 = tmpvar_20;
  mediump vec4 c_25;
  mediump vec3 specCol_26;
  highp float nh_27;
  mediump float tmpvar_28;
  tmpvar_28 = max (0.0, dot (tmpvar_3, normalize((lightDir_22 + viewDir_23))));
  nh_27 = tmpvar_28;
  mediump float arg1_29;
  arg1_29 = (32.0 * _Shininess);
  highp vec3 tmpvar_30;
  tmpvar_30 = (pow (nh_27, arg1_29) * tmpvar_18);
  specCol_26 = tmpvar_30;
  c_25.xyz = ((((tmpvar_2 * _LightColor0.xyz) * max (0.0, dot (tmpvar_3, lightDir_22))) + (_LightColor0.xyz * specCol_26)) * (atten_24 * 2.0));
  c_25.w = 0.0;
  c_1 = c_25;
  mediump vec3 tmpvar_31;
  tmpvar_31 = (c_1.xyz + (tmpvar_2 * xlv_TEXCOORD3));
  c_1.xyz = tmpvar_31;
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
Vector 33 [_BumpMap_ST]
"agal_vs
c34 1.0 0.0 0.5 0.0
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
aaaaaaaaaeaaaiacccaaaaaaabaaaaaaaaaaaaaaaaaaaaaa mov r4.w, c34.x
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
abaaaaaaabaaapacacaaaaoeacaaaaaaccaaaaaaabaaaaaa add r1, r2, c34.x
bdaaaaaaacaaaeacaeaaaaoeacaaaaaabjaaaaoeabaaaaaa dp4 r2.z, r4, c25
bdaaaaaaacaaacacaeaaaaoeacaaaaaabiaaaaoeabaaaaaa dp4 r2.y, r4, c24
bdaaaaaaacaaabacaeaaaaoeacaaaaaabhaaaaoeabaaaaaa dp4 r2.x, r4, c23
afaaaaaaabaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rcp r1.x, r1.x
afaaaaaaabaaacacabaaaaffacaaaaaaaaaaaaaaaaaaaaaa rcp r1.y, r1.y
afaaaaaaabaaaiacabaaaappacaaaaaaaaaaaaaaaaaaaaaa rcp r1.w, r1.w
afaaaaaaabaaaeacabaaaakkacaaaaaaaaaaaaaaaaaaaaaa rcp r1.z, r1.z
ahaaaaaaaaaaapacaaaaaaoeacaaaaaaccaaaaffabaaaaaa max r0, r0, c34.y
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
abaaaaaaacaaahacacaaaakeacaaaaaaaaaaaakeacaaaaaa add r2.xyz, r2.xyzz, r0.xyzz
abaaaaaaadaaahaeacaaaakeacaaaaaaabaaaakeacaaaaaa add v3.xyz, r2.xyzz, r1.xyzz
aaaaaaaaaaaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, a5
adaaaaaaabaaahacabaaaancaaaaaaaaaaaaaaajacaaaaaa mul r1.xyz, a1.zxyw, r0.yzxx
aaaaaaaaaaaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, a5
adaaaaaaafaaahacabaaaamjaaaaaaaaaaaaaafcacaaaaaa mul r5.xyz, a1.yzxw, r0.zxyy
acaaaaaaaaaaahacafaaaakeacaaaaaaabaaaakeacaaaaaa sub r0.xyz, r5.xyzz, r1.xyzz
adaaaaaaadaaahacaaaaaakeacaaaaaaafaaaappaaaaaaaa mul r3.xyz, r0.xyzz, a5.w
aaaaaaaaaaaaapacakaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0, c10
bdaaaaaaaeaaaeacaoaaaaoeabaaaaaaaaaaaaoeacaaaaaa dp4 r4.z, c14, r0
aaaaaaaaaaaaapacajaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0, c9
bdaaaaaaaeaaacacaoaaaaoeabaaaaaaaaaaaaoeacaaaaaa dp4 r4.y, c14, r0
aaaaaaaaabaaaiacccaaaaaaabaaaaaaaaaaaaaaaaaaaaaa mov r1.w, c34.x
aaaaaaaaabaaahacamaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r1.xyz, c12
bdaaaaaaaaaaaiacaaaaaaoeaaaaaaaaadaaaaoeabaaaaaa dp4 r0.w, a0, c3
bdaaaaaaaaaaaeacaaaaaaoeaaaaaaaaacaaaaoeabaaaaaa dp4 r0.z, a0, c2
bdaaaaaaacaaaeacabaaaaoeacaaaaaaakaaaaoeabaaaaaa dp4 r2.z, r1, c10
bdaaaaaaacaaabacabaaaaoeacaaaaaaaiaaaaoeabaaaaaa dp4 r2.x, r1, c8
bdaaaaaaacaaacacabaaaaoeacaaaaaaajaaaaoeabaaaaaa dp4 r2.y, r1, c9
adaaaaaaafaaahacacaaaakeacaaaaaaboaaaappabaaaaaa mul r5.xyz, r2.xyzz, c30.w
acaaaaaaacaaahacafaaaakeacaaaaaaaaaaaaoeaaaaaaaa sub r2.xyz, r5.xyzz, a0
aaaaaaaaabaaapacaiaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r1, c8
bdaaaaaaaeaaabacaoaaaaoeabaaaaaaabaaaaoeacaaaaaa dp4 r4.x, c14, r1
bdaaaaaaaaaaabacaaaaaaoeaaaaaaaaaaaaaaoeabaaaaaa dp4 r0.x, a0, c0
bdaaaaaaaaaaacacaaaaaaoeaaaaaaaaabaaaaoeabaaaaaa dp4 r0.y, a0, c1
adaaaaaaabaaahacaaaaaapeacaaaaaaccaaaakkabaaaaaa mul r1.xyz, r0.xyww, c34.z
adaaaaaaabaaacacabaaaaffacaaaaaaanaaaaaaabaaaaaa mul r1.y, r1.y, c13.x
abaaaaaaabaaadacabaaaafeacaaaaaaabaaaakkacaaaaaa add r1.xy, r1.xyyy, r1.z
bcaaaaaaabaaacaeacaaaakeacaaaaaaadaaaakeacaaaaaa dp3 v1.y, r2.xyzz, r3.xyzz
bcaaaaaaacaaacaeadaaaakeacaaaaaaaeaaaakeacaaaaaa dp3 v2.y, r3.xyzz, r4.xyzz
bcaaaaaaabaaaeaeabaaaaoeaaaaaaaaacaaaakeacaaaaaa dp3 v1.z, a1, r2.xyzz
bcaaaaaaabaaabaeacaaaakeacaaaaaaafaaaaoeaaaaaaaa dp3 v1.x, r2.xyzz, a5
bcaaaaaaacaaaeaeabaaaaoeaaaaaaaaaeaaaakeacaaaaaa dp3 v2.z, a1, r4.xyzz
bcaaaaaaacaaabaeafaaaaoeaaaaaaaaaeaaaakeacaaaaaa dp3 v2.x, a5, r4.xyzz
adaaaaaaaeaaadaeabaaaafeacaaaaaabpaaaaoeabaaaaaa mul v4.xy, r1.xyyy, c31
aaaaaaaaaaaaapadaaaaaaoeacaaaaaaaaaaaaaaaaaaaaaa mov o0, r0
aaaaaaaaaeaaamaeaaaaaaopacaaaaaaaaaaaaaaaaaaaaaa mov v4.zw, r0.wwzw
adaaaaaaafaaamacadaaaaeeaaaaaaaacbaaaaeeabaaaaaa mul r5.zw, a3.xyxy, c33.xyxy
abaaaaaaaaaaamaeafaaaaopacaaaaaacbaaaaoeabaaaaaa add v0.zw, r5.wwzw, c33
adaaaaaaafaaadacadaaaaoeaaaaaaaacaaaaaoeabaaaaaa mul r5.xy, a3, c32
abaaaaaaaaaaadaeafaaaafeacaaaaaacaaaaaooabaaaaaa add v0.xy, r5.xyyy, c32.zwzw
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
varying lowp vec3 xlv_TEXCOORD3;
varying lowp vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 _BumpMap_ST;
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
  highp vec4 tmpvar_4;
  lowp vec3 tmpvar_5;
  lowp vec3 tmpvar_6;
  tmpvar_4.xy = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  tmpvar_4.zw = ((_glesMultiTexCoord0.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
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
  tmpvar_5 = tmpvar_11;
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
  tmpvar_6 = shlight_3;
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = tmpvar_4;
  xlv_TEXCOORD1 = (tmpvar_10 * (((_World2Object * tmpvar_12).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = tmpvar_5;
  xlv_TEXCOORD3 = tmpvar_6;
  xlv_TEXCOORD4 = (unity_World2Shadow[0] * (_Object2World * _glesVertex));
}



#endif
#ifdef FRAGMENT

#extension GL_EXT_shadow_samplers : enable
varying highp vec4 xlv_TEXCOORD4;
varying lowp vec3 xlv_TEXCOORD3;
varying lowp vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp float _Parallax;
uniform mediump float _Gloss;
uniform mediump float _Shininess;
uniform lowp vec4 _Color;
uniform sampler2D _ParallaxMap;
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
  mediump vec4 spec_4;
  mediump float h_5;
  lowp float tmpvar_6;
  tmpvar_6 = texture2D (_ParallaxMap, xlv_TEXCOORD0.zw).w;
  h_5 = tmpvar_6;
  highp vec2 tmpvar_7;
  mediump float height_8;
  height_8 = _Parallax;
  mediump vec3 viewDir_9;
  viewDir_9 = xlv_TEXCOORD1;
  highp vec3 v_10;
  mediump float tmpvar_11;
  tmpvar_11 = ((h_5 * height_8) - (height_8 / 2.0));
  mediump vec3 tmpvar_12;
  tmpvar_12 = normalize(viewDir_9);
  v_10 = tmpvar_12;
  v_10.z = (v_10.z + 0.42);
  tmpvar_7 = (tmpvar_11 * (v_10.xy / v_10.z));
  highp vec2 tmpvar_13;
  tmpvar_13 = (xlv_TEXCOORD0.xy + tmpvar_7);
  highp vec2 tmpvar_14;
  tmpvar_14 = (xlv_TEXCOORD0.zw + tmpvar_7);
  lowp vec4 tmpvar_15;
  tmpvar_15 = texture2D (_MainTex, tmpvar_13);
  lowp vec3 tmpvar_16;
  tmpvar_16 = (tmpvar_15.xyz * _Color.xyz);
  tmpvar_2 = tmpvar_16;
  lowp vec4 tmpvar_17;
  tmpvar_17 = texture2D (_SpecMap, tmpvar_13);
  spec_4 = tmpvar_17;
  mediump vec3 tmpvar_18;
  tmpvar_18 = ((tmpvar_15.w * spec_4.xyz) * _Gloss);
  lowp vec3 tmpvar_19;
  tmpvar_19 = ((texture2D (_BumpMap, tmpvar_14).xyz * 2.0) - 1.0);
  tmpvar_3 = tmpvar_19;
  lowp float shadow_20;
  lowp float tmpvar_21;
  tmpvar_21 = shadow2DEXT (_ShadowMapTexture, xlv_TEXCOORD4.xyz);
  highp float tmpvar_22;
  tmpvar_22 = (_LightShadowData.x + (tmpvar_21 * (1.0 - _LightShadowData.x)));
  shadow_20 = tmpvar_22;
  highp vec3 tmpvar_23;
  tmpvar_23 = normalize(xlv_TEXCOORD1);
  mediump vec3 lightDir_24;
  lightDir_24 = xlv_TEXCOORD2;
  mediump vec3 viewDir_25;
  viewDir_25 = tmpvar_23;
  mediump float atten_26;
  atten_26 = shadow_20;
  mediump vec4 c_27;
  mediump vec3 specCol_28;
  highp float nh_29;
  mediump float tmpvar_30;
  tmpvar_30 = max (0.0, dot (tmpvar_3, normalize((lightDir_24 + viewDir_25))));
  nh_29 = tmpvar_30;
  mediump float arg1_31;
  arg1_31 = (32.0 * _Shininess);
  highp vec3 tmpvar_32;
  tmpvar_32 = (pow (nh_29, arg1_31) * tmpvar_18);
  specCol_28 = tmpvar_32;
  c_27.xyz = ((((tmpvar_2 * _LightColor0.xyz) * max (0.0, dot (tmpvar_3, lightDir_24))) + (_LightColor0.xyz * specCol_28)) * (atten_26 * 2.0));
  c_27.w = 0.0;
  c_1 = c_27;
  mediump vec3 tmpvar_33;
  tmpvar_33 = (c_1.xyz + (tmpvar_2 * xlv_TEXCOORD3));
  c_1.xyz = tmpvar_33;
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
varying highp vec4 xlv_TEXCOORD3;
varying highp vec2 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 _BumpMap_ST;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 unity_LightmapST;
uniform highp vec4 unity_Scale;
uniform highp mat4 _World2Object;
uniform highp mat4 _Object2World;

uniform highp mat4 unity_World2Shadow[4];
uniform highp vec3 _WorldSpaceCameraPos;
attribute vec4 _glesTANGENT;
attribute vec4 _glesMultiTexCoord1;
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
  highp vec4 tmpvar_3;
  tmpvar_3.xy = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  tmpvar_3.zw = ((_glesMultiTexCoord0.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
  highp vec3 tmpvar_4;
  highp vec3 tmpvar_5;
  tmpvar_4 = tmpvar_1.xyz;
  tmpvar_5 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_6;
  tmpvar_6[0].x = tmpvar_4.x;
  tmpvar_6[0].y = tmpvar_5.x;
  tmpvar_6[0].z = tmpvar_2.x;
  tmpvar_6[1].x = tmpvar_4.y;
  tmpvar_6[1].y = tmpvar_5.y;
  tmpvar_6[1].z = tmpvar_2.y;
  tmpvar_6[2].x = tmpvar_4.z;
  tmpvar_6[2].y = tmpvar_5.z;
  tmpvar_6[2].z = tmpvar_2.z;
  highp vec4 tmpvar_7;
  tmpvar_7.w = 1.0;
  tmpvar_7.xyz = _WorldSpaceCameraPos;
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = tmpvar_3;
  xlv_TEXCOORD1 = (tmpvar_6 * (((_World2Object * tmpvar_7).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = ((_glesMultiTexCoord1.xy * unity_LightmapST.xy) + unity_LightmapST.zw);
  xlv_TEXCOORD3 = (unity_World2Shadow[0] * (_Object2World * _glesVertex));
}



#endif
#ifdef FRAGMENT

#extension GL_EXT_shadow_samplers : enable
varying highp vec4 xlv_TEXCOORD3;
varying highp vec2 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform sampler2D unity_Lightmap;
uniform highp float _Parallax;
uniform lowp vec4 _Color;
uniform sampler2D _ParallaxMap;
uniform sampler2D _MainTex;
uniform sampler2DShadow _ShadowMapTexture;
uniform highp vec4 _LightShadowData;
void main ()
{
  lowp vec4 c_1;
  mediump vec3 tmpvar_2;
  mediump float h_3;
  lowp float tmpvar_4;
  tmpvar_4 = texture2D (_ParallaxMap, xlv_TEXCOORD0.zw).w;
  h_3 = tmpvar_4;
  mediump float height_5;
  height_5 = _Parallax;
  mediump vec3 viewDir_6;
  viewDir_6 = xlv_TEXCOORD1;
  highp vec3 v_7;
  mediump float tmpvar_8;
  tmpvar_8 = ((h_3 * height_5) - (height_5 / 2.0));
  mediump vec3 tmpvar_9;
  tmpvar_9 = normalize(viewDir_6);
  v_7 = tmpvar_9;
  v_7.z = (v_7.z + 0.42);
  highp vec2 tmpvar_10;
  tmpvar_10 = (xlv_TEXCOORD0.xy + (tmpvar_8 * (v_7.xy / v_7.z)));
  lowp vec3 tmpvar_11;
  tmpvar_11 = (texture2D (_MainTex, tmpvar_10).xyz * _Color.xyz);
  tmpvar_2 = tmpvar_11;
  lowp float shadow_12;
  lowp float tmpvar_13;
  tmpvar_13 = shadow2DEXT (_ShadowMapTexture, xlv_TEXCOORD3.xyz);
  highp float tmpvar_14;
  tmpvar_14 = (_LightShadowData.x + (tmpvar_13 * (1.0 - _LightShadowData.x)));
  shadow_12 = tmpvar_14;
  lowp vec3 tmpvar_15;
  tmpvar_15 = min ((2.0 * texture2D (unity_Lightmap, xlv_TEXCOORD2).xyz), vec3((shadow_12 * 2.0)));
  mediump vec3 tmpvar_16;
  tmpvar_16 = (tmpvar_2 * tmpvar_15);
  c_1.xyz = tmpvar_16;
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
varying lowp vec3 xlv_TEXCOORD3;
varying lowp vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 _BumpMap_ST;
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
  highp vec4 tmpvar_4;
  lowp vec3 tmpvar_5;
  lowp vec3 tmpvar_6;
  tmpvar_4.xy = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  tmpvar_4.zw = ((_glesMultiTexCoord0.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
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
  tmpvar_5 = tmpvar_12;
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
  tmpvar_6 = shlight_3;
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
  tmpvar_36 = (tmpvar_6 + ((((unity_LightColor[0].xyz * tmpvar_35.x) + (unity_LightColor[1].xyz * tmpvar_35.y)) + (unity_LightColor[2].xyz * tmpvar_35.z)) + (unity_LightColor[3].xyz * tmpvar_35.w)));
  tmpvar_6 = tmpvar_36;
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = tmpvar_4;
  xlv_TEXCOORD1 = (tmpvar_11 * (((_World2Object * tmpvar_13).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = tmpvar_5;
  xlv_TEXCOORD3 = tmpvar_6;
  xlv_TEXCOORD4 = (unity_World2Shadow[0] * (_Object2World * _glesVertex));
}



#endif
#ifdef FRAGMENT

#extension GL_EXT_shadow_samplers : enable
varying highp vec4 xlv_TEXCOORD4;
varying lowp vec3 xlv_TEXCOORD3;
varying lowp vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp float _Parallax;
uniform mediump float _Gloss;
uniform mediump float _Shininess;
uniform lowp vec4 _Color;
uniform sampler2D _ParallaxMap;
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
  mediump vec4 spec_4;
  mediump float h_5;
  lowp float tmpvar_6;
  tmpvar_6 = texture2D (_ParallaxMap, xlv_TEXCOORD0.zw).w;
  h_5 = tmpvar_6;
  highp vec2 tmpvar_7;
  mediump float height_8;
  height_8 = _Parallax;
  mediump vec3 viewDir_9;
  viewDir_9 = xlv_TEXCOORD1;
  highp vec3 v_10;
  mediump float tmpvar_11;
  tmpvar_11 = ((h_5 * height_8) - (height_8 / 2.0));
  mediump vec3 tmpvar_12;
  tmpvar_12 = normalize(viewDir_9);
  v_10 = tmpvar_12;
  v_10.z = (v_10.z + 0.42);
  tmpvar_7 = (tmpvar_11 * (v_10.xy / v_10.z));
  highp vec2 tmpvar_13;
  tmpvar_13 = (xlv_TEXCOORD0.xy + tmpvar_7);
  highp vec2 tmpvar_14;
  tmpvar_14 = (xlv_TEXCOORD0.zw + tmpvar_7);
  lowp vec4 tmpvar_15;
  tmpvar_15 = texture2D (_MainTex, tmpvar_13);
  lowp vec3 tmpvar_16;
  tmpvar_16 = (tmpvar_15.xyz * _Color.xyz);
  tmpvar_2 = tmpvar_16;
  lowp vec4 tmpvar_17;
  tmpvar_17 = texture2D (_SpecMap, tmpvar_13);
  spec_4 = tmpvar_17;
  mediump vec3 tmpvar_18;
  tmpvar_18 = ((tmpvar_15.w * spec_4.xyz) * _Gloss);
  lowp vec3 tmpvar_19;
  tmpvar_19 = ((texture2D (_BumpMap, tmpvar_14).xyz * 2.0) - 1.0);
  tmpvar_3 = tmpvar_19;
  lowp float shadow_20;
  lowp float tmpvar_21;
  tmpvar_21 = shadow2DEXT (_ShadowMapTexture, xlv_TEXCOORD4.xyz);
  highp float tmpvar_22;
  tmpvar_22 = (_LightShadowData.x + (tmpvar_21 * (1.0 - _LightShadowData.x)));
  shadow_20 = tmpvar_22;
  highp vec3 tmpvar_23;
  tmpvar_23 = normalize(xlv_TEXCOORD1);
  mediump vec3 lightDir_24;
  lightDir_24 = xlv_TEXCOORD2;
  mediump vec3 viewDir_25;
  viewDir_25 = tmpvar_23;
  mediump float atten_26;
  atten_26 = shadow_20;
  mediump vec4 c_27;
  mediump vec3 specCol_28;
  highp float nh_29;
  mediump float tmpvar_30;
  tmpvar_30 = max (0.0, dot (tmpvar_3, normalize((lightDir_24 + viewDir_25))));
  nh_29 = tmpvar_30;
  mediump float arg1_31;
  arg1_31 = (32.0 * _Shininess);
  highp vec3 tmpvar_32;
  tmpvar_32 = (pow (nh_29, arg1_31) * tmpvar_18);
  specCol_28 = tmpvar_32;
  c_27.xyz = ((((tmpvar_2 * _LightColor0.xyz) * max (0.0, dot (tmpvar_3, lightDir_24))) + (_LightColor0.xyz * specCol_28)) * (atten_26 * 2.0));
  c_27.w = 0.0;
  c_1 = c_27;
  mediump vec3 tmpvar_33;
  tmpvar_33 = (c_1.xyz + (tmpvar_2 * xlv_TEXCOORD3));
  c_1.xyz = tmpvar_33;
  gl_FragData[0] = c_1;
}



#endif"
}

}
Program "fp" {
// Fragment combos: 4
//   opengl - ALU: 18 to 45, TEX: 3 to 5
//   d3d9 - ALU: 18 to 49, TEX: 3 to 5
//   d3d11 - ALU: 9 to 24, TEX: 3 to 5, FLOW: 1 to 1
//   d3d11_9x - ALU: 9 to 22, TEX: 3 to 4, FLOW: 1 to 1
SubProgram "opengl " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
Float 4 [_Parallax]
SetTexture 0 [_ParallaxMap] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 2 [_SpecMap] 2D
SetTexture 3 [_BumpMap] 2D
"!!ARBfp1.0
OPTION ARB_precision_hint_fastest;
# 43 ALU, 4 TEX
PARAM c[7] = { program.local[0..4],
		{ 0.5, 0.41999999, 2, 1 },
		{ 0, 32 } };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
TEX R0.w, fragment.texcoord[0].zwzw, texture[0], 2D;
DP3 R0.x, fragment.texcoord[1], fragment.texcoord[1];
RSQ R2.z, R0.x;
MUL R1.xyz, R2.z, fragment.texcoord[1];
ADD R0.y, R1.z, c[5];
RCP R0.y, R0.y;
MOV R0.x, c[4];
MUL R0.x, R0, c[5];
MOV R3.xyz, fragment.texcoord[2];
MAD R3.xyz, R2.z, fragment.texcoord[1], R3;
DP3 R2.z, R3, R3;
RSQ R2.z, R2.z;
MUL R1.zw, R1.xyxy, R0.y;
MAD R0.x, R0.w, c[4], -R0;
MAD R1.xy, R0.x, R1.zwzw, fragment.texcoord[0];
MAD R1.zw, R0.x, R1, fragment.texcoord[0];
MUL R3.xyz, R2.z, R3;
MOV result.color.w, c[6].x;
TEX R0, R1, texture[1], 2D;
TEX R2.yw, R1.zwzw, texture[3], 2D;
TEX R1.xyz, R1, texture[2], 2D;
MAD R2.xy, R2.wyzw, c[5].z, -c[5].w;
MUL R1.w, R2.y, R2.y;
MAD R1.w, -R2.x, R2.x, -R1;
MUL R1.xyz, R0.w, R1;
ADD R1.w, R1, c[5];
RSQ R1.w, R1.w;
RCP R2.z, R1.w;
DP3 R1.w, R2, R3;
MOV R2.w, c[6].y;
MUL R0.xyz, R0, c[1];
MAX R1.w, R1, c[6].x;
MUL R0.w, R2, c[2].x;
POW R0.w, R1.w, R0.w;
MUL R1.xyz, R1, c[3].x;
MUL R1.xyz, R0.w, R1;
DP3 R0.w, R2, fragment.texcoord[2];
MUL R2.xyz, R0, fragment.texcoord[3];
MUL R1.xyz, R1, c[0];
MAX R0.w, R0, c[6].x;
MUL R0.xyz, R0, c[0];
MAD R0.xyz, R0, R0.w, R1;
MAD result.color.xyz, R0, c[5].z, R2;
END
# 43 instructions, 4 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
Float 4 [_Parallax]
SetTexture 0 [_ParallaxMap] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 2 [_SpecMap] 2D
SetTexture 3 [_BumpMap] 2D
"ps_2_0
; 48 ALU, 4 TEX
dcl_2d s0
dcl_2d s1
dcl_2d s2
dcl_2d s3
def c5, 0.50000000, 0.41999999, 2.00000000, -1.00000000
def c6, 1.00000000, 0.00000000, 32.00000000, 0
dcl t0
dcl t1.xyz
dcl t2.xyz
dcl t3.xyz
mov r4.y, t0.w
mov r4.x, t0.z
mov r0.y, t0.w
mov r0.x, t0.z
texld r0, r0, s0
dp3_pp r0.x, t1, t1
rsq_pp r0.x, r0.x
mul_pp r3.xyz, r0.x, t1
add r1.x, r3.z, c5.y
rcp r2.x, r1.x
mov_pp r1.x, c5
mul_pp r1.x, c4, r1
mad_pp r1.x, r0.w, c4, -r1
mul r2.xy, r3, r2.x
mad r3.xy, r1.x, r2, t0
mad r1.xy, r1.x, r2, r4
mov_pp r4.xyz, t2
mad_pp r4.xyz, r0.x, t1, r4
mov_pp r0.w, c6.y
texld r2, r3, s1
texld r1, r1, s3
texld r3, r3, s2
mov r1.x, r1.w
mad_pp r5.xy, r1, c5.z, c5.w
mul_pp r1.x, r5.y, r5.y
mad_pp r1.x, -r5, r5, -r1
add_pp r0.x, r1, c6
dp3_pp r1.x, r4, r4
rsq_pp r0.x, r0.x
rcp_pp r5.z, r0.x
rsq_pp r1.x, r1.x
mul_pp r1.xyz, r1.x, r4
dp3_pp r1.x, r5, r1
mov_pp r0.x, c2
mul_pp r0.x, c6.z, r0
max_pp r1.x, r1, c6.y
pow r4.x, r1.x, r0.x
mul_pp r0.xyz, r2.w, r3
mul_pp r2.xyz, r2, c1
mul_pp r1.xyz, r0, c3.x
mov r0.x, r4.x
mul r0.xyz, r0.x, r1
mul_pp r1.xyz, r0, c0
dp3_pp r0.x, r5, t2
max_pp r0.x, r0, c6.y
mul_pp r3.xyz, r2, c0
mad_pp r0.xyz, r3, r0.x, r1
mul_pp r1.xyz, r2, t3
mad_pp r0.xyz, r0, c5.z, r1
mov_pp oC0, r0
"
}

SubProgram "xbox360 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Vector 1 [_Color]
Float 3 [_Gloss]
Vector 0 [_LightColor0]
Float 4 [_Parallax]
Float 2 [_Shininess]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_BumpMap] 2D
SetTexture 2 [_SpecMap] 2D
SetTexture 3 [_ParallaxMap] 2D
// Shader Timing Estimate, in Cycles/64 pixel vector:
// ALU: 30.67 (23 instructions), vertex: 0, texture: 16,
//   sequencer: 12, interpolator: 16;    7 GPRs, 27 threads,
// Performance (if enough threads): ~30 cycles per vector
// * Texture cycle estimates are assuming an 8bit/component texture with no
//     aniso or trilinear filtering.

"ps_360
backbbaaaaaaabpaaaaaableaaaaaaaaaaaaaaceaaaaabjiaaaaabmaaaaaaaaa
aaaaaaaaaaaaabhaaaaaaabmaaaaabgeppppadaaaaaaaaajaaaaaabmaaaaaaaa
aaaaabfnaaaaaanaaaadaaabaaabaaaaaaaaaanmaaaaaaaaaaaaaaomaaacaaab
aaabaaaaaaaaaapeaaaaaaaaaaaaabaeaaacaaadaaabaaaaaaaaabamaaaaaaaa
aaaaabbmaaacaaaaaaabaaaaaaaaaapeaaaaaaaaaaaaabcjaaadaaaaaaabaaaa
aaaaaanmaaaaaaaaaaaaabdcaaacaaaeaaabaaaaaaaaabamaaaaaaaaaaaaabdm
aaadaaadaaabaaaaaaaaaanmaaaaaaaaaaaaabejaaacaaacaaabaaaaaaaaabam
aaaaaaaaaaaaabfeaaadaaacaaabaaaaaaaaaanmaaaaaaaafpechfgnhaengbha
aaklklklaaaeaaamaaabaaabaaabaaaaaaaaaaaafpedgpgmgphcaaklaaabaaad
aaabaaaeaaabaaaaaaaaaaaafpehgmgphdhdaaklaaaaaaadaaabaaabaaabaaaa
aaaaaaaafpemgjghgiheedgpgmgphcdaaafpengbgjgofegfhiaafpfagbhcgbgm
gmgbhiaafpfagbhcgbgmgmgbhiengbhaaafpfdgigjgogjgogfhdhdaafpfdhagf
gdengbhaaahahdfpddfpdaaadccodacodcdadddfddcodaaaaaaaaaaaaaaaaaab
aaaaaaaaaaaaaaaaaaaaaabeabpmaabaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaeaaaaaabhebaaaagaaaaaaaaaeaaaaaaaaaaaadeieaaapaaapaaaaaaab
aaaapafaaaaahbfbaaaahcfcaaaahdfdaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaeaaaaaaadpiaaaaaecaaaaaaaaaaaaaa
dpaaaaaaaaaaaaaalpiaaaaadonhakdnaaajgaadgaajbcaabcaaafeaaaaaaaaa
gaapmeaabcaaaaaaaaaagabfdablbcaaccaaaaaadidadaabbpbpphppaaaaeaaa
miaiaaacaaloloaapaababaafiiiacabaagmgmblcbaeppicmiaiaaababblgmbl
kladaeabmiahaaabaablloaaobacabaaleihacafaagfmaaamaabacppemiiacaf
aagmmgblcbacpoacmiadaaabaamfblaaobabacaamiapaaabaakablaaolababaa
baciaacbbpbppoiiaaaaeaaalibigacbbpbpppnjaaaaeaaabaaibacbbpbppefi
aaaaeaaamiahaaaeaabemaaakbababaamiafaaabaagngmmgilagpoppmiaoaaaa
aalbpmaaobabaaaamiacaaabaegogolbnbababpokaibabaaaalololbpaafafib
fibcaaabaamplogmpaabaciamiahaaacaamagmaaobafaaaakibbacabaalompeb
naacabadkicdacabaalalbecicabppadeaehabafaalelegmkbaeaaibkiepacab
aadepbedmbafabaddiihaaaaaamamablobaeadabmiahaaacaaleblaaobacaaaa
miahaaabaamaleleklacaaabmiahmaaaaalegmmaklabpoaaaaaaaaaaaaaaaaaa
aaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
Float 4 [_Parallax]
SetTexture 0 [_ParallaxMap] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 2 [_SpecMap] 2D
SetTexture 3 [_BumpMap] 2D
"sce_fp_rsx // 56 instructions using 4 registers
[Configuration]
24
ffffffff0003c020000ffff0000000000000840004000000
[Offsets]
5
_LightColor0 2 0
00000360000002e0
_Color 1 0
000002b0
_Shininess 1 0
000001f0
_Gloss 1 0
00000140
_Parallax 2 0
0000004000000020
[Microcode]
896
900017005c011c9dc8000001c8003fe102800240fe001c9d00020000c8000001
000000000000000000000000000000001088014000021c9cc8000001c8000001
00000000000000000000000000000000ae883940c8011c9dc8000029c800bfe1
08040300c9101c9d00020000c80000010a3d3ed7000000000000000000000000
02820440ff101c9daa020000c9000001000000000000bf000000000000000000
06003a00c9101c9d54080001c80000010602020001041c9cc8000001c8000001
9e060100c8011c9dc8000001c8003fe1060003005c0c1c9dc8040001c8000001
18060300800c1c9c80040000c800000114001706c8001c9dc8000001c8000001
18820440ee001c9c00020000aa020000000040000000bf800000000000000000
1e0217025c0c1c9dc8000001c800000110860240c8041c9d00020000c8000001
0000000000000000000000000000000002800240ff041c9dff040001c8000001
108a044055041c9f5504000101000002ce8c0140c8011c9dc8000001c8003fe1
0e040340c9181c9dc9100001c80000010e0017045c0c1c9dc8000001c8000001
0e883940c8081c9dc8000029c800000110880340c9141c9d00020000c8000001
00003f800000000000000000000000000e8a0240ff0c1c9dc8000001c8000001
1080014000021c9cc8000001c800000100000000000000000000000000000000
02823b40ff103c9dff100001c8000001028005401d041c9cc9100001c8000001
108c0240c9001c9d00020000c800000100004200000000000000000000000000
1002090001001c9c00020000c800000100000000000000000000000000000000
ee800140c8011c9dc8000001c8003fe1108005401d041c9cc9180001c8000001
08001d00fe041c9dc8000001c80000011000020054001c9dc9180001c8000001
0e840240c8041c9dc8020001c800000100000000000000000000000000000000
08001c00fe001c9dc8000001c80000010e860240c9081c9dc8020001c8000001
0000000000000000000000000000000010800900c9001c9dc8020001c8000001
000000000000000000000000000000000e860240c90c1c9dff000001c8000001
1080014000021c9cc8000001c800000100000000000000000000000000000000
0e82020054001c9dc9140001c80000010e820440c9041c9dc8021001c90c0001
000000000000000000000000000000000e810440c9081c9dc9000001c9040001
"
}

SubProgram "d3d11 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
ConstBuffer "$Globals" 112 // 76 used size, 9 vars
Vector 16 [_LightColor0] 4
Vector 48 [_Color] 4
Float 64 [_Shininess]
Float 68 [_Gloss]
Float 72 [_Parallax]
BindCB "$Globals" 0
SetTexture 0 [_ParallaxMap] 2D 3
SetTexture 1 [_MainTex] 2D 0
SetTexture 2 [_SpecMap] 2D 2
SetTexture 3 [_BumpMap] 2D 1
// 37 instructions, 4 temp regs, 0 temp arrays:
// ALU 22 float, 0 int, 0 uint
// TEX 4 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0
eefiecedphhoecfaecmcegjabcieacifoilmoilbabaaaaaafeagaaaaadaaaaaa
cmaaaaaammaaaaaaaaabaaaaejfdeheojiaaaaaaafaaaaaaaiaaaaaaiaaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaaimaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaahahaaaaimaaaaaa
adaaaaaaaaaaaaaaadaaaaaaaeaaaaaaahahaaaafdfgfpfaepfdejfeejepeoaa
feeffiedepepfceeaaklklklepfdeheocmaaaaaaabaaaaaaaiaaaaaacaaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaafdfgfpfegbhcghgfheaaklkl
fdeieefcemafaaaaeaaaaaaafdabaaaafjaaaaaeegiocaaaaaaaaaaaafaaaaaa
fkaaaaadaagabaaaaaaaaaaafkaaaaadaagabaaaabaaaaaafkaaaaadaagabaaa
acaaaaaafkaaaaadaagabaaaadaaaaaafibiaaaeaahabaaaaaaaaaaaffffaaaa
fibiaaaeaahabaaaabaaaaaaffffaaaafibiaaaeaahabaaaacaaaaaaffffaaaa
fibiaaaeaahabaaaadaaaaaaffffaaaagcbaaaadpcbabaaaabaaaaaagcbaaaad
hcbabaaaacaaaaaagcbaaaadhcbabaaaadaaaaaagcbaaaadhcbabaaaaeaaaaaa
gfaaaaadpccabaaaaaaaaaaagiaaaaacaeaaaaaabaaaaaahbcaabaaaaaaaaaaa
egbcbaaaacaaaaaaegbcbaaaacaaaaaaeeaaaaafbcaabaaaaaaaaaaaakaabaaa
aaaaaaaadcaaaaajocaabaaaaaaaaaaaagbjbaaaacaaaaaaagaabaaaaaaaaaaa
agbjbaaaadaaaaaabaaaaaahbcaabaaaabaaaaaajgahbaaaaaaaaaaajgahbaaa
aaaaaaaaeeaaaaafbcaabaaaabaaaaaaakaabaaaabaaaaaadiaaaaahocaabaaa
aaaaaaaafgaobaaaaaaaaaaaagaabaaaabaaaaaadiaaaaahdcaabaaaabaaaaaa
agaabaaaaaaaaaaaegbabaaaacaaaaaadcaaaaajbcaabaaaaaaaaaaackbabaaa
acaaaaaaakaabaaaaaaaaaaaabeaaaaadnaknhdoaoaaaaahpcaabaaaabaaaaaa
egaebaaaabaaaaaaagaabaaaaaaaaaaaefaaaaajpcaabaaaacaaaaaaogbkbaaa
abaaaaaaeghobaaaaaaaaaaaaagabaaaadaaaaaadiaaaaaldcaabaaaacaaaaaa
cgikcaaaaaaaaaaaaeaaaaaaaceaaaaaaaaaaadpaaaaaaecaaaaaaaaaaaaaaaa
dcaaaaalbcaabaaaaaaaaaaadkaabaaaacaaaaaackiacaaaaaaaaaaaaeaaaaaa
akaabaiaebaaaaaaacaaaaaadcaaaaajpcaabaaaabaaaaaaagaabaaaaaaaaaaa
egaobaaaabaaaaaaegbobaaaabaaaaaaefaaaaajpcaabaaaadaaaaaaogakbaaa
abaaaaaaeghobaaaadaaaaaaaagabaaaabaaaaaadcaaaaapdcaabaaaadaaaaaa
hgapbaaaadaaaaaaaceaaaaaaaaaaaeaaaaaaaeaaaaaaaaaaaaaaaaaaceaaaaa
aaaaialpaaaaialpaaaaaaaaaaaaaaaadcaaaaakbcaabaaaaaaaaaaaakaabaia
ebaaaaaaadaaaaaaakaabaaaadaaaaaaabeaaaaaaaaaiadpdcaaaaakbcaabaaa
aaaaaaaabkaabaiaebaaaaaaadaaaaaabkaabaaaadaaaaaaakaabaaaaaaaaaaa
elaaaaafecaabaaaadaaaaaaakaabaaaaaaaaaaabaaaaaahbcaabaaaaaaaaaaa
egacbaaaadaaaaaajgahbaaaaaaaaaaabaaaaaahccaabaaaaaaaaaaaegacbaaa
adaaaaaaegbcbaaaadaaaaaadeaaaaakdcaabaaaaaaaaaaaegaabaaaaaaaaaaa
aceaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaacpaaaaafbcaabaaaaaaaaaaa
akaabaaaaaaaaaaadiaaaaahbcaabaaaaaaaaaaaakaabaaaaaaaaaaabkaabaaa
acaaaaaabjaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaaefaaaaajpcaabaaa
acaaaaaaegaabaaaabaaaaaaeghobaaaacaaaaaaaagabaaaacaaaaaaefaaaaaj
pcaabaaaabaaaaaaegaabaaaabaaaaaaeghobaaaabaaaaaaaagabaaaaaaaaaaa
diaaaaahhcaabaaaacaaaaaaegacbaaaacaaaaaapgapbaaaabaaaaaadiaaaaai
hcaabaaaabaaaaaaegacbaaaabaaaaaaegiccaaaaaaaaaaaadaaaaaadiaaaaai
hcaabaaaacaaaaaaegacbaaaacaaaaaafgifcaaaaaaaaaaaaeaaaaaadiaaaaah
ncaabaaaaaaaaaaaagaabaaaaaaaaaaaagajbaaaacaaaaaadiaaaaaincaabaaa
aaaaaaaaagaobaaaaaaaaaaaagijcaaaaaaaaaaaabaaaaaadiaaaaaihcaabaaa
acaaaaaaegacbaaaabaaaaaaegiccaaaaaaaaaaaabaaaaaadcaaaaajhcaabaaa
aaaaaaaaegacbaaaacaaaaaafgafbaaaaaaaaaaaigadbaaaaaaaaaaaaaaaaaah
hcaabaaaaaaaaaaaegacbaaaaaaaaaaaegacbaaaaaaaaaaadcaaaaajhccabaaa
aaaaaaaaegacbaaaabaaaaaaegbcbaaaaeaaaaaaegacbaaaaaaaaaaadgaaaaaf
iccabaaaaaaaaaaaabeaaaaaaaaaaaaadoaaaaab"
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
Float 4 [_Parallax]
SetTexture 0 [_ParallaxMap] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 2 [_SpecMap] 2D
SetTexture 3 [_BumpMap] 2D
"agal_ps
c5 0.5 0.42 2.0 -1.0
c6 1.0 0.0 32.0 0.0
[bc]
aaaaaaaaaeaaacacaaaaaappaeaaaaaaaaaaaaaaaaaaaaaa mov r4.y, v0.w
aaaaaaaaaeaaabacaaaaaakkaeaaaaaaaaaaaaaaaaaaaaaa mov r4.x, v0.z
aaaaaaaaaaaaacacaaaaaappaeaaaaaaaaaaaaaaaaaaaaaa mov r0.y, v0.w
aaaaaaaaaaaaabacaaaaaakkaeaaaaaaaaaaaaaaaaaaaaaa mov r0.x, v0.z
ciaaaaaaaaaaapacaaaaaafeacaaaaaaaaaaaaaaafaababb tex r0, r0.xyyy, s0 <2d wrap linear point>
bcaaaaaaaaaaabacabaaaaoeaeaaaaaaabaaaaoeaeaaaaaa dp3 r0.x, v1, v1
akaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r0.x, r0.x
adaaaaaaadaaahacaaaaaaaaacaaaaaaabaaaaoeaeaaaaaa mul r3.xyz, r0.x, v1
abaaaaaaabaaabacadaaaakkacaaaaaaafaaaaffabaaaaaa add r1.x, r3.z, c5.y
afaaaaaaacaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rcp r2.x, r1.x
aaaaaaaaabaaabacafaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r1.x, c5
adaaaaaaabaaabacaeaaaaoeabaaaaaaabaaaaaaacaaaaaa mul r1.x, c4, r1.x
adaaaaaaadaaaiacaaaaaappacaaaaaaaeaaaaoeabaaaaaa mul r3.w, r0.w, c4
acaaaaaaabaaabacadaaaappacaaaaaaabaaaaaaacaaaaaa sub r1.x, r3.w, r1.x
adaaaaaaacaaadacadaaaafeacaaaaaaacaaaaaaacaaaaaa mul r2.xy, r3.xyyy, r2.x
adaaaaaaadaaadacabaaaaaaacaaaaaaacaaaafeacaaaaaa mul r3.xy, r1.x, r2.xyyy
abaaaaaaadaaadacadaaaafeacaaaaaaaaaaaaoeaeaaaaaa add r3.xy, r3.xyyy, v0
adaaaaaaabaaadacabaaaaaaacaaaaaaacaaaafeacaaaaaa mul r1.xy, r1.x, r2.xyyy
abaaaaaaabaaadacabaaaafeacaaaaaaaeaaaafeacaaaaaa add r1.xy, r1.xyyy, r4.xyyy
aaaaaaaaaeaaahacacaaaaoeaeaaaaaaaaaaaaaaaaaaaaaa mov r4.xyz, v2
adaaaaaaafaaahacaaaaaaaaacaaaaaaabaaaaoeaeaaaaaa mul r5.xyz, r0.x, v1
abaaaaaaaeaaahacafaaaakeacaaaaaaaeaaaakeacaaaaaa add r4.xyz, r5.xyzz, r4.xyzz
aaaaaaaaaaaaaiacagaaaaffabaaaaaaaaaaaaaaaaaaaaaa mov r0.w, c6.y
ciaaaaaaacaaapacadaaaafeacaaaaaaabaaaaaaafaababb tex r2, r3.xyyy, s1 <2d wrap linear point>
ciaaaaaaabaaapacabaaaafeacaaaaaaadaaaaaaafaababb tex r1, r1.xyyy, s3 <2d wrap linear point>
ciaaaaaaadaaapacadaaaafeacaaaaaaacaaaaaaafaababb tex r3, r3.xyyy, s2 <2d wrap linear point>
aaaaaaaaabaaabacabaaaappacaaaaaaaaaaaaaaaaaaaaaa mov r1.x, r1.w
adaaaaaaafaaadacabaaaafeacaaaaaaafaaaakkabaaaaaa mul r5.xy, r1.xyyy, c5.z
abaaaaaaafaaadacafaaaafeacaaaaaaafaaaappabaaaaaa add r5.xy, r5.xyyy, c5.w
adaaaaaaabaaabacafaaaaffacaaaaaaafaaaaffacaaaaaa mul r1.x, r5.y, r5.y
bfaaaaaaaeaaaiacafaaaaaaacaaaaaaaaaaaaaaaaaaaaaa neg r4.w, r5.x
adaaaaaaaeaaaiacaeaaaappacaaaaaaafaaaaaaacaaaaaa mul r4.w, r4.w, r5.x
acaaaaaaabaaabacaeaaaappacaaaaaaabaaaaaaacaaaaaa sub r1.x, r4.w, r1.x
abaaaaaaaaaaabacabaaaaaaacaaaaaaagaaaaoeabaaaaaa add r0.x, r1.x, c6
bcaaaaaaabaaabacaeaaaakeacaaaaaaaeaaaakeacaaaaaa dp3 r1.x, r4.xyzz, r4.xyzz
akaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r0.x, r0.x
afaaaaaaafaaaeacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rcp r5.z, r0.x
akaaaaaaabaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r1.x, r1.x
adaaaaaaabaaahacabaaaaaaacaaaaaaaeaaaakeacaaaaaa mul r1.xyz, r1.x, r4.xyzz
bcaaaaaaabaaabacafaaaakeacaaaaaaabaaaakeacaaaaaa dp3 r1.x, r5.xyzz, r1.xyzz
aaaaaaaaaaaaabacacaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0.x, c2
adaaaaaaaaaaabacagaaaakkabaaaaaaaaaaaaaaacaaaaaa mul r0.x, c6.z, r0.x
ahaaaaaaabaaabacabaaaaaaacaaaaaaagaaaaffabaaaaaa max r1.x, r1.x, c6.y
alaaaaaaaeaaapacabaaaaaaacaaaaaaaaaaaaaaacaaaaaa pow r4, r1.x, r0.x
adaaaaaaaaaaahacacaaaappacaaaaaaadaaaakeacaaaaaa mul r0.xyz, r2.w, r3.xyzz
adaaaaaaacaaahacacaaaakeacaaaaaaabaaaaoeabaaaaaa mul r2.xyz, r2.xyzz, c1
adaaaaaaabaaahacaaaaaakeacaaaaaaadaaaaaaabaaaaaa mul r1.xyz, r0.xyzz, c3.x
aaaaaaaaaaaaabacaeaaaaaaacaaaaaaaaaaaaaaaaaaaaaa mov r0.x, r4.x
adaaaaaaaaaaahacaaaaaaaaacaaaaaaabaaaakeacaaaaaa mul r0.xyz, r0.x, r1.xyzz
adaaaaaaabaaahacaaaaaakeacaaaaaaaaaaaaoeabaaaaaa mul r1.xyz, r0.xyzz, c0
bcaaaaaaaaaaabacafaaaakeacaaaaaaacaaaaoeaeaaaaaa dp3 r0.x, r5.xyzz, v2
ahaaaaaaaaaaabacaaaaaaaaacaaaaaaagaaaaffabaaaaaa max r0.x, r0.x, c6.y
adaaaaaaadaaahacacaaaakeacaaaaaaaaaaaaoeabaaaaaa mul r3.xyz, r2.xyzz, c0
adaaaaaaaaaaahacadaaaakeacaaaaaaaaaaaaaaacaaaaaa mul r0.xyz, r3.xyzz, r0.x
abaaaaaaaaaaahacaaaaaakeacaaaaaaabaaaakeacaaaaaa add r0.xyz, r0.xyzz, r1.xyzz
adaaaaaaabaaahacacaaaakeacaaaaaaadaaaaoeaeaaaaaa mul r1.xyz, r2.xyzz, v3
adaaaaaaaaaaahacaaaaaakeacaaaaaaafaaaakkabaaaaaa mul r0.xyz, r0.xyzz, c5.z
abaaaaaaaaaaahacaaaaaakeacaaaaaaabaaaakeacaaaaaa add r0.xyz, r0.xyzz, r1.xyzz
aaaaaaaaaaaaapadaaaaaaoeacaaaaaaaaaaaaaaaaaaaaaa mov o0, r0
"
}

SubProgram "d3d11_9x " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
ConstBuffer "$Globals" 112 // 76 used size, 9 vars
Vector 16 [_LightColor0] 4
Vector 48 [_Color] 4
Float 64 [_Shininess]
Float 68 [_Gloss]
Float 72 [_Parallax]
BindCB "$Globals" 0
SetTexture 0 [_ParallaxMap] 2D 3
SetTexture 1 [_MainTex] 2D 0
SetTexture 2 [_SpecMap] 2D 2
SetTexture 3 [_BumpMap] 2D 1
// 37 instructions, 4 temp regs, 0 temp arrays:
// ALU 22 float, 0 int, 0 uint
// TEX 4 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0_level_9_3
eefiecedhehfeafhcihpjjlddpoglgkicgbchphiabaaaaaameajaaaaaeaaaaaa
daaaaaaajmadaaaapaaiaaaajaajaaaaebgpgodjgeadaaaageadaaaaaaacpppp
biadaaaaemaaaaaaacaadeaaaaaaemaaaaaaemaaaeaaceaaaaaaemaaabaaaaaa
adababaaacacacaaaaadadaaaaaaabaaabaaaaaaaaaaaaaaaaaaadaaacaaabaa
aaaaaaaaabacppppfbaaaaafadaaapkaaaaaaadpaaaaaaecdnaknhdoaaaaiadp
fbaaaaafaeaaapkaaaaaaaeaaaaaialpaaaaaaaaaaaaaaaabpaaaaacaaaaaaia
aaaaaplabpaaaaacaaaaaaiaabaachlabpaaaaacaaaaaaiaacaachlabpaaaaac
aaaaaaiaadaachlabpaaaaacaaaaaajaaaaiapkabpaaaaacaaaaaajaabaiapka
bpaaaaacaaaaaajaacaiapkabpaaaaacaaaaaajaadaiapkaaiaaaaadaaaaciia
abaaoelaabaaoelaahaaaaacaaaacbiaaaaappiaaeaaaaaeaaaaaciaabaakkla
aaaaaaiaadaakkkaagaaaaacaaaaaciaaaaaffiaafaaaaadaaaaamiaaaaaaaia
abaaeelaabaaaaacabaaahiaabaaoelaaeaaaaaeabaachiaabaaoeiaaaaaaaia
acaaoelaceaaaaacacaachiaabaaoeiaafaaaaadaaaaadiaaaaaffiaaaaaooia
abaaaaacabaaadiaaaaaoolaecaaaaadabaacpiaabaaoeiaadaioekaabaaaaac
abaaafiaacaaoekaafaaaaadaaaaamiaabaaceiaadaaeekaaeaaaaaeacaaciia
abaappiaacaakkkaaaaakkibaeaaaaaeabaaadiaacaappiaaaaaoeiaaaaaoola
aeaaaaaeaaaaadiaacaappiaaaaaoeiaaaaaoelaecaaaaadabaacpiaabaaoeia
abaioekaaeaaaaaeabaacdiaabaaohiaaeaaaakaaeaaffkaaeaaaaaeabaaciia
abaaaaiaabaaaaibadaappkaaeaaaaaeabaaciiaabaaffiaabaaffibabaappia
ahaaaaacabaaciiaabaappiaagaaaaacabaaceiaabaappiaaiaaaaadabaaciia
abaaoeiaacaaoeiaaiaaaaadaaaaceiaabaaoeiaacaaoelaalaaaaadabaacbia
aaaakkiaaeaakkkaalaaaaadaaaaaeiaabaappiaaeaakkkacaaaaaadabaaacia
aaaakkiaaaaappiaecaaaaadacaacpiaaaaaoeiaaaaioekaecaaaaadaaaacpia
aaaaoeiaacaioekaafaaaaadaaaachiaaaaaoeiaacaappiaafaaaaadacaachia
acaaoeiaabaaoekaafaaaaadaaaachiaaaaaoeiaacaaffkaafaaaaadaaaachia
aaaaoeiaabaaffiaafaaaaadaaaachiaaaaaoeiaaaaaoekaafaaaaadabaacoia
acaajaiaaaaajakaaeaaaaaeaaaachiaabaapjiaabaaaaiaaaaaoeiaacaaaaad
aaaachiaaaaaoeiaaaaaoeiaaeaaaaaeaaaachiaacaaoeiaadaaoelaaaaaoeia
abaaaaacaaaaciiaaeaakkkaabaaaaacaaaicpiaaaaaoeiappppaaaafdeieefc
emafaaaaeaaaaaaafdabaaaafjaaaaaeegiocaaaaaaaaaaaafaaaaaafkaaaaad
aagabaaaaaaaaaaafkaaaaadaagabaaaabaaaaaafkaaaaadaagabaaaacaaaaaa
fkaaaaadaagabaaaadaaaaaafibiaaaeaahabaaaaaaaaaaaffffaaaafibiaaae
aahabaaaabaaaaaaffffaaaafibiaaaeaahabaaaacaaaaaaffffaaaafibiaaae
aahabaaaadaaaaaaffffaaaagcbaaaadpcbabaaaabaaaaaagcbaaaadhcbabaaa
acaaaaaagcbaaaadhcbabaaaadaaaaaagcbaaaadhcbabaaaaeaaaaaagfaaaaad
pccabaaaaaaaaaaagiaaaaacaeaaaaaabaaaaaahbcaabaaaaaaaaaaaegbcbaaa
acaaaaaaegbcbaaaacaaaaaaeeaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaa
dcaaaaajocaabaaaaaaaaaaaagbjbaaaacaaaaaaagaabaaaaaaaaaaaagbjbaaa
adaaaaaabaaaaaahbcaabaaaabaaaaaajgahbaaaaaaaaaaajgahbaaaaaaaaaaa
eeaaaaafbcaabaaaabaaaaaaakaabaaaabaaaaaadiaaaaahocaabaaaaaaaaaaa
fgaobaaaaaaaaaaaagaabaaaabaaaaaadiaaaaahdcaabaaaabaaaaaaagaabaaa
aaaaaaaaegbabaaaacaaaaaadcaaaaajbcaabaaaaaaaaaaackbabaaaacaaaaaa
akaabaaaaaaaaaaaabeaaaaadnaknhdoaoaaaaahpcaabaaaabaaaaaaegaebaaa
abaaaaaaagaabaaaaaaaaaaaefaaaaajpcaabaaaacaaaaaaogbkbaaaabaaaaaa
eghobaaaaaaaaaaaaagabaaaadaaaaaadiaaaaaldcaabaaaacaaaaaacgikcaaa
aaaaaaaaaeaaaaaaaceaaaaaaaaaaadpaaaaaaecaaaaaaaaaaaaaaaadcaaaaal
bcaabaaaaaaaaaaadkaabaaaacaaaaaackiacaaaaaaaaaaaaeaaaaaaakaabaia
ebaaaaaaacaaaaaadcaaaaajpcaabaaaabaaaaaaagaabaaaaaaaaaaaegaobaaa
abaaaaaaegbobaaaabaaaaaaefaaaaajpcaabaaaadaaaaaaogakbaaaabaaaaaa
eghobaaaadaaaaaaaagabaaaabaaaaaadcaaaaapdcaabaaaadaaaaaahgapbaaa
adaaaaaaaceaaaaaaaaaaaeaaaaaaaeaaaaaaaaaaaaaaaaaaceaaaaaaaaaialp
aaaaialpaaaaaaaaaaaaaaaadcaaaaakbcaabaaaaaaaaaaaakaabaiaebaaaaaa
adaaaaaaakaabaaaadaaaaaaabeaaaaaaaaaiadpdcaaaaakbcaabaaaaaaaaaaa
bkaabaiaebaaaaaaadaaaaaabkaabaaaadaaaaaaakaabaaaaaaaaaaaelaaaaaf
ecaabaaaadaaaaaaakaabaaaaaaaaaaabaaaaaahbcaabaaaaaaaaaaaegacbaaa
adaaaaaajgahbaaaaaaaaaaabaaaaaahccaabaaaaaaaaaaaegacbaaaadaaaaaa
egbcbaaaadaaaaaadeaaaaakdcaabaaaaaaaaaaaegaabaaaaaaaaaaaaceaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaacpaaaaafbcaabaaaaaaaaaaaakaabaaa
aaaaaaaadiaaaaahbcaabaaaaaaaaaaaakaabaaaaaaaaaaabkaabaaaacaaaaaa
bjaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaaefaaaaajpcaabaaaacaaaaaa
egaabaaaabaaaaaaeghobaaaacaaaaaaaagabaaaacaaaaaaefaaaaajpcaabaaa
abaaaaaaegaabaaaabaaaaaaeghobaaaabaaaaaaaagabaaaaaaaaaaadiaaaaah
hcaabaaaacaaaaaaegacbaaaacaaaaaapgapbaaaabaaaaaadiaaaaaihcaabaaa
abaaaaaaegacbaaaabaaaaaaegiccaaaaaaaaaaaadaaaaaadiaaaaaihcaabaaa
acaaaaaaegacbaaaacaaaaaafgifcaaaaaaaaaaaaeaaaaaadiaaaaahncaabaaa
aaaaaaaaagaabaaaaaaaaaaaagajbaaaacaaaaaadiaaaaaincaabaaaaaaaaaaa
agaobaaaaaaaaaaaagijcaaaaaaaaaaaabaaaaaadiaaaaaihcaabaaaacaaaaaa
egacbaaaabaaaaaaegiccaaaaaaaaaaaabaaaaaadcaaaaajhcaabaaaaaaaaaaa
egacbaaaacaaaaaafgafbaaaaaaaaaaaigadbaaaaaaaaaaaaaaaaaahhcaabaaa
aaaaaaaaegacbaaaaaaaaaaaegacbaaaaaaaaaaadcaaaaajhccabaaaaaaaaaaa
egacbaaaabaaaaaaegbcbaaaaeaaaaaaegacbaaaaaaaaaaadgaaaaaficcabaaa
aaaaaaaaabeaaaaaaaaaaaaadoaaaaabejfdeheojiaaaaaaafaaaaaaaiaaaaaa
iaaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaabaaaaaaapapaaaaimaaaaaaabaaaaaaaaaaaaaaadaaaaaa
acaaaaaaahahaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaahahaaaa
imaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaahahaaaafdfgfpfaepfdejfe
ejepeoaafeeffiedepepfceeaaklklklepfdeheocmaaaaaaabaaaaaaaiaaaaaa
caaaaaaaaaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaafdfgfpfegbhcghgf
heaaklkl"
}

SubProgram "opengl " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Vector 0 [_Color]
Float 1 [_Parallax]
SetTexture 0 [_ParallaxMap] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 3 [unity_Lightmap] 2D
"!!ARBfp1.0
OPTION ARB_precision_hint_fastest;
# 18 ALU, 3 TEX
PARAM c[3] = { program.local[0..1],
		{ 0.5, 0, 0.41999999, 8 } };
TEMP R0;
TEMP R1;
TEX R0.w, fragment.texcoord[0].zwzw, texture[0], 2D;
DP3 R0.x, fragment.texcoord[1], fragment.texcoord[1];
RSQ R0.x, R0.x;
MUL R0.xyz, R0.x, fragment.texcoord[1];
ADD R1.x, R0.z, c[2].z;
RCP R1.x, R1.x;
MOV R0.z, c[1].x;
MUL R0.z, R0, c[2].x;
MUL R0.xy, R0, R1.x;
MAD R0.z, R0.w, c[1].x, -R0;
MAD R0.xy, R0.z, R0, fragment.texcoord[0];
MOV result.color.w, c[2].y;
TEX R1.xyz, R0, texture[1], 2D;
TEX R0, fragment.texcoord[2], texture[3], 2D;
MUL R1.xyz, R1, c[0];
MUL R0.xyz, R0.w, R0;
MUL R0.xyz, R0, R1;
MUL result.color.xyz, R0, c[2].w;
END
# 18 instructions, 2 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Vector 0 [_Color]
Float 1 [_Parallax]
SetTexture 0 [_ParallaxMap] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 3 [unity_Lightmap] 2D
"ps_2_0
; 18 ALU, 3 TEX
dcl_2d s0
dcl_2d s1
dcl_2d s3
def c2, 0.50000000, 0.41999999, 8.00000000, 0.00000000
dcl t0
dcl t1.xyz
dcl t2.xy
mov r0.y, t0.w
mov r0.x, t0.z
texld r0, r0, s0
dp3_pp r0.x, t1, t1
rsq_pp r0.x, r0.x
mul_pp r2.xyz, r0.x, t1
add r0.x, r2.z, c2.y
rcp r1.x, r0.x
mov_pp r0.x, c2
mul_pp r0.x, c1, r0
mul r1.xy, r2, r1.x
mad_pp r0.x, r0.w, c1, -r0
mad r0.xy, r0.x, r1, t0
texld r1, r0, s1
texld r0, t2, s3
mul_pp r0.xyz, r0.w, r0
mul_pp r1.xyz, r1, c0
mul_pp r0.xyz, r0, r1
mul_pp r0.xyz, r0, c2.z
mov_pp r0.w, c2
mov_pp oC0, r0
"
}

SubProgram "xbox360 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Vector 0 [_Color]
Float 1 [_Parallax]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_ParallaxMap] 2D
SetTexture 2 [unity_Lightmap] 2D
// Shader Timing Estimate, in Cycles/64 pixel vector:
// ALU: 14.67 (11 instructions), vertex: 0, texture: 12,
//   sequencer: 8, interpolator: 12;    3 GPRs, 63 threads,
// Performance (if enough threads): ~14 cycles per vector
// * Texture cycle estimates are assuming an 8bit/component texture with no
//     aniso or trilinear filtering.

"ps_360
backbbaaaaaaabhmaaaaabamaaaaaaaaaaaaaaceaaaaabciaaaaabfaaaaaaaaa
aaaaaaaaaaaaabaaaaaaaabmaaaaaapdppppadaaaaaaaaafaaaaaabmaaaaaaaa
aaaaaaomaaaaaaiaaaacaaaaaaabaaaaaaaaaaiiaaaaaaaaaaaaaajiaaadaaaa
aaabaaaaaaaaaakeaaaaaaaaaaaaaaleaaacaaabaaabaaaaaaaaaamaaaaaaaaa
aaaaaanaaaadaaabaaabaaaaaaaaaakeaaaaaaaaaaaaaannaaadaaacaaabaaaa
aaaaaakeaaaaaaaafpedgpgmgphcaaklaaabaaadaaabaaaeaaabaaaaaaaaaaaa
fpengbgjgofegfhiaaklklklaaaeaaamaaabaaabaaabaaaaaaaaaaaafpfagbhc
gbgmgmgbhiaaklklaaaaaaadaaabaaabaaabaaaaaaaaaaaafpfagbhcgbgmgmgb
hiengbhaaahfgogjhehjfpemgjghgihegngbhaaahahdfpddfpdaaadccodacodc
dadddfddcodaaaklaaaaaaaaaaaaaaabaaaaaaaaaaaaaaaaaaaaaabeabpmaaba
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaeaaaaaaammbaaaacaaaaaaaaae
aaaaaaaaaaaacegdaaahaaahaaaaaaabaaaapafaaaaahbfbaaaadcfcaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaebaaaaaadpaaaaaadonhakdnaaaaaaaaaaajgaac
faaibcaabcaaabeaaaaaaaaadaanmeaaccaaaaaadibacaabbpbpphppaaaaeaaa
miaeaaacaaloloaapaababaafieiacabaagmlbmgcbabppicmiaiaaababblgmbl
klacababmiahaaabaamgloaaobacabaaleeaacaaaaaaaamamcaaaappemeaacaa
aaaaaamgocaaaaacmiadaaabaamfmgaaobabacaamiadaaaaaalabllaolababaa
baaibaabbpbppoiiaaaaeaaabaciaaebbpbppgiiaaaaeaaakiihababaamamaed
ibabaappmiahaaaaaablmaaaobabaaaamiahmaaaaamamaaaobabaaaaaaaaaaaa
aaaaaaaaaaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Vector 0 [_Color]
Float 1 [_Parallax]
SetTexture 0 [_ParallaxMap] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 3 [unity_Lightmap] 2D
"sce_fp_rsx // 21 instructions using 3 registers
[Configuration]
24
ffffffff0001c0200007fffc000000000000840003000000
[Offsets]
2
_Color 1 0
00000100
_Parallax 2 0
0000009000000070
[Microcode]
336
de021706c8011c9dc8000001c8003fe1ae803940c8011c9dc8000029c800bfe1
0400030055001c9d00020000c80000010a3d3ed7000000000000000000000000
9800010080011c9cc8000001c8003fe1900417005c011c9dc8000001c8003fe1
02880240fe081c9d00020000c800000100000000000000000000000000000000
1088014000021c9cc8000001c800000100000000000000000000000000000000
06003a00c9001c9daa000000c800000102880440ff101c9d00020000c9100001
0000bf000000000000000000000000000600040001101c9cc80000015c000001
0e001702c8001c9dc8000001c80000010e800240c8001c9dc8020001c8000001
0000000000000000000000000000000010800140c8021c9dc8000001c8000001
000000000000000000000000000000000e800240fe041c9dc9000001c8000001
0e810240c9001c9dc8043001c8000001
"
}

SubProgram "d3d11 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
ConstBuffer "$Globals" 128 // 76 used size, 10 vars
Vector 48 [_Color] 4
Float 72 [_Parallax]
BindCB "$Globals" 0
SetTexture 0 [_ParallaxMap] 2D 1
SetTexture 1 [_MainTex] 2D 0
SetTexture 2 [unity_Lightmap] 2D 2
// 17 instructions, 2 temp regs, 0 temp arrays:
// ALU 9 float, 0 int, 0 uint
// TEX 3 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0
eefiecedaokllhhgifndodiaiapmhcdbhkedmlbbabaaaaaaiiadaaaaadaaaaaa
cmaaaaaaleaaaaaaoiaaaaaaejfdeheoiaaaaaaaaeaaaaaaaiaaaaaagiaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaheaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaaheaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaaheaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaadadaaaafdfgfpfa
epfdejfeejepeoaafeeffiedepepfceeaaklklklepfdeheocmaaaaaaabaaaaaa
aiaaaaaacaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaafdfgfpfe
gbhcghgfheaaklklfdeieefcjiacaaaaeaaaaaaakgaaaaaafjaaaaaeegiocaaa
aaaaaaaaafaaaaaafkaaaaadaagabaaaaaaaaaaafkaaaaadaagabaaaabaaaaaa
fkaaaaadaagabaaaacaaaaaafibiaaaeaahabaaaaaaaaaaaffffaaaafibiaaae
aahabaaaabaaaaaaffffaaaafibiaaaeaahabaaaacaaaaaaffffaaaagcbaaaad
pcbabaaaabaaaaaagcbaaaadhcbabaaaacaaaaaagcbaaaaddcbabaaaadaaaaaa
gfaaaaadpccabaaaaaaaaaaagiaaaaacacaaaaaabaaaaaahbcaabaaaaaaaaaaa
egbcbaaaacaaaaaaegbcbaaaacaaaaaaeeaaaaafbcaabaaaaaaaaaaaakaabaaa
aaaaaaaadiaaaaahgcaabaaaaaaaaaaaagaabaaaaaaaaaaaagbbbaaaacaaaaaa
dcaaaaajbcaabaaaaaaaaaaackbabaaaacaaaaaaakaabaaaaaaaaaaaabeaaaaa
dnaknhdoaoaaaaahdcaabaaaaaaaaaaajgafbaaaaaaaaaaaagaabaaaaaaaaaaa
efaaaaajpcaabaaaabaaaaaaogbkbaaaabaaaaaaeghobaaaaaaaaaaaaagabaaa
abaaaaaadiaaaaaiecaabaaaaaaaaaaackiacaaaaaaaaaaaaeaaaaaaabeaaaaa
aaaaaadpdcaaaaalecaabaaaaaaaaaaadkaabaaaabaaaaaackiacaaaaaaaaaaa
aeaaaaaackaabaiaebaaaaaaaaaaaaaadcaaaaajdcaabaaaaaaaaaaakgakbaaa
aaaaaaaaegaabaaaaaaaaaaaegbabaaaabaaaaaaefaaaaajpcaabaaaaaaaaaaa
egaabaaaaaaaaaaaeghobaaaabaaaaaaaagabaaaaaaaaaaadiaaaaaihcaabaaa
aaaaaaaaegacbaaaaaaaaaaaegiccaaaaaaaaaaaadaaaaaaefaaaaajpcaabaaa
abaaaaaaegbabaaaadaaaaaaeghobaaaacaaaaaaaagabaaaacaaaaaadiaaaaah
icaabaaaaaaaaaaadkaabaaaabaaaaaaabeaaaaaaaaaaaebdiaaaaahhcaabaaa
abaaaaaaegacbaaaabaaaaaapgapbaaaaaaaaaaadiaaaaahhccabaaaaaaaaaaa
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
Float 1 [_Parallax]
SetTexture 0 [_ParallaxMap] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 3 [unity_Lightmap] 2D
"agal_ps
c2 0.5 0.42 8.0 0.0
[bc]
aaaaaaaaaaaaacacaaaaaappaeaaaaaaaaaaaaaaaaaaaaaa mov r0.y, v0.w
aaaaaaaaaaaaabacaaaaaakkaeaaaaaaaaaaaaaaaaaaaaaa mov r0.x, v0.z
ciaaaaaaaaaaapacaaaaaafeacaaaaaaaaaaaaaaafaababb tex r0, r0.xyyy, s0 <2d wrap linear point>
bcaaaaaaaaaaabacabaaaaoeaeaaaaaaabaaaaoeaeaaaaaa dp3 r0.x, v1, v1
akaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r0.x, r0.x
adaaaaaaacaaahacaaaaaaaaacaaaaaaabaaaaoeaeaaaaaa mul r2.xyz, r0.x, v1
abaaaaaaaaaaabacacaaaakkacaaaaaaacaaaaffabaaaaaa add r0.x, r2.z, c2.y
afaaaaaaabaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rcp r1.x, r0.x
aaaaaaaaaaaaabacacaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0.x, c2
adaaaaaaaaaaabacabaaaaoeabaaaaaaaaaaaaaaacaaaaaa mul r0.x, c1, r0.x
adaaaaaaabaaadacacaaaafeacaaaaaaabaaaaaaacaaaaaa mul r1.xy, r2.xyyy, r1.x
adaaaaaaabaaaiacaaaaaappacaaaaaaabaaaaoeabaaaaaa mul r1.w, r0.w, c1
acaaaaaaaaaaabacabaaaappacaaaaaaaaaaaaaaacaaaaaa sub r0.x, r1.w, r0.x
adaaaaaaaaaaadacaaaaaaaaacaaaaaaabaaaafeacaaaaaa mul r0.xy, r0.x, r1.xyyy
abaaaaaaaaaaadacaaaaaafeacaaaaaaaaaaaaoeaeaaaaaa add r0.xy, r0.xyyy, v0
ciaaaaaaabaaapacaaaaaafeacaaaaaaabaaaaaaafaababb tex r1, r0.xyyy, s1 <2d wrap linear point>
ciaaaaaaaaaaapacacaaaaoeaeaaaaaaadaaaaaaafaababb tex r0, v2, s3 <2d wrap linear point>
adaaaaaaaaaaahacaaaaaappacaaaaaaaaaaaakeacaaaaaa mul r0.xyz, r0.w, r0.xyzz
adaaaaaaabaaahacabaaaakeacaaaaaaaaaaaaoeabaaaaaa mul r1.xyz, r1.xyzz, c0
adaaaaaaaaaaahacaaaaaakeacaaaaaaabaaaakeacaaaaaa mul r0.xyz, r0.xyzz, r1.xyzz
adaaaaaaaaaaahacaaaaaakeacaaaaaaacaaaakkabaaaaaa mul r0.xyz, r0.xyzz, c2.z
aaaaaaaaaaaaaiacacaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0.w, c2
aaaaaaaaaaaaapadaaaaaaoeacaaaaaaaaaaaaaaaaaaaaaa mov o0, r0
"
}

SubProgram "d3d11_9x " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
ConstBuffer "$Globals" 128 // 76 used size, 10 vars
Vector 48 [_Color] 4
Float 72 [_Parallax]
BindCB "$Globals" 0
SetTexture 0 [_ParallaxMap] 2D 1
SetTexture 1 [_MainTex] 2D 0
SetTexture 2 [unity_Lightmap] 2D 2
// 17 instructions, 2 temp regs, 0 temp arrays:
// ALU 9 float, 0 int, 0 uint
// TEX 3 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0_level_9_3
eefiecedbmbiabogfeedejbmgdejonlbgilecejhabaaaaaagmafaaaaaeaaaaaa
daaaaaaabaacaaaalaaeaaaadiafaaaaebgpgodjniabaaaaniabaaaaaaacpppp
jmabaaaadmaaaaaaabaadaaaaaaadmaaaaaadmaaadaaceaaaaaadmaaabaaaaaa
aaababaaacacacaaaaaaadaaacaaaaaaaaaaaaaaabacppppfbaaaaafacaaapka
aaaaaadpdnaknhdoaaaaaaebaaaaaaaabpaaaaacaaaaaaiaaaaaaplabpaaaaac
aaaaaaiaabaaahlabpaaaaacaaaaaaiaacaaadlabpaaaaacaaaaaajaaaaiapka
bpaaaaacaaaaaajaabaiapkabpaaaaacaaaaaajaacaiapkaaiaaaaadaaaaciia
abaaoelaabaaoelaahaaaaacaaaacbiaaaaappiaaeaaaaaeaaaaaciaabaakkla
aaaaaaiaacaaffkaafaaaaadaaaaafiaaaaaaaiaabaanelaagaaaaacaaaaacia
aaaaffiaafaaaaadaaaaadiaaaaaffiaaaaaoiiaabaaaaacabaaadiaaaaaoola
ecaaaaadabaacpiaabaaoeiaabaioekaabaaaaacabaaabiaacaaaakaafaaaaad
aaaaceiaabaaaaiaabaakkkaaeaaaaaeaaaaceiaabaappiaabaakkkaaaaakkib
aeaaaaaeaaaaadiaaaaakkiaaaaaoeiaaaaaoelaecaaaaadabaacpiaacaaoela
acaioekaecaaaaadaaaacpiaaaaaoeiaaaaioekaafaaaaadaaaachiaaaaaoeia
aaaaoekaafaaaaadaaaaciiaabaappiaacaakkkaafaaaaadabaachiaabaaoeia
aaaappiaafaaaaadaaaachiaaaaaoeiaabaaoeiaabaaaaacaaaaciiaacaappka
abaaaaacaaaicpiaaaaaoeiappppaaaafdeieefcjiacaaaaeaaaaaaakgaaaaaa
fjaaaaaeegiocaaaaaaaaaaaafaaaaaafkaaaaadaagabaaaaaaaaaaafkaaaaad
aagabaaaabaaaaaafkaaaaadaagabaaaacaaaaaafibiaaaeaahabaaaaaaaaaaa
ffffaaaafibiaaaeaahabaaaabaaaaaaffffaaaafibiaaaeaahabaaaacaaaaaa
ffffaaaagcbaaaadpcbabaaaabaaaaaagcbaaaadhcbabaaaacaaaaaagcbaaaad
dcbabaaaadaaaaaagfaaaaadpccabaaaaaaaaaaagiaaaaacacaaaaaabaaaaaah
bcaabaaaaaaaaaaaegbcbaaaacaaaaaaegbcbaaaacaaaaaaeeaaaaafbcaabaaa
aaaaaaaaakaabaaaaaaaaaaadiaaaaahgcaabaaaaaaaaaaaagaabaaaaaaaaaaa
agbbbaaaacaaaaaadcaaaaajbcaabaaaaaaaaaaackbabaaaacaaaaaaakaabaaa
aaaaaaaaabeaaaaadnaknhdoaoaaaaahdcaabaaaaaaaaaaajgafbaaaaaaaaaaa
agaabaaaaaaaaaaaefaaaaajpcaabaaaabaaaaaaogbkbaaaabaaaaaaeghobaaa
aaaaaaaaaagabaaaabaaaaaadiaaaaaiecaabaaaaaaaaaaackiacaaaaaaaaaaa
aeaaaaaaabeaaaaaaaaaaadpdcaaaaalecaabaaaaaaaaaaadkaabaaaabaaaaaa
ckiacaaaaaaaaaaaaeaaaaaackaabaiaebaaaaaaaaaaaaaadcaaaaajdcaabaaa
aaaaaaaakgakbaaaaaaaaaaaegaabaaaaaaaaaaaegbabaaaabaaaaaaefaaaaaj
pcaabaaaaaaaaaaaegaabaaaaaaaaaaaeghobaaaabaaaaaaaagabaaaaaaaaaaa
diaaaaaihcaabaaaaaaaaaaaegacbaaaaaaaaaaaegiccaaaaaaaaaaaadaaaaaa
efaaaaajpcaabaaaabaaaaaaegbabaaaadaaaaaaeghobaaaacaaaaaaaagabaaa
acaaaaaadiaaaaahicaabaaaaaaaaaaadkaabaaaabaaaaaaabeaaaaaaaaaaaeb
diaaaaahhcaabaaaabaaaaaaegacbaaaabaaaaaapgapbaaaaaaaaaaadiaaaaah
hccabaaaaaaaaaaaegacbaaaaaaaaaaaegacbaaaabaaaaaadgaaaaaficcabaaa
aaaaaaaaabeaaaaaaaaaaaaadoaaaaabejfdeheoiaaaaaaaaeaaaaaaaiaaaaaa
giaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaheaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaabaaaaaaapapaaaaheaaaaaaabaaaaaaaaaaaaaaadaaaaaa
acaaaaaaahahaaaaheaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaadadaaaa
fdfgfpfaepfdejfeejepeoaafeeffiedepepfceeaaklklklepfdeheocmaaaaaa
abaaaaaaaiaaaaaacaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaa
fdfgfpfegbhcghgfheaaklkl"
}

SubProgram "opengl " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
Float 4 [_Parallax]
SetTexture 0 [_ParallaxMap] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 2 [_SpecMap] 2D
SetTexture 3 [_BumpMap] 2D
SetTexture 4 [_ShadowMapTexture] 2D
"!!ARBfp1.0
OPTION ARB_precision_hint_fastest;
# 45 ALU, 5 TEX
PARAM c[7] = { program.local[0..4],
		{ 0.5, 0.41999999, 2, 1 },
		{ 0, 32 } };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
TEX R0.w, fragment.texcoord[0].zwzw, texture[0], 2D;
DP3 R0.x, fragment.texcoord[1], fragment.texcoord[1];
RSQ R2.w, R0.x;
MUL R1.xyz, R2.w, fragment.texcoord[1];
ADD R0.y, R1.z, c[5];
RCP R0.y, R0.y;
MOV R0.x, c[4];
MUL R0.x, R0, c[5];
MUL R1.xy, R1, R0.y;
MAD R0.x, R0.w, c[4], -R0;
MAD R0.zw, R0.x, R1.xyxy, fragment.texcoord[0].xyxy;
MAD R0.xy, R0.x, R1, fragment.texcoord[0].zwzw;
MOV result.color.w, c[6].x;
TEX R1, R0.zwzw, texture[1], 2D;
TEX R2.xyz, R0.zwzw, texture[2], 2D;
TEX R0.yw, R0, texture[3], 2D;
TXP R0.x, fragment.texcoord[4], texture[4], 2D;
MAD R3.xy, R0.wyzw, c[5].z, -c[5].w;
MOV R0.yzw, fragment.texcoord[2].xxyz;
MUL R2.xyz, R1.w, R2;
MUL R3.z, R3.y, R3.y;
MAD R0.yzw, R2.w, fragment.texcoord[1].xxyz, R0;
MAD R2.w, -R3.x, R3.x, -R3.z;
DP3 R3.z, R0.yzww, R0.yzww;
RSQ R3.z, R3.z;
ADD R2.w, R2, c[5];
MUL R0.yzw, R3.z, R0;
RSQ R2.w, R2.w;
RCP R3.z, R2.w;
DP3 R0.y, R3, R0.yzww;
MOV R0.z, c[6].y;
MUL R0.z, R0, c[2].x;
MAX R0.y, R0, c[6].x;
POW R0.y, R0.y, R0.z;
MUL R2.xyz, R2, c[3].x;
MUL R2.xyz, R0.y, R2;
DP3 R0.y, R3, fragment.texcoord[2];
MUL R1.xyz, R1, c[1];
MUL R3.xyz, R1, c[0];
MAX R0.y, R0, c[6].x;
MUL R2.xyz, R2, c[0];
MAD R2.xyz, R3, R0.y, R2;
MUL R1.xyz, R1, fragment.texcoord[3];
MUL R0.xyz, R0.x, R2;
MAD result.color.xyz, R0, c[5].z, R1;
END
# 45 instructions, 4 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
Float 4 [_Parallax]
SetTexture 0 [_ParallaxMap] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 2 [_SpecMap] 2D
SetTexture 3 [_BumpMap] 2D
SetTexture 4 [_ShadowMapTexture] 2D
"ps_2_0
; 49 ALU, 5 TEX
dcl_2d s0
dcl_2d s1
dcl_2d s2
dcl_2d s3
dcl_2d s4
def c5, 0.50000000, 0.41999999, 2.00000000, -1.00000000
def c6, 1.00000000, 0.00000000, 32.00000000, 0
dcl t0
dcl t1.xyz
dcl t2.xyz
dcl t3.xyz
dcl t4
texldp r6, t4, s4
mov r4.y, t0.w
mov r4.x, t0.z
mov r0.y, t0.w
mov r0.x, t0.z
mov_pp r5.xyz, t2
texld r0, r0, s0
dp3_pp r0.x, t1, t1
rsq_pp r0.x, r0.x
mul_pp r3.xyz, r0.x, t1
add r1.x, r3.z, c5.y
rcp r2.x, r1.x
mov_pp r1.x, c5
mul_pp r1.x, c4, r1
mad_pp r1.x, r0.w, c4, -r1
mul r2.xy, r3, r2.x
mad r3.xy, r1.x, r2, t0
mad r1.xy, r1.x, r2, r4
mad_pp r5.xyz, r0.x, t1, r5
mov_pp r0.w, c6.y
texld r2, r3, s1
texld r1, r1, s3
texld r3, r3, s2
mov r1.x, r1.w
mad_pp r4.xy, r1, c5.z, c5.w
mul_pp r1.x, r4.y, r4.y
mad_pp r1.x, -r4, r4, -r1
add_pp r0.x, r1, c6
dp3_pp r1.x, r5, r5
rsq_pp r0.x, r0.x
rcp_pp r4.z, r0.x
rsq_pp r1.x, r1.x
mul_pp r1.xyz, r1.x, r5
dp3_pp r1.x, r4, r1
mov_pp r0.x, c2
mul_pp r0.x, c6.z, r0
max_pp r1.x, r1, c6.y
pow r5.x, r1.x, r0.x
mul_pp r0.xyz, r2.w, r3
mul_pp r2.xyz, r2, c1
mul_pp r1.xyz, r0, c3.x
mov r0.x, r5.x
mul r0.xyz, r0.x, r1
mul_pp r1.xyz, r0, c0
dp3_pp r0.x, r4, t2
max_pp r0.x, r0, c6.y
mul_pp r3.xyz, r2, c0
mad_pp r0.xyz, r3, r0.x, r1
mul_pp r0.xyz, r6.x, r0
mul_pp r1.xyz, r2, t3
mad_pp r0.xyz, r0, c5.z, r1
mov_pp oC0, r0
"
}

SubProgram "xbox360 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Vector 1 [_Color]
Float 3 [_Gloss]
Vector 0 [_LightColor0]
Float 4 [_Parallax]
Float 2 [_Shininess]
SetTexture 0 [_ShadowMapTexture] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_SpecMap] 2D
SetTexture 4 [_ParallaxMap] 2D
// Shader Timing Estimate, in Cycles/64 pixel vector:
// ALU: 33.33 (25 instructions), vertex: 0, texture: 20,
//   sequencer: 14, interpolator: 20;    7 GPRs, 27 threads,
// Performance (if enough threads): ~33 cycles per vector
// * Texture cycle estimates are assuming an 8bit/component texture with no
//     aniso or trilinear filtering.

"ps_360
backbbaaaaaaacbmaaaaaboeaaaaaaaaaaaaaaceaaaaabmaaaaaaboiaaaaaaaa
aaaaaaaaaaaaabjiaaaaaabmaaaaabikppppadaaaaaaaaakaaaaaabmaaaaaaaa
aaaaabidaaaaaaoeaaadaaacaaabaaaaaaaaaapaaaaaaaaaaaaaabaaaaacaaab
aaabaaaaaaaaabaiaaaaaaaaaaaaabbiaaacaaadaaabaaaaaaaaabcaaaaaaaaa
aaaaabdaaaacaaaaaaabaaaaaaaaabaiaaaaaaaaaaaaabdnaaadaaabaaabaaaa
aaaaaapaaaaaaaaaaaaaabegaaacaaaeaaabaaaaaaaaabcaaaaaaaaaaaaaabfa
aaadaaaeaaabaaaaaaaaaapaaaaaaaaaaaaaabfnaaadaaaaaaabaaaaaaaaaapa
aaaaaaaaaaaaabgpaaacaaacaaabaaaaaaaaabcaaaaaaaaaaaaaabhkaaadaaad
aaabaaaaaaaaaapaaaaaaaaafpechfgnhaengbhaaaklklklaaaeaaamaaabaaab
aaabaaaaaaaaaaaafpedgpgmgphcaaklaaabaaadaaabaaaeaaabaaaaaaaaaaaa
fpehgmgphdhdaaklaaaaaaadaaabaaabaaabaaaaaaaaaaaafpemgjghgiheedgp
gmgphcdaaafpengbgjgofegfhiaafpfagbhcgbgmgmgbhiaafpfagbhcgbgmgmgb
hiengbhaaafpfdgigbgegphhengbhafegfhihehfhcgfaafpfdgigjgogjgogfhd
hdaafpfdhagfgdengbhaaahahdfpddfpdaaadccodacodcdadddfddcodaaaklkl
aaaaaaaaaaaaaaabaaaaaaaaaaaaaaaaaaaaaabeabpmaabaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaeaaaaaabkebaaaagaaaaaaaaaeaaaaaaaaaaaaeekf
aabpaabpaaaaaaabaaaapafaaaaahbfbaaaahcfcaaaahdfdaaaapefeaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaalpiaaaaa
ecaaaaaaaaaaaaaaaaaaaaaadpaaaaaaaaaaaaaadonhakdndpiaaaaaaaajgaae
gaakbcaabcaaafeaaaabbabaaaaabcaameaaaaaaaaaagabbgabhbcaabcaaaaaa
aaaafabnaaaaccaaaaaaaaaadieadaabbpbpphppaaaaeaaamiaiaaacaaloloaa
paababaafiiiacabaagmgmblcbaeppicmiaiaaababblgmblkladaeabmiahaaaf
aablleaaobacabaaembeababaalbmgblkaafppaeemedababaagmlamgobabaeab
miadaaaeaamemgaaobafabaamiapaaaaaakablaaolaeabaabaaibacbbpbppppi
aaaaeaaalicibaabbpbppompaaaaeaaabadieaabbpbppoiiaaaaeaaababiaaab
bpbppgecaaaaeaaamiahaaaeaablmaaaobaaaeaamiahaaabaaleleaaoaababaa
beahaaafaalemagmmaafacacambgagagaambgmlbiaabpopomiaiaaabaelclcbl
nbagagppbeciaaacaalololbpaafafaafiihacaeaamagmblkbaeadickaihagaf
aamablblobafacibkibeafafaalomdebnaafagabkiciafafaamdloicnaagacab
kiegafacaabglbmaicafppabeaboacagaapmpmlbkbafaaicmiapaaacaaaameaa
obagacaadiihaaaaaamamagmobafadacmiahaaadaamablaaobaeaaaamiahaaac
aamamabfkladaaacmiahmaaaaamagmmaolacabaaaaaaaaaaaaaaaaaaaaaaaaaa
"
}

SubProgram "ps3 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
Float 4 [_Parallax]
SetTexture 0 [_ParallaxMap] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 2 [_SpecMap] 2D
SetTexture 3 [_BumpMap] 2D
SetTexture 4 [_ShadowMapTexture] 2D
"sce_fp_rsx // 60 instructions using 4 registers
[Configuration]
24
ffffffff0007c020001fffe0000000000000840004000000
[Offsets]
5
_LightColor0 2 0
000003a0000002e0
_Color 1 0
00000210
_Shininess 1 0
00000240
_Gloss 1 0
00000160
_Parallax 2 0
0000004000000020
[Microcode]
960
900017005c011c9dc8000001c8003fe102860240fe001c9d00020000c8000001
000000000000000000000000000000001084014000021c9cc8000001c8000001
00000000000000000000000000000000ae843940c8011c9dc8000029c800bfe1
0200030055081c9d00020000c80000010a3d3ed7000000000000000000000000
9e060100c8011c9dc8000001c8003fe102820440ff081c9d00020000c90c0001
0000bf0000000000000000000000000006003a00c9081c9dc8000001c8000001
08000100c8001c9dc8000001c80000011802020001041c9c80000000c8000001
060003005c0c1c9d5c040001c800000114001706c8001c9dc8000001c8000001
18060300800c1c9cc8040001c80000011e0417025c0c1c9dc8000001c8000001
06860440ce001c9d00020000aa020000000040000000bf800000000000000000
1e7e7d00c8001c9dc8000001c800000110800240c8081c9d00020000c8000001
00000000000000000000000000000000ce820140c8011c9dc8000001c8003fe1
1602034049041c9d49080001c80000010e0617045c0c1c9dc8000001c8000001
0e84394068041c9dc8000029c80000010e8c0240ff001c9dc80c0001c8000001
02800240ab0c1c9cab0c0000c800000110840440010c1c9e010c000001000002
10800340c9081c9dc8020001c800000100000000000000000000000000003f80
0e8e0240c8081c9dc8020001c800000100000000000000000000000000000000
1e7e7d00c8001c9dc8000001c80000011084014000021c9cc8000001c8000001
0000000000000000000000000000000008863b40ff003c9dff000001c8000001
10800540c90c1c9dc9080001c800000110840240c9081c9d00020000c8000001
0000420000000000000000000000000008800540c90c1c9dc9040001c8000001
10000900c9001c9d00020000c800000100000000000000000000000000000000
02021d00fe001c9dc8000001c80000010e820240c91c1c9dc8020001c8000001
000000000000000000000000000000001002020000041c9cc9080001c8000001
ee840240c91c1c9dc8015001c8003fe108061c00fe041c9dc8000001c8000001
0e860200540c1c9dc9180001c80000011080090055001c9d00020000c8000001
000000000000000000000000000000000e820240c9041c9dff000001c8000001
1080014000021c9cc8000001c800000100000000000000000000000000000000
02001809c8011c9dc8000001c8003fe10e820440c90c1c9dc8020001c9040001
000000000000000000000000000000000e81044000001c9cc9041001c9080001
"
}

SubProgram "d3d11 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
ConstBuffer "$Globals" 176 // 140 used size, 10 vars
Vector 16 [_LightColor0] 4
Vector 112 [_Color] 4
Float 128 [_Shininess]
Float 132 [_Gloss]
Float 136 [_Parallax]
BindCB "$Globals" 0
SetTexture 0 [_ParallaxMap] 2D 4
SetTexture 1 [_MainTex] 2D 1
SetTexture 2 [_SpecMap] 2D 3
SetTexture 3 [_BumpMap] 2D 2
SetTexture 4 [_ShadowMapTexture] 2D 0
// 40 instructions, 4 temp regs, 0 temp arrays:
// ALU 24 float, 0 int, 0 uint
// TEX 5 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0
eefiecedknplffiffhhaghjpbgckiokdaigcoaddabaaaaaapaagaaaaadaaaaaa
cmaaaaaaoeaaaaaabiabaaaaejfdeheolaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaakeaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaakeaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaakeaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaahahaaaakeaaaaaa
adaaaaaaaaaaaaaaadaaaaaaaeaaaaaaahahaaaakeaaaaaaaeaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapalaaaafdfgfpfaepfdejfeejepeoaafeeffiedepepfcee
aaklklklepfdeheocmaaaaaaabaaaaaaaiaaaaaacaaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaaaaaaaaaapaaaaaafdfgfpfegbhcghgfheaaklklfdeieefcnaafaaaa
eaaaaaaaheabaaaafjaaaaaeegiocaaaaaaaaaaaajaaaaaafkaaaaadaagabaaa
aaaaaaaafkaaaaadaagabaaaabaaaaaafkaaaaadaagabaaaacaaaaaafkaaaaad
aagabaaaadaaaaaafkaaaaadaagabaaaaeaaaaaafibiaaaeaahabaaaaaaaaaaa
ffffaaaafibiaaaeaahabaaaabaaaaaaffffaaaafibiaaaeaahabaaaacaaaaaa
ffffaaaafibiaaaeaahabaaaadaaaaaaffffaaaafibiaaaeaahabaaaaeaaaaaa
ffffaaaagcbaaaadpcbabaaaabaaaaaagcbaaaadhcbabaaaacaaaaaagcbaaaad
hcbabaaaadaaaaaagcbaaaadhcbabaaaaeaaaaaagcbaaaadlcbabaaaafaaaaaa
gfaaaaadpccabaaaaaaaaaaagiaaaaacaeaaaaaabaaaaaahbcaabaaaaaaaaaaa
egbcbaaaacaaaaaaegbcbaaaacaaaaaaeeaaaaafbcaabaaaaaaaaaaaakaabaaa
aaaaaaaadcaaaaajocaabaaaaaaaaaaaagbjbaaaacaaaaaaagaabaaaaaaaaaaa
agbjbaaaadaaaaaabaaaaaahbcaabaaaabaaaaaajgahbaaaaaaaaaaajgahbaaa
aaaaaaaaeeaaaaafbcaabaaaabaaaaaaakaabaaaabaaaaaadiaaaaahocaabaaa
aaaaaaaafgaobaaaaaaaaaaaagaabaaaabaaaaaadiaaaaahdcaabaaaabaaaaaa
agaabaaaaaaaaaaaegbabaaaacaaaaaadcaaaaajbcaabaaaaaaaaaaackbabaaa
acaaaaaaakaabaaaaaaaaaaaabeaaaaadnaknhdoaoaaaaahpcaabaaaabaaaaaa
egaebaaaabaaaaaaagaabaaaaaaaaaaaefaaaaajpcaabaaaacaaaaaaogbkbaaa
abaaaaaaeghobaaaaaaaaaaaaagabaaaaeaaaaaadiaaaaaldcaabaaaacaaaaaa
cgikcaaaaaaaaaaaaiaaaaaaaceaaaaaaaaaaadpaaaaaaecaaaaaaaaaaaaaaaa
dcaaaaalbcaabaaaaaaaaaaadkaabaaaacaaaaaackiacaaaaaaaaaaaaiaaaaaa
akaabaiaebaaaaaaacaaaaaadcaaaaajpcaabaaaabaaaaaaagaabaaaaaaaaaaa
egaobaaaabaaaaaaegbobaaaabaaaaaaefaaaaajpcaabaaaadaaaaaaogakbaaa
abaaaaaaeghobaaaadaaaaaaaagabaaaacaaaaaadcaaaaapdcaabaaaadaaaaaa
hgapbaaaadaaaaaaaceaaaaaaaaaaaeaaaaaaaeaaaaaaaaaaaaaaaaaaceaaaaa
aaaaialpaaaaialpaaaaaaaaaaaaaaaadcaaaaakbcaabaaaaaaaaaaaakaabaia
ebaaaaaaadaaaaaaakaabaaaadaaaaaaabeaaaaaaaaaiadpdcaaaaakbcaabaaa
aaaaaaaabkaabaiaebaaaaaaadaaaaaabkaabaaaadaaaaaaakaabaaaaaaaaaaa
elaaaaafecaabaaaadaaaaaaakaabaaaaaaaaaaabaaaaaahbcaabaaaaaaaaaaa
egacbaaaadaaaaaajgahbaaaaaaaaaaabaaaaaahccaabaaaaaaaaaaaegacbaaa
adaaaaaaegbcbaaaadaaaaaadeaaaaakdcaabaaaaaaaaaaaegaabaaaaaaaaaaa
aceaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaacpaaaaafbcaabaaaaaaaaaaa
akaabaaaaaaaaaaadiaaaaahbcaabaaaaaaaaaaaakaabaaaaaaaaaaabkaabaaa
acaaaaaabjaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaaefaaaaajpcaabaaa
acaaaaaaegaabaaaabaaaaaaeghobaaaacaaaaaaaagabaaaadaaaaaaefaaaaaj
pcaabaaaabaaaaaaegaabaaaabaaaaaaeghobaaaabaaaaaaaagabaaaabaaaaaa
diaaaaahhcaabaaaacaaaaaaegacbaaaacaaaaaapgapbaaaabaaaaaadiaaaaai
hcaabaaaabaaaaaaegacbaaaabaaaaaaegiccaaaaaaaaaaaahaaaaaadiaaaaai
hcaabaaaacaaaaaaegacbaaaacaaaaaafgifcaaaaaaaaaaaaiaaaaaadiaaaaah
ncaabaaaaaaaaaaaagaabaaaaaaaaaaaagajbaaaacaaaaaadiaaaaaincaabaaa
aaaaaaaaagaobaaaaaaaaaaaagijcaaaaaaaaaaaabaaaaaadiaaaaaihcaabaaa
acaaaaaaegacbaaaabaaaaaaegiccaaaaaaaaaaaabaaaaaadiaaaaahhcaabaaa
abaaaaaaegacbaaaabaaaaaaegbcbaaaaeaaaaaadcaaaaajhcaabaaaaaaaaaaa
egacbaaaacaaaaaafgafbaaaaaaaaaaaigadbaaaaaaaaaaaaoaaaaahdcaabaaa
acaaaaaaegbabaaaafaaaaaapgbpbaaaafaaaaaaefaaaaajpcaabaaaacaaaaaa
egaabaaaacaaaaaaeghobaaaaeaaaaaaaagabaaaaaaaaaaaaaaaaaahicaabaaa
aaaaaaaaakaabaaaacaaaaaaakaabaaaacaaaaaadcaaaaajhccabaaaaaaaaaaa
egacbaaaaaaaaaaapgapbaaaaaaaaaaaegacbaaaabaaaaaadgaaaaaficcabaaa
aaaaaaaaabeaaaaaaaaaaaaadoaaaaab"
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
Float 4 [_Parallax]
SetTexture 0 [_ParallaxMap] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 2 [_SpecMap] 2D
SetTexture 3 [_BumpMap] 2D
SetTexture 4 [_ShadowMapTexture] 2D
"agal_ps
c5 0.5 0.42 2.0 -1.0
c6 1.0 0.0 32.0 0.0
[bc]
aeaaaaaaaaaaapacaeaaaaoeaeaaaaaaaeaaaappaeaaaaaa div r0, v4, v4.w
ciaaaaaaagaaapacaaaaaafeacaaaaaaaeaaaaaaafaababb tex r6, r0.xyyy, s4 <2d wrap linear point>
aaaaaaaaaeaaacacaaaaaappaeaaaaaaaaaaaaaaaaaaaaaa mov r4.y, v0.w
aaaaaaaaaeaaabacaaaaaakkaeaaaaaaaaaaaaaaaaaaaaaa mov r4.x, v0.z
aaaaaaaaaaaaacacaaaaaappaeaaaaaaaaaaaaaaaaaaaaaa mov r0.y, v0.w
aaaaaaaaaaaaabacaaaaaakkaeaaaaaaaaaaaaaaaaaaaaaa mov r0.x, v0.z
aaaaaaaaafaaahacacaaaaoeaeaaaaaaaaaaaaaaaaaaaaaa mov r5.xyz, v2
ciaaaaaaaaaaapacaaaaaafeacaaaaaaaaaaaaaaafaababb tex r0, r0.xyyy, s0 <2d wrap linear point>
bcaaaaaaaaaaabacabaaaaoeaeaaaaaaabaaaaoeaeaaaaaa dp3 r0.x, v1, v1
akaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r0.x, r0.x
adaaaaaaadaaahacaaaaaaaaacaaaaaaabaaaaoeaeaaaaaa mul r3.xyz, r0.x, v1
abaaaaaaabaaabacadaaaakkacaaaaaaafaaaaffabaaaaaa add r1.x, r3.z, c5.y
afaaaaaaacaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rcp r2.x, r1.x
aaaaaaaaabaaabacafaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r1.x, c5
adaaaaaaabaaabacaeaaaaoeabaaaaaaabaaaaaaacaaaaaa mul r1.x, c4, r1.x
adaaaaaaadaaaiacaaaaaappacaaaaaaaeaaaaoeabaaaaaa mul r3.w, r0.w, c4
acaaaaaaabaaabacadaaaappacaaaaaaabaaaaaaacaaaaaa sub r1.x, r3.w, r1.x
adaaaaaaacaaadacadaaaafeacaaaaaaacaaaaaaacaaaaaa mul r2.xy, r3.xyyy, r2.x
adaaaaaaadaaadacabaaaaaaacaaaaaaacaaaafeacaaaaaa mul r3.xy, r1.x, r2.xyyy
abaaaaaaadaaadacadaaaafeacaaaaaaaaaaaaoeaeaaaaaa add r3.xy, r3.xyyy, v0
adaaaaaaabaaadacabaaaaaaacaaaaaaacaaaafeacaaaaaa mul r1.xy, r1.x, r2.xyyy
abaaaaaaabaaadacabaaaafeacaaaaaaaeaaaafeacaaaaaa add r1.xy, r1.xyyy, r4.xyyy
adaaaaaaagaaaoacaaaaaaaaacaaaaaaabaaaaoeaeaaaaaa mul r6.yzw, r0.x, v1
abaaaaaaafaaahacagaaaapjacaaaaaaafaaaakeacaaaaaa add r5.xyz, r6.yzww, r5.xyzz
aaaaaaaaaaaaaiacagaaaaffabaaaaaaaaaaaaaaaaaaaaaa mov r0.w, c6.y
ciaaaaaaacaaapacadaaaafeacaaaaaaabaaaaaaafaababb tex r2, r3.xyyy, s1 <2d wrap linear point>
ciaaaaaaabaaapacabaaaafeacaaaaaaadaaaaaaafaababb tex r1, r1.xyyy, s3 <2d wrap linear point>
ciaaaaaaadaaapacadaaaafeacaaaaaaacaaaaaaafaababb tex r3, r3.xyyy, s2 <2d wrap linear point>
aaaaaaaaabaaabacabaaaappacaaaaaaaaaaaaaaaaaaaaaa mov r1.x, r1.w
adaaaaaaaeaaadacabaaaafeacaaaaaaafaaaakkabaaaaaa mul r4.xy, r1.xyyy, c5.z
abaaaaaaaeaaadacaeaaaafeacaaaaaaafaaaappabaaaaaa add r4.xy, r4.xyyy, c5.w
adaaaaaaabaaabacaeaaaaffacaaaaaaaeaaaaffacaaaaaa mul r1.x, r4.y, r4.y
bfaaaaaaaeaaaiacaeaaaaaaacaaaaaaaaaaaaaaaaaaaaaa neg r4.w, r4.x
adaaaaaaaeaaaiacaeaaaappacaaaaaaaeaaaaaaacaaaaaa mul r4.w, r4.w, r4.x
acaaaaaaabaaabacaeaaaappacaaaaaaabaaaaaaacaaaaaa sub r1.x, r4.w, r1.x
abaaaaaaaaaaabacabaaaaaaacaaaaaaagaaaaoeabaaaaaa add r0.x, r1.x, c6
bcaaaaaaabaaabacafaaaakeacaaaaaaafaaaakeacaaaaaa dp3 r1.x, r5.xyzz, r5.xyzz
akaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r0.x, r0.x
afaaaaaaaeaaaeacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rcp r4.z, r0.x
akaaaaaaabaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r1.x, r1.x
adaaaaaaabaaahacabaaaaaaacaaaaaaafaaaakeacaaaaaa mul r1.xyz, r1.x, r5.xyzz
bcaaaaaaabaaabacaeaaaakeacaaaaaaabaaaakeacaaaaaa dp3 r1.x, r4.xyzz, r1.xyzz
aaaaaaaaaaaaabacacaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0.x, c2
adaaaaaaaaaaabacagaaaakkabaaaaaaaaaaaaaaacaaaaaa mul r0.x, c6.z, r0.x
ahaaaaaaabaaabacabaaaaaaacaaaaaaagaaaaffabaaaaaa max r1.x, r1.x, c6.y
alaaaaaaafaaapacabaaaaaaacaaaaaaaaaaaaaaacaaaaaa pow r5, r1.x, r0.x
adaaaaaaaaaaahacacaaaappacaaaaaaadaaaakeacaaaaaa mul r0.xyz, r2.w, r3.xyzz
adaaaaaaacaaahacacaaaakeacaaaaaaabaaaaoeabaaaaaa mul r2.xyz, r2.xyzz, c1
adaaaaaaabaaahacaaaaaakeacaaaaaaadaaaaaaabaaaaaa mul r1.xyz, r0.xyzz, c3.x
aaaaaaaaaaaaabacafaaaaaaacaaaaaaaaaaaaaaaaaaaaaa mov r0.x, r5.x
adaaaaaaaaaaahacaaaaaaaaacaaaaaaabaaaakeacaaaaaa mul r0.xyz, r0.x, r1.xyzz
adaaaaaaabaaahacaaaaaakeacaaaaaaaaaaaaoeabaaaaaa mul r1.xyz, r0.xyzz, c0
bcaaaaaaaaaaabacaeaaaakeacaaaaaaacaaaaoeaeaaaaaa dp3 r0.x, r4.xyzz, v2
ahaaaaaaaaaaabacaaaaaaaaacaaaaaaagaaaaffabaaaaaa max r0.x, r0.x, c6.y
adaaaaaaadaaahacacaaaakeacaaaaaaaaaaaaoeabaaaaaa mul r3.xyz, r2.xyzz, c0
adaaaaaaaaaaahacadaaaakeacaaaaaaaaaaaaaaacaaaaaa mul r0.xyz, r3.xyzz, r0.x
abaaaaaaaaaaahacaaaaaakeacaaaaaaabaaaakeacaaaaaa add r0.xyz, r0.xyzz, r1.xyzz
adaaaaaaaaaaahacagaaaaaaacaaaaaaaaaaaakeacaaaaaa mul r0.xyz, r6.x, r0.xyzz
adaaaaaaabaaahacacaaaakeacaaaaaaadaaaaoeaeaaaaaa mul r1.xyz, r2.xyzz, v3
adaaaaaaaaaaahacaaaaaakeacaaaaaaafaaaakkabaaaaaa mul r0.xyz, r0.xyzz, c5.z
abaaaaaaaaaaahacaaaaaakeacaaaaaaabaaaakeacaaaaaa add r0.xyz, r0.xyzz, r1.xyzz
aaaaaaaaaaaaapadaaaaaaoeacaaaaaaaaaaaaaaaaaaaaaa mov o0, r0
"
}

SubProgram "opengl " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Vector 0 [_Color]
Float 1 [_Parallax]
SetTexture 0 [_ParallaxMap] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 3 [_ShadowMapTexture] 2D
SetTexture 4 [unity_Lightmap] 2D
"!!ARBfp1.0
OPTION ARB_precision_hint_fastest;
# 24 ALU, 4 TEX
PARAM c[4] = { program.local[0..1],
		{ 0.5, 0, 0.41999999, 8 },
		{ 2 } };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
TEX R0.w, fragment.texcoord[0].zwzw, texture[0], 2D;
DP3 R0.x, fragment.texcoord[1], fragment.texcoord[1];
RSQ R0.x, R0.x;
MUL R1.xyz, R0.x, fragment.texcoord[1];
ADD R0.y, R1.z, c[2].z;
RCP R0.y, R0.y;
MOV R0.x, c[1];
MUL R0.x, R0, c[2];
MUL R1.xy, R1, R0.y;
MAD R0.x, R0.w, c[1], -R0;
MAD R0.xy, R0.x, R1, fragment.texcoord[0];
MOV result.color.w, c[2].y;
TEX R2.xyz, R0, texture[1], 2D;
TEX R1, fragment.texcoord[2], texture[4], 2D;
TXP R0.x, fragment.texcoord[3], texture[3], 2D;
MUL R0.yzw, R1.w, R1.xxyz;
MUL R1.xyz, R1, R0.x;
MUL R0.yzw, R0, c[2].w;
MUL R1.xyz, R1, c[3].x;
MUL R3.xyz, R0.yzww, R0.x;
MIN R0.xyz, R0.yzww, R1;
MAX R0.xyz, R0, R3;
MUL R1.xyz, R2, c[0];
MUL result.color.xyz, R1, R0;
END
# 24 instructions, 4 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Vector 0 [_Color]
Float 1 [_Parallax]
SetTexture 0 [_ParallaxMap] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 3 [_ShadowMapTexture] 2D
SetTexture 4 [unity_Lightmap] 2D
"ps_2_0
; 23 ALU, 4 TEX
dcl_2d s0
dcl_2d s1
dcl_2d s3
dcl_2d s4
def c2, 0.50000000, 0.41999999, 8.00000000, 2.00000000
def c3, 0.00000000, 0, 0, 0
dcl t0
dcl t1.xyz
dcl t2.xy
dcl t3
texldp r3, t3, s3
mov r0.y, t0.w
mov r0.x, t0.z
texld r0, r0, s0
dp3_pp r0.x, t1, t1
rsq_pp r0.x, r0.x
mul_pp r2.xyz, r0.x, t1
add r0.x, r2.z, c2.y
rcp r1.x, r0.x
mov_pp r0.x, c2
mul_pp r0.x, c1, r0
mul r1.xy, r2, r1.x
mad_pp r0.x, r0.w, c1, -r0
mad r0.xy, r0.x, r1, t0
texld r1, r0, s1
texld r0, t2, s4
mul_pp r2.xyz, r0, r3.x
mul_pp r0.xyz, r0.w, r0
mul_pp r0.xyz, r0, c2.z
mul_pp r2.xyz, r2, c2.w
min_pp r2.xyz, r0, r2
mul_pp r0.xyz, r0, r3.x
max_pp r0.xyz, r2, r0
mul_pp r1.xyz, r1, c0
mul_pp r0.xyz, r1, r0
mov_pp r0.w, c3.x
mov_pp oC0, r0
"
}

SubProgram "xbox360 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Vector 0 [_Color]
Float 1 [_Parallax]
SetTexture 0 [_ShadowMapTexture] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 2 [_ParallaxMap] 2D
SetTexture 3 [unity_Lightmap] 2D
// Shader Timing Estimate, in Cycles/64 pixel vector:
// ALU: 20.00 (15 instructions), vertex: 0, texture: 16,
//   sequencer: 10, interpolator: 16;    5 GPRs, 36 threads,
// Performance (if enough threads): ~20 cycles per vector
// * Texture cycle estimates are assuming an 8bit/component texture with no
//     aniso or trilinear filtering.

"ps_360
backbbaaaaaaabkiaaaaabfeaaaaaaaaaaaaaaceaaaaabfaaaaaabhiaaaaaaaa
aaaaaaaaaaaaabciaaaaaabmaaaaabbjppppadaaaaaaaaagaaaaaabmaaaaaaaa
aaaaabbcaaaaaajeaaacaaaaaaabaaaaaaaaaajmaaaaaaaaaaaaaakmaaadaaab
aaabaaaaaaaaaaliaaaaaaaaaaaaaamiaaacaaabaaabaaaaaaaaaaneaaaaaaaa
aaaaaaoeaaadaaacaaabaaaaaaaaaaliaaaaaaaaaaaaaapbaaadaaaaaaabaaaa
aaaaaaliaaaaaaaaaaaaabadaaadaaadaaabaaaaaaaaaaliaaaaaaaafpedgpgm
gphcaaklaaabaaadaaabaaaeaaabaaaaaaaaaaaafpengbgjgofegfhiaaklklkl
aaaeaaamaaabaaabaaabaaaaaaaaaaaafpfagbhcgbgmgmgbhiaaklklaaaaaaad
aaabaaabaaabaaaaaaaaaaaafpfagbhcgbgmgmgbhiengbhaaafpfdgigbgegphh
engbhafegfhihehfhcgfaahfgogjhehjfpemgjghgihegngbhaaahahdfpddfpda
aadccodacodcdadddfddcodaaaklklklaaaaaaaaaaaaaaabaaaaaaaaaaaaaaaa
aaaaaabeabpmaabaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaeaaaaaabbe
baaaaeaaaaaaaaaeaaaaaaaaaaaadeieaaapaaapaaaaaaabaaaapafaaaaahbfb
aaaadcfcaaaapdfdaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaebaaaaaadpaaaaaa
donhakdnaaaaaaaaaaajgaadgaajbcaabcaaafeaaaaaaaaagaapmeaabcaaaaaa
aaaababfaaaaccaaaaaaaaaadicacaabbpbpphppaaaaeaaamiaeaaacaaloloaa
paababaafieiacabaagmlbmgcbabppicmiaiaaababblgmblklacababmiahaaae
aamgmaaaobacabaaembeababaamgmgblkaaeppademedababaagmlamgobabadab
miamaaacaakmmgaaobaeabaamiadaaaaaabkbllaolacabaababiaaabbpbppoec
aaaaeaaabadicaebbpbppgiiaaaaeaaabaaiaacbbpbppbppaaaaeaaaaabiabab
aablgmblkbacppaamiahaaabaagmmaaaobabacaamiaoaaacaablpmaaobabacaa
kibhacadaabfblebmbacaaaakichacabaabfmaicmdacabaakiehacabaamamama
mcadabaamiahmaaaaamamaaaobacabaaaaaaaaaaaaaaaaaaaaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Vector 0 [_Color]
Float 1 [_Parallax]
SetTexture 0 [_ParallaxMap] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 3 [_ShadowMapTexture] 2D
SetTexture 4 [unity_Lightmap] 2D
"sce_fp_rsx // 26 instructions using 3 registers
[Configuration]
24
ffffffff0003c020000ffff4000000000000840003000000
[Offsets]
2
_Color 1 0
00000150
_Parallax 2 0
000000c000000040
[Microcode]
416
de041708c8011c9dc8000001c8003fe1ae843940c8011c9dc8000029c800bfe1
900017005c011c9dc8000001c8003fe110820240c8001c9d00020000c8000001
000000000000000000000000000000000e820240fe081c9dc8083001c8000001
e2001806c8011c9dc8000001c8003fe10e880240c8081c9d00001000c8000001
1004030055081c9d00020000c80000010a3d3ed7000000000000000000000000
18043a0081081c9cfe080001c80000011088014000021c9cc8000001c8000001
000000000000000000000000000000000e860240c9041c9d00000000c8000001
86000100c8011c9dc8000001c8003fe110820440c9101c9d00020000c9040001
0000bf000000000000000000000000000e840840c9041c9dc9100001c8000001
06000400ff041c9d5c080001c80000010e001702c8001c9dc8000001c8000001
0e800240c8001c9dc8020001c800000100000000000000000000000000000000
10800140c8021c9dc8000001c800000100000000000000000000000000000000
0e820940c9081c9dc90c0001c80000010e810240c9001c9dc9040001c8000001
"
}

SubProgram "d3d11 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
ConstBuffer "$Globals" 192 // 140 used size, 11 vars
Vector 112 [_Color] 4
Float 136 [_Parallax]
BindCB "$Globals" 0
SetTexture 0 [_ParallaxMap] 2D 2
SetTexture 1 [_MainTex] 2D 1
SetTexture 2 [_ShadowMapTexture] 2D 0
SetTexture 3 [unity_Lightmap] 2D 3
// 24 instructions, 3 temp regs, 0 temp arrays:
// ALU 15 float, 0 int, 0 uint
// TEX 4 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0
eefiecedbdfaanacohdnnjhbpgkjlkondfngbhljabaaaaaajeaeaaaaadaaaaaa
cmaaaaaammaaaaaaaaabaaaaejfdeheojiaaaaaaafaaaaaaaiaaaaaaiaaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaaimaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaadadaaaaimaaaaaa
adaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapalaaaafdfgfpfaepfdejfeejepeoaa
feeffiedepepfceeaaklklklepfdeheocmaaaaaaabaaaaaaaiaaaaaacaaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaafdfgfpfegbhcghgfheaaklkl
fdeieefcimadaaaaeaaaaaaaodaaaaaafjaaaaaeegiocaaaaaaaaaaaajaaaaaa
fkaaaaadaagabaaaaaaaaaaafkaaaaadaagabaaaabaaaaaafkaaaaadaagabaaa
acaaaaaafkaaaaadaagabaaaadaaaaaafibiaaaeaahabaaaaaaaaaaaffffaaaa
fibiaaaeaahabaaaabaaaaaaffffaaaafibiaaaeaahabaaaacaaaaaaffffaaaa
fibiaaaeaahabaaaadaaaaaaffffaaaagcbaaaadpcbabaaaabaaaaaagcbaaaad
hcbabaaaacaaaaaagcbaaaaddcbabaaaadaaaaaagcbaaaadlcbabaaaaeaaaaaa
gfaaaaadpccabaaaaaaaaaaagiaaaaacadaaaaaabaaaaaahbcaabaaaaaaaaaaa
egbcbaaaacaaaaaaegbcbaaaacaaaaaaeeaaaaafbcaabaaaaaaaaaaaakaabaaa
aaaaaaaadiaaaaahgcaabaaaaaaaaaaaagaabaaaaaaaaaaaagbbbaaaacaaaaaa
dcaaaaajbcaabaaaaaaaaaaackbabaaaacaaaaaaakaabaaaaaaaaaaaabeaaaaa
dnaknhdoaoaaaaahdcaabaaaaaaaaaaajgafbaaaaaaaaaaaagaabaaaaaaaaaaa
efaaaaajpcaabaaaabaaaaaaogbkbaaaabaaaaaaeghobaaaaaaaaaaaaagabaaa
acaaaaaadiaaaaaiecaabaaaaaaaaaaackiacaaaaaaaaaaaaiaaaaaaabeaaaaa
aaaaaadpdcaaaaalecaabaaaaaaaaaaadkaabaaaabaaaaaackiacaaaaaaaaaaa
aiaaaaaackaabaiaebaaaaaaaaaaaaaadcaaaaajdcaabaaaaaaaaaaakgakbaaa
aaaaaaaaegaabaaaaaaaaaaaegbabaaaabaaaaaaefaaaaajpcaabaaaaaaaaaaa
egaabaaaaaaaaaaaeghobaaaabaaaaaaaagabaaaabaaaaaadiaaaaaihcaabaaa
aaaaaaaaegacbaaaaaaaaaaaegiccaaaaaaaaaaaahaaaaaaaoaaaaahdcaabaaa
abaaaaaaegbabaaaaeaaaaaapgbpbaaaaeaaaaaaefaaaaajpcaabaaaabaaaaaa
egaabaaaabaaaaaaeghobaaaacaaaaaaaagabaaaaaaaaaaaaaaaaaahicaabaaa
aaaaaaaaakaabaaaabaaaaaaakaabaaaabaaaaaaefaaaaajpcaabaaaacaaaaaa
egbabaaaadaaaaaaeghobaaaadaaaaaaaagabaaaadaaaaaadiaaaaahocaabaaa
abaaaaaapgapbaaaaaaaaaaaagajbaaaacaaaaaadiaaaaahicaabaaaaaaaaaaa
dkaabaaaacaaaaaaabeaaaaaaaaaaaebdiaaaaahhcaabaaaacaaaaaaegacbaaa
acaaaaaapgapbaaaaaaaaaaaddaaaaahocaabaaaabaaaaaafgaobaaaabaaaaaa
agajbaaaacaaaaaadiaaaaahhcaabaaaacaaaaaaagaabaaaabaaaaaaegacbaaa
acaaaaaadeaaaaahhcaabaaaabaaaaaajgahbaaaabaaaaaaegacbaaaacaaaaaa
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
Float 1 [_Parallax]
SetTexture 0 [_ParallaxMap] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 3 [_ShadowMapTexture] 2D
SetTexture 4 [unity_Lightmap] 2D
"agal_ps
c2 0.5 0.42 8.0 2.0
c3 0.0 0.0 0.0 0.0
[bc]
aeaaaaaaaaaaapacadaaaaoeaeaaaaaaadaaaappaeaaaaaa div r0, v3, v3.w
ciaaaaaaadaaapacaaaaaafeacaaaaaaadaaaaaaafaababb tex r3, r0.xyyy, s3 <2d wrap linear point>
aaaaaaaaaaaaacacaaaaaappaeaaaaaaaaaaaaaaaaaaaaaa mov r0.y, v0.w
aaaaaaaaaaaaabacaaaaaakkaeaaaaaaaaaaaaaaaaaaaaaa mov r0.x, v0.z
ciaaaaaaaaaaapacaaaaaafeacaaaaaaaaaaaaaaafaababb tex r0, r0.xyyy, s0 <2d wrap linear point>
bcaaaaaaaaaaabacabaaaaoeaeaaaaaaabaaaaoeaeaaaaaa dp3 r0.x, v1, v1
akaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r0.x, r0.x
adaaaaaaacaaahacaaaaaaaaacaaaaaaabaaaaoeaeaaaaaa mul r2.xyz, r0.x, v1
abaaaaaaaaaaabacacaaaakkacaaaaaaacaaaaffabaaaaaa add r0.x, r2.z, c2.y
afaaaaaaabaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rcp r1.x, r0.x
aaaaaaaaaaaaabacacaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0.x, c2
adaaaaaaaaaaabacabaaaaoeabaaaaaaaaaaaaaaacaaaaaa mul r0.x, c1, r0.x
adaaaaaaabaaadacacaaaafeacaaaaaaabaaaaaaacaaaaaa mul r1.xy, r2.xyyy, r1.x
adaaaaaaabaaaiacaaaaaappacaaaaaaabaaaaoeabaaaaaa mul r1.w, r0.w, c1
acaaaaaaaaaaabacabaaaappacaaaaaaaaaaaaaaacaaaaaa sub r0.x, r1.w, r0.x
adaaaaaaaaaaadacaaaaaaaaacaaaaaaabaaaafeacaaaaaa mul r0.xy, r0.x, r1.xyyy
abaaaaaaaaaaadacaaaaaafeacaaaaaaaaaaaaoeaeaaaaaa add r0.xy, r0.xyyy, v0
ciaaaaaaabaaapacaaaaaafeacaaaaaaabaaaaaaafaababb tex r1, r0.xyyy, s1 <2d wrap linear point>
ciaaaaaaaaaaapacacaaaaoeaeaaaaaaaeaaaaaaafaababb tex r0, v2, s4 <2d wrap linear point>
adaaaaaaacaaahacaaaaaakeacaaaaaaadaaaaaaacaaaaaa mul r2.xyz, r0.xyzz, r3.x
adaaaaaaaaaaahacaaaaaappacaaaaaaaaaaaakeacaaaaaa mul r0.xyz, r0.w, r0.xyzz
adaaaaaaaaaaahacaaaaaakeacaaaaaaacaaaakkabaaaaaa mul r0.xyz, r0.xyzz, c2.z
adaaaaaaacaaahacacaaaakeacaaaaaaacaaaappabaaaaaa mul r2.xyz, r2.xyzz, c2.w
agaaaaaaacaaahacaaaaaakeacaaaaaaacaaaakeacaaaaaa min r2.xyz, r0.xyzz, r2.xyzz
adaaaaaaaaaaahacaaaaaakeacaaaaaaadaaaaaaacaaaaaa mul r0.xyz, r0.xyzz, r3.x
ahaaaaaaaaaaahacacaaaakeacaaaaaaaaaaaakeacaaaaaa max r0.xyz, r2.xyzz, r0.xyzz
adaaaaaaabaaahacabaaaakeacaaaaaaaaaaaaoeabaaaaaa mul r1.xyz, r1.xyzz, c0
adaaaaaaaaaaahacabaaaakeacaaaaaaaaaaaakeacaaaaaa mul r0.xyz, r1.xyzz, r0.xyzz
aaaaaaaaaaaaaiacadaaaaaaabaaaaaaaaaaaaaaaaaaaaaa mov r0.w, c3.x
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
//   opengl - ALU: 26 to 35
//   d3d9 - ALU: 29 to 38
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
Vector 21 [_BumpMap_ST]
"!!ARBvp1.0
# 34 ALU
PARAM c[22] = { { 1 },
		state.matrix.mvp,
		program.local[5..21] };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
MOV R0.xyz, vertex.attrib[14];
MUL R2.xyz, vertex.normal.zxyw, R0.yzxw;
MAD R0.xyz, vertex.normal.yzxw, R0.zxyw, -R2;
MOV R1, c[18];
MOV R0.w, c[0].x;
DP4 R2.z, R1, c[11];
DP4 R2.x, R1, c[9];
DP4 R2.y, R1, c[10];
MAD R2.xyz, R2, c[19].w, -vertex.position;
MUL R1.xyz, R0, vertex.attrib[14].w;
MOV R0.xyz, c[17];
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
MAD result.texcoord[0].zw, vertex.texcoord[0].xyxy, c[21].xyxy, c[21];
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
Vector 20 [_BumpMap_ST]
"vs_2_0
; 37 ALU
def c21, 1.00000000, 0, 0, 0
dcl_position0 v0
dcl_tangent0 v1
dcl_normal0 v2
dcl_texcoord0 v3
mov r1.xyz, v1
mov r0, c10
dp4 r3.z, c17, r0
mov r0, c9
dp4 r3.y, c17, r0
mul r2.xyz, v2.zxyw, r1.yzxw
mov r1.xyz, v1
mad r2.xyz, v2.yzxw, r1.zxyw, -r2
mov r1, c8
dp4 r3.x, c17, r1
mad r0.xyz, r3, c18.w, -v0
mul r2.xyz, r2, v1.w
mov r1.xyz, c16
mov r1.w, c21.x
dp3 oT2.y, r2, r0
dp3 oT2.z, v2, r0
dp3 oT2.x, v1, r0
dp4 r0.w, v0, c7
dp4 r0.z, v0, c6
dp4 r0.x, v0, c4
dp4 r0.y, v0, c5
dp4 r3.z, r1, c10
dp4 r3.x, r1, c8
dp4 r3.y, r1, c9
mad r1.xyz, r3, c18.w, -v0
dp3 oT1.y, r1, r2
dp3 oT1.z, v2, r1
dp3 oT1.x, r1, v1
dp4 oT3.z, r0, c14
dp4 oT3.y, r0, c13
dp4 oT3.x, r0, c12
mad oT0.zw, v3.xyxy, c20.xyxy, c20
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
Vector 20 [_BumpMap_ST]
Matrix 15 [_LightMatrix0] 4
Vector 19 [_MainTex_ST]
Matrix 6 [_Object2World] 4
Matrix 10 [_World2Object] 4
Vector 0 [_WorldSpaceCameraPos]
Vector 1 [_WorldSpaceLightPos0]
Matrix 2 [glstate_matrix_mvp] 4
Vector 14 [unity_Scale]
// Shader Timing Estimate, in Cycles/64 vertex vector:
// ALU: 42.67 (32 instructions), vertex: 32, texture: 0,
//   sequencer: 18,  8 GPRs, 24 threads,
// Performance (if enough threads): ~42 cycles per vector
// * Vertex cycle estimates are assuming 3 vfetch_minis for every vfetch_full,
//     with <= 32 bytes per vfetch_full group.

"vs_360
backbbabaaaaacdiaaaaabpiaaaaaaaaaaaaaaceaaaaaaaaaaaaabmmaaaaaaaa
aaaaaaaaaaaaabkeaaaaaabmaaaaabjhpppoadaaaaaaaaajaaaaaabmaaaaaaaa
aaaaabjaaaaaaanaaaacaabeaaabaaaaaaaaaanmaaaaaaaaaaaaaaomaaacaaap
aaaeaaaaaaaaaapmaaaaaaaaaaaaabamaaacaabdaaabaaaaaaaaaanmaaaaaaaa
aaaaabbiaaacaaagaaaeaaaaaaaaaapmaaaaaaaaaaaaabcgaaacaaakaaaeaaaa
aaaaaapmaaaaaaaaaaaaabdeaaacaaaaaaabaaaaaaaaabemaaaaaaaaaaaaabfm
aaacaaabaaabaaaaaaaaaanmaaaaaaaaaaaaabhbaaacaaacaaaeaaaaaaaaaapm
aaaaaaaaaaaaabieaaacaaaoaaabaaaaaaaaaanmaaaaaaaafpechfgnhaengbha
fpfdfeaaaaabaaadaaabaaaeaaabaaaaaaaaaaaafpemgjghgiheengbhehcgjhi
daaaklklaaadaaadaaaeaaaeaaabaaaaaaaaaaaafpengbgjgofegfhifpfdfeaa
fpepgcgkgfgdhedcfhgphcgmgeaafpfhgphcgmgedcepgcgkgfgdheaafpfhgphc
gmgefdhagbgdgfedgbgngfhcgbfagphdaaklklklaaabaaadaaabaaadaaabaaaa
aaaaaaaafpfhgphcgmgefdhagbgdgfemgjghgihefagphddaaaghgmhdhegbhegf
fpgngbhehcgjhifpgnhghaaahfgogjhehjfpfdgdgbgmgfaahghdfpddfpdaaadc
codacodcdadddfddcodaaaklaaaaaaaaaaaaabpiaadbaaahaaaaaaaaaaaaaaaa
aaaadeieaaaaaaabaaaaaaaeaaaaaaajaaaaacjaaabaaaafaaaagaagaaaadaah
aacafaaiaaaapafaaaachbfbaaafhcfcaaaihdfdaaaaaacdaaaabaceaaaaaabn
aaaaaaboaaaababpaaaaaacaaaaaaacbaaaabaccaaaabacipaffeaafaaaabcaa
mcaaaaaaaaaaeaajaaaabcaameaaaaaaaaaagaangabdbcaabcaaaaaaaaaagabj
gabpbcaabcaaaaaaaaaaeacfaaaaccaaaaaaaaaaafpidaaaaaaaagiiaaaaaaaa
afpifaaaaaaaagiiaaaaaaaaafpicaaaaaaaaoiiaaaaaaaaafpibaaaaaaaapmi
aaaaaaaamiapaaaaaabliiaakbadafaamiapaaaaaamgiiaakladaeaamiapaaaa
aalbdejekladadaamiapiadoaagmaadekladacaamiahaaaaaaleblaacbanabaa
miahaaagaamamgmaalamaaanmiahaaaeaalogfaaobacafaamiahaaagaalelble
clalaaagmiahaaahaamamgleclamabaamiapaaaaaabliiaakbadajaamiapaaaa
aamgiiaakladaiaamiahaaahaalelbleclalabahmiahaaagaamagmleclakaaag
miahaaaeabgflomaolacafaemiahaaaeaamablaaobaeafaamiahaaagabmablma
klagaoadmiahaaahaamagmleclakabahmiapaaaaaalbdejekladahaamiapaaaa
aagmejhkkladagaamiahaaadabmablmaklahaoadmiabiaabaaloloaapaagafaa
miaciaabaaloloaapaaeagaamiaeiaabaaloloaapaagacaamiabiaacaaloloaa
paadafaamiaciaacaaloloaapaaeadaamiaeiaacaaloloaapaadacaamiadiaaa
aalalabkilabbdbdmiamiaaaaakmkmagilabbebemiahaaabaalbleaakbaabcaa
miahaaabaamgmaleklaabbabmiahaaaaaagmleleklaabaabmiahiaadaablmale
klaaapaaaaaaaaaaaaaaaaaaaaaaaaaa"
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
Vector 463 [_BumpMap_ST]
"sce_vp_rsx // 33 instructions using 5 registers
[Configuration]
8
0000002141050500
[Microcode]
528
00009c6c005d200d8186c0836041fffc00019c6c00400e0c0106c0836041dffc
00011c6c005d300c0186c0836041dffc401f9c6c011cf800810040d560607f9c
401f9c6c011d0808010400d740619f9c401f9c6c01d0300d8106c0c360403f80
401f9c6c01d0200d8106c0c360405f80401f9c6c01d0100d8106c0c360409f80
401f9c6c01d0000d8106c0c360411f8000001c6c01d0700d8106c0c360403ffc
00001c6c01d0600d8106c0c360405ffc00001c6c01d0500d8106c0c360409ffc
00001c6c01d0400d8106c0c360411ffc00021c6c01d0a00d8286c0c360405ffc
00021c6c01d0900d8286c0c360409ffc00021c6c01d0800d8286c0c360411ffc
00009c6c0190a00c0486c0c360405ffc00009c6c0190900c0486c0c360409ffc
00009c6c0190800c0486c0c360411ffc00011c6c00800243011843436041dffc
00011c6c01000230812183630121dffc401f9c6c01d0e00d8086c0c360405fa8
401f9c6c01d0d00d8086c0c360409fa8401f9c6c01d0c00d8086c0c360411fa8
00001c6c011d100c08bfc0e30041dffc00009c6c011d100c02bfc0e30041dffc
401f9c6c0140020c0106004360405fa4401f9c6c01400e0c0106004360411fa4
00011c6c00800e0c04bfc0836041dffc401f9c6c0140020c0106014360405fa0
401f9c6c01400e0c0286008360411fa0401f9c6c0140000c0486004360409fa4
401f9c6c0140000c0286024360409fa1
"
}

SubProgram "d3d11 " {
Keywords { "POINT" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 176 // 176 used size, 10 vars
Matrix 48 [_LightMatrix0] 4
Vector 144 [_MainTex_ST] 4
Vector 160 [_BumpMap_ST] 4
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
// 34 instructions, 2 temp regs, 0 temp arrays:
// ALU 14 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0
eefiecedipgdcampblaklmipimpabbddhdnfeapfabaaaaaaceahaaaaadaaaaaa
cmaaaaaapeaaaaaajeabaaaaejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaa
abaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaljaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfc
enebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheojiaaaaaaafaaaaaa
aiaaaaaaiaaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaabaaaaaaapaaaaaaimaaaaaaabaaaaaaaaaaaaaa
adaaaaaaacaaaaaaahaiaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaa
ahaiaaaaimaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaahaiaaaafdfgfpfa
epfdejfeejepeoaafeeffiedepepfceeaaklklklfdeieefciiafaaaaeaaaabaa
gcabaaaafjaaaaaeegiocaaaaaaaaaaaalaaaaaafjaaaaaeegiocaaaabaaaaaa
afaaaaaafjaaaaaeegiocaaaacaaaaaaabaaaaaafjaaaaaeegiocaaaadaaaaaa
bfaaaaaafpaaaaadpcbabaaaaaaaaaaafpaaaaadpcbabaaaabaaaaaafpaaaaad
hcbabaaaacaaaaaafpaaaaaddcbabaaaadaaaaaaghaaaaaepccabaaaaaaaaaaa
abaaaaaagfaaaaadpccabaaaabaaaaaagfaaaaadhccabaaaacaaaaaagfaaaaad
hccabaaaadaaaaaagfaaaaadhccabaaaaeaaaaaagiaaaaacacaaaaaadiaaaaai
pcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaadaaaaaaabaaaaaadcaaaaak
pcaabaaaaaaaaaaaegiocaaaadaaaaaaaaaaaaaaagbabaaaaaaaaaaaegaobaaa
aaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaacaaaaaakgbkbaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaakpccabaaaaaaaaaaaegiocaaaadaaaaaa
adaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaaldccabaaaabaaaaaa
egbabaaaadaaaaaaegiacaaaaaaaaaaaajaaaaaaogikcaaaaaaaaaaaajaaaaaa
dcaaaaalmccabaaaabaaaaaaagbebaaaadaaaaaaagiecaaaaaaaaaaaakaaaaaa
kgiocaaaaaaaaaaaakaaaaaadiaaaaahhcaabaaaaaaaaaaajgbebaaaabaaaaaa
cgbjbaaaacaaaaaadcaaaaakhcaabaaaaaaaaaaajgbebaaaacaaaaaacgbjbaaa
abaaaaaaegacbaiaebaaaaaaaaaaaaaadiaaaaahhcaabaaaaaaaaaaaegacbaaa
aaaaaaaapgbpbaaaabaaaaaadiaaaaajhcaabaaaabaaaaaafgifcaaaabaaaaaa
aeaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaa
adaaaaaabaaaaaaaagiacaaaabaaaaaaaeaaaaaaegacbaaaabaaaaaadcaaaaal
hcaabaaaabaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaaabaaaaaaaeaaaaaa
egacbaaaabaaaaaaaaaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaaegiccaaa
adaaaaaabdaaaaaadcaaaaalhcaabaaaabaaaaaaegacbaaaabaaaaaapgipcaaa
adaaaaaabeaaaaaaegbcbaiaebaaaaaaaaaaaaaabaaaaaahcccabaaaacaaaaaa
egacbaaaaaaaaaaaegacbaaaabaaaaaabaaaaaahbccabaaaacaaaaaaegbcbaaa
abaaaaaaegacbaaaabaaaaaabaaaaaaheccabaaaacaaaaaaegbcbaaaacaaaaaa
egacbaaaabaaaaaadiaaaaajhcaabaaaabaaaaaafgifcaaaacaaaaaaaaaaaaaa
egiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaa
baaaaaaaagiacaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaa
abaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaaacaaaaaaaaaaaaaaegacbaaa
abaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabdaaaaaapgipcaaa
acaaaaaaaaaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegacbaaa
abaaaaaapgipcaaaadaaaaaabeaaaaaaegbcbaiaebaaaaaaaaaaaaaabaaaaaah
cccabaaaadaaaaaaegacbaaaaaaaaaaaegacbaaaabaaaaaabaaaaaahbccabaaa
adaaaaaaegbcbaaaabaaaaaaegacbaaaabaaaaaabaaaaaaheccabaaaadaaaaaa
egbcbaaaacaaaaaaegacbaaaabaaaaaadiaaaaaipcaabaaaaaaaaaaafgbfbaaa
aaaaaaaaegiocaaaadaaaaaaanaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaa
adaaaaaaamaaaaaaagbabaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaa
aaaaaaaaegiocaaaadaaaaaaaoaaaaaakgbkbaaaaaaaaaaaegaobaaaaaaaaaaa
dcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaapaaaaaapgbpbaaaaaaaaaaa
egaobaaaaaaaaaaadiaaaaaihcaabaaaabaaaaaafgafbaaaaaaaaaaaegiccaaa
aaaaaaaaaeaaaaaadcaaaaakhcaabaaaabaaaaaaegiccaaaaaaaaaaaadaaaaaa
agaabaaaaaaaaaaaegacbaaaabaaaaaadcaaaaakhcaabaaaaaaaaaaaegiccaaa
aaaaaaaaafaaaaaakgakbaaaaaaaaaaaegacbaaaabaaaaaadcaaaaakhccabaaa
aeaaaaaaegiccaaaaaaaaaaaagaaaaaapgapbaaaaaaaaaaaegacbaaaaaaaaaaa
doaaaaab"
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
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 _BumpMap_ST;
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
  highp vec4 tmpvar_3;
  mediump vec3 tmpvar_4;
  tmpvar_3.xy = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  tmpvar_3.zw = ((_glesMultiTexCoord0.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
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
  tmpvar_4 = tmpvar_8;
  highp vec4 tmpvar_9;
  tmpvar_9.w = 1.0;
  tmpvar_9.xyz = _WorldSpaceCameraPos;
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = tmpvar_3;
  xlv_TEXCOORD1 = (tmpvar_7 * (((_World2Object * tmpvar_9).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = tmpvar_4;
  xlv_TEXCOORD3 = (_LightMatrix0 * (_Object2World * _glesVertex)).xyz;
}



#endif
#ifdef FRAGMENT

varying highp vec3 xlv_TEXCOORD3;
varying mediump vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp float _Parallax;
uniform mediump float _Gloss;
uniform mediump float _Shininess;
uniform lowp vec4 _Color;
uniform sampler2D _ParallaxMap;
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
  mediump vec4 spec_5;
  mediump float h_6;
  lowp float tmpvar_7;
  tmpvar_7 = texture2D (_ParallaxMap, xlv_TEXCOORD0.zw).w;
  h_6 = tmpvar_7;
  highp vec2 tmpvar_8;
  mediump float height_9;
  height_9 = _Parallax;
  mediump vec3 viewDir_10;
  viewDir_10 = xlv_TEXCOORD1;
  highp vec3 v_11;
  mediump float tmpvar_12;
  tmpvar_12 = ((h_6 * height_9) - (height_9 / 2.0));
  mediump vec3 tmpvar_13;
  tmpvar_13 = normalize(viewDir_10);
  v_11 = tmpvar_13;
  v_11.z = (v_11.z + 0.42);
  tmpvar_8 = (tmpvar_12 * (v_11.xy / v_11.z));
  highp vec2 tmpvar_14;
  tmpvar_14 = (xlv_TEXCOORD0.xy + tmpvar_8);
  highp vec2 tmpvar_15;
  tmpvar_15 = (xlv_TEXCOORD0.zw + tmpvar_8);
  lowp vec4 tmpvar_16;
  tmpvar_16 = texture2D (_MainTex, tmpvar_14);
  lowp vec3 tmpvar_17;
  tmpvar_17 = (tmpvar_16.xyz * _Color.xyz);
  tmpvar_3 = tmpvar_17;
  lowp vec4 tmpvar_18;
  tmpvar_18 = texture2D (_SpecMap, tmpvar_14);
  spec_5 = tmpvar_18;
  mediump vec3 tmpvar_19;
  tmpvar_19 = ((tmpvar_16.w * spec_5.xyz) * _Gloss);
  lowp vec3 tmpvar_20;
  tmpvar_20 = ((texture2D (_BumpMap, tmpvar_15).xyz * 2.0) - 1.0);
  tmpvar_4 = tmpvar_20;
  mediump vec3 tmpvar_21;
  tmpvar_21 = normalize(xlv_TEXCOORD2);
  lightDir_2 = tmpvar_21;
  highp vec3 tmpvar_22;
  tmpvar_22 = normalize(xlv_TEXCOORD1);
  highp float tmpvar_23;
  tmpvar_23 = dot (xlv_TEXCOORD3, xlv_TEXCOORD3);
  lowp vec4 tmpvar_24;
  tmpvar_24 = texture2D (_LightTexture0, vec2(tmpvar_23));
  mediump vec3 lightDir_25;
  lightDir_25 = lightDir_2;
  mediump vec3 viewDir_26;
  viewDir_26 = tmpvar_22;
  mediump float atten_27;
  atten_27 = tmpvar_24.w;
  mediump vec4 c_28;
  mediump vec3 specCol_29;
  highp float nh_30;
  mediump float tmpvar_31;
  tmpvar_31 = max (0.0, dot (tmpvar_4, normalize((lightDir_25 + viewDir_26))));
  nh_30 = tmpvar_31;
  mediump float arg1_32;
  arg1_32 = (32.0 * _Shininess);
  highp vec3 tmpvar_33;
  tmpvar_33 = (pow (nh_30, arg1_32) * tmpvar_19);
  specCol_29 = tmpvar_33;
  c_28.xyz = ((((tmpvar_3 * _LightColor0.xyz) * max (0.0, dot (tmpvar_4, lightDir_25))) + (_LightColor0.xyz * specCol_29)) * (atten_27 * 2.0));
  c_28.w = 0.0;
  c_1.xyz = c_28.xyz;
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
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 _BumpMap_ST;
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
  highp vec4 tmpvar_3;
  mediump vec3 tmpvar_4;
  tmpvar_3.xy = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  tmpvar_3.zw = ((_glesMultiTexCoord0.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
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
  tmpvar_4 = tmpvar_8;
  highp vec4 tmpvar_9;
  tmpvar_9.w = 1.0;
  tmpvar_9.xyz = _WorldSpaceCameraPos;
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = tmpvar_3;
  xlv_TEXCOORD1 = (tmpvar_7 * (((_World2Object * tmpvar_9).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = tmpvar_4;
  xlv_TEXCOORD3 = (_LightMatrix0 * (_Object2World * _glesVertex)).xyz;
}



#endif
#ifdef FRAGMENT

varying highp vec3 xlv_TEXCOORD3;
varying mediump vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp float _Parallax;
uniform mediump float _Gloss;
uniform mediump float _Shininess;
uniform lowp vec4 _Color;
uniform sampler2D _ParallaxMap;
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
  mediump vec4 spec_5;
  mediump float h_6;
  lowp float tmpvar_7;
  tmpvar_7 = texture2D (_ParallaxMap, xlv_TEXCOORD0.zw).w;
  h_6 = tmpvar_7;
  highp vec2 tmpvar_8;
  mediump float height_9;
  height_9 = _Parallax;
  mediump vec3 viewDir_10;
  viewDir_10 = xlv_TEXCOORD1;
  highp vec3 v_11;
  mediump float tmpvar_12;
  tmpvar_12 = ((h_6 * height_9) - (height_9 / 2.0));
  mediump vec3 tmpvar_13;
  tmpvar_13 = normalize(viewDir_10);
  v_11 = tmpvar_13;
  v_11.z = (v_11.z + 0.42);
  tmpvar_8 = (tmpvar_12 * (v_11.xy / v_11.z));
  highp vec2 tmpvar_14;
  tmpvar_14 = (xlv_TEXCOORD0.xy + tmpvar_8);
  highp vec2 tmpvar_15;
  tmpvar_15 = (xlv_TEXCOORD0.zw + tmpvar_8);
  lowp vec4 tmpvar_16;
  tmpvar_16 = texture2D (_MainTex, tmpvar_14);
  lowp vec3 tmpvar_17;
  tmpvar_17 = (tmpvar_16.xyz * _Color.xyz);
  tmpvar_3 = tmpvar_17;
  lowp vec4 tmpvar_18;
  tmpvar_18 = texture2D (_SpecMap, tmpvar_14);
  spec_5 = tmpvar_18;
  mediump vec3 tmpvar_19;
  tmpvar_19 = ((tmpvar_16.w * spec_5.xyz) * _Gloss);
  lowp vec3 normal_20;
  normal_20.xy = ((texture2D (_BumpMap, tmpvar_15).wy * 2.0) - 1.0);
  normal_20.z = sqrt(((1.0 - (normal_20.x * normal_20.x)) - (normal_20.y * normal_20.y)));
  tmpvar_4 = normal_20;
  mediump vec3 tmpvar_21;
  tmpvar_21 = normalize(xlv_TEXCOORD2);
  lightDir_2 = tmpvar_21;
  highp vec3 tmpvar_22;
  tmpvar_22 = normalize(xlv_TEXCOORD1);
  highp float tmpvar_23;
  tmpvar_23 = dot (xlv_TEXCOORD3, xlv_TEXCOORD3);
  lowp vec4 tmpvar_24;
  tmpvar_24 = texture2D (_LightTexture0, vec2(tmpvar_23));
  mediump vec3 lightDir_25;
  lightDir_25 = lightDir_2;
  mediump vec3 viewDir_26;
  viewDir_26 = tmpvar_22;
  mediump float atten_27;
  atten_27 = tmpvar_24.w;
  mediump vec4 c_28;
  mediump vec3 specCol_29;
  highp float nh_30;
  mediump float tmpvar_31;
  tmpvar_31 = max (0.0, dot (tmpvar_4, normalize((lightDir_25 + viewDir_26))));
  nh_30 = tmpvar_31;
  mediump float arg1_32;
  arg1_32 = (32.0 * _Shininess);
  highp vec3 tmpvar_33;
  tmpvar_33 = (pow (nh_30, arg1_32) * tmpvar_19);
  specCol_29 = tmpvar_33;
  c_28.xyz = ((((tmpvar_3 * _LightColor0.xyz) * max (0.0, dot (tmpvar_4, lightDir_25))) + (_LightColor0.xyz * specCol_29)) * (atten_27 * 2.0));
  c_28.w = 0.0;
  c_1.xyz = c_28.xyz;
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
Vector 20 [_BumpMap_ST]
"agal_vs
c21 1.0 0.0 0.0 0.0
[bc]
aaaaaaaaabaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r1.xyz, a5
aaaaaaaaaaaaapacakaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0, c10
bdaaaaaaadaaaeacbbaaaaoeabaaaaaaaaaaaaoeacaaaaaa dp4 r3.z, c17, r0
aaaaaaaaaaaaapacajaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0, c9
bdaaaaaaadaaacacbbaaaaoeabaaaaaaaaaaaaoeacaaaaaa dp4 r3.y, c17, r0
adaaaaaaacaaahacabaaaancaaaaaaaaabaaaaajacaaaaaa mul r2.xyz, a1.zxyw, r1.yzxx
aaaaaaaaabaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r1.xyz, a5
adaaaaaaaeaaahacabaaaamjaaaaaaaaabaaaafcacaaaaaa mul r4.xyz, a1.yzxw, r1.zxyy
acaaaaaaacaaahacaeaaaakeacaaaaaaacaaaakeacaaaaaa sub r2.xyz, r4.xyzz, r2.xyzz
aaaaaaaaabaaapacaiaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r1, c8
bdaaaaaaadaaabacbbaaaaoeabaaaaaaabaaaaoeacaaaaaa dp4 r3.x, c17, r1
adaaaaaaaeaaahacadaaaakeacaaaaaabcaaaappabaaaaaa mul r4.xyz, r3.xyzz, c18.w
acaaaaaaaaaaahacaeaaaakeacaaaaaaaaaaaaoeaaaaaaaa sub r0.xyz, r4.xyzz, a0
adaaaaaaacaaahacacaaaakeacaaaaaaafaaaappaaaaaaaa mul r2.xyz, r2.xyzz, a5.w
aaaaaaaaabaaahacbaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r1.xyz, c16
aaaaaaaaabaaaiacbfaaaaaaabaaaaaaaaaaaaaaaaaaaaaa mov r1.w, c21.x
bcaaaaaaacaaacaeacaaaakeacaaaaaaaaaaaakeacaaaaaa dp3 v2.y, r2.xyzz, r0.xyzz
bcaaaaaaacaaaeaeabaaaaoeaaaaaaaaaaaaaakeacaaaaaa dp3 v2.z, a1, r0.xyzz
bcaaaaaaacaaabaeafaaaaoeaaaaaaaaaaaaaakeacaaaaaa dp3 v2.x, a5, r0.xyzz
bdaaaaaaaaaaaiacaaaaaaoeaaaaaaaaahaaaaoeabaaaaaa dp4 r0.w, a0, c7
bdaaaaaaaaaaaeacaaaaaaoeaaaaaaaaagaaaaoeabaaaaaa dp4 r0.z, a0, c6
bdaaaaaaaaaaabacaaaaaaoeaaaaaaaaaeaaaaoeabaaaaaa dp4 r0.x, a0, c4
bdaaaaaaaaaaacacaaaaaaoeaaaaaaaaafaaaaoeabaaaaaa dp4 r0.y, a0, c5
bdaaaaaaadaaaeacabaaaaoeacaaaaaaakaaaaoeabaaaaaa dp4 r3.z, r1, c10
bdaaaaaaadaaabacabaaaaoeacaaaaaaaiaaaaoeabaaaaaa dp4 r3.x, r1, c8
bdaaaaaaadaaacacabaaaaoeacaaaaaaajaaaaoeabaaaaaa dp4 r3.y, r1, c9
adaaaaaaaeaaahacadaaaakeacaaaaaabcaaaappabaaaaaa mul r4.xyz, r3.xyzz, c18.w
acaaaaaaabaaahacaeaaaakeacaaaaaaaaaaaaoeaaaaaaaa sub r1.xyz, r4.xyzz, a0
bcaaaaaaabaaacaeabaaaakeacaaaaaaacaaaakeacaaaaaa dp3 v1.y, r1.xyzz, r2.xyzz
bcaaaaaaabaaaeaeabaaaaoeaaaaaaaaabaaaakeacaaaaaa dp3 v1.z, a1, r1.xyzz
bcaaaaaaabaaabaeabaaaakeacaaaaaaafaaaaoeaaaaaaaa dp3 v1.x, r1.xyzz, a5
bdaaaaaaadaaaeaeaaaaaaoeacaaaaaaaoaaaaoeabaaaaaa dp4 v3.z, r0, c14
bdaaaaaaadaaacaeaaaaaaoeacaaaaaaanaaaaoeabaaaaaa dp4 v3.y, r0, c13
bdaaaaaaadaaabaeaaaaaaoeacaaaaaaamaaaaoeabaaaaaa dp4 v3.x, r0, c12
adaaaaaaaeaaamacadaaaaeeaaaaaaaabeaaaaeeabaaaaaa mul r4.zw, a3.xyxy, c20.xyxy
abaaaaaaaaaaamaeaeaaaaopacaaaaaabeaaaaoeabaaaaaa add v0.zw, r4.wwzw, c20
adaaaaaaaeaaadacadaaaaoeaaaaaaaabdaaaaoeabaaaaaa mul r4.xy, a3, c19
abaaaaaaaaaaadaeaeaaaafeacaaaaaabdaaaaooabaaaaaa add v0.xy, r4.xyyy, c19.zwzw
bdaaaaaaaaaaaiadaaaaaaoeaaaaaaaaadaaaaoeabaaaaaa dp4 o0.w, a0, c3
bdaaaaaaaaaaaeadaaaaaaoeaaaaaaaaacaaaaoeabaaaaaa dp4 o0.z, a0, c2
bdaaaaaaaaaaacadaaaaaaoeaaaaaaaaabaaaaoeabaaaaaa dp4 o0.y, a0, c1
bdaaaaaaaaaaabadaaaaaaoeaaaaaaaaaaaaaaoeabaaaaaa dp4 o0.x, a0, c0
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
ConstBuffer "$Globals" 176 // 176 used size, 10 vars
Matrix 48 [_LightMatrix0] 4
Vector 144 [_MainTex_ST] 4
Vector 160 [_BumpMap_ST] 4
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
// 34 instructions, 2 temp regs, 0 temp arrays:
// ALU 14 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0_level_9_3
eefiecedmojjpcpiaakmdikedfpgjkanpehfmgdgabaaaaaahiakaaaaaeaaaaaa
daaaaaaaiaadaaaabaajaaaaniajaaaaebgpgodjeiadaaaaeiadaaaaaaacpopp
niacaaaahaaaaaaaagaaceaaaaaagmaaaaaagmaaaaaaceaaabaagmaaaaaaadaa
aeaaabaaaaaaaaaaaaaaajaaacaaafaaaaaaaaaaabaaaeaaabaaahaaaaaaaaaa
acaaaaaaabaaaiaaaaaaaaaaadaaaaaaaeaaajaaaaaaaaaaadaaamaaajaaanaa
aaaaaaaaaaaaaaaaabacpoppbpaaaaacafaaaaiaaaaaapjabpaaaaacafaaabia
abaaapjabpaaaaacafaaaciaacaaapjabpaaaaacafaaadiaadaaapjaaeaaaaae
aaaaadoaadaaoejaafaaoekaafaaookaaeaaaaaeaaaaamoaadaaeejaagaaeeka
agaaoekaabaaaaacaaaaapiaaiaaoekaafaaaaadabaaahiaaaaaffiabcaaoeka
aeaaaaaeabaaahiabbaaoekaaaaaaaiaabaaoeiaaeaaaaaeaaaaahiabdaaoeka
aaaakkiaabaaoeiaaeaaaaaeaaaaahiabeaaoekaaaaappiaaaaaoeiaaeaaaaae
aaaaahiaaaaaoeiabfaappkaaaaaoejbaiaaaaadacaaaboaabaaoejaaaaaoeia
abaaaaacabaaahiaabaaoejaafaaaaadacaaahiaabaamjiaacaancjaaeaaaaae
abaaahiaacaamjjaabaanciaacaaoeibafaaaaadabaaahiaabaaoeiaabaappja
aiaaaaadacaaacoaabaaoeiaaaaaoeiaaiaaaaadacaaaeoaacaaoejaaaaaoeia
abaaaaacaaaaahiaahaaoekaafaaaaadacaaahiaaaaaffiabcaaoekaaeaaaaae
aaaaaliabbaakekaaaaaaaiaacaakeiaaeaaaaaeaaaaahiabdaaoekaaaaakkia
aaaapeiaacaaaaadaaaaahiaaaaaoeiabeaaoekaaeaaaaaeaaaaahiaaaaaoeia
bfaappkaaaaaoejbaiaaaaadabaaaboaabaaoejaaaaaoeiaaiaaaaadabaaacoa
abaaoeiaaaaaoeiaaiaaaaadabaaaeoaacaaoejaaaaaoeiaafaaaaadaaaaapia
aaaaffjaaoaaoekaaeaaaaaeaaaaapiaanaaoekaaaaaaajaaaaaoeiaaeaaaaae
aaaaapiaapaaoekaaaaakkjaaaaaoeiaaeaaaaaeaaaaapiabaaaoekaaaaappja
aaaaoeiaafaaaaadabaaahiaaaaaffiaacaaoekaaeaaaaaeabaaahiaabaaoeka
aaaaaaiaabaaoeiaaeaaaaaeaaaaahiaadaaoekaaaaakkiaabaaoeiaaeaaaaae
adaaahoaaeaaoekaaaaappiaaaaaoeiaafaaaaadaaaaapiaaaaaffjaakaaoeka
aeaaaaaeaaaaapiaajaaoekaaaaaaajaaaaaoeiaaeaaaaaeaaaaapiaalaaoeka
aaaakkjaaaaaoeiaaeaaaaaeaaaaapiaamaaoekaaaaappjaaaaaoeiaaeaaaaae
aaaaadmaaaaappiaaaaaoekaaaaaoeiaabaaaaacaaaaammaaaaaoeiappppaaaa
fdeieefciiafaaaaeaaaabaagcabaaaafjaaaaaeegiocaaaaaaaaaaaalaaaaaa
fjaaaaaeegiocaaaabaaaaaaafaaaaaafjaaaaaeegiocaaaacaaaaaaabaaaaaa
fjaaaaaeegiocaaaadaaaaaabfaaaaaafpaaaaadpcbabaaaaaaaaaaafpaaaaad
pcbabaaaabaaaaaafpaaaaadhcbabaaaacaaaaaafpaaaaaddcbabaaaadaaaaaa
ghaaaaaepccabaaaaaaaaaaaabaaaaaagfaaaaadpccabaaaabaaaaaagfaaaaad
hccabaaaacaaaaaagfaaaaadhccabaaaadaaaaaagfaaaaadhccabaaaaeaaaaaa
giaaaaacacaaaaaadiaaaaaipcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaa
adaaaaaaabaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaaaaaaaaa
agbabaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaa
adaaaaaaacaaaaaakgbkbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpccabaaa
aaaaaaaaegiocaaaadaaaaaaadaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaa
dcaaaaaldccabaaaabaaaaaaegbabaaaadaaaaaaegiacaaaaaaaaaaaajaaaaaa
ogikcaaaaaaaaaaaajaaaaaadcaaaaalmccabaaaabaaaaaaagbebaaaadaaaaaa
agiecaaaaaaaaaaaakaaaaaakgiocaaaaaaaaaaaakaaaaaadiaaaaahhcaabaaa
aaaaaaaajgbebaaaabaaaaaacgbjbaaaacaaaaaadcaaaaakhcaabaaaaaaaaaaa
jgbebaaaacaaaaaacgbjbaaaabaaaaaaegacbaiaebaaaaaaaaaaaaaadiaaaaah
hcaabaaaaaaaaaaaegacbaaaaaaaaaaapgbpbaaaabaaaaaadiaaaaajhcaabaaa
abaaaaaafgifcaaaabaaaaaaaeaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaal
hcaabaaaabaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaaabaaaaaaaeaaaaaa
egacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabcaaaaaa
kgikcaaaabaaaaaaaeaaaaaaegacbaaaabaaaaaaaaaaaaaihcaabaaaabaaaaaa
egacbaaaabaaaaaaegiccaaaadaaaaaabdaaaaaadcaaaaalhcaabaaaabaaaaaa
egacbaaaabaaaaaapgipcaaaadaaaaaabeaaaaaaegbcbaiaebaaaaaaaaaaaaaa
baaaaaahcccabaaaacaaaaaaegacbaaaaaaaaaaaegacbaaaabaaaaaabaaaaaah
bccabaaaacaaaaaaegbcbaaaabaaaaaaegacbaaaabaaaaaabaaaaaaheccabaaa
acaaaaaaegbcbaaaacaaaaaaegacbaaaabaaaaaadiaaaaajhcaabaaaabaaaaaa
fgifcaaaacaaaaaaaaaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaa
abaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaaacaaaaaaaaaaaaaaegacbaaa
abaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaa
acaaaaaaaaaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaa
adaaaaaabdaaaaaapgipcaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaadcaaaaal
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
imaaaaaaaaaaaaaaaaaaaaaaadaaaaaaabaaaaaaapaaaaaaimaaaaaaabaaaaaa
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
Vector 13 [_BumpMap_ST]
"!!ARBvp1.0
# 26 ALU
PARAM c[14] = { { 1 },
		state.matrix.mvp,
		program.local[5..13] };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
MOV R0.xyz, vertex.attrib[14];
MUL R1.xyz, vertex.normal.zxyw, R0.yzxw;
MAD R1.xyz, vertex.normal.yzxw, R0.zxyw, -R1;
MOV R0.xyz, c[9];
MOV R0.w, c[0].x;
DP4 R2.z, R0, c[7];
DP4 R2.x, R0, c[5];
DP4 R2.y, R0, c[6];
MAD R0.xyz, R2, c[11].w, -vertex.position;
MUL R2.xyz, R1, vertex.attrib[14].w;
MOV R1, c[10];
DP4 R3.z, R1, c[7];
DP4 R3.x, R1, c[5];
DP4 R3.y, R1, c[6];
DP3 result.texcoord[1].y, R0, R2;
DP3 result.texcoord[2].y, R2, R3;
DP3 result.texcoord[1].z, vertex.normal, R0;
DP3 result.texcoord[1].x, R0, vertex.attrib[14];
DP3 result.texcoord[2].z, vertex.normal, R3;
DP3 result.texcoord[2].x, vertex.attrib[14], R3;
MAD result.texcoord[0].zw, vertex.texcoord[0].xyxy, c[13].xyxy, c[13];
MAD result.texcoord[0].xy, vertex.texcoord[0], c[12], c[12].zwzw;
DP4 result.position.w, vertex.position, c[4];
DP4 result.position.z, vertex.position, c[3];
DP4 result.position.y, vertex.position, c[2];
DP4 result.position.x, vertex.position, c[1];
END
# 26 instructions, 4 R-regs
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
Vector 12 [_BumpMap_ST]
"vs_2_0
; 29 ALU
def c13, 1.00000000, 0, 0, 0
dcl_position0 v0
dcl_tangent0 v1
dcl_normal0 v2
dcl_texcoord0 v3
mov r0.xyz, v1
mul r1.xyz, v2.zxyw, r0.yzxw
mov r0.xyz, v1
mad r0.xyz, v2.yzxw, r0.zxyw, -r1
mul r3.xyz, r0, v1.w
mov r0, c6
dp4 r4.z, c9, r0
mov r0, c5
mov r1.w, c13.x
mov r1.xyz, c8
dp4 r4.y, c9, r0
dp4 r2.z, r1, c6
dp4 r2.x, r1, c4
dp4 r2.y, r1, c5
mad r2.xyz, r2, c10.w, -v0
mov r1, c4
dp4 r4.x, c9, r1
dp3 oT1.y, r2, r3
dp3 oT2.y, r3, r4
dp3 oT1.z, v2, r2
dp3 oT1.x, r2, v1
dp3 oT2.z, v2, r4
dp3 oT2.x, v1, r4
mad oT0.zw, v3.xyxy, c12.xyxy, c12
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
Vector 12 [_BumpMap_ST]
Vector 11 [_MainTex_ST]
Matrix 6 [_World2Object] 4
Vector 0 [_WorldSpaceCameraPos]
Vector 1 [_WorldSpaceLightPos0]
Matrix 2 [glstate_matrix_mvp] 4
Vector 10 [unity_Scale]
// Shader Timing Estimate, in Cycles/64 vertex vector:
// ALU: 30.67 (23 instructions), vertex: 32, texture: 0,
//   sequencer: 16,  7 GPRs, 27 threads,
// Performance (if enough threads): ~32 cycles per vector
// * Vertex cycle estimates are assuming 3 vfetch_minis for every vfetch_full,
//     with <= 32 bytes per vfetch_full group.

"vs_360
backbbabaaaaabomaaaaabiaaaaaaaaaaaaaaaceaaaaaaaaaaaaabiiaaaaaaaa
aaaaaaaaaaaaabgaaaaaaabmaaaaabfdpppoadaaaaaaaaahaaaaaabmaaaaaaaa
aaaaabemaaaaaakiaaacaaamaaabaaaaaaaaaaleaaaaaaaaaaaaaameaaacaaal
aaabaaaaaaaaaaleaaaaaaaaaaaaaanaaaacaaagaaaeaaaaaaaaaaoaaaaaaaaa
aaaaaapaaaacaaaaaaabaaaaaaaaabaiaaaaaaaaaaaaabbiaaacaaabaaabaaaa
aaaaaaleaaaaaaaaaaaaabcnaaacaaacaaaeaaaaaaaaaaoaaaaaaaaaaaaaabea
aaacaaakaaabaaaaaaaaaaleaaaaaaaafpechfgnhaengbhafpfdfeaaaaabaaad
aaabaaaeaaabaaaaaaaaaaaafpengbgjgofegfhifpfdfeaafpfhgphcgmgedcep
gcgkgfgdheaaklklaaadaaadaaaeaaaeaaabaaaaaaaaaaaafpfhgphcgmgefdha
gbgdgfedgbgngfhcgbfagphdaaklklklaaabaaadaaabaaadaaabaaaaaaaaaaaa
fpfhgphcgmgefdhagbgdgfemgjghgihefagphddaaaghgmhdhegbhegffpgngbhe
hcgjhifpgnhghaaahfgogjhehjfpfdgdgbgmgfaahghdfpddfpdaaadccodacodc
dadddfddcodaaaklaaaaaaaaaaaaabiaaacbaaagaaaaaaaaaaaaaaaaaaaacigd
aaaaaaabaaaaaaaeaaaaaaaiaaaaacjaaabaaaaeaaaagaafaaaadaagaadafaah
aaaapafaaaachbfbaaafhcfcaaaaaabnaaaababoaaaaaabhaaaaaabiaaaababj
aaaaaabkaaaaaablaaaababmpaffeaaeaaaabcaamcaaaaaaaaaaeaaiaaaabcaa
meaaaaaaaaaagaamgabcbcaabcaaaaaaaaaagabibabobcaaccaaaaaaafpifaaa
aaaaagiiaaaaaaaaafpieaaaaaaaagiiaaaaaaaaafpibaaaaaaaaoiiaaaaaaaa
afpiaaaaaaaaapmiaaaaaaaamiapaaacaabliiaakbafafaamiapaaacaamgiiaa
klafaeacmiapaaacaalbdejeklafadacmiapiadoaagmaadeklafacacmiahaaag
aamamgmaalaiaaajmiahaaacaaleblaacbajabaamiahaaacaamamgleclaiabac
miahaaadaalogfaaobabaeaamiahaaagaalelbleclahaaagmiahaaagaamagmle
clagaaagmiahaaadabgflomaolabaeadmiahaaacaalelbleclahabacmiahaaac
aamagmleclagabacmiahaaadaamablaaobadaeaamiahaaafabmablmaklagakaf
miabiaabaaloloaapaafaeaamiaciaabaaloloaapaadafaamiaeiaabaaloloaa
paafabaamiabiaacaaloloaapaacaeaamiaciaacaaloloaapaadacaamiaeiaac
aaloloaapaacabaamiadiaaaaalalabkilaaalalmiamiaaaaakmkmagilaaamam
aaaaaaaaaaaaaaaaaaaaaaaa"
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
Vector 463 [_BumpMap_ST]
"sce_vp_rsx // 25 instructions using 4 registers
[Configuration]
8
0000001941050400
[Microcode]
400
00009c6c005d200d8186c0836041fffc00011c6c00400e0c0106c0836041dffc
00001c6c005d300c0186c0836041dffc401f9c6c011cf800810040d560607f9c
401f9c6c011d0808010400d740619f9c401f9c6c01d0300d8106c0c360403f80
401f9c6c01d0200d8106c0c360405f80401f9c6c01d0100d8106c0c360409f80
401f9c6c01d0000d8106c0c360411f8000019c6c01d0600d8286c0c360405ffc
00019c6c01d0500d8286c0c360409ffc00019c6c01d0400d8286c0c360411ffc
00009c6c0190600c0086c0c360405ffc00009c6c0190500c0086c0c360409ffc
00009c6c0190400c0086c0c360411ffc00001c6c00800243011842436041dffc
00001c6c01000230812182630021dffc00009c6c011d100c02bfc0e30041dffc
401f9c6c0140020c0106034360405fa4401f9c6c01400e0c0106034360411fa4
00001c6c00800e0c00bfc0836041dffc401f9c6c0140020c0106014360405fa0
401f9c6c01400e0c0286008360411fa0401f9c6c0140000c0086034360409fa4
401f9c6c0140000c0286004360409fa1
"
}

SubProgram "d3d11 " {
Keywords { "DIRECTIONAL" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 112 // 112 used size, 9 vars
Vector 80 [_MainTex_ST] 4
Vector 96 [_BumpMap_ST] 4
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
// 25 instructions, 2 temp regs, 0 temp arrays:
// ALU 12 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0
eefiecedoocgiajiilgjjgeolnhaiicdpelikbhdabaaaaaakeafaaaaadaaaaaa
cmaaaaaapeaaaaaahmabaaaaejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaa
abaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaljaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfc
enebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheoiaaaaaaaaeaaaaaa
aiaaaaaagiaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaheaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaabaaaaaaapaaaaaaheaaaaaaabaaaaaaaaaaaaaa
adaaaaaaacaaaaaaahaiaaaaheaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaa
ahaiaaaafdfgfpfaepfdejfeejepeoaafeeffiedepepfceeaaklklklfdeieefc
caaeaaaaeaaaabaaaiabaaaafjaaaaaeegiocaaaaaaaaaaaahaaaaaafjaaaaae
egiocaaaabaaaaaaafaaaaaafjaaaaaeegiocaaaacaaaaaaabaaaaaafjaaaaae
egiocaaaadaaaaaabfaaaaaafpaaaaadpcbabaaaaaaaaaaafpaaaaadpcbabaaa
abaaaaaafpaaaaadhcbabaaaacaaaaaafpaaaaaddcbabaaaadaaaaaaghaaaaae
pccabaaaaaaaaaaaabaaaaaagfaaaaadpccabaaaabaaaaaagfaaaaadhccabaaa
acaaaaaagfaaaaadhccabaaaadaaaaaagiaaaaacacaaaaaadiaaaaaipcaabaaa
aaaaaaaafgbfbaaaaaaaaaaaegiocaaaadaaaaaaabaaaaaadcaaaaakpcaabaaa
aaaaaaaaegiocaaaadaaaaaaaaaaaaaaagbabaaaaaaaaaaaegaobaaaaaaaaaaa
dcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaacaaaaaakgbkbaaaaaaaaaaa
egaobaaaaaaaaaaadcaaaaakpccabaaaaaaaaaaaegiocaaaadaaaaaaadaaaaaa
pgbpbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaaldccabaaaabaaaaaaegbabaaa
adaaaaaaegiacaaaaaaaaaaaafaaaaaaogikcaaaaaaaaaaaafaaaaaadcaaaaal
mccabaaaabaaaaaaagbebaaaadaaaaaaagiecaaaaaaaaaaaagaaaaaakgiocaaa
aaaaaaaaagaaaaaadiaaaaahhcaabaaaaaaaaaaajgbebaaaabaaaaaacgbjbaaa
acaaaaaadcaaaaakhcaabaaaaaaaaaaajgbebaaaacaaaaaacgbjbaaaabaaaaaa
egacbaiaebaaaaaaaaaaaaaadiaaaaahhcaabaaaaaaaaaaaegacbaaaaaaaaaaa
pgbpbaaaabaaaaaadiaaaaajhcaabaaaabaaaaaafgifcaaaabaaaaaaaeaaaaaa
egiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaa
baaaaaaaagiacaaaabaaaaaaaeaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaa
abaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaaabaaaaaaaeaaaaaaegacbaaa
abaaaaaaaaaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaaegiccaaaadaaaaaa
bdaaaaaadcaaaaalhcaabaaaabaaaaaaegacbaaaabaaaaaapgipcaaaadaaaaaa
beaaaaaaegbcbaiaebaaaaaaaaaaaaaabaaaaaahcccabaaaacaaaaaaegacbaaa
aaaaaaaaegacbaaaabaaaaaabaaaaaahbccabaaaacaaaaaaegbcbaaaabaaaaaa
egacbaaaabaaaaaabaaaaaaheccabaaaacaaaaaaegbcbaaaacaaaaaaegacbaaa
abaaaaaadiaaaaajhcaabaaaabaaaaaafgifcaaaacaaaaaaaaaaaaaaegiccaaa
adaaaaaabbaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabaaaaaaa
agiacaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaa
egiccaaaadaaaaaabcaaaaaakgikcaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaa
dcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabdaaaaaapgipcaaaacaaaaaa
aaaaaaaaegacbaaaabaaaaaabaaaaaahcccabaaaadaaaaaaegacbaaaaaaaaaaa
egacbaaaabaaaaaabaaaaaahbccabaaaadaaaaaaegbcbaaaabaaaaaaegacbaaa
abaaaaaabaaaaaaheccabaaaadaaaaaaegbcbaaaacaaaaaaegacbaaaabaaaaaa
doaaaaab"
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
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 _BumpMap_ST;
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
  highp vec4 tmpvar_3;
  mediump vec3 tmpvar_4;
  tmpvar_3.xy = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  tmpvar_3.zw = ((_glesMultiTexCoord0.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
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
  tmpvar_4 = tmpvar_8;
  highp vec4 tmpvar_9;
  tmpvar_9.w = 1.0;
  tmpvar_9.xyz = _WorldSpaceCameraPos;
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = tmpvar_3;
  xlv_TEXCOORD1 = (tmpvar_7 * (((_World2Object * tmpvar_9).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = tmpvar_4;
}



#endif
#ifdef FRAGMENT

varying mediump vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp float _Parallax;
uniform mediump float _Gloss;
uniform mediump float _Shininess;
uniform lowp vec4 _Color;
uniform sampler2D _ParallaxMap;
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
  mediump vec4 spec_5;
  mediump float h_6;
  lowp float tmpvar_7;
  tmpvar_7 = texture2D (_ParallaxMap, xlv_TEXCOORD0.zw).w;
  h_6 = tmpvar_7;
  highp vec2 tmpvar_8;
  mediump float height_9;
  height_9 = _Parallax;
  mediump vec3 viewDir_10;
  viewDir_10 = xlv_TEXCOORD1;
  highp vec3 v_11;
  mediump float tmpvar_12;
  tmpvar_12 = ((h_6 * height_9) - (height_9 / 2.0));
  mediump vec3 tmpvar_13;
  tmpvar_13 = normalize(viewDir_10);
  v_11 = tmpvar_13;
  v_11.z = (v_11.z + 0.42);
  tmpvar_8 = (tmpvar_12 * (v_11.xy / v_11.z));
  highp vec2 tmpvar_14;
  tmpvar_14 = (xlv_TEXCOORD0.xy + tmpvar_8);
  highp vec2 tmpvar_15;
  tmpvar_15 = (xlv_TEXCOORD0.zw + tmpvar_8);
  lowp vec4 tmpvar_16;
  tmpvar_16 = texture2D (_MainTex, tmpvar_14);
  lowp vec3 tmpvar_17;
  tmpvar_17 = (tmpvar_16.xyz * _Color.xyz);
  tmpvar_3 = tmpvar_17;
  lowp vec4 tmpvar_18;
  tmpvar_18 = texture2D (_SpecMap, tmpvar_14);
  spec_5 = tmpvar_18;
  mediump vec3 tmpvar_19;
  tmpvar_19 = ((tmpvar_16.w * spec_5.xyz) * _Gloss);
  lowp vec3 tmpvar_20;
  tmpvar_20 = ((texture2D (_BumpMap, tmpvar_15).xyz * 2.0) - 1.0);
  tmpvar_4 = tmpvar_20;
  lightDir_2 = xlv_TEXCOORD2;
  highp vec3 tmpvar_21;
  tmpvar_21 = normalize(xlv_TEXCOORD1);
  mediump vec3 lightDir_22;
  lightDir_22 = lightDir_2;
  mediump vec3 viewDir_23;
  viewDir_23 = tmpvar_21;
  mediump vec4 c_24;
  mediump vec3 specCol_25;
  highp float nh_26;
  mediump float tmpvar_27;
  tmpvar_27 = max (0.0, dot (tmpvar_4, normalize((lightDir_22 + viewDir_23))));
  nh_26 = tmpvar_27;
  mediump float arg1_28;
  arg1_28 = (32.0 * _Shininess);
  highp vec3 tmpvar_29;
  tmpvar_29 = (pow (nh_26, arg1_28) * tmpvar_19);
  specCol_25 = tmpvar_29;
  c_24.xyz = ((((tmpvar_3 * _LightColor0.xyz) * max (0.0, dot (tmpvar_4, lightDir_22))) + (_LightColor0.xyz * specCol_25)) * 2.0);
  c_24.w = 0.0;
  c_1.xyz = c_24.xyz;
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
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 _BumpMap_ST;
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
  highp vec4 tmpvar_3;
  mediump vec3 tmpvar_4;
  tmpvar_3.xy = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  tmpvar_3.zw = ((_glesMultiTexCoord0.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
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
  tmpvar_4 = tmpvar_8;
  highp vec4 tmpvar_9;
  tmpvar_9.w = 1.0;
  tmpvar_9.xyz = _WorldSpaceCameraPos;
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = tmpvar_3;
  xlv_TEXCOORD1 = (tmpvar_7 * (((_World2Object * tmpvar_9).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = tmpvar_4;
}



#endif
#ifdef FRAGMENT

varying mediump vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp float _Parallax;
uniform mediump float _Gloss;
uniform mediump float _Shininess;
uniform lowp vec4 _Color;
uniform sampler2D _ParallaxMap;
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
  mediump vec4 spec_5;
  mediump float h_6;
  lowp float tmpvar_7;
  tmpvar_7 = texture2D (_ParallaxMap, xlv_TEXCOORD0.zw).w;
  h_6 = tmpvar_7;
  highp vec2 tmpvar_8;
  mediump float height_9;
  height_9 = _Parallax;
  mediump vec3 viewDir_10;
  viewDir_10 = xlv_TEXCOORD1;
  highp vec3 v_11;
  mediump float tmpvar_12;
  tmpvar_12 = ((h_6 * height_9) - (height_9 / 2.0));
  mediump vec3 tmpvar_13;
  tmpvar_13 = normalize(viewDir_10);
  v_11 = tmpvar_13;
  v_11.z = (v_11.z + 0.42);
  tmpvar_8 = (tmpvar_12 * (v_11.xy / v_11.z));
  highp vec2 tmpvar_14;
  tmpvar_14 = (xlv_TEXCOORD0.xy + tmpvar_8);
  highp vec2 tmpvar_15;
  tmpvar_15 = (xlv_TEXCOORD0.zw + tmpvar_8);
  lowp vec4 tmpvar_16;
  tmpvar_16 = texture2D (_MainTex, tmpvar_14);
  lowp vec3 tmpvar_17;
  tmpvar_17 = (tmpvar_16.xyz * _Color.xyz);
  tmpvar_3 = tmpvar_17;
  lowp vec4 tmpvar_18;
  tmpvar_18 = texture2D (_SpecMap, tmpvar_14);
  spec_5 = tmpvar_18;
  mediump vec3 tmpvar_19;
  tmpvar_19 = ((tmpvar_16.w * spec_5.xyz) * _Gloss);
  lowp vec3 normal_20;
  normal_20.xy = ((texture2D (_BumpMap, tmpvar_15).wy * 2.0) - 1.0);
  normal_20.z = sqrt(((1.0 - (normal_20.x * normal_20.x)) - (normal_20.y * normal_20.y)));
  tmpvar_4 = normal_20;
  lightDir_2 = xlv_TEXCOORD2;
  highp vec3 tmpvar_21;
  tmpvar_21 = normalize(xlv_TEXCOORD1);
  mediump vec3 lightDir_22;
  lightDir_22 = lightDir_2;
  mediump vec3 viewDir_23;
  viewDir_23 = tmpvar_21;
  mediump vec4 c_24;
  mediump vec3 specCol_25;
  highp float nh_26;
  mediump float tmpvar_27;
  tmpvar_27 = max (0.0, dot (tmpvar_4, normalize((lightDir_22 + viewDir_23))));
  nh_26 = tmpvar_27;
  mediump float arg1_28;
  arg1_28 = (32.0 * _Shininess);
  highp vec3 tmpvar_29;
  tmpvar_29 = (pow (nh_26, arg1_28) * tmpvar_19);
  specCol_25 = tmpvar_29;
  c_24.xyz = ((((tmpvar_3 * _LightColor0.xyz) * max (0.0, dot (tmpvar_4, lightDir_22))) + (_LightColor0.xyz * specCol_25)) * 2.0);
  c_24.w = 0.0;
  c_1.xyz = c_24.xyz;
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
Vector 12 [_BumpMap_ST]
"agal_vs
c13 1.0 0.0 0.0 0.0
[bc]
aaaaaaaaaaaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, a5
adaaaaaaabaaahacabaaaancaaaaaaaaaaaaaaajacaaaaaa mul r1.xyz, a1.zxyw, r0.yzxx
aaaaaaaaaaaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, a5
adaaaaaaacaaahacabaaaamjaaaaaaaaaaaaaafcacaaaaaa mul r2.xyz, a1.yzxw, r0.zxyy
acaaaaaaaaaaahacacaaaakeacaaaaaaabaaaakeacaaaaaa sub r0.xyz, r2.xyzz, r1.xyzz
adaaaaaaadaaahacaaaaaakeacaaaaaaafaaaappaaaaaaaa mul r3.xyz, r0.xyzz, a5.w
aaaaaaaaaaaaapacagaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0, c6
bdaaaaaaaeaaaeacajaaaaoeabaaaaaaaaaaaaoeacaaaaaa dp4 r4.z, c9, r0
aaaaaaaaaaaaapacafaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0, c5
aaaaaaaaabaaaiacanaaaaaaabaaaaaaaaaaaaaaaaaaaaaa mov r1.w, c13.x
aaaaaaaaabaaahacaiaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r1.xyz, c8
bdaaaaaaaeaaacacajaaaaoeabaaaaaaaaaaaaoeacaaaaaa dp4 r4.y, c9, r0
bdaaaaaaacaaaeacabaaaaoeacaaaaaaagaaaaoeabaaaaaa dp4 r2.z, r1, c6
bdaaaaaaacaaabacabaaaaoeacaaaaaaaeaaaaoeabaaaaaa dp4 r2.x, r1, c4
bdaaaaaaacaaacacabaaaaoeacaaaaaaafaaaaoeabaaaaaa dp4 r2.y, r1, c5
adaaaaaaaaaaahacacaaaakeacaaaaaaakaaaappabaaaaaa mul r0.xyz, r2.xyzz, c10.w
acaaaaaaacaaahacaaaaaakeacaaaaaaaaaaaaoeaaaaaaaa sub r2.xyz, r0.xyzz, a0
aaaaaaaaabaaapacaeaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r1, c4
bdaaaaaaaeaaabacajaaaaoeabaaaaaaabaaaaoeacaaaaaa dp4 r4.x, c9, r1
bcaaaaaaabaaacaeacaaaakeacaaaaaaadaaaakeacaaaaaa dp3 v1.y, r2.xyzz, r3.xyzz
bcaaaaaaacaaacaeadaaaakeacaaaaaaaeaaaakeacaaaaaa dp3 v2.y, r3.xyzz, r4.xyzz
bcaaaaaaabaaaeaeabaaaaoeaaaaaaaaacaaaakeacaaaaaa dp3 v1.z, a1, r2.xyzz
bcaaaaaaabaaabaeacaaaakeacaaaaaaafaaaaoeaaaaaaaa dp3 v1.x, r2.xyzz, a5
bcaaaaaaacaaaeaeabaaaaoeaaaaaaaaaeaaaakeacaaaaaa dp3 v2.z, a1, r4.xyzz
bcaaaaaaacaaabaeafaaaaoeaaaaaaaaaeaaaakeacaaaaaa dp3 v2.x, a5, r4.xyzz
adaaaaaaaaaaamacadaaaaeeaaaaaaaaamaaaaeeabaaaaaa mul r0.zw, a3.xyxy, c12.xyxy
abaaaaaaaaaaamaeaaaaaaopacaaaaaaamaaaaoeabaaaaaa add v0.zw, r0.wwzw, c12
adaaaaaaaaaaadacadaaaaoeaaaaaaaaalaaaaoeabaaaaaa mul r0.xy, a3, c11
abaaaaaaaaaaadaeaaaaaafeacaaaaaaalaaaaooabaaaaaa add v0.xy, r0.xyyy, c11.zwzw
bdaaaaaaaaaaaiadaaaaaaoeaaaaaaaaadaaaaoeabaaaaaa dp4 o0.w, a0, c3
bdaaaaaaaaaaaeadaaaaaaoeaaaaaaaaacaaaaoeabaaaaaa dp4 o0.z, a0, c2
bdaaaaaaaaaaacadaaaaaaoeaaaaaaaaabaaaaoeabaaaaaa dp4 o0.y, a0, c1
bdaaaaaaaaaaabadaaaaaaoeaaaaaaaaaaaaaaoeabaaaaaa dp4 o0.x, a0, c0
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
ConstBuffer "$Globals" 112 // 112 used size, 9 vars
Vector 80 [_MainTex_ST] 4
Vector 96 [_BumpMap_ST] 4
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
// 25 instructions, 2 temp regs, 0 temp arrays:
// ALU 12 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0_level_9_3
eefiecedbncdeaoomgdgfeflmkjfcmcoppbomhafabaaaaaaeaaiaaaaaeaaaaaa
daaaaaaamiacaaaapaagaaaaliahaaaaebgpgodjjaacaaaajaacaaaaaaacpopp
cmacaaaageaaaaaaafaaceaaaaaagaaaaaaagaaaaaaaceaaabaagaaaaaaaafaa
acaaabaaaaaaaaaaabaaaeaaabaaadaaaaaaaaaaacaaaaaaabaaaeaaaaaaaaaa
adaaaaaaaeaaafaaaaaaaaaaadaabaaaafaaajaaaaaaaaaaaaaaaaaaabacpopp
bpaaaaacafaaaaiaaaaaapjabpaaaaacafaaabiaabaaapjabpaaaaacafaaacia
acaaapjabpaaaaacafaaadiaadaaapjaaeaaaaaeaaaaadoaadaaoejaabaaoeka
abaaookaaeaaaaaeaaaaamoaadaaeejaacaaeekaacaaoekaabaaaaacaaaaapia
aeaaoekaafaaaaadabaaahiaaaaaffiaakaaoekaaeaaaaaeabaaahiaajaaoeka
aaaaaaiaabaaoeiaaeaaaaaeaaaaahiaalaaoekaaaaakkiaabaaoeiaaeaaaaae
aaaaahiaamaaoekaaaaappiaaaaaoeiaaiaaaaadacaaaboaabaaoejaaaaaoeia
abaaaaacabaaahiaabaaoejaafaaaaadacaaahiaabaamjiaacaancjaaeaaaaae
abaaahiaacaamjjaabaanciaacaaoeibafaaaaadabaaahiaabaaoeiaabaappja
aiaaaaadacaaacoaabaaoeiaaaaaoeiaaiaaaaadacaaaeoaacaaoejaaaaaoeia
abaaaaacaaaaahiaadaaoekaafaaaaadacaaahiaaaaaffiaakaaoekaaeaaaaae
aaaaaliaajaakekaaaaaaaiaacaakeiaaeaaaaaeaaaaahiaalaaoekaaaaakkia
aaaapeiaacaaaaadaaaaahiaaaaaoeiaamaaoekaaeaaaaaeaaaaahiaaaaaoeia
anaappkaaaaaoejbaiaaaaadabaaaboaabaaoejaaaaaoeiaaiaaaaadabaaacoa
abaaoeiaaaaaoeiaaiaaaaadabaaaeoaacaaoejaaaaaoeiaafaaaaadaaaaapia
aaaaffjaagaaoekaaeaaaaaeaaaaapiaafaaoekaaaaaaajaaaaaoeiaaeaaaaae
aaaaapiaahaaoekaaaaakkjaaaaaoeiaaeaaaaaeaaaaapiaaiaaoekaaaaappja
aaaaoeiaaeaaaaaeaaaaadmaaaaappiaaaaaoekaaaaaoeiaabaaaaacaaaaamma
aaaaoeiappppaaaafdeieefccaaeaaaaeaaaabaaaiabaaaafjaaaaaeegiocaaa
aaaaaaaaahaaaaaafjaaaaaeegiocaaaabaaaaaaafaaaaaafjaaaaaeegiocaaa
acaaaaaaabaaaaaafjaaaaaeegiocaaaadaaaaaabfaaaaaafpaaaaadpcbabaaa
aaaaaaaafpaaaaadpcbabaaaabaaaaaafpaaaaadhcbabaaaacaaaaaafpaaaaad
dcbabaaaadaaaaaaghaaaaaepccabaaaaaaaaaaaabaaaaaagfaaaaadpccabaaa
abaaaaaagfaaaaadhccabaaaacaaaaaagfaaaaadhccabaaaadaaaaaagiaaaaac
acaaaaaadiaaaaaipcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaadaaaaaa
abaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaaaaaaaaaagbabaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaa
acaaaaaakgbkbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpccabaaaaaaaaaaa
egiocaaaadaaaaaaadaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaal
dccabaaaabaaaaaaegbabaaaadaaaaaaegiacaaaaaaaaaaaafaaaaaaogikcaaa
aaaaaaaaafaaaaaadcaaaaalmccabaaaabaaaaaaagbebaaaadaaaaaaagiecaaa
aaaaaaaaagaaaaaakgiocaaaaaaaaaaaagaaaaaadiaaaaahhcaabaaaaaaaaaaa
jgbebaaaabaaaaaacgbjbaaaacaaaaaadcaaaaakhcaabaaaaaaaaaaajgbebaaa
acaaaaaacgbjbaaaabaaaaaaegacbaiaebaaaaaaaaaaaaaadiaaaaahhcaabaaa
aaaaaaaaegacbaaaaaaaaaaapgbpbaaaabaaaaaadiaaaaajhcaabaaaabaaaaaa
fgifcaaaabaaaaaaaeaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaa
abaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaaabaaaaaaaeaaaaaaegacbaaa
abaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaa
abaaaaaaaeaaaaaaegacbaaaabaaaaaaaaaaaaaihcaabaaaabaaaaaaegacbaaa
abaaaaaaegiccaaaadaaaaaabdaaaaaadcaaaaalhcaabaaaabaaaaaaegacbaaa
abaaaaaapgipcaaaadaaaaaabeaaaaaaegbcbaiaebaaaaaaaaaaaaaabaaaaaah
cccabaaaacaaaaaaegacbaaaaaaaaaaaegacbaaaabaaaaaabaaaaaahbccabaaa
acaaaaaaegbcbaaaabaaaaaaegacbaaaabaaaaaabaaaaaaheccabaaaacaaaaaa
egbcbaaaacaaaaaaegacbaaaabaaaaaadiaaaaajhcaabaaaabaaaaaafgifcaaa
acaaaaaaaaaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaaabaaaaaa
egiccaaaadaaaaaabaaaaaaaagiacaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaa
dcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaaacaaaaaa
aaaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaa
bdaaaaaapgipcaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaabaaaaaahcccabaaa
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
heaaaaaaaaaaaaaaaaaaaaaaadaaaaaaabaaaaaaapaaaaaaheaaaaaaabaaaaaa
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
Vector 21 [_BumpMap_ST]
"!!ARBvp1.0
# 35 ALU
PARAM c[22] = { { 1 },
		state.matrix.mvp,
		program.local[5..21] };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
MOV R0.xyz, vertex.attrib[14];
MUL R2.xyz, vertex.normal.zxyw, R0.yzxw;
MAD R0.xyz, vertex.normal.yzxw, R0.zxyw, -R2;
MOV R1, c[18];
MOV R0.w, c[0].x;
DP4 R2.z, R1, c[11];
DP4 R2.x, R1, c[9];
DP4 R2.y, R1, c[10];
MAD R2.xyz, R2, c[19].w, -vertex.position;
MUL R1.xyz, R0, vertex.attrib[14].w;
MOV R0.xyz, c[17];
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
MAD result.texcoord[0].zw, vertex.texcoord[0].xyxy, c[21].xyxy, c[21];
MAD result.texcoord[0].xy, vertex.texcoord[0], c[20], c[20].zwzw;
DP4 result.position.w, vertex.position, c[4];
DP4 result.position.z, vertex.position, c[3];
DP4 result.position.y, vertex.position, c[2];
DP4 result.position.x, vertex.position, c[1];
END
# 35 instructions, 4 R-regs
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
Vector 20 [_BumpMap_ST]
"vs_2_0
; 38 ALU
def c21, 1.00000000, 0, 0, 0
dcl_position0 v0
dcl_tangent0 v1
dcl_normal0 v2
dcl_texcoord0 v3
mov r1.xyz, v1
mov r0, c10
dp4 r3.z, c17, r0
mov r0, c9
dp4 r3.y, c17, r0
mul r2.xyz, v2.zxyw, r1.yzxw
mov r1.xyz, v1
mad r2.xyz, v2.yzxw, r1.zxyw, -r2
mov r1, c8
dp4 r3.x, c17, r1
mad r0.xyz, r3, c18.w, -v0
mul r2.xyz, r2, v1.w
mov r1.xyz, c16
mov r1.w, c21.x
dp4 r0.w, v0, c7
dp3 oT2.y, r2, r0
dp3 oT2.z, v2, r0
dp3 oT2.x, v1, r0
dp4 r0.z, v0, c6
dp4 r0.x, v0, c4
dp4 r0.y, v0, c5
dp4 r3.z, r1, c10
dp4 r3.x, r1, c8
dp4 r3.y, r1, c9
mad r1.xyz, r3, c18.w, -v0
dp3 oT1.y, r1, r2
dp3 oT1.z, v2, r1
dp3 oT1.x, r1, v1
dp4 oT3.w, r0, c15
dp4 oT3.z, r0, c14
dp4 oT3.y, r0, c13
dp4 oT3.x, r0, c12
mad oT0.zw, v3.xyxy, c20.xyxy, c20
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
Vector 20 [_BumpMap_ST]
Matrix 15 [_LightMatrix0] 4
Vector 19 [_MainTex_ST]
Matrix 6 [_Object2World] 4
Matrix 10 [_World2Object] 4
Vector 0 [_WorldSpaceCameraPos]
Vector 1 [_WorldSpaceLightPos0]
Matrix 2 [glstate_matrix_mvp] 4
Vector 14 [unity_Scale]
// Shader Timing Estimate, in Cycles/64 vertex vector:
// ALU: 42.67 (32 instructions), vertex: 32, texture: 0,
//   sequencer: 18,  8 GPRs, 24 threads,
// Performance (if enough threads): ~42 cycles per vector
// * Vertex cycle estimates are assuming 3 vfetch_minis for every vfetch_full,
//     with <= 32 bytes per vfetch_full group.

"vs_360
backbbabaaaaacdiaaaaabpiaaaaaaaaaaaaaaceaaaaaaaaaaaaabmmaaaaaaaa
aaaaaaaaaaaaabkeaaaaaabmaaaaabjhpppoadaaaaaaaaajaaaaaabmaaaaaaaa
aaaaabjaaaaaaanaaaacaabeaaabaaaaaaaaaanmaaaaaaaaaaaaaaomaaacaaap
aaaeaaaaaaaaaapmaaaaaaaaaaaaabamaaacaabdaaabaaaaaaaaaanmaaaaaaaa
aaaaabbiaaacaaagaaaeaaaaaaaaaapmaaaaaaaaaaaaabcgaaacaaakaaaeaaaa
aaaaaapmaaaaaaaaaaaaabdeaaacaaaaaaabaaaaaaaaabemaaaaaaaaaaaaabfm
aaacaaabaaabaaaaaaaaaanmaaaaaaaaaaaaabhbaaacaaacaaaeaaaaaaaaaapm
aaaaaaaaaaaaabieaaacaaaoaaabaaaaaaaaaanmaaaaaaaafpechfgnhaengbha
fpfdfeaaaaabaaadaaabaaaeaaabaaaaaaaaaaaafpemgjghgiheengbhehcgjhi
daaaklklaaadaaadaaaeaaaeaaabaaaaaaaaaaaafpengbgjgofegfhifpfdfeaa
fpepgcgkgfgdhedcfhgphcgmgeaafpfhgphcgmgedcepgcgkgfgdheaafpfhgphc
gmgefdhagbgdgfedgbgngfhcgbfagphdaaklklklaaabaaadaaabaaadaaabaaaa
aaaaaaaafpfhgphcgmgefdhagbgdgfemgjghgihefagphddaaaghgmhdhegbhegf
fpgngbhehcgjhifpgnhghaaahfgogjhehjfpfdgdgbgmgfaahghdfpddfpdaaadc
codacodcdadddfddcodaaaklaaaaaaaaaaaaabpiaadbaaahaaaaaaaaaaaaaaaa
aaaadiieaaaaaaabaaaaaaaeaaaaaaajaaaaacjaaabaaaafaaaagaagaaaadaah
aadafaaiaaaapafaaaachbfbaaafhcfcaaaipdfdaaaaaacdaaaabaceaaaaaabn
aaaaaaboaaaababpaaaaaacaaaaaaacbaaaabaccaaaabacipaffeaafaaaabcaa
mcaaaaaaaaaaeaajaaaabcaameaaaaaaaaaagaangabdbcaabcaaaaaaaaaagabj
gabpbcaabcaaaaaaaaaaeacfaaaaccaaaaaaaaaaafpidaaaaaaaagiiaaaaaaaa
afpifaaaaaaaagiiaaaaaaaaafpicaaaaaaaaoiiaaaaaaaaafpiaaaaaaaaapmi
aaaaaaaamiapaaabaabliiaakbadafaamiapaaabaamgiiaakladaeabmiapaaab
aalbdejekladadabmiapiadoaagmaadekladacabmiahaaabaaleblaacbanabaa
miahaaagaamamgmaalamaaanmiahaaaeaalogfaaobacafaamiahaaagaalelble
clalaaagmiahaaahaamamgleclamababmiapaaabaabliiaakbadajaamiapaaab
aamgiiaakladaiabmiahaaahaalelbleclalabahmiahaaagaamagmleclakaaag
miahaaaeabgflomaolacafaemiahaaaeaamablaaobaeafaamiahaaagabmablma
klagaoadmiahaaahaamagmleclakabahmiapaaabaalbdejekladahabmiapaaab
aagmnajekladagabmiahaaadabmablmaklahaoadmiabiaabaaloloaapaagafaa
miaciaabaaloloaapaaeagaamiaeiaabaaloloaapaagacaamiabiaacaaloloaa
paadafaamiaciaacaaloloaapaaeadaamiaeiaacaaloloaapaadacaamiadiaaa
aalalabkilaabdbdmiamiaaaaakmkmagilaabebemiapaaaaaamgiiaakbabbcaa
miapaaaaaabliiaaklabbbaamiapaaaaaalbdejeklabbaaamiapiaadaagmaade
klabapaaaaaaaaaaaaaaaaaaaaaaaaaa"
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
Vector 463 [_BumpMap_ST]
"sce_vp_rsx // 34 instructions using 5 registers
[Configuration]
8
0000002241050500
[Microcode]
544
00009c6c005d200d8186c0836041fffc00019c6c00400e0c0106c0836041dffc
00011c6c005d300c0186c0836041dffc401f9c6c011cf800810040d560607f9c
401f9c6c011d0808010400d740619f9c401f9c6c01d0300d8106c0c360403f80
401f9c6c01d0200d8106c0c360405f80401f9c6c01d0100d8106c0c360409f80
401f9c6c01d0000d8106c0c360411f8000001c6c01d0700d8106c0c360403ffc
00001c6c01d0600d8106c0c360405ffc00001c6c01d0500d8106c0c360409ffc
00001c6c01d0400d8106c0c360411ffc00021c6c01d0a00d8286c0c360405ffc
00021c6c01d0900d8286c0c360409ffc00021c6c01d0800d8286c0c360411ffc
00009c6c0190a00c0486c0c360405ffc00009c6c0190900c0486c0c360409ffc
00009c6c0190800c0486c0c360411ffc00011c6c00800243011843436041dffc
00011c6c01000230812183630121dffc401f9c6c01d0f00d8086c0c360403fa8
401f9c6c01d0e00d8086c0c360405fa8401f9c6c01d0d00d8086c0c360409fa8
401f9c6c01d0c00d8086c0c360411fa800001c6c011d100c08bfc0e30041dffc
00009c6c011d100c02bfc0e30041dffc401f9c6c0140020c0106004360405fa4
401f9c6c01400e0c0106004360411fa400011c6c00800e0c04bfc0836041dffc
401f9c6c0140020c0106014360405fa0401f9c6c01400e0c0286008360411fa0
401f9c6c0140000c0486004360409fa4401f9c6c0140000c0286024360409fa1
"
}

SubProgram "d3d11 " {
Keywords { "SPOT" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 176 // 176 used size, 10 vars
Matrix 48 [_LightMatrix0] 4
Vector 144 [_MainTex_ST] 4
Vector 160 [_BumpMap_ST] 4
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
// 34 instructions, 2 temp regs, 0 temp arrays:
// ALU 14 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0
eefiecediiemhpgchhaknncnahgaeffojgfkokgeabaaaaaaceahaaaaadaaaaaa
cmaaaaaapeaaaaaajeabaaaaejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaa
abaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaljaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfc
enebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheojiaaaaaaafaaaaaa
aiaaaaaaiaaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaabaaaaaaapaaaaaaimaaaaaaabaaaaaaaaaaaaaa
adaaaaaaacaaaaaaahaiaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaa
ahaiaaaaimaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaafdfgfpfa
epfdejfeejepeoaafeeffiedepepfceeaaklklklfdeieefciiafaaaaeaaaabaa
gcabaaaafjaaaaaeegiocaaaaaaaaaaaalaaaaaafjaaaaaeegiocaaaabaaaaaa
afaaaaaafjaaaaaeegiocaaaacaaaaaaabaaaaaafjaaaaaeegiocaaaadaaaaaa
bfaaaaaafpaaaaadpcbabaaaaaaaaaaafpaaaaadpcbabaaaabaaaaaafpaaaaad
hcbabaaaacaaaaaafpaaaaaddcbabaaaadaaaaaaghaaaaaepccabaaaaaaaaaaa
abaaaaaagfaaaaadpccabaaaabaaaaaagfaaaaadhccabaaaacaaaaaagfaaaaad
hccabaaaadaaaaaagfaaaaadpccabaaaaeaaaaaagiaaaaacacaaaaaadiaaaaai
pcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaadaaaaaaabaaaaaadcaaaaak
pcaabaaaaaaaaaaaegiocaaaadaaaaaaaaaaaaaaagbabaaaaaaaaaaaegaobaaa
aaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaacaaaaaakgbkbaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaakpccabaaaaaaaaaaaegiocaaaadaaaaaa
adaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaaldccabaaaabaaaaaa
egbabaaaadaaaaaaegiacaaaaaaaaaaaajaaaaaaogikcaaaaaaaaaaaajaaaaaa
dcaaaaalmccabaaaabaaaaaaagbebaaaadaaaaaaagiecaaaaaaaaaaaakaaaaaa
kgiocaaaaaaaaaaaakaaaaaadiaaaaahhcaabaaaaaaaaaaajgbebaaaabaaaaaa
cgbjbaaaacaaaaaadcaaaaakhcaabaaaaaaaaaaajgbebaaaacaaaaaacgbjbaaa
abaaaaaaegacbaiaebaaaaaaaaaaaaaadiaaaaahhcaabaaaaaaaaaaaegacbaaa
aaaaaaaapgbpbaaaabaaaaaadiaaaaajhcaabaaaabaaaaaafgifcaaaabaaaaaa
aeaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaa
adaaaaaabaaaaaaaagiacaaaabaaaaaaaeaaaaaaegacbaaaabaaaaaadcaaaaal
hcaabaaaabaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaaabaaaaaaaeaaaaaa
egacbaaaabaaaaaaaaaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaaegiccaaa
adaaaaaabdaaaaaadcaaaaalhcaabaaaabaaaaaaegacbaaaabaaaaaapgipcaaa
adaaaaaabeaaaaaaegbcbaiaebaaaaaaaaaaaaaabaaaaaahcccabaaaacaaaaaa
egacbaaaaaaaaaaaegacbaaaabaaaaaabaaaaaahbccabaaaacaaaaaaegbcbaaa
abaaaaaaegacbaaaabaaaaaabaaaaaaheccabaaaacaaaaaaegbcbaaaacaaaaaa
egacbaaaabaaaaaadiaaaaajhcaabaaaabaaaaaafgifcaaaacaaaaaaaaaaaaaa
egiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaa
baaaaaaaagiacaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaa
abaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaaacaaaaaaaaaaaaaaegacbaaa
abaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabdaaaaaapgipcaaa
acaaaaaaaaaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegacbaaa
abaaaaaapgipcaaaadaaaaaabeaaaaaaegbcbaiaebaaaaaaaaaaaaaabaaaaaah
cccabaaaadaaaaaaegacbaaaaaaaaaaaegacbaaaabaaaaaabaaaaaahbccabaaa
adaaaaaaegbcbaaaabaaaaaaegacbaaaabaaaaaabaaaaaaheccabaaaadaaaaaa
egbcbaaaacaaaaaaegacbaaaabaaaaaadiaaaaaipcaabaaaaaaaaaaafgbfbaaa
aaaaaaaaegiocaaaadaaaaaaanaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaa
adaaaaaaamaaaaaaagbabaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaa
aaaaaaaaegiocaaaadaaaaaaaoaaaaaakgbkbaaaaaaaaaaaegaobaaaaaaaaaaa
dcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaapaaaaaapgbpbaaaaaaaaaaa
egaobaaaaaaaaaaadiaaaaaipcaabaaaabaaaaaafgafbaaaaaaaaaaaegiocaaa
aaaaaaaaaeaaaaaadcaaaaakpcaabaaaabaaaaaaegiocaaaaaaaaaaaadaaaaaa
agaabaaaaaaaaaaaegaobaaaabaaaaaadcaaaaakpcaabaaaabaaaaaaegiocaaa
aaaaaaaaafaaaaaakgakbaaaaaaaaaaaegaobaaaabaaaaaadcaaaaakpccabaaa
aeaaaaaaegiocaaaaaaaaaaaagaaaaaapgapbaaaaaaaaaaaegaobaaaabaaaaaa
doaaaaab"
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
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 _BumpMap_ST;
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
  highp vec4 tmpvar_3;
  mediump vec3 tmpvar_4;
  tmpvar_3.xy = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  tmpvar_3.zw = ((_glesMultiTexCoord0.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
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
  tmpvar_4 = tmpvar_8;
  highp vec4 tmpvar_9;
  tmpvar_9.w = 1.0;
  tmpvar_9.xyz = _WorldSpaceCameraPos;
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = tmpvar_3;
  xlv_TEXCOORD1 = (tmpvar_7 * (((_World2Object * tmpvar_9).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = tmpvar_4;
  xlv_TEXCOORD3 = (_LightMatrix0 * (_Object2World * _glesVertex));
}



#endif
#ifdef FRAGMENT

varying highp vec4 xlv_TEXCOORD3;
varying mediump vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp float _Parallax;
uniform mediump float _Gloss;
uniform mediump float _Shininess;
uniform lowp vec4 _Color;
uniform sampler2D _ParallaxMap;
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
  mediump vec4 spec_5;
  mediump float h_6;
  lowp float tmpvar_7;
  tmpvar_7 = texture2D (_ParallaxMap, xlv_TEXCOORD0.zw).w;
  h_6 = tmpvar_7;
  highp vec2 tmpvar_8;
  mediump float height_9;
  height_9 = _Parallax;
  mediump vec3 viewDir_10;
  viewDir_10 = xlv_TEXCOORD1;
  highp vec3 v_11;
  mediump float tmpvar_12;
  tmpvar_12 = ((h_6 * height_9) - (height_9 / 2.0));
  mediump vec3 tmpvar_13;
  tmpvar_13 = normalize(viewDir_10);
  v_11 = tmpvar_13;
  v_11.z = (v_11.z + 0.42);
  tmpvar_8 = (tmpvar_12 * (v_11.xy / v_11.z));
  highp vec2 tmpvar_14;
  tmpvar_14 = (xlv_TEXCOORD0.xy + tmpvar_8);
  highp vec2 tmpvar_15;
  tmpvar_15 = (xlv_TEXCOORD0.zw + tmpvar_8);
  lowp vec4 tmpvar_16;
  tmpvar_16 = texture2D (_MainTex, tmpvar_14);
  lowp vec3 tmpvar_17;
  tmpvar_17 = (tmpvar_16.xyz * _Color.xyz);
  tmpvar_3 = tmpvar_17;
  lowp vec4 tmpvar_18;
  tmpvar_18 = texture2D (_SpecMap, tmpvar_14);
  spec_5 = tmpvar_18;
  mediump vec3 tmpvar_19;
  tmpvar_19 = ((tmpvar_16.w * spec_5.xyz) * _Gloss);
  lowp vec3 tmpvar_20;
  tmpvar_20 = ((texture2D (_BumpMap, tmpvar_15).xyz * 2.0) - 1.0);
  tmpvar_4 = tmpvar_20;
  mediump vec3 tmpvar_21;
  tmpvar_21 = normalize(xlv_TEXCOORD2);
  lightDir_2 = tmpvar_21;
  highp vec3 tmpvar_22;
  tmpvar_22 = normalize(xlv_TEXCOORD1);
  lowp vec4 tmpvar_23;
  highp vec2 P_24;
  P_24 = ((xlv_TEXCOORD3.xy / xlv_TEXCOORD3.w) + 0.5);
  tmpvar_23 = texture2D (_LightTexture0, P_24);
  highp float tmpvar_25;
  tmpvar_25 = dot (xlv_TEXCOORD3.xyz, xlv_TEXCOORD3.xyz);
  lowp vec4 tmpvar_26;
  tmpvar_26 = texture2D (_LightTextureB0, vec2(tmpvar_25));
  mediump vec3 lightDir_27;
  lightDir_27 = lightDir_2;
  mediump vec3 viewDir_28;
  viewDir_28 = tmpvar_22;
  mediump float atten_29;
  atten_29 = ((float((xlv_TEXCOORD3.z > 0.0)) * tmpvar_23.w) * tmpvar_26.w);
  mediump vec4 c_30;
  mediump vec3 specCol_31;
  highp float nh_32;
  mediump float tmpvar_33;
  tmpvar_33 = max (0.0, dot (tmpvar_4, normalize((lightDir_27 + viewDir_28))));
  nh_32 = tmpvar_33;
  mediump float arg1_34;
  arg1_34 = (32.0 * _Shininess);
  highp vec3 tmpvar_35;
  tmpvar_35 = (pow (nh_32, arg1_34) * tmpvar_19);
  specCol_31 = tmpvar_35;
  c_30.xyz = ((((tmpvar_3 * _LightColor0.xyz) * max (0.0, dot (tmpvar_4, lightDir_27))) + (_LightColor0.xyz * specCol_31)) * (atten_29 * 2.0));
  c_30.w = 0.0;
  c_1.xyz = c_30.xyz;
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
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 _BumpMap_ST;
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
  highp vec4 tmpvar_3;
  mediump vec3 tmpvar_4;
  tmpvar_3.xy = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  tmpvar_3.zw = ((_glesMultiTexCoord0.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
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
  tmpvar_4 = tmpvar_8;
  highp vec4 tmpvar_9;
  tmpvar_9.w = 1.0;
  tmpvar_9.xyz = _WorldSpaceCameraPos;
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = tmpvar_3;
  xlv_TEXCOORD1 = (tmpvar_7 * (((_World2Object * tmpvar_9).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = tmpvar_4;
  xlv_TEXCOORD3 = (_LightMatrix0 * (_Object2World * _glesVertex));
}



#endif
#ifdef FRAGMENT

varying highp vec4 xlv_TEXCOORD3;
varying mediump vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp float _Parallax;
uniform mediump float _Gloss;
uniform mediump float _Shininess;
uniform lowp vec4 _Color;
uniform sampler2D _ParallaxMap;
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
  mediump vec4 spec_5;
  mediump float h_6;
  lowp float tmpvar_7;
  tmpvar_7 = texture2D (_ParallaxMap, xlv_TEXCOORD0.zw).w;
  h_6 = tmpvar_7;
  highp vec2 tmpvar_8;
  mediump float height_9;
  height_9 = _Parallax;
  mediump vec3 viewDir_10;
  viewDir_10 = xlv_TEXCOORD1;
  highp vec3 v_11;
  mediump float tmpvar_12;
  tmpvar_12 = ((h_6 * height_9) - (height_9 / 2.0));
  mediump vec3 tmpvar_13;
  tmpvar_13 = normalize(viewDir_10);
  v_11 = tmpvar_13;
  v_11.z = (v_11.z + 0.42);
  tmpvar_8 = (tmpvar_12 * (v_11.xy / v_11.z));
  highp vec2 tmpvar_14;
  tmpvar_14 = (xlv_TEXCOORD0.xy + tmpvar_8);
  highp vec2 tmpvar_15;
  tmpvar_15 = (xlv_TEXCOORD0.zw + tmpvar_8);
  lowp vec4 tmpvar_16;
  tmpvar_16 = texture2D (_MainTex, tmpvar_14);
  lowp vec3 tmpvar_17;
  tmpvar_17 = (tmpvar_16.xyz * _Color.xyz);
  tmpvar_3 = tmpvar_17;
  lowp vec4 tmpvar_18;
  tmpvar_18 = texture2D (_SpecMap, tmpvar_14);
  spec_5 = tmpvar_18;
  mediump vec3 tmpvar_19;
  tmpvar_19 = ((tmpvar_16.w * spec_5.xyz) * _Gloss);
  lowp vec3 normal_20;
  normal_20.xy = ((texture2D (_BumpMap, tmpvar_15).wy * 2.0) - 1.0);
  normal_20.z = sqrt(((1.0 - (normal_20.x * normal_20.x)) - (normal_20.y * normal_20.y)));
  tmpvar_4 = normal_20;
  mediump vec3 tmpvar_21;
  tmpvar_21 = normalize(xlv_TEXCOORD2);
  lightDir_2 = tmpvar_21;
  highp vec3 tmpvar_22;
  tmpvar_22 = normalize(xlv_TEXCOORD1);
  lowp vec4 tmpvar_23;
  highp vec2 P_24;
  P_24 = ((xlv_TEXCOORD3.xy / xlv_TEXCOORD3.w) + 0.5);
  tmpvar_23 = texture2D (_LightTexture0, P_24);
  highp float tmpvar_25;
  tmpvar_25 = dot (xlv_TEXCOORD3.xyz, xlv_TEXCOORD3.xyz);
  lowp vec4 tmpvar_26;
  tmpvar_26 = texture2D (_LightTextureB0, vec2(tmpvar_25));
  mediump vec3 lightDir_27;
  lightDir_27 = lightDir_2;
  mediump vec3 viewDir_28;
  viewDir_28 = tmpvar_22;
  mediump float atten_29;
  atten_29 = ((float((xlv_TEXCOORD3.z > 0.0)) * tmpvar_23.w) * tmpvar_26.w);
  mediump vec4 c_30;
  mediump vec3 specCol_31;
  highp float nh_32;
  mediump float tmpvar_33;
  tmpvar_33 = max (0.0, dot (tmpvar_4, normalize((lightDir_27 + viewDir_28))));
  nh_32 = tmpvar_33;
  mediump float arg1_34;
  arg1_34 = (32.0 * _Shininess);
  highp vec3 tmpvar_35;
  tmpvar_35 = (pow (nh_32, arg1_34) * tmpvar_19);
  specCol_31 = tmpvar_35;
  c_30.xyz = ((((tmpvar_3 * _LightColor0.xyz) * max (0.0, dot (tmpvar_4, lightDir_27))) + (_LightColor0.xyz * specCol_31)) * (atten_29 * 2.0));
  c_30.w = 0.0;
  c_1.xyz = c_30.xyz;
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
Vector 20 [_BumpMap_ST]
"agal_vs
c21 1.0 0.0 0.0 0.0
[bc]
aaaaaaaaabaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r1.xyz, a5
aaaaaaaaaaaaapacakaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0, c10
bdaaaaaaadaaaeacbbaaaaoeabaaaaaaaaaaaaoeacaaaaaa dp4 r3.z, c17, r0
aaaaaaaaaaaaapacajaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0, c9
bdaaaaaaadaaacacbbaaaaoeabaaaaaaaaaaaaoeacaaaaaa dp4 r3.y, c17, r0
adaaaaaaacaaahacabaaaancaaaaaaaaabaaaaajacaaaaaa mul r2.xyz, a1.zxyw, r1.yzxx
aaaaaaaaabaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r1.xyz, a5
adaaaaaaaeaaahacabaaaamjaaaaaaaaabaaaafcacaaaaaa mul r4.xyz, a1.yzxw, r1.zxyy
acaaaaaaacaaahacaeaaaakeacaaaaaaacaaaakeacaaaaaa sub r2.xyz, r4.xyzz, r2.xyzz
aaaaaaaaabaaapacaiaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r1, c8
bdaaaaaaadaaabacbbaaaaoeabaaaaaaabaaaaoeacaaaaaa dp4 r3.x, c17, r1
adaaaaaaaeaaahacadaaaakeacaaaaaabcaaaappabaaaaaa mul r4.xyz, r3.xyzz, c18.w
acaaaaaaaaaaahacaeaaaakeacaaaaaaaaaaaaoeaaaaaaaa sub r0.xyz, r4.xyzz, a0
adaaaaaaacaaahacacaaaakeacaaaaaaafaaaappaaaaaaaa mul r2.xyz, r2.xyzz, a5.w
aaaaaaaaabaaahacbaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r1.xyz, c16
aaaaaaaaabaaaiacbfaaaaaaabaaaaaaaaaaaaaaaaaaaaaa mov r1.w, c21.x
bdaaaaaaaaaaaiacaaaaaaoeaaaaaaaaahaaaaoeabaaaaaa dp4 r0.w, a0, c7
bcaaaaaaacaaacaeacaaaakeacaaaaaaaaaaaakeacaaaaaa dp3 v2.y, r2.xyzz, r0.xyzz
bcaaaaaaacaaaeaeabaaaaoeaaaaaaaaaaaaaakeacaaaaaa dp3 v2.z, a1, r0.xyzz
bcaaaaaaacaaabaeafaaaaoeaaaaaaaaaaaaaakeacaaaaaa dp3 v2.x, a5, r0.xyzz
bdaaaaaaaaaaaeacaaaaaaoeaaaaaaaaagaaaaoeabaaaaaa dp4 r0.z, a0, c6
bdaaaaaaaaaaabacaaaaaaoeaaaaaaaaaeaaaaoeabaaaaaa dp4 r0.x, a0, c4
bdaaaaaaaaaaacacaaaaaaoeaaaaaaaaafaaaaoeabaaaaaa dp4 r0.y, a0, c5
bdaaaaaaadaaaeacabaaaaoeacaaaaaaakaaaaoeabaaaaaa dp4 r3.z, r1, c10
bdaaaaaaadaaabacabaaaaoeacaaaaaaaiaaaaoeabaaaaaa dp4 r3.x, r1, c8
bdaaaaaaadaaacacabaaaaoeacaaaaaaajaaaaoeabaaaaaa dp4 r3.y, r1, c9
adaaaaaaaeaaahacadaaaakeacaaaaaabcaaaappabaaaaaa mul r4.xyz, r3.xyzz, c18.w
acaaaaaaabaaahacaeaaaakeacaaaaaaaaaaaaoeaaaaaaaa sub r1.xyz, r4.xyzz, a0
bcaaaaaaabaaacaeabaaaakeacaaaaaaacaaaakeacaaaaaa dp3 v1.y, r1.xyzz, r2.xyzz
bcaaaaaaabaaaeaeabaaaaoeaaaaaaaaabaaaakeacaaaaaa dp3 v1.z, a1, r1.xyzz
bcaaaaaaabaaabaeabaaaakeacaaaaaaafaaaaoeaaaaaaaa dp3 v1.x, r1.xyzz, a5
bdaaaaaaadaaaiaeaaaaaaoeacaaaaaaapaaaaoeabaaaaaa dp4 v3.w, r0, c15
bdaaaaaaadaaaeaeaaaaaaoeacaaaaaaaoaaaaoeabaaaaaa dp4 v3.z, r0, c14
bdaaaaaaadaaacaeaaaaaaoeacaaaaaaanaaaaoeabaaaaaa dp4 v3.y, r0, c13
bdaaaaaaadaaabaeaaaaaaoeacaaaaaaamaaaaoeabaaaaaa dp4 v3.x, r0, c12
adaaaaaaaeaaamacadaaaaeeaaaaaaaabeaaaaeeabaaaaaa mul r4.zw, a3.xyxy, c20.xyxy
abaaaaaaaaaaamaeaeaaaaopacaaaaaabeaaaaoeabaaaaaa add v0.zw, r4.wwzw, c20
adaaaaaaaeaaadacadaaaaoeaaaaaaaabdaaaaoeabaaaaaa mul r4.xy, a3, c19
abaaaaaaaaaaadaeaeaaaafeacaaaaaabdaaaaooabaaaaaa add v0.xy, r4.xyyy, c19.zwzw
bdaaaaaaaaaaaiadaaaaaaoeaaaaaaaaadaaaaoeabaaaaaa dp4 o0.w, a0, c3
bdaaaaaaaaaaaeadaaaaaaoeaaaaaaaaacaaaaoeabaaaaaa dp4 o0.z, a0, c2
bdaaaaaaaaaaacadaaaaaaoeaaaaaaaaabaaaaoeabaaaaaa dp4 o0.y, a0, c1
bdaaaaaaaaaaabadaaaaaaoeaaaaaaaaaaaaaaoeabaaaaaa dp4 o0.x, a0, c0
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
ConstBuffer "$Globals" 176 // 176 used size, 10 vars
Matrix 48 [_LightMatrix0] 4
Vector 144 [_MainTex_ST] 4
Vector 160 [_BumpMap_ST] 4
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
// 34 instructions, 2 temp regs, 0 temp arrays:
// ALU 14 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0_level_9_3
eefieceddebgdeijmckkcihegagpipfkdldhjdgcabaaaaaahiakaaaaaeaaaaaa
daaaaaaaiaadaaaabaajaaaaniajaaaaebgpgodjeiadaaaaeiadaaaaaaacpopp
niacaaaahaaaaaaaagaaceaaaaaagmaaaaaagmaaaaaaceaaabaagmaaaaaaadaa
aeaaabaaaaaaaaaaaaaaajaaacaaafaaaaaaaaaaabaaaeaaabaaahaaaaaaaaaa
acaaaaaaabaaaiaaaaaaaaaaadaaaaaaaeaaajaaaaaaaaaaadaaamaaajaaanaa
aaaaaaaaaaaaaaaaabacpoppbpaaaaacafaaaaiaaaaaapjabpaaaaacafaaabia
abaaapjabpaaaaacafaaaciaacaaapjabpaaaaacafaaadiaadaaapjaaeaaaaae
aaaaadoaadaaoejaafaaoekaafaaookaaeaaaaaeaaaaamoaadaaeejaagaaeeka
agaaoekaabaaaaacaaaaapiaaiaaoekaafaaaaadabaaahiaaaaaffiabcaaoeka
aeaaaaaeabaaahiabbaaoekaaaaaaaiaabaaoeiaaeaaaaaeaaaaahiabdaaoeka
aaaakkiaabaaoeiaaeaaaaaeaaaaahiabeaaoekaaaaappiaaaaaoeiaaeaaaaae
aaaaahiaaaaaoeiabfaappkaaaaaoejbaiaaaaadacaaaboaabaaoejaaaaaoeia
abaaaaacabaaahiaabaaoejaafaaaaadacaaahiaabaamjiaacaancjaaeaaaaae
abaaahiaacaamjjaabaanciaacaaoeibafaaaaadabaaahiaabaaoeiaabaappja
aiaaaaadacaaacoaabaaoeiaaaaaoeiaaiaaaaadacaaaeoaacaaoejaaaaaoeia
abaaaaacaaaaahiaahaaoekaafaaaaadacaaahiaaaaaffiabcaaoekaaeaaaaae
aaaaaliabbaakekaaaaaaaiaacaakeiaaeaaaaaeaaaaahiabdaaoekaaaaakkia
aaaapeiaacaaaaadaaaaahiaaaaaoeiabeaaoekaaeaaaaaeaaaaahiaaaaaoeia
bfaappkaaaaaoejbaiaaaaadabaaaboaabaaoejaaaaaoeiaaiaaaaadabaaacoa
abaaoeiaaaaaoeiaaiaaaaadabaaaeoaacaaoejaaaaaoeiaafaaaaadaaaaapia
aaaaffjaaoaaoekaaeaaaaaeaaaaapiaanaaoekaaaaaaajaaaaaoeiaaeaaaaae
aaaaapiaapaaoekaaaaakkjaaaaaoeiaaeaaaaaeaaaaapiabaaaoekaaaaappja
aaaaoeiaafaaaaadabaaapiaaaaaffiaacaaoekaaeaaaaaeabaaapiaabaaoeka
aaaaaaiaabaaoeiaaeaaaaaeabaaapiaadaaoekaaaaakkiaabaaoeiaaeaaaaae
adaaapoaaeaaoekaaaaappiaabaaoeiaafaaaaadaaaaapiaaaaaffjaakaaoeka
aeaaaaaeaaaaapiaajaaoekaaaaaaajaaaaaoeiaaeaaaaaeaaaaapiaalaaoeka
aaaakkjaaaaaoeiaaeaaaaaeaaaaapiaamaaoekaaaaappjaaaaaoeiaaeaaaaae
aaaaadmaaaaappiaaaaaoekaaaaaoeiaabaaaaacaaaaammaaaaaoeiappppaaaa
fdeieefciiafaaaaeaaaabaagcabaaaafjaaaaaeegiocaaaaaaaaaaaalaaaaaa
fjaaaaaeegiocaaaabaaaaaaafaaaaaafjaaaaaeegiocaaaacaaaaaaabaaaaaa
fjaaaaaeegiocaaaadaaaaaabfaaaaaafpaaaaadpcbabaaaaaaaaaaafpaaaaad
pcbabaaaabaaaaaafpaaaaadhcbabaaaacaaaaaafpaaaaaddcbabaaaadaaaaaa
ghaaaaaepccabaaaaaaaaaaaabaaaaaagfaaaaadpccabaaaabaaaaaagfaaaaad
hccabaaaacaaaaaagfaaaaadhccabaaaadaaaaaagfaaaaadpccabaaaaeaaaaaa
giaaaaacacaaaaaadiaaaaaipcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaa
adaaaaaaabaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaaaaaaaaa
agbabaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaa
adaaaaaaacaaaaaakgbkbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpccabaaa
aaaaaaaaegiocaaaadaaaaaaadaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaa
dcaaaaaldccabaaaabaaaaaaegbabaaaadaaaaaaegiacaaaaaaaaaaaajaaaaaa
ogikcaaaaaaaaaaaajaaaaaadcaaaaalmccabaaaabaaaaaaagbebaaaadaaaaaa
agiecaaaaaaaaaaaakaaaaaakgiocaaaaaaaaaaaakaaaaaadiaaaaahhcaabaaa
aaaaaaaajgbebaaaabaaaaaacgbjbaaaacaaaaaadcaaaaakhcaabaaaaaaaaaaa
jgbebaaaacaaaaaacgbjbaaaabaaaaaaegacbaiaebaaaaaaaaaaaaaadiaaaaah
hcaabaaaaaaaaaaaegacbaaaaaaaaaaapgbpbaaaabaaaaaadiaaaaajhcaabaaa
abaaaaaafgifcaaaabaaaaaaaeaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaal
hcaabaaaabaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaaabaaaaaaaeaaaaaa
egacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabcaaaaaa
kgikcaaaabaaaaaaaeaaaaaaegacbaaaabaaaaaaaaaaaaaihcaabaaaabaaaaaa
egacbaaaabaaaaaaegiccaaaadaaaaaabdaaaaaadcaaaaalhcaabaaaabaaaaaa
egacbaaaabaaaaaapgipcaaaadaaaaaabeaaaaaaegbcbaiaebaaaaaaaaaaaaaa
baaaaaahcccabaaaacaaaaaaegacbaaaaaaaaaaaegacbaaaabaaaaaabaaaaaah
bccabaaaacaaaaaaegbcbaaaabaaaaaaegacbaaaabaaaaaabaaaaaaheccabaaa
acaaaaaaegbcbaaaacaaaaaaegacbaaaabaaaaaadiaaaaajhcaabaaaabaaaaaa
fgifcaaaacaaaaaaaaaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaa
abaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaaacaaaaaaaaaaaaaaegacbaaa
abaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaa
acaaaaaaaaaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaa
adaaaaaabdaaaaaapgipcaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaadcaaaaal
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
imaaaaaaaaaaaaaaaaaaaaaaadaaaaaaabaaaaaaapaaaaaaimaaaaaaabaaaaaa
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
Vector 21 [_BumpMap_ST]
"!!ARBvp1.0
# 34 ALU
PARAM c[22] = { { 1 },
		state.matrix.mvp,
		program.local[5..21] };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
MOV R0.xyz, vertex.attrib[14];
MUL R2.xyz, vertex.normal.zxyw, R0.yzxw;
MAD R0.xyz, vertex.normal.yzxw, R0.zxyw, -R2;
MOV R1, c[18];
MOV R0.w, c[0].x;
DP4 R2.z, R1, c[11];
DP4 R2.x, R1, c[9];
DP4 R2.y, R1, c[10];
MAD R2.xyz, R2, c[19].w, -vertex.position;
MUL R1.xyz, R0, vertex.attrib[14].w;
MOV R0.xyz, c[17];
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
MAD result.texcoord[0].zw, vertex.texcoord[0].xyxy, c[21].xyxy, c[21];
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
Vector 20 [_BumpMap_ST]
"vs_2_0
; 37 ALU
def c21, 1.00000000, 0, 0, 0
dcl_position0 v0
dcl_tangent0 v1
dcl_normal0 v2
dcl_texcoord0 v3
mov r1.xyz, v1
mov r0, c10
dp4 r3.z, c17, r0
mov r0, c9
dp4 r3.y, c17, r0
mul r2.xyz, v2.zxyw, r1.yzxw
mov r1.xyz, v1
mad r2.xyz, v2.yzxw, r1.zxyw, -r2
mov r1, c8
dp4 r3.x, c17, r1
mad r0.xyz, r3, c18.w, -v0
mul r2.xyz, r2, v1.w
mov r1.xyz, c16
mov r1.w, c21.x
dp3 oT2.y, r2, r0
dp3 oT2.z, v2, r0
dp3 oT2.x, v1, r0
dp4 r0.w, v0, c7
dp4 r0.z, v0, c6
dp4 r0.x, v0, c4
dp4 r0.y, v0, c5
dp4 r3.z, r1, c10
dp4 r3.x, r1, c8
dp4 r3.y, r1, c9
mad r1.xyz, r3, c18.w, -v0
dp3 oT1.y, r1, r2
dp3 oT1.z, v2, r1
dp3 oT1.x, r1, v1
dp4 oT3.z, r0, c14
dp4 oT3.y, r0, c13
dp4 oT3.x, r0, c12
mad oT0.zw, v3.xyxy, c20.xyxy, c20
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
Vector 20 [_BumpMap_ST]
Matrix 15 [_LightMatrix0] 4
Vector 19 [_MainTex_ST]
Matrix 6 [_Object2World] 4
Matrix 10 [_World2Object] 4
Vector 0 [_WorldSpaceCameraPos]
Vector 1 [_WorldSpaceLightPos0]
Matrix 2 [glstate_matrix_mvp] 4
Vector 14 [unity_Scale]
// Shader Timing Estimate, in Cycles/64 vertex vector:
// ALU: 42.67 (32 instructions), vertex: 32, texture: 0,
//   sequencer: 18,  8 GPRs, 24 threads,
// Performance (if enough threads): ~42 cycles per vector
// * Vertex cycle estimates are assuming 3 vfetch_minis for every vfetch_full,
//     with <= 32 bytes per vfetch_full group.

"vs_360
backbbabaaaaacdiaaaaabpiaaaaaaaaaaaaaaceaaaaaaaaaaaaabmmaaaaaaaa
aaaaaaaaaaaaabkeaaaaaabmaaaaabjhpppoadaaaaaaaaajaaaaaabmaaaaaaaa
aaaaabjaaaaaaanaaaacaabeaaabaaaaaaaaaanmaaaaaaaaaaaaaaomaaacaaap
aaaeaaaaaaaaaapmaaaaaaaaaaaaabamaaacaabdaaabaaaaaaaaaanmaaaaaaaa
aaaaabbiaaacaaagaaaeaaaaaaaaaapmaaaaaaaaaaaaabcgaaacaaakaaaeaaaa
aaaaaapmaaaaaaaaaaaaabdeaaacaaaaaaabaaaaaaaaabemaaaaaaaaaaaaabfm
aaacaaabaaabaaaaaaaaaanmaaaaaaaaaaaaabhbaaacaaacaaaeaaaaaaaaaapm
aaaaaaaaaaaaabieaaacaaaoaaabaaaaaaaaaanmaaaaaaaafpechfgnhaengbha
fpfdfeaaaaabaaadaaabaaaeaaabaaaaaaaaaaaafpemgjghgiheengbhehcgjhi
daaaklklaaadaaadaaaeaaaeaaabaaaaaaaaaaaafpengbgjgofegfhifpfdfeaa
fpepgcgkgfgdhedcfhgphcgmgeaafpfhgphcgmgedcepgcgkgfgdheaafpfhgphc
gmgefdhagbgdgfedgbgngfhcgbfagphdaaklklklaaabaaadaaabaaadaaabaaaa
aaaaaaaafpfhgphcgmgefdhagbgdgfemgjghgihefagphddaaaghgmhdhegbhegf
fpgngbhehcgjhifpgnhghaaahfgogjhehjfpfdgdgbgmgfaahghdfpddfpdaaadc
codacodcdadddfddcodaaaklaaaaaaaaaaaaabpiaadbaaahaaaaaaaaaaaaaaaa
aaaadeieaaaaaaabaaaaaaaeaaaaaaajaaaaacjaaabaaaafaaaagaagaaaadaah
aacafaaiaaaapafaaaachbfbaaafhcfcaaaihdfdaaaaaacdaaaabaceaaaaaabn
aaaaaaboaaaababpaaaaaacaaaaaaacbaaaabaccaaaabacipaffeaafaaaabcaa
mcaaaaaaaaaaeaajaaaabcaameaaaaaaaaaagaangabdbcaabcaaaaaaaaaagabj
gabpbcaabcaaaaaaaaaaeacfaaaaccaaaaaaaaaaafpidaaaaaaaagiiaaaaaaaa
afpifaaaaaaaagiiaaaaaaaaafpicaaaaaaaaoiiaaaaaaaaafpibaaaaaaaapmi
aaaaaaaamiapaaaaaabliiaakbadafaamiapaaaaaamgiiaakladaeaamiapaaaa
aalbdejekladadaamiapiadoaagmaadekladacaamiahaaaaaaleblaacbanabaa
miahaaagaamamgmaalamaaanmiahaaaeaalogfaaobacafaamiahaaagaalelble
clalaaagmiahaaahaamamgleclamabaamiapaaaaaabliiaakbadajaamiapaaaa
aamgiiaakladaiaamiahaaahaalelbleclalabahmiahaaagaamagmleclakaaag
miahaaaeabgflomaolacafaemiahaaaeaamablaaobaeafaamiahaaagabmablma
klagaoadmiahaaahaamagmleclakabahmiapaaaaaalbdejekladahaamiapaaaa
aagmejhkkladagaamiahaaadabmablmaklahaoadmiabiaabaaloloaapaagafaa
miaciaabaaloloaapaaeagaamiaeiaabaaloloaapaagacaamiabiaacaaloloaa
paadafaamiaciaacaaloloaapaaeadaamiaeiaacaaloloaapaadacaamiadiaaa
aalalabkilabbdbdmiamiaaaaakmkmagilabbebemiahaaabaalbleaakbaabcaa
miahaaabaamgmaleklaabbabmiahaaaaaagmleleklaabaabmiahiaadaablmale
klaaapaaaaaaaaaaaaaaaaaaaaaaaaaa"
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
Vector 463 [_BumpMap_ST]
"sce_vp_rsx // 33 instructions using 5 registers
[Configuration]
8
0000002141050500
[Microcode]
528
00009c6c005d200d8186c0836041fffc00019c6c00400e0c0106c0836041dffc
00011c6c005d300c0186c0836041dffc401f9c6c011cf800810040d560607f9c
401f9c6c011d0808010400d740619f9c401f9c6c01d0300d8106c0c360403f80
401f9c6c01d0200d8106c0c360405f80401f9c6c01d0100d8106c0c360409f80
401f9c6c01d0000d8106c0c360411f8000001c6c01d0700d8106c0c360403ffc
00001c6c01d0600d8106c0c360405ffc00001c6c01d0500d8106c0c360409ffc
00001c6c01d0400d8106c0c360411ffc00021c6c01d0a00d8286c0c360405ffc
00021c6c01d0900d8286c0c360409ffc00021c6c01d0800d8286c0c360411ffc
00009c6c0190a00c0486c0c360405ffc00009c6c0190900c0486c0c360409ffc
00009c6c0190800c0486c0c360411ffc00011c6c00800243011843436041dffc
00011c6c01000230812183630121dffc401f9c6c01d0e00d8086c0c360405fa8
401f9c6c01d0d00d8086c0c360409fa8401f9c6c01d0c00d8086c0c360411fa8
00001c6c011d100c08bfc0e30041dffc00009c6c011d100c02bfc0e30041dffc
401f9c6c0140020c0106004360405fa4401f9c6c01400e0c0106004360411fa4
00011c6c00800e0c04bfc0836041dffc401f9c6c0140020c0106014360405fa0
401f9c6c01400e0c0286008360411fa0401f9c6c0140000c0486004360409fa4
401f9c6c0140000c0286024360409fa1
"
}

SubProgram "d3d11 " {
Keywords { "POINT_COOKIE" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 176 // 176 used size, 10 vars
Matrix 48 [_LightMatrix0] 4
Vector 144 [_MainTex_ST] 4
Vector 160 [_BumpMap_ST] 4
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
// 34 instructions, 2 temp regs, 0 temp arrays:
// ALU 14 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0
eefiecedipgdcampblaklmipimpabbddhdnfeapfabaaaaaaceahaaaaadaaaaaa
cmaaaaaapeaaaaaajeabaaaaejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaa
abaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaljaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfc
enebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheojiaaaaaaafaaaaaa
aiaaaaaaiaaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaabaaaaaaapaaaaaaimaaaaaaabaaaaaaaaaaaaaa
adaaaaaaacaaaaaaahaiaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaa
ahaiaaaaimaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaahaiaaaafdfgfpfa
epfdejfeejepeoaafeeffiedepepfceeaaklklklfdeieefciiafaaaaeaaaabaa
gcabaaaafjaaaaaeegiocaaaaaaaaaaaalaaaaaafjaaaaaeegiocaaaabaaaaaa
afaaaaaafjaaaaaeegiocaaaacaaaaaaabaaaaaafjaaaaaeegiocaaaadaaaaaa
bfaaaaaafpaaaaadpcbabaaaaaaaaaaafpaaaaadpcbabaaaabaaaaaafpaaaaad
hcbabaaaacaaaaaafpaaaaaddcbabaaaadaaaaaaghaaaaaepccabaaaaaaaaaaa
abaaaaaagfaaaaadpccabaaaabaaaaaagfaaaaadhccabaaaacaaaaaagfaaaaad
hccabaaaadaaaaaagfaaaaadhccabaaaaeaaaaaagiaaaaacacaaaaaadiaaaaai
pcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaadaaaaaaabaaaaaadcaaaaak
pcaabaaaaaaaaaaaegiocaaaadaaaaaaaaaaaaaaagbabaaaaaaaaaaaegaobaaa
aaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaacaaaaaakgbkbaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaakpccabaaaaaaaaaaaegiocaaaadaaaaaa
adaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaaldccabaaaabaaaaaa
egbabaaaadaaaaaaegiacaaaaaaaaaaaajaaaaaaogikcaaaaaaaaaaaajaaaaaa
dcaaaaalmccabaaaabaaaaaaagbebaaaadaaaaaaagiecaaaaaaaaaaaakaaaaaa
kgiocaaaaaaaaaaaakaaaaaadiaaaaahhcaabaaaaaaaaaaajgbebaaaabaaaaaa
cgbjbaaaacaaaaaadcaaaaakhcaabaaaaaaaaaaajgbebaaaacaaaaaacgbjbaaa
abaaaaaaegacbaiaebaaaaaaaaaaaaaadiaaaaahhcaabaaaaaaaaaaaegacbaaa
aaaaaaaapgbpbaaaabaaaaaadiaaaaajhcaabaaaabaaaaaafgifcaaaabaaaaaa
aeaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaa
adaaaaaabaaaaaaaagiacaaaabaaaaaaaeaaaaaaegacbaaaabaaaaaadcaaaaal
hcaabaaaabaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaaabaaaaaaaeaaaaaa
egacbaaaabaaaaaaaaaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaaegiccaaa
adaaaaaabdaaaaaadcaaaaalhcaabaaaabaaaaaaegacbaaaabaaaaaapgipcaaa
adaaaaaabeaaaaaaegbcbaiaebaaaaaaaaaaaaaabaaaaaahcccabaaaacaaaaaa
egacbaaaaaaaaaaaegacbaaaabaaaaaabaaaaaahbccabaaaacaaaaaaegbcbaaa
abaaaaaaegacbaaaabaaaaaabaaaaaaheccabaaaacaaaaaaegbcbaaaacaaaaaa
egacbaaaabaaaaaadiaaaaajhcaabaaaabaaaaaafgifcaaaacaaaaaaaaaaaaaa
egiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaa
baaaaaaaagiacaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaa
abaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaaacaaaaaaaaaaaaaaegacbaaa
abaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabdaaaaaapgipcaaa
acaaaaaaaaaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegacbaaa
abaaaaaapgipcaaaadaaaaaabeaaaaaaegbcbaiaebaaaaaaaaaaaaaabaaaaaah
cccabaaaadaaaaaaegacbaaaaaaaaaaaegacbaaaabaaaaaabaaaaaahbccabaaa
adaaaaaaegbcbaaaabaaaaaaegacbaaaabaaaaaabaaaaaaheccabaaaadaaaaaa
egbcbaaaacaaaaaaegacbaaaabaaaaaadiaaaaaipcaabaaaaaaaaaaafgbfbaaa
aaaaaaaaegiocaaaadaaaaaaanaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaa
adaaaaaaamaaaaaaagbabaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaa
aaaaaaaaegiocaaaadaaaaaaaoaaaaaakgbkbaaaaaaaaaaaegaobaaaaaaaaaaa
dcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaapaaaaaapgbpbaaaaaaaaaaa
egaobaaaaaaaaaaadiaaaaaihcaabaaaabaaaaaafgafbaaaaaaaaaaaegiccaaa
aaaaaaaaaeaaaaaadcaaaaakhcaabaaaabaaaaaaegiccaaaaaaaaaaaadaaaaaa
agaabaaaaaaaaaaaegacbaaaabaaaaaadcaaaaakhcaabaaaaaaaaaaaegiccaaa
aaaaaaaaafaaaaaakgakbaaaaaaaaaaaegacbaaaabaaaaaadcaaaaakhccabaaa
aeaaaaaaegiccaaaaaaaaaaaagaaaaaapgapbaaaaaaaaaaaegacbaaaaaaaaaaa
doaaaaab"
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
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 _BumpMap_ST;
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
  highp vec4 tmpvar_3;
  mediump vec3 tmpvar_4;
  tmpvar_3.xy = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  tmpvar_3.zw = ((_glesMultiTexCoord0.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
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
  tmpvar_4 = tmpvar_8;
  highp vec4 tmpvar_9;
  tmpvar_9.w = 1.0;
  tmpvar_9.xyz = _WorldSpaceCameraPos;
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = tmpvar_3;
  xlv_TEXCOORD1 = (tmpvar_7 * (((_World2Object * tmpvar_9).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = tmpvar_4;
  xlv_TEXCOORD3 = (_LightMatrix0 * (_Object2World * _glesVertex)).xyz;
}



#endif
#ifdef FRAGMENT

varying highp vec3 xlv_TEXCOORD3;
varying mediump vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp float _Parallax;
uniform mediump float _Gloss;
uniform mediump float _Shininess;
uniform lowp vec4 _Color;
uniform sampler2D _ParallaxMap;
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
  mediump vec4 spec_5;
  mediump float h_6;
  lowp float tmpvar_7;
  tmpvar_7 = texture2D (_ParallaxMap, xlv_TEXCOORD0.zw).w;
  h_6 = tmpvar_7;
  highp vec2 tmpvar_8;
  mediump float height_9;
  height_9 = _Parallax;
  mediump vec3 viewDir_10;
  viewDir_10 = xlv_TEXCOORD1;
  highp vec3 v_11;
  mediump float tmpvar_12;
  tmpvar_12 = ((h_6 * height_9) - (height_9 / 2.0));
  mediump vec3 tmpvar_13;
  tmpvar_13 = normalize(viewDir_10);
  v_11 = tmpvar_13;
  v_11.z = (v_11.z + 0.42);
  tmpvar_8 = (tmpvar_12 * (v_11.xy / v_11.z));
  highp vec2 tmpvar_14;
  tmpvar_14 = (xlv_TEXCOORD0.xy + tmpvar_8);
  highp vec2 tmpvar_15;
  tmpvar_15 = (xlv_TEXCOORD0.zw + tmpvar_8);
  lowp vec4 tmpvar_16;
  tmpvar_16 = texture2D (_MainTex, tmpvar_14);
  lowp vec3 tmpvar_17;
  tmpvar_17 = (tmpvar_16.xyz * _Color.xyz);
  tmpvar_3 = tmpvar_17;
  lowp vec4 tmpvar_18;
  tmpvar_18 = texture2D (_SpecMap, tmpvar_14);
  spec_5 = tmpvar_18;
  mediump vec3 tmpvar_19;
  tmpvar_19 = ((tmpvar_16.w * spec_5.xyz) * _Gloss);
  lowp vec3 tmpvar_20;
  tmpvar_20 = ((texture2D (_BumpMap, tmpvar_15).xyz * 2.0) - 1.0);
  tmpvar_4 = tmpvar_20;
  mediump vec3 tmpvar_21;
  tmpvar_21 = normalize(xlv_TEXCOORD2);
  lightDir_2 = tmpvar_21;
  highp vec3 tmpvar_22;
  tmpvar_22 = normalize(xlv_TEXCOORD1);
  highp float tmpvar_23;
  tmpvar_23 = dot (xlv_TEXCOORD3, xlv_TEXCOORD3);
  lowp vec4 tmpvar_24;
  tmpvar_24 = texture2D (_LightTextureB0, vec2(tmpvar_23));
  lowp vec4 tmpvar_25;
  tmpvar_25 = textureCube (_LightTexture0, xlv_TEXCOORD3);
  mediump vec3 lightDir_26;
  lightDir_26 = lightDir_2;
  mediump vec3 viewDir_27;
  viewDir_27 = tmpvar_22;
  mediump float atten_28;
  atten_28 = (tmpvar_24.w * tmpvar_25.w);
  mediump vec4 c_29;
  mediump vec3 specCol_30;
  highp float nh_31;
  mediump float tmpvar_32;
  tmpvar_32 = max (0.0, dot (tmpvar_4, normalize((lightDir_26 + viewDir_27))));
  nh_31 = tmpvar_32;
  mediump float arg1_33;
  arg1_33 = (32.0 * _Shininess);
  highp vec3 tmpvar_34;
  tmpvar_34 = (pow (nh_31, arg1_33) * tmpvar_19);
  specCol_30 = tmpvar_34;
  c_29.xyz = ((((tmpvar_3 * _LightColor0.xyz) * max (0.0, dot (tmpvar_4, lightDir_26))) + (_LightColor0.xyz * specCol_30)) * (atten_28 * 2.0));
  c_29.w = 0.0;
  c_1.xyz = c_29.xyz;
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
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 _BumpMap_ST;
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
  highp vec4 tmpvar_3;
  mediump vec3 tmpvar_4;
  tmpvar_3.xy = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  tmpvar_3.zw = ((_glesMultiTexCoord0.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
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
  tmpvar_4 = tmpvar_8;
  highp vec4 tmpvar_9;
  tmpvar_9.w = 1.0;
  tmpvar_9.xyz = _WorldSpaceCameraPos;
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = tmpvar_3;
  xlv_TEXCOORD1 = (tmpvar_7 * (((_World2Object * tmpvar_9).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = tmpvar_4;
  xlv_TEXCOORD3 = (_LightMatrix0 * (_Object2World * _glesVertex)).xyz;
}



#endif
#ifdef FRAGMENT

varying highp vec3 xlv_TEXCOORD3;
varying mediump vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp float _Parallax;
uniform mediump float _Gloss;
uniform mediump float _Shininess;
uniform lowp vec4 _Color;
uniform sampler2D _ParallaxMap;
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
  mediump vec4 spec_5;
  mediump float h_6;
  lowp float tmpvar_7;
  tmpvar_7 = texture2D (_ParallaxMap, xlv_TEXCOORD0.zw).w;
  h_6 = tmpvar_7;
  highp vec2 tmpvar_8;
  mediump float height_9;
  height_9 = _Parallax;
  mediump vec3 viewDir_10;
  viewDir_10 = xlv_TEXCOORD1;
  highp vec3 v_11;
  mediump float tmpvar_12;
  tmpvar_12 = ((h_6 * height_9) - (height_9 / 2.0));
  mediump vec3 tmpvar_13;
  tmpvar_13 = normalize(viewDir_10);
  v_11 = tmpvar_13;
  v_11.z = (v_11.z + 0.42);
  tmpvar_8 = (tmpvar_12 * (v_11.xy / v_11.z));
  highp vec2 tmpvar_14;
  tmpvar_14 = (xlv_TEXCOORD0.xy + tmpvar_8);
  highp vec2 tmpvar_15;
  tmpvar_15 = (xlv_TEXCOORD0.zw + tmpvar_8);
  lowp vec4 tmpvar_16;
  tmpvar_16 = texture2D (_MainTex, tmpvar_14);
  lowp vec3 tmpvar_17;
  tmpvar_17 = (tmpvar_16.xyz * _Color.xyz);
  tmpvar_3 = tmpvar_17;
  lowp vec4 tmpvar_18;
  tmpvar_18 = texture2D (_SpecMap, tmpvar_14);
  spec_5 = tmpvar_18;
  mediump vec3 tmpvar_19;
  tmpvar_19 = ((tmpvar_16.w * spec_5.xyz) * _Gloss);
  lowp vec3 normal_20;
  normal_20.xy = ((texture2D (_BumpMap, tmpvar_15).wy * 2.0) - 1.0);
  normal_20.z = sqrt(((1.0 - (normal_20.x * normal_20.x)) - (normal_20.y * normal_20.y)));
  tmpvar_4 = normal_20;
  mediump vec3 tmpvar_21;
  tmpvar_21 = normalize(xlv_TEXCOORD2);
  lightDir_2 = tmpvar_21;
  highp vec3 tmpvar_22;
  tmpvar_22 = normalize(xlv_TEXCOORD1);
  highp float tmpvar_23;
  tmpvar_23 = dot (xlv_TEXCOORD3, xlv_TEXCOORD3);
  lowp vec4 tmpvar_24;
  tmpvar_24 = texture2D (_LightTextureB0, vec2(tmpvar_23));
  lowp vec4 tmpvar_25;
  tmpvar_25 = textureCube (_LightTexture0, xlv_TEXCOORD3);
  mediump vec3 lightDir_26;
  lightDir_26 = lightDir_2;
  mediump vec3 viewDir_27;
  viewDir_27 = tmpvar_22;
  mediump float atten_28;
  atten_28 = (tmpvar_24.w * tmpvar_25.w);
  mediump vec4 c_29;
  mediump vec3 specCol_30;
  highp float nh_31;
  mediump float tmpvar_32;
  tmpvar_32 = max (0.0, dot (tmpvar_4, normalize((lightDir_26 + viewDir_27))));
  nh_31 = tmpvar_32;
  mediump float arg1_33;
  arg1_33 = (32.0 * _Shininess);
  highp vec3 tmpvar_34;
  tmpvar_34 = (pow (nh_31, arg1_33) * tmpvar_19);
  specCol_30 = tmpvar_34;
  c_29.xyz = ((((tmpvar_3 * _LightColor0.xyz) * max (0.0, dot (tmpvar_4, lightDir_26))) + (_LightColor0.xyz * specCol_30)) * (atten_28 * 2.0));
  c_29.w = 0.0;
  c_1.xyz = c_29.xyz;
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
Vector 20 [_BumpMap_ST]
"agal_vs
c21 1.0 0.0 0.0 0.0
[bc]
aaaaaaaaabaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r1.xyz, a5
aaaaaaaaaaaaapacakaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0, c10
bdaaaaaaadaaaeacbbaaaaoeabaaaaaaaaaaaaoeacaaaaaa dp4 r3.z, c17, r0
aaaaaaaaaaaaapacajaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0, c9
bdaaaaaaadaaacacbbaaaaoeabaaaaaaaaaaaaoeacaaaaaa dp4 r3.y, c17, r0
adaaaaaaacaaahacabaaaancaaaaaaaaabaaaaajacaaaaaa mul r2.xyz, a1.zxyw, r1.yzxx
aaaaaaaaabaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r1.xyz, a5
adaaaaaaaeaaahacabaaaamjaaaaaaaaabaaaafcacaaaaaa mul r4.xyz, a1.yzxw, r1.zxyy
acaaaaaaacaaahacaeaaaakeacaaaaaaacaaaakeacaaaaaa sub r2.xyz, r4.xyzz, r2.xyzz
aaaaaaaaabaaapacaiaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r1, c8
bdaaaaaaadaaabacbbaaaaoeabaaaaaaabaaaaoeacaaaaaa dp4 r3.x, c17, r1
adaaaaaaaeaaahacadaaaakeacaaaaaabcaaaappabaaaaaa mul r4.xyz, r3.xyzz, c18.w
acaaaaaaaaaaahacaeaaaakeacaaaaaaaaaaaaoeaaaaaaaa sub r0.xyz, r4.xyzz, a0
adaaaaaaacaaahacacaaaakeacaaaaaaafaaaappaaaaaaaa mul r2.xyz, r2.xyzz, a5.w
aaaaaaaaabaaahacbaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r1.xyz, c16
aaaaaaaaabaaaiacbfaaaaaaabaaaaaaaaaaaaaaaaaaaaaa mov r1.w, c21.x
bcaaaaaaacaaacaeacaaaakeacaaaaaaaaaaaakeacaaaaaa dp3 v2.y, r2.xyzz, r0.xyzz
bcaaaaaaacaaaeaeabaaaaoeaaaaaaaaaaaaaakeacaaaaaa dp3 v2.z, a1, r0.xyzz
bcaaaaaaacaaabaeafaaaaoeaaaaaaaaaaaaaakeacaaaaaa dp3 v2.x, a5, r0.xyzz
bdaaaaaaaaaaaiacaaaaaaoeaaaaaaaaahaaaaoeabaaaaaa dp4 r0.w, a0, c7
bdaaaaaaaaaaaeacaaaaaaoeaaaaaaaaagaaaaoeabaaaaaa dp4 r0.z, a0, c6
bdaaaaaaaaaaabacaaaaaaoeaaaaaaaaaeaaaaoeabaaaaaa dp4 r0.x, a0, c4
bdaaaaaaaaaaacacaaaaaaoeaaaaaaaaafaaaaoeabaaaaaa dp4 r0.y, a0, c5
bdaaaaaaadaaaeacabaaaaoeacaaaaaaakaaaaoeabaaaaaa dp4 r3.z, r1, c10
bdaaaaaaadaaabacabaaaaoeacaaaaaaaiaaaaoeabaaaaaa dp4 r3.x, r1, c8
bdaaaaaaadaaacacabaaaaoeacaaaaaaajaaaaoeabaaaaaa dp4 r3.y, r1, c9
adaaaaaaaeaaahacadaaaakeacaaaaaabcaaaappabaaaaaa mul r4.xyz, r3.xyzz, c18.w
acaaaaaaabaaahacaeaaaakeacaaaaaaaaaaaaoeaaaaaaaa sub r1.xyz, r4.xyzz, a0
bcaaaaaaabaaacaeabaaaakeacaaaaaaacaaaakeacaaaaaa dp3 v1.y, r1.xyzz, r2.xyzz
bcaaaaaaabaaaeaeabaaaaoeaaaaaaaaabaaaakeacaaaaaa dp3 v1.z, a1, r1.xyzz
bcaaaaaaabaaabaeabaaaakeacaaaaaaafaaaaoeaaaaaaaa dp3 v1.x, r1.xyzz, a5
bdaaaaaaadaaaeaeaaaaaaoeacaaaaaaaoaaaaoeabaaaaaa dp4 v3.z, r0, c14
bdaaaaaaadaaacaeaaaaaaoeacaaaaaaanaaaaoeabaaaaaa dp4 v3.y, r0, c13
bdaaaaaaadaaabaeaaaaaaoeacaaaaaaamaaaaoeabaaaaaa dp4 v3.x, r0, c12
adaaaaaaaeaaamacadaaaaeeaaaaaaaabeaaaaeeabaaaaaa mul r4.zw, a3.xyxy, c20.xyxy
abaaaaaaaaaaamaeaeaaaaopacaaaaaabeaaaaoeabaaaaaa add v0.zw, r4.wwzw, c20
adaaaaaaaeaaadacadaaaaoeaaaaaaaabdaaaaoeabaaaaaa mul r4.xy, a3, c19
abaaaaaaaaaaadaeaeaaaafeacaaaaaabdaaaaooabaaaaaa add v0.xy, r4.xyyy, c19.zwzw
bdaaaaaaaaaaaiadaaaaaaoeaaaaaaaaadaaaaoeabaaaaaa dp4 o0.w, a0, c3
bdaaaaaaaaaaaeadaaaaaaoeaaaaaaaaacaaaaoeabaaaaaa dp4 o0.z, a0, c2
bdaaaaaaaaaaacadaaaaaaoeaaaaaaaaabaaaaoeabaaaaaa dp4 o0.y, a0, c1
bdaaaaaaaaaaabadaaaaaaoeaaaaaaaaaaaaaaoeabaaaaaa dp4 o0.x, a0, c0
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
ConstBuffer "$Globals" 176 // 176 used size, 10 vars
Matrix 48 [_LightMatrix0] 4
Vector 144 [_MainTex_ST] 4
Vector 160 [_BumpMap_ST] 4
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
// 34 instructions, 2 temp regs, 0 temp arrays:
// ALU 14 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0_level_9_3
eefiecedmojjpcpiaakmdikedfpgjkanpehfmgdgabaaaaaahiakaaaaaeaaaaaa
daaaaaaaiaadaaaabaajaaaaniajaaaaebgpgodjeiadaaaaeiadaaaaaaacpopp
niacaaaahaaaaaaaagaaceaaaaaagmaaaaaagmaaaaaaceaaabaagmaaaaaaadaa
aeaaabaaaaaaaaaaaaaaajaaacaaafaaaaaaaaaaabaaaeaaabaaahaaaaaaaaaa
acaaaaaaabaaaiaaaaaaaaaaadaaaaaaaeaaajaaaaaaaaaaadaaamaaajaaanaa
aaaaaaaaaaaaaaaaabacpoppbpaaaaacafaaaaiaaaaaapjabpaaaaacafaaabia
abaaapjabpaaaaacafaaaciaacaaapjabpaaaaacafaaadiaadaaapjaaeaaaaae
aaaaadoaadaaoejaafaaoekaafaaookaaeaaaaaeaaaaamoaadaaeejaagaaeeka
agaaoekaabaaaaacaaaaapiaaiaaoekaafaaaaadabaaahiaaaaaffiabcaaoeka
aeaaaaaeabaaahiabbaaoekaaaaaaaiaabaaoeiaaeaaaaaeaaaaahiabdaaoeka
aaaakkiaabaaoeiaaeaaaaaeaaaaahiabeaaoekaaaaappiaaaaaoeiaaeaaaaae
aaaaahiaaaaaoeiabfaappkaaaaaoejbaiaaaaadacaaaboaabaaoejaaaaaoeia
abaaaaacabaaahiaabaaoejaafaaaaadacaaahiaabaamjiaacaancjaaeaaaaae
abaaahiaacaamjjaabaanciaacaaoeibafaaaaadabaaahiaabaaoeiaabaappja
aiaaaaadacaaacoaabaaoeiaaaaaoeiaaiaaaaadacaaaeoaacaaoejaaaaaoeia
abaaaaacaaaaahiaahaaoekaafaaaaadacaaahiaaaaaffiabcaaoekaaeaaaaae
aaaaaliabbaakekaaaaaaaiaacaakeiaaeaaaaaeaaaaahiabdaaoekaaaaakkia
aaaapeiaacaaaaadaaaaahiaaaaaoeiabeaaoekaaeaaaaaeaaaaahiaaaaaoeia
bfaappkaaaaaoejbaiaaaaadabaaaboaabaaoejaaaaaoeiaaiaaaaadabaaacoa
abaaoeiaaaaaoeiaaiaaaaadabaaaeoaacaaoejaaaaaoeiaafaaaaadaaaaapia
aaaaffjaaoaaoekaaeaaaaaeaaaaapiaanaaoekaaaaaaajaaaaaoeiaaeaaaaae
aaaaapiaapaaoekaaaaakkjaaaaaoeiaaeaaaaaeaaaaapiabaaaoekaaaaappja
aaaaoeiaafaaaaadabaaahiaaaaaffiaacaaoekaaeaaaaaeabaaahiaabaaoeka
aaaaaaiaabaaoeiaaeaaaaaeaaaaahiaadaaoekaaaaakkiaabaaoeiaaeaaaaae
adaaahoaaeaaoekaaaaappiaaaaaoeiaafaaaaadaaaaapiaaaaaffjaakaaoeka
aeaaaaaeaaaaapiaajaaoekaaaaaaajaaaaaoeiaaeaaaaaeaaaaapiaalaaoeka
aaaakkjaaaaaoeiaaeaaaaaeaaaaapiaamaaoekaaaaappjaaaaaoeiaaeaaaaae
aaaaadmaaaaappiaaaaaoekaaaaaoeiaabaaaaacaaaaammaaaaaoeiappppaaaa
fdeieefciiafaaaaeaaaabaagcabaaaafjaaaaaeegiocaaaaaaaaaaaalaaaaaa
fjaaaaaeegiocaaaabaaaaaaafaaaaaafjaaaaaeegiocaaaacaaaaaaabaaaaaa
fjaaaaaeegiocaaaadaaaaaabfaaaaaafpaaaaadpcbabaaaaaaaaaaafpaaaaad
pcbabaaaabaaaaaafpaaaaadhcbabaaaacaaaaaafpaaaaaddcbabaaaadaaaaaa
ghaaaaaepccabaaaaaaaaaaaabaaaaaagfaaaaadpccabaaaabaaaaaagfaaaaad
hccabaaaacaaaaaagfaaaaadhccabaaaadaaaaaagfaaaaadhccabaaaaeaaaaaa
giaaaaacacaaaaaadiaaaaaipcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaa
adaaaaaaabaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaaaaaaaaa
agbabaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaa
adaaaaaaacaaaaaakgbkbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpccabaaa
aaaaaaaaegiocaaaadaaaaaaadaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaa
dcaaaaaldccabaaaabaaaaaaegbabaaaadaaaaaaegiacaaaaaaaaaaaajaaaaaa
ogikcaaaaaaaaaaaajaaaaaadcaaaaalmccabaaaabaaaaaaagbebaaaadaaaaaa
agiecaaaaaaaaaaaakaaaaaakgiocaaaaaaaaaaaakaaaaaadiaaaaahhcaabaaa
aaaaaaaajgbebaaaabaaaaaacgbjbaaaacaaaaaadcaaaaakhcaabaaaaaaaaaaa
jgbebaaaacaaaaaacgbjbaaaabaaaaaaegacbaiaebaaaaaaaaaaaaaadiaaaaah
hcaabaaaaaaaaaaaegacbaaaaaaaaaaapgbpbaaaabaaaaaadiaaaaajhcaabaaa
abaaaaaafgifcaaaabaaaaaaaeaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaal
hcaabaaaabaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaaabaaaaaaaeaaaaaa
egacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabcaaaaaa
kgikcaaaabaaaaaaaeaaaaaaegacbaaaabaaaaaaaaaaaaaihcaabaaaabaaaaaa
egacbaaaabaaaaaaegiccaaaadaaaaaabdaaaaaadcaaaaalhcaabaaaabaaaaaa
egacbaaaabaaaaaapgipcaaaadaaaaaabeaaaaaaegbcbaiaebaaaaaaaaaaaaaa
baaaaaahcccabaaaacaaaaaaegacbaaaaaaaaaaaegacbaaaabaaaaaabaaaaaah
bccabaaaacaaaaaaegbcbaaaabaaaaaaegacbaaaabaaaaaabaaaaaaheccabaaa
acaaaaaaegbcbaaaacaaaaaaegacbaaaabaaaaaadiaaaaajhcaabaaaabaaaaaa
fgifcaaaacaaaaaaaaaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaa
abaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaaacaaaaaaaaaaaaaaegacbaaa
abaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaa
acaaaaaaaaaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaa
adaaaaaabdaaaaaapgipcaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaadcaaaaal
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
imaaaaaaaaaaaaaaaaaaaaaaadaaaaaaabaaaaaaapaaaaaaimaaaaaaabaaaaaa
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
Vector 21 [_BumpMap_ST]
"!!ARBvp1.0
# 32 ALU
PARAM c[22] = { { 1 },
		state.matrix.mvp,
		program.local[5..21] };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
MOV R0.xyz, vertex.attrib[14];
MUL R1.xyz, vertex.normal.zxyw, R0.yzxw;
MAD R1.xyz, vertex.normal.yzxw, R0.zxyw, -R1;
MOV R0.w, c[0].x;
MOV R0.xyz, c[17];
DP4 R2.z, R0, c[11];
DP4 R2.x, R0, c[9];
DP4 R2.y, R0, c[10];
MAD R0.xyz, R2, c[19].w, -vertex.position;
MUL R2.xyz, R1, vertex.attrib[14].w;
MOV R1, c[18];
DP3 result.texcoord[1].y, R0, R2;
DP4 R3.z, R1, c[11];
DP4 R3.x, R1, c[9];
DP4 R3.y, R1, c[10];
DP3 result.texcoord[1].z, vertex.normal, R0;
DP3 result.texcoord[1].x, R0, vertex.attrib[14];
DP4 R0.w, vertex.position, c[8];
DP4 R0.z, vertex.position, c[7];
DP4 R0.x, vertex.position, c[5];
DP4 R0.y, vertex.position, c[6];
DP3 result.texcoord[2].y, R2, R3;
DP3 result.texcoord[2].z, vertex.normal, R3;
DP3 result.texcoord[2].x, vertex.attrib[14], R3;
DP4 result.texcoord[3].y, R0, c[14];
DP4 result.texcoord[3].x, R0, c[13];
MAD result.texcoord[0].zw, vertex.texcoord[0].xyxy, c[21].xyxy, c[21];
MAD result.texcoord[0].xy, vertex.texcoord[0], c[20], c[20].zwzw;
DP4 result.position.w, vertex.position, c[4];
DP4 result.position.z, vertex.position, c[3];
DP4 result.position.y, vertex.position, c[2];
DP4 result.position.x, vertex.position, c[1];
END
# 32 instructions, 4 R-regs
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
Vector 20 [_BumpMap_ST]
"vs_2_0
; 35 ALU
def c21, 1.00000000, 0, 0, 0
dcl_position0 v0
dcl_tangent0 v1
dcl_normal0 v2
dcl_texcoord0 v3
mov r0.xyz, v1
mul r1.xyz, v2.zxyw, r0.yzxw
mov r0.xyz, v1
mad r0.xyz, v2.yzxw, r0.zxyw, -r1
mul r3.xyz, r0, v1.w
mov r0, c10
dp4 r4.z, c17, r0
mov r0, c9
dp4 r4.y, c17, r0
mov r1.w, c21.x
mov r1.xyz, c16
dp4 r2.z, r1, c10
dp4 r2.x, r1, c8
dp4 r2.y, r1, c9
mad r2.xyz, r2, c18.w, -v0
mov r1, c8
dp4 r4.x, c17, r1
dp4 r0.w, v0, c7
dp4 r0.z, v0, c6
dp4 r0.x, v0, c4
dp4 r0.y, v0, c5
dp3 oT1.y, r2, r3
dp3 oT2.y, r3, r4
dp3 oT1.z, v2, r2
dp3 oT1.x, r2, v1
dp3 oT2.z, v2, r4
dp3 oT2.x, v1, r4
dp4 oT3.y, r0, c13
dp4 oT3.x, r0, c12
mad oT0.zw, v3.xyxy, c20.xyxy, c20
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
Vector 20 [_BumpMap_ST]
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
backbbabaaaaacdiaaaaabomaaaaaaaaaaaaaaceaaaaaaaaaaaaabmmaaaaaaaa
aaaaaaaaaaaaabkeaaaaaabmaaaaabjhpppoadaaaaaaaaajaaaaaabmaaaaaaaa
aaaaabjaaaaaaanaaaacaabeaaabaaaaaaaaaanmaaaaaaaaaaaaaaomaaacaaap
aaaeaaaaaaaaaapmaaaaaaaaaaaaabamaaacaabdaaabaaaaaaaaaanmaaaaaaaa
aaaaabbiaaacaaagaaaeaaaaaaaaaapmaaaaaaaaaaaaabcgaaacaaakaaaeaaaa
aaaaaapmaaaaaaaaaaaaabdeaaacaaaaaaabaaaaaaaaabemaaaaaaaaaaaaabfm
aaacaaabaaabaaaaaaaaaanmaaaaaaaaaaaaabhbaaacaaacaaaeaaaaaaaaaapm
aaaaaaaaaaaaabieaaacaaaoaaabaaaaaaaaaanmaaaaaaaafpechfgnhaengbha
fpfdfeaaaaabaaadaaabaaaeaaabaaaaaaaaaaaafpemgjghgiheengbhehcgjhi
daaaklklaaadaaadaaaeaaaeaaabaaaaaaaaaaaafpengbgjgofegfhifpfdfeaa
fpepgcgkgfgdhedcfhgphcgmgeaafpfhgphcgmgedcepgcgkgfgdheaafpfhgphc
gmgefdhagbgdgfedgbgngfhcgbfagphdaaklklklaaabaaadaaabaaadaaabaaaa
aaaaaaaafpfhgphcgmgefdhagbgdgfemgjghgihefagphddaaaghgmhdhegbhegf
fpgngbhehcgjhifpgnhghaaahfgogjhehjfpfdgdgbgmgfaahghdfpddfpdaaadc
codacodcdadddfddcodaaaklaaaaaaaaaaaaabomaadbaaahaaaaaaaaaaaaaaaa
aaaadaieaaaaaaabaaaaaaaeaaaaaaajaaaaacjaaabaaaafaaaagaagaaaadaah
aacafaaiaaaapafaaaachbfbaaafhcfcaaaiddfdaaaaaaccaaaabacdaaaaaabm
aaaaaabnaaaababoaaaaaabpaaaaaacaaaaabacbaaaabachpaffeaafaaaabcaa
mcaaaaaaaaaaeaajaaaabcaameaaaaaaaaaagaangabdbcaabcaaaaaaaaaagabj
gabpbcaabcaaaaaaaaaadacfaaaaccaaaaaaaaaaafpihaaaaaaaagiiaaaaaaaa
afpifaaaaaaaagiiaaaaaaaaafpicaaaaaaaaoiiaaaaaaaaafpibaaaaaaaapmi
aaaaaaaamiapaaaaaabliiaakbahafaamiapaaaaaamgiiaaklahaeaamiapaaaa
aalbdejeklahadaamiapiadoaagmaadeklahacaamiahaaaaaamamgmaalamaaan
miahaaadaaleblaacbanabaamiahaaadaamamgleclamabadmiahaaaeaalogfaa
obacafaamiahaaagaalelbleclalaaaamiapaaaaaabliiaakbahajaamiapaaaa
aamgiiaaklahaiaamiahaaagaamagmleclakaaagmiahaaaeabgflomaolacafae
miahaaadaalelbleclalabadmiahaaadaamagmleclakabadmiahaaaeaamablaa
obaeafaamiahaaagabmablmaklagaoahmiapaaaaaalbdejeklahahaamiapaaaa
aagmojkkklahagaamiabiaabaaloloaapaagafaamiaciaabaaloloaapaaeagaa
miaeiaabaaloloaapaagacaamiabiaacaaloloaapaadafaamiaciaacaaloloaa
paaeadaamiaeiaacaaloloaapaadacaamiadiaaaaalalabkilabbdbdmiamiaaa
aakmkmagilabbebemiadaaabaalblaaakbaabcaamiadaaabaabllalaklaabbab
miadaaaaaagmlalaklaabaabmiadiaadaamglalaklaaapaaaaaaaaaaaaaaaaaa
aaaaaaaa"
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
Vector 463 [_BumpMap_ST]
"sce_vp_rsx // 31 instructions using 5 registers
[Configuration]
8
0000001f41050500
[Microcode]
496
00009c6c005d200d8186c0836041fffc00019c6c00400e0c0106c0836041dffc
00011c6c005d300c0186c0836041dffc401f9c6c011cf800810040d560607f9c
401f9c6c011d0808010400d740619f9c401f9c6c01d0300d8106c0c360403f80
401f9c6c01d0200d8106c0c360405f80401f9c6c01d0100d8106c0c360409f80
401f9c6c01d0000d8106c0c360411f8000001c6c01d0700d8106c0c360403ffc
00001c6c01d0600d8106c0c360405ffc00001c6c01d0500d8106c0c360409ffc
00001c6c01d0400d8106c0c360411ffc00021c6c01d0a00d8286c0c360405ffc
00021c6c01d0900d8286c0c360409ffc00021c6c01d0800d8286c0c360411ffc
00009c6c0190a00c0486c0c360405ffc00009c6c0190900c0486c0c360409ffc
00009c6c0190800c0486c0c360411ffc00011c6c00800243011843436041dffc
00011c6c01000230812183630121dffc401f9c6c01d0d00d8086c0c360409fa8
401f9c6c01d0c00d8086c0c360411fa800001c6c011d100c02bfc0e30041dffc
401f9c6c0140020c0106044360405fa4401f9c6c01400e0c0106044360411fa4
00009c6c00800e0c04bfc0836041dffc401f9c6c0140020c0106004360405fa0
401f9c6c01400e0c0086008360411fa0401f9c6c0140000c0286044360409fa4
401f9c6c0140000c0086014360409fa1
"
}

SubProgram "d3d11 " {
Keywords { "DIRECTIONAL_COOKIE" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 176 // 176 used size, 10 vars
Matrix 48 [_LightMatrix0] 4
Vector 144 [_MainTex_ST] 4
Vector 160 [_BumpMap_ST] 4
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
eefiecednmbbbjigbghnaekofbepdhecmpenflagabaaaaaapiagaaaaadaaaaaa
cmaaaaaapeaaaaaajeabaaaaejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaa
abaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaljaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfc
enebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheojiaaaaaaafaaaaaa
aiaaaaaaiaaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaabaaaaaaapaaaaaaimaaaaaaabaaaaaaaaaaaaaa
adaaaaaaacaaaaaaahaiaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaa
ahaiaaaaimaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaadamaaaafdfgfpfa
epfdejfeejepeoaafeeffiedepepfceeaaklklklfdeieefcfmafaaaaeaaaabaa
fhabaaaafjaaaaaeegiocaaaaaaaaaaaalaaaaaafjaaaaaeegiocaaaabaaaaaa
afaaaaaafjaaaaaeegiocaaaacaaaaaaabaaaaaafjaaaaaeegiocaaaadaaaaaa
bfaaaaaafpaaaaadpcbabaaaaaaaaaaafpaaaaadpcbabaaaabaaaaaafpaaaaad
hcbabaaaacaaaaaafpaaaaaddcbabaaaadaaaaaaghaaaaaepccabaaaaaaaaaaa
abaaaaaagfaaaaadpccabaaaabaaaaaagfaaaaadhccabaaaacaaaaaagfaaaaad
hccabaaaadaaaaaagfaaaaaddccabaaaaeaaaaaagiaaaaacacaaaaaadiaaaaai
pcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaadaaaaaaabaaaaaadcaaaaak
pcaabaaaaaaaaaaaegiocaaaadaaaaaaaaaaaaaaagbabaaaaaaaaaaaegaobaaa
aaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaacaaaaaakgbkbaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaakpccabaaaaaaaaaaaegiocaaaadaaaaaa
adaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaaldccabaaaabaaaaaa
egbabaaaadaaaaaaegiacaaaaaaaaaaaajaaaaaaogikcaaaaaaaaaaaajaaaaaa
dcaaaaalmccabaaaabaaaaaaagbebaaaadaaaaaaagiecaaaaaaaaaaaakaaaaaa
kgiocaaaaaaaaaaaakaaaaaadiaaaaahhcaabaaaaaaaaaaajgbebaaaabaaaaaa
cgbjbaaaacaaaaaadcaaaaakhcaabaaaaaaaaaaajgbebaaaacaaaaaacgbjbaaa
abaaaaaaegacbaiaebaaaaaaaaaaaaaadiaaaaahhcaabaaaaaaaaaaaegacbaaa
aaaaaaaapgbpbaaaabaaaaaadiaaaaajhcaabaaaabaaaaaafgifcaaaabaaaaaa
aeaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaa
adaaaaaabaaaaaaaagiacaaaabaaaaaaaeaaaaaaegacbaaaabaaaaaadcaaaaal
hcaabaaaabaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaaabaaaaaaaeaaaaaa
egacbaaaabaaaaaaaaaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaaegiccaaa
adaaaaaabdaaaaaadcaaaaalhcaabaaaabaaaaaaegacbaaaabaaaaaapgipcaaa
adaaaaaabeaaaaaaegbcbaiaebaaaaaaaaaaaaaabaaaaaahcccabaaaacaaaaaa
egacbaaaaaaaaaaaegacbaaaabaaaaaabaaaaaahbccabaaaacaaaaaaegbcbaaa
abaaaaaaegacbaaaabaaaaaabaaaaaaheccabaaaacaaaaaaegbcbaaaacaaaaaa
egacbaaaabaaaaaadiaaaaajhcaabaaaabaaaaaafgifcaaaacaaaaaaaaaaaaaa
egiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaa
baaaaaaaagiacaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaa
abaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaaacaaaaaaaaaaaaaaegacbaaa
abaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabdaaaaaapgipcaaa
acaaaaaaaaaaaaaaegacbaaaabaaaaaabaaaaaahcccabaaaadaaaaaaegacbaaa
aaaaaaaaegacbaaaabaaaaaabaaaaaahbccabaaaadaaaaaaegbcbaaaabaaaaaa
egacbaaaabaaaaaabaaaaaaheccabaaaadaaaaaaegbcbaaaacaaaaaaegacbaaa
abaaaaaadiaaaaaipcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaadaaaaaa
anaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaamaaaaaaagbabaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaa
aoaaaaaakgbkbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaa
egiocaaaadaaaaaaapaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadiaaaaai
dcaabaaaabaaaaaafgafbaaaaaaaaaaaegiacaaaaaaaaaaaaeaaaaaadcaaaaak
dcaabaaaaaaaaaaaegiacaaaaaaaaaaaadaaaaaaagaabaaaaaaaaaaaegaabaaa
abaaaaaadcaaaaakdcaabaaaaaaaaaaaegiacaaaaaaaaaaaafaaaaaakgakbaaa
aaaaaaaaegaabaaaaaaaaaaadcaaaaakdccabaaaaeaaaaaaegiacaaaaaaaaaaa
agaaaaaapgapbaaaaaaaaaaaegaabaaaaaaaaaaadoaaaaab"
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
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 _BumpMap_ST;
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
  highp vec4 tmpvar_3;
  mediump vec3 tmpvar_4;
  tmpvar_3.xy = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  tmpvar_3.zw = ((_glesMultiTexCoord0.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
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
  tmpvar_4 = tmpvar_8;
  highp vec4 tmpvar_9;
  tmpvar_9.w = 1.0;
  tmpvar_9.xyz = _WorldSpaceCameraPos;
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = tmpvar_3;
  xlv_TEXCOORD1 = (tmpvar_7 * (((_World2Object * tmpvar_9).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = tmpvar_4;
  xlv_TEXCOORD3 = (_LightMatrix0 * (_Object2World * _glesVertex)).xy;
}



#endif
#ifdef FRAGMENT

varying highp vec2 xlv_TEXCOORD3;
varying mediump vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp float _Parallax;
uniform mediump float _Gloss;
uniform mediump float _Shininess;
uniform lowp vec4 _Color;
uniform sampler2D _ParallaxMap;
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
  mediump vec4 spec_5;
  mediump float h_6;
  lowp float tmpvar_7;
  tmpvar_7 = texture2D (_ParallaxMap, xlv_TEXCOORD0.zw).w;
  h_6 = tmpvar_7;
  highp vec2 tmpvar_8;
  mediump float height_9;
  height_9 = _Parallax;
  mediump vec3 viewDir_10;
  viewDir_10 = xlv_TEXCOORD1;
  highp vec3 v_11;
  mediump float tmpvar_12;
  tmpvar_12 = ((h_6 * height_9) - (height_9 / 2.0));
  mediump vec3 tmpvar_13;
  tmpvar_13 = normalize(viewDir_10);
  v_11 = tmpvar_13;
  v_11.z = (v_11.z + 0.42);
  tmpvar_8 = (tmpvar_12 * (v_11.xy / v_11.z));
  highp vec2 tmpvar_14;
  tmpvar_14 = (xlv_TEXCOORD0.xy + tmpvar_8);
  highp vec2 tmpvar_15;
  tmpvar_15 = (xlv_TEXCOORD0.zw + tmpvar_8);
  lowp vec4 tmpvar_16;
  tmpvar_16 = texture2D (_MainTex, tmpvar_14);
  lowp vec3 tmpvar_17;
  tmpvar_17 = (tmpvar_16.xyz * _Color.xyz);
  tmpvar_3 = tmpvar_17;
  lowp vec4 tmpvar_18;
  tmpvar_18 = texture2D (_SpecMap, tmpvar_14);
  spec_5 = tmpvar_18;
  mediump vec3 tmpvar_19;
  tmpvar_19 = ((tmpvar_16.w * spec_5.xyz) * _Gloss);
  lowp vec3 tmpvar_20;
  tmpvar_20 = ((texture2D (_BumpMap, tmpvar_15).xyz * 2.0) - 1.0);
  tmpvar_4 = tmpvar_20;
  lightDir_2 = xlv_TEXCOORD2;
  highp vec3 tmpvar_21;
  tmpvar_21 = normalize(xlv_TEXCOORD1);
  lowp vec4 tmpvar_22;
  tmpvar_22 = texture2D (_LightTexture0, xlv_TEXCOORD3);
  mediump vec3 lightDir_23;
  lightDir_23 = lightDir_2;
  mediump vec3 viewDir_24;
  viewDir_24 = tmpvar_21;
  mediump float atten_25;
  atten_25 = tmpvar_22.w;
  mediump vec4 c_26;
  mediump vec3 specCol_27;
  highp float nh_28;
  mediump float tmpvar_29;
  tmpvar_29 = max (0.0, dot (tmpvar_4, normalize((lightDir_23 + viewDir_24))));
  nh_28 = tmpvar_29;
  mediump float arg1_30;
  arg1_30 = (32.0 * _Shininess);
  highp vec3 tmpvar_31;
  tmpvar_31 = (pow (nh_28, arg1_30) * tmpvar_19);
  specCol_27 = tmpvar_31;
  c_26.xyz = ((((tmpvar_3 * _LightColor0.xyz) * max (0.0, dot (tmpvar_4, lightDir_23))) + (_LightColor0.xyz * specCol_27)) * (atten_25 * 2.0));
  c_26.w = 0.0;
  c_1.xyz = c_26.xyz;
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
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 _BumpMap_ST;
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
  highp vec4 tmpvar_3;
  mediump vec3 tmpvar_4;
  tmpvar_3.xy = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  tmpvar_3.zw = ((_glesMultiTexCoord0.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
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
  tmpvar_4 = tmpvar_8;
  highp vec4 tmpvar_9;
  tmpvar_9.w = 1.0;
  tmpvar_9.xyz = _WorldSpaceCameraPos;
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = tmpvar_3;
  xlv_TEXCOORD1 = (tmpvar_7 * (((_World2Object * tmpvar_9).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = tmpvar_4;
  xlv_TEXCOORD3 = (_LightMatrix0 * (_Object2World * _glesVertex)).xy;
}



#endif
#ifdef FRAGMENT

varying highp vec2 xlv_TEXCOORD3;
varying mediump vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp float _Parallax;
uniform mediump float _Gloss;
uniform mediump float _Shininess;
uniform lowp vec4 _Color;
uniform sampler2D _ParallaxMap;
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
  mediump vec4 spec_5;
  mediump float h_6;
  lowp float tmpvar_7;
  tmpvar_7 = texture2D (_ParallaxMap, xlv_TEXCOORD0.zw).w;
  h_6 = tmpvar_7;
  highp vec2 tmpvar_8;
  mediump float height_9;
  height_9 = _Parallax;
  mediump vec3 viewDir_10;
  viewDir_10 = xlv_TEXCOORD1;
  highp vec3 v_11;
  mediump float tmpvar_12;
  tmpvar_12 = ((h_6 * height_9) - (height_9 / 2.0));
  mediump vec3 tmpvar_13;
  tmpvar_13 = normalize(viewDir_10);
  v_11 = tmpvar_13;
  v_11.z = (v_11.z + 0.42);
  tmpvar_8 = (tmpvar_12 * (v_11.xy / v_11.z));
  highp vec2 tmpvar_14;
  tmpvar_14 = (xlv_TEXCOORD0.xy + tmpvar_8);
  highp vec2 tmpvar_15;
  tmpvar_15 = (xlv_TEXCOORD0.zw + tmpvar_8);
  lowp vec4 tmpvar_16;
  tmpvar_16 = texture2D (_MainTex, tmpvar_14);
  lowp vec3 tmpvar_17;
  tmpvar_17 = (tmpvar_16.xyz * _Color.xyz);
  tmpvar_3 = tmpvar_17;
  lowp vec4 tmpvar_18;
  tmpvar_18 = texture2D (_SpecMap, tmpvar_14);
  spec_5 = tmpvar_18;
  mediump vec3 tmpvar_19;
  tmpvar_19 = ((tmpvar_16.w * spec_5.xyz) * _Gloss);
  lowp vec3 normal_20;
  normal_20.xy = ((texture2D (_BumpMap, tmpvar_15).wy * 2.0) - 1.0);
  normal_20.z = sqrt(((1.0 - (normal_20.x * normal_20.x)) - (normal_20.y * normal_20.y)));
  tmpvar_4 = normal_20;
  lightDir_2 = xlv_TEXCOORD2;
  highp vec3 tmpvar_21;
  tmpvar_21 = normalize(xlv_TEXCOORD1);
  lowp vec4 tmpvar_22;
  tmpvar_22 = texture2D (_LightTexture0, xlv_TEXCOORD3);
  mediump vec3 lightDir_23;
  lightDir_23 = lightDir_2;
  mediump vec3 viewDir_24;
  viewDir_24 = tmpvar_21;
  mediump float atten_25;
  atten_25 = tmpvar_22.w;
  mediump vec4 c_26;
  mediump vec3 specCol_27;
  highp float nh_28;
  mediump float tmpvar_29;
  tmpvar_29 = max (0.0, dot (tmpvar_4, normalize((lightDir_23 + viewDir_24))));
  nh_28 = tmpvar_29;
  mediump float arg1_30;
  arg1_30 = (32.0 * _Shininess);
  highp vec3 tmpvar_31;
  tmpvar_31 = (pow (nh_28, arg1_30) * tmpvar_19);
  specCol_27 = tmpvar_31;
  c_26.xyz = ((((tmpvar_3 * _LightColor0.xyz) * max (0.0, dot (tmpvar_4, lightDir_23))) + (_LightColor0.xyz * specCol_27)) * (atten_25 * 2.0));
  c_26.w = 0.0;
  c_1.xyz = c_26.xyz;
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
Vector 20 [_BumpMap_ST]
"agal_vs
c21 1.0 0.0 0.0 0.0
[bc]
aaaaaaaaaaaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, a5
adaaaaaaabaaahacabaaaancaaaaaaaaaaaaaaajacaaaaaa mul r1.xyz, a1.zxyw, r0.yzxx
aaaaaaaaaaaaahacafaaaaoeaaaaaaaaaaaaaaaaaaaaaaaa mov r0.xyz, a5
adaaaaaaacaaahacabaaaamjaaaaaaaaaaaaaafcacaaaaaa mul r2.xyz, a1.yzxw, r0.zxyy
acaaaaaaaaaaahacacaaaakeacaaaaaaabaaaakeacaaaaaa sub r0.xyz, r2.xyzz, r1.xyzz
adaaaaaaadaaahacaaaaaakeacaaaaaaafaaaappaaaaaaaa mul r3.xyz, r0.xyzz, a5.w
aaaaaaaaaaaaapacakaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0, c10
bdaaaaaaaeaaaeacbbaaaaoeabaaaaaaaaaaaaoeacaaaaaa dp4 r4.z, c17, r0
aaaaaaaaaaaaapacajaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0, c9
bdaaaaaaaeaaacacbbaaaaoeabaaaaaaaaaaaaoeacaaaaaa dp4 r4.y, c17, r0
aaaaaaaaabaaaiacbfaaaaaaabaaaaaaaaaaaaaaaaaaaaaa mov r1.w, c21.x
aaaaaaaaabaaahacbaaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r1.xyz, c16
bdaaaaaaacaaaeacabaaaaoeacaaaaaaakaaaaoeabaaaaaa dp4 r2.z, r1, c10
bdaaaaaaacaaabacabaaaaoeacaaaaaaaiaaaaoeabaaaaaa dp4 r2.x, r1, c8
bdaaaaaaacaaacacabaaaaoeacaaaaaaajaaaaoeabaaaaaa dp4 r2.y, r1, c9
adaaaaaaafaaahacacaaaakeacaaaaaabcaaaappabaaaaaa mul r5.xyz, r2.xyzz, c18.w
acaaaaaaacaaahacafaaaakeacaaaaaaaaaaaaoeaaaaaaaa sub r2.xyz, r5.xyzz, a0
aaaaaaaaabaaapacaiaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r1, c8
bdaaaaaaaeaaabacbbaaaaoeabaaaaaaabaaaaoeacaaaaaa dp4 r4.x, c17, r1
bdaaaaaaaaaaaiacaaaaaaoeaaaaaaaaahaaaaoeabaaaaaa dp4 r0.w, a0, c7
bdaaaaaaaaaaaeacaaaaaaoeaaaaaaaaagaaaaoeabaaaaaa dp4 r0.z, a0, c6
bdaaaaaaaaaaabacaaaaaaoeaaaaaaaaaeaaaaoeabaaaaaa dp4 r0.x, a0, c4
bdaaaaaaaaaaacacaaaaaaoeaaaaaaaaafaaaaoeabaaaaaa dp4 r0.y, a0, c5
bcaaaaaaabaaacaeacaaaakeacaaaaaaadaaaakeacaaaaaa dp3 v1.y, r2.xyzz, r3.xyzz
bcaaaaaaacaaacaeadaaaakeacaaaaaaaeaaaakeacaaaaaa dp3 v2.y, r3.xyzz, r4.xyzz
bcaaaaaaabaaaeaeabaaaaoeaaaaaaaaacaaaakeacaaaaaa dp3 v1.z, a1, r2.xyzz
bcaaaaaaabaaabaeacaaaakeacaaaaaaafaaaaoeaaaaaaaa dp3 v1.x, r2.xyzz, a5
bcaaaaaaacaaaeaeabaaaaoeaaaaaaaaaeaaaakeacaaaaaa dp3 v2.z, a1, r4.xyzz
bcaaaaaaacaaabaeafaaaaoeaaaaaaaaaeaaaakeacaaaaaa dp3 v2.x, a5, r4.xyzz
bdaaaaaaadaaacaeaaaaaaoeacaaaaaaanaaaaoeabaaaaaa dp4 v3.y, r0, c13
bdaaaaaaadaaabaeaaaaaaoeacaaaaaaamaaaaoeabaaaaaa dp4 v3.x, r0, c12
adaaaaaaafaaamacadaaaaeeaaaaaaaabeaaaaeeabaaaaaa mul r5.zw, a3.xyxy, c20.xyxy
abaaaaaaaaaaamaeafaaaaopacaaaaaabeaaaaoeabaaaaaa add v0.zw, r5.wwzw, c20
adaaaaaaafaaadacadaaaaoeaaaaaaaabdaaaaoeabaaaaaa mul r5.xy, a3, c19
abaaaaaaaaaaadaeafaaaafeacaaaaaabdaaaaooabaaaaaa add v0.xy, r5.xyyy, c19.zwzw
bdaaaaaaaaaaaiadaaaaaaoeaaaaaaaaadaaaaoeabaaaaaa dp4 o0.w, a0, c3
bdaaaaaaaaaaaeadaaaaaaoeaaaaaaaaacaaaaoeabaaaaaa dp4 o0.z, a0, c2
bdaaaaaaaaaaacadaaaaaaoeaaaaaaaaabaaaaoeabaaaaaa dp4 o0.y, a0, c1
bdaaaaaaaaaaabadaaaaaaoeaaaaaaaaaaaaaaoeabaaaaaa dp4 o0.x, a0, c0
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
ConstBuffer "$Globals" 176 // 176 used size, 10 vars
Matrix 48 [_LightMatrix0] 4
Vector 144 [_MainTex_ST] 4
Vector 160 [_BumpMap_ST] 4
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
eefiecedamflpgkopememjolddkjdjdcijmbmjfeabaaaaaadiakaaaaaeaaaaaa
daaaaaaagmadaaaanaaiaaaajiajaaaaebgpgodjdeadaaaadeadaaaaaaacpopp
meacaaaahaaaaaaaagaaceaaaaaagmaaaaaagmaaaaaaceaaabaagmaaaaaaadaa
aeaaabaaaaaaaaaaaaaaajaaacaaafaaaaaaaaaaabaaaeaaabaaahaaaaaaaaaa
acaaaaaaabaaaiaaaaaaaaaaadaaaaaaaeaaajaaaaaaaaaaadaaamaaajaaanaa
aaaaaaaaaaaaaaaaabacpoppbpaaaaacafaaaaiaaaaaapjabpaaaaacafaaabia
abaaapjabpaaaaacafaaaciaacaaapjabpaaaaacafaaadiaadaaapjaaeaaaaae
aaaaadoaadaaoejaafaaoekaafaaookaaeaaaaaeaaaaamoaadaaeejaagaaeeka
agaaoekaabaaaaacaaaaapiaaiaaoekaafaaaaadabaaahiaaaaaffiabcaaoeka
aeaaaaaeabaaahiabbaaoekaaaaaaaiaabaaoeiaaeaaaaaeaaaaahiabdaaoeka
aaaakkiaabaaoeiaaeaaaaaeaaaaahiabeaaoekaaaaappiaaaaaoeiaaiaaaaad
acaaaboaabaaoejaaaaaoeiaabaaaaacabaaahiaabaaoejaafaaaaadacaaahia
abaamjiaacaancjaaeaaaaaeabaaahiaacaamjjaabaanciaacaaoeibafaaaaad
abaaahiaabaaoeiaabaappjaaiaaaaadacaaacoaabaaoeiaaaaaoeiaaiaaaaad
acaaaeoaacaaoejaaaaaoeiaabaaaaacaaaaahiaahaaoekaafaaaaadacaaahia
aaaaffiabcaaoekaaeaaaaaeaaaaaliabbaakekaaaaaaaiaacaakeiaaeaaaaae
aaaaahiabdaaoekaaaaakkiaaaaapeiaacaaaaadaaaaahiaaaaaoeiabeaaoeka
aeaaaaaeaaaaahiaaaaaoeiabfaappkaaaaaoejbaiaaaaadabaaaboaabaaoeja
aaaaoeiaaiaaaaadabaaacoaabaaoeiaaaaaoeiaaiaaaaadabaaaeoaacaaoeja
aaaaoeiaafaaaaadaaaaapiaaaaaffjaaoaaoekaaeaaaaaeaaaaapiaanaaoeka
aaaaaajaaaaaoeiaaeaaaaaeaaaaapiaapaaoekaaaaakkjaaaaaoeiaaeaaaaae
aaaaapiabaaaoekaaaaappjaaaaaoeiaafaaaaadabaaadiaaaaaffiaacaaoeka
aeaaaaaeaaaaadiaabaaoekaaaaaaaiaabaaoeiaaeaaaaaeaaaaadiaadaaoeka
aaaakkiaaaaaoeiaaeaaaaaeadaaadoaaeaaoekaaaaappiaaaaaoeiaafaaaaad
aaaaapiaaaaaffjaakaaoekaaeaaaaaeaaaaapiaajaaoekaaaaaaajaaaaaoeia
aeaaaaaeaaaaapiaalaaoekaaaaakkjaaaaaoeiaaeaaaaaeaaaaapiaamaaoeka
aaaappjaaaaaoeiaaeaaaaaeaaaaadmaaaaappiaaaaaoekaaaaaoeiaabaaaaac
aaaaammaaaaaoeiappppaaaafdeieefcfmafaaaaeaaaabaafhabaaaafjaaaaae
egiocaaaaaaaaaaaalaaaaaafjaaaaaeegiocaaaabaaaaaaafaaaaaafjaaaaae
egiocaaaacaaaaaaabaaaaaafjaaaaaeegiocaaaadaaaaaabfaaaaaafpaaaaad
pcbabaaaaaaaaaaafpaaaaadpcbabaaaabaaaaaafpaaaaadhcbabaaaacaaaaaa
fpaaaaaddcbabaaaadaaaaaaghaaaaaepccabaaaaaaaaaaaabaaaaaagfaaaaad
pccabaaaabaaaaaagfaaaaadhccabaaaacaaaaaagfaaaaadhccabaaaadaaaaaa
gfaaaaaddccabaaaaeaaaaaagiaaaaacacaaaaaadiaaaaaipcaabaaaaaaaaaaa
fgbfbaaaaaaaaaaaegiocaaaadaaaaaaabaaaaaadcaaaaakpcaabaaaaaaaaaaa
egiocaaaadaaaaaaaaaaaaaaagbabaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaak
pcaabaaaaaaaaaaaegiocaaaadaaaaaaacaaaaaakgbkbaaaaaaaaaaaegaobaaa
aaaaaaaadcaaaaakpccabaaaaaaaaaaaegiocaaaadaaaaaaadaaaaaapgbpbaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaaldccabaaaabaaaaaaegbabaaaadaaaaaa
egiacaaaaaaaaaaaajaaaaaaogikcaaaaaaaaaaaajaaaaaadcaaaaalmccabaaa
abaaaaaaagbebaaaadaaaaaaagiecaaaaaaaaaaaakaaaaaakgiocaaaaaaaaaaa
akaaaaaadiaaaaahhcaabaaaaaaaaaaajgbebaaaabaaaaaacgbjbaaaacaaaaaa
dcaaaaakhcaabaaaaaaaaaaajgbebaaaacaaaaaacgbjbaaaabaaaaaaegacbaia
ebaaaaaaaaaaaaaadiaaaaahhcaabaaaaaaaaaaaegacbaaaaaaaaaaapgbpbaaa
abaaaaaadiaaaaajhcaabaaaabaaaaaafgifcaaaabaaaaaaaeaaaaaaegiccaaa
adaaaaaabbaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabaaaaaaa
agiacaaaabaaaaaaaeaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaa
egiccaaaadaaaaaabcaaaaaakgikcaaaabaaaaaaaeaaaaaaegacbaaaabaaaaaa
aaaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaaegiccaaaadaaaaaabdaaaaaa
dcaaaaalhcaabaaaabaaaaaaegacbaaaabaaaaaapgipcaaaadaaaaaabeaaaaaa
egbcbaiaebaaaaaaaaaaaaaabaaaaaahcccabaaaacaaaaaaegacbaaaaaaaaaaa
egacbaaaabaaaaaabaaaaaahbccabaaaacaaaaaaegbcbaaaabaaaaaaegacbaaa
abaaaaaabaaaaaaheccabaaaacaaaaaaegbcbaaaacaaaaaaegacbaaaabaaaaaa
diaaaaajhcaabaaaabaaaaaafgifcaaaacaaaaaaaaaaaaaaegiccaaaadaaaaaa
bbaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaa
acaaaaaaaaaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaa
adaaaaaabcaaaaaakgikcaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaadcaaaaal
hcaabaaaabaaaaaaegiccaaaadaaaaaabdaaaaaapgipcaaaacaaaaaaaaaaaaaa
egacbaaaabaaaaaabaaaaaahcccabaaaadaaaaaaegacbaaaaaaaaaaaegacbaaa
abaaaaaabaaaaaahbccabaaaadaaaaaaegbcbaaaabaaaaaaegacbaaaabaaaaaa
baaaaaaheccabaaaadaaaaaaegbcbaaaacaaaaaaegacbaaaabaaaaaadiaaaaai
pcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaadaaaaaaanaaaaaadcaaaaak
pcaabaaaaaaaaaaaegiocaaaadaaaaaaamaaaaaaagbabaaaaaaaaaaaegaobaaa
aaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaaoaaaaaakgbkbaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaa
apaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadiaaaaaidcaabaaaabaaaaaa
fgafbaaaaaaaaaaaegiacaaaaaaaaaaaaeaaaaaadcaaaaakdcaabaaaaaaaaaaa
egiacaaaaaaaaaaaadaaaaaaagaabaaaaaaaaaaaegaabaaaabaaaaaadcaaaaak
dcaabaaaaaaaaaaaegiacaaaaaaaaaaaafaaaaaakgakbaaaaaaaaaaaegaabaaa
aaaaaaaadcaaaaakdccabaaaaeaaaaaaegiacaaaaaaaaaaaagaaaaaapgapbaaa
aaaaaaaaegaabaaaaaaaaaaadoaaaaabejfdeheomaaaaaaaagaaaaaaaiaaaaaa
jiaaaaaaaaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaa
acaaaaaaahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaa
laaaaaaaabaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaljaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofe
aaeoepfcenebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheojiaaaaaa
afaaaaaaaiaaaaaaiaaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaa
imaaaaaaaaaaaaaaaaaaaaaaadaaaaaaabaaaaaaapaaaaaaimaaaaaaabaaaaaa
aaaaaaaaadaaaaaaacaaaaaaahaiaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaa
adaaaaaaahaiaaaaimaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaadamaaaa
fdfgfpfaepfdejfeejepeoaafeeffiedepepfceeaaklklkl"
}

}
Program "fp" {
// Fragment combos: 5
//   opengl - ALU: 42 to 53, TEX: 4 to 6
//   d3d9 - ALU: 47 to 57, TEX: 4 to 6
//   d3d11 - ALU: 22 to 32, TEX: 4 to 6, FLOW: 1 to 1
//   d3d11_9x - ALU: 22 to 32, TEX: 4 to 6, FLOW: 1 to 1
SubProgram "opengl " {
Keywords { "POINT" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
Float 4 [_Parallax]
SetTexture 0 [_ParallaxMap] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 2 [_SpecMap] 2D
SetTexture 3 [_BumpMap] 2D
SetTexture 4 [_LightTexture0] 2D
"!!ARBfp1.0
OPTION ARB_precision_hint_fastest;
# 47 ALU, 5 TEX
PARAM c[7] = { program.local[0..4],
		{ 0, 0.5, 0.41999999, 2 },
		{ 1, 32 } };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
TEMP R4;
TEX R0.w, fragment.texcoord[0].zwzw, texture[0], 2D;
DP3 R0.x, fragment.texcoord[1], fragment.texcoord[1];
RSQ R4.x, R0.x;
MUL R1.xyz, R4.x, fragment.texcoord[1];
ADD R0.y, R1.z, c[5].z;
RCP R0.y, R0.y;
MOV R0.x, c[4];
MUL R0.x, R0, c[5].y;
MAD R0.x, R0.w, c[4], -R0;
MUL R1.xy, R1, R0.y;
MAD R2.xy, R0.x, R1, fragment.texcoord[0].zwzw;
MAD R0.xy, R0.x, R1, fragment.texcoord[0];
DP3 R0.w, fragment.texcoord[3], fragment.texcoord[3];
MOV R2.zw, c[6].xyxy;
MOV result.color.w, c[5].x;
TEX R1, R0, texture[1], 2D;
TEX R3.yw, R2, texture[3], 2D;
TEX R0.xyz, R0, texture[2], 2D;
TEX R0.w, R0.w, texture[4], 2D;
MUL R0.xyz, R1.w, R0;
DP3 R2.x, fragment.texcoord[2], fragment.texcoord[2];
RSQ R3.x, R2.x;
MAD R2.xy, R3.wyzw, c[5].w, -R2.z;
MUL R3.xyz, R3.x, fragment.texcoord[2];
MAD R4.xyz, R4.x, fragment.texcoord[1], R3;
DP3 R3.w, R4, R4;
RSQ R3.w, R3.w;
MUL R2.z, R2.y, R2.y;
MAD R2.z, -R2.x, R2.x, -R2;
ADD R2.z, R2, c[6].x;
RSQ R2.z, R2.z;
MUL R1.xyz, R1, c[1];
RCP R2.z, R2.z;
MUL R4.xyz, R3.w, R4;
DP3 R3.w, R2, R4;
MAX R3.w, R3, c[5].x;
MUL R1.w, R2, c[2].x;
POW R1.w, R3.w, R1.w;
MUL R0.xyz, R0, c[3].x;
MUL R0.xyz, R1.w, R0;
DP3 R1.w, R2, R3;
MUL R0.xyz, R0, c[0];
MAX R1.w, R1, c[5].x;
MUL R1.xyz, R1, c[0];
MAD R0.xyz, R1, R1.w, R0;
MUL R0.xyz, R0.w, R0;
MUL result.color.xyz, R0, c[5].w;
END
# 47 instructions, 5 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "POINT" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
Float 4 [_Parallax]
SetTexture 0 [_ParallaxMap] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 2 [_SpecMap] 2D
SetTexture 3 [_BumpMap] 2D
SetTexture 4 [_LightTexture0] 2D
"ps_2_0
; 53 ALU, 5 TEX
dcl_2d s0
dcl_2d s1
dcl_2d s2
dcl_2d s3
dcl_2d s4
def c5, 0.50000000, 0.41999999, 2.00000000, -1.00000000
def c6, 1.00000000, 0.00000000, 32.00000000, 0
dcl t0
dcl t1.xyz
dcl t2.xyz
dcl t3.xyz
mov_pp r1.x, c5
mov r2.y, t0.w
mov r0.y, t0.w
mov r0.x, t0.z
mul_pp r1.x, c4, r1
texld r0, r0, s0
dp3_pp r0.x, t1, t1
rsq_pp r0.x, r0.x
mul_pp r3.xyz, r0.x, t1
mad_pp r1.x, r0.w, c4, -r1
add r2.x, r3.z, c5.y
rcp r2.x, r2.x
mul r3.xy, r3, r2.x
mov r2.x, t0.z
mad r5.xy, r1.x, r3, r2
mad r1.xy, r1.x, r3, t0
dp3 r2.x, t3, t3
mov r4.xy, r2.x
mov_pp r0.w, c6.y
texld r7, r4, s4
texld r2, r1, s1
texld r3, r1, s2
texld r1, r5, s3
mul_pp r2.xyz, r2, c1
mov r4.y, r1
mov r4.x, r1.w
mad_pp r6.xy, r4, c5.z, c5.w
dp3_pp r1.x, t2, t2
rsq_pp r4.x, r1.x
mul_pp r4.xyz, r4.x, t2
mul_pp r1.x, r6.y, r6.y
mad_pp r5.xyz, r0.x, t1, r4
mad_pp r1.x, -r6, r6, -r1
add_pp r0.x, r1, c6
dp3_pp r1.x, r5, r5
rsq_pp r0.x, r0.x
rcp_pp r6.z, r0.x
rsq_pp r1.x, r1.x
mul_pp r1.xyz, r1.x, r5
dp3_pp r1.x, r6, r1
mov_pp r0.x, c2
mul_pp r0.x, c6.z, r0
max_pp r1.x, r1, c6.y
pow r5.x, r1.x, r0.x
mul_pp r0.xyz, r2.w, r3
mul_pp r1.xyz, r0, c3.x
mov r0.x, r5.x
mul r0.xyz, r0.x, r1
mul_pp r1.xyz, r0, c0
dp3_pp r0.x, r6, r4
max_pp r0.x, r0, c6.y
mul_pp r2.xyz, r2, c0
mad_pp r0.xyz, r2, r0.x, r1
mul_pp r0.xyz, r7.x, r0
mul_pp r0.xyz, r0, c5.z
mov_pp oC0, r0
"
}

SubProgram "xbox360 " {
Keywords { "POINT" }
Vector 1 [_Color]
Float 3 [_Gloss]
Vector 0 [_LightColor0]
Float 4 [_Parallax]
Float 2 [_Shininess]
SetTexture 0 [_LightTexture0] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_SpecMap] 2D
SetTexture 4 [_ParallaxMap] 2D
// Shader Timing Estimate, in Cycles/64 pixel vector:
// ALU: 34.67 (26 instructions), vertex: 0, texture: 20,
//   sequencer: 14, interpolator: 16;    7 GPRs, 27 threads,
// Performance (if enough threads): ~34 cycles per vector
// * Texture cycle estimates are assuming an 8bit/component texture with no
//     aniso or trilinear filtering.

"ps_360
backbbaaaaaaacbeaaaaabpaaaaaaaaaaaaaaaceaaaaablmaaaaaboeaaaaaaaa
aaaaaaaaaaaaabjeaaaaaabmaaaaabihppppadaaaaaaaaakaaaaaabmaaaaaaaa
aaaaabiaaaaaaaoeaaadaaacaaabaaaaaaaaaapaaaaaaaaaaaaaabaaaaacaaab
aaabaaaaaaaaabaiaaaaaaaaaaaaabbiaaacaaadaaabaaaaaaaaabcaaaaaaaaa
aaaaabdaaaacaaaaaaabaaaaaaaaabaiaaaaaaaaaaaaabdnaaadaaaaaaabaaaa
aaaaaapaaaaaaaaaaaaaabemaaadaaabaaabaaaaaaaaaapaaaaaaaaaaaaaabff
aaacaaaeaaabaaaaaaaaabcaaaaaaaaaaaaaabfpaaadaaaeaaabaaaaaaaaaapa
aaaaaaaaaaaaabgmaaacaaacaaabaaaaaaaaabcaaaaaaaaaaaaaabhhaaadaaad
aaabaaaaaaaaaapaaaaaaaaafpechfgnhaengbhaaaklklklaaaeaaamaaabaaab
aaabaaaaaaaaaaaafpedgpgmgphcaaklaaabaaadaaabaaaeaaabaaaaaaaaaaaa
fpehgmgphdhdaaklaaaaaaadaaabaaabaaabaaaaaaaaaaaafpemgjghgiheedgp
gmgphcdaaafpemgjghgihefegfhihehfhcgfdaaafpengbgjgofegfhiaafpfagb
hcgbgmgmgbhiaafpfagbhcgbgmgmgbhiengbhaaafpfdgigjgogjgogfhdhdaafp
fdhagfgdengbhaaahahdfpddfpdaaadccodacodcdadddfddcodaaaklaaaaaaaa
aaaaaaabaaaaaaaaaaaaaaaaaaaaaabeabpmaabaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaeaaaaaablabaaaagaaaaaaaaaeaaaaaaaaaaaadeieaaapaaap
aaaaaaabaaaapafaaaaahbfbaaaahcfcaaaahdfdaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaalpiaaaaaecaaaaaaaaaaaaaa
aaaaaaaadpaaaaaaaaaaaaaadonhakdndpiaaaaaaaajgaaegaakbcaabcaaafea
aaabbabaaaaabcaameaaaaaaaaaagabbgabhbcaabcaaaaaaaaaagabnaaaaccaa
aaaaaaaadieadaabbpbpphppaaaaeaaamiaiaaabaaloloaapaababaafiiiabac
aagmgmblcbaeppibmiaiaaadabblgmblkladaeacmiahaaabaablloaaobababaa
lebiaeabaalolomanaacacppembiadacaalologmpaadadaemiadaaadaamfgmaa
obabadaamiapaaaaaalmbljeoladadaadadigaabbpbppoiiaaaaeaaadabifaab
bpbppgiiaaaaeaaapmaiaaebbpbppppiaaaaeaaageciaaabbpbppompaaaaeaaa
fiiiaaaeaagmlbblcbacpoibaabhadaeaamamagmkbafabaaaachadafaablmamg
obaaacaaaaehadacaamagflboaafabaamiagaaaaaambgmaakaadpoaamiabaaaa
aelclcblnbaaaappmiaiaaaaaaloloaapaacacaafiihaaabaablmablobafagia
kaihaaacaamablgmobacaaiamiabaaaaaalomdaapaafaaaamiacaaaaaalomdaa
paacaaaamiadaaaaaalalbaakcaappaaeaehaaaeaamamalbkbaeaaiamiapaaaa
aaaaomaaobaeaaaadiboababaapmgmblkbabadaamiahaaabaabfgmaaobababaa
miahaaaaaamamamaklabaaaamiahmaaaaagmmaaaobadaaaaaaaaaaaaaaaaaaaa
aaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "POINT" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
Float 4 [_Parallax]
SetTexture 0 [_ParallaxMap] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 2 [_SpecMap] 2D
SetTexture 3 [_BumpMap] 2D
SetTexture 4 [_LightTexture0] 2D
"sce_fp_rsx // 61 instructions using 4 registers
[Configuration]
24
ffffffff0003c020000ffff0000000000000840004000000
[Offsets]
5
_LightColor0 2 0
000003b000000340
_Color 1 0
00000320
_Shininess 1 0
000000d0
_Gloss 1 0
000001b0
_Parallax 2 0
0000004000000020
[Microcode]
976
900017005c011c9dc8000001c8003fe102860240fe001c9d00020000c8000001
000000000000000000000000000000001084014000021c9cc8000001c8000001
000000000000000000000000000000009e060100c8011c9dc8000001c8003fe1
02820440ff081c9d00020000c90c00010000bf00000000000000000000000000
ae843940c8011c9dc8000029c800bfe11000030055081c9d00020000c8000001
0a3d3ed700000000000000000000000006003a00c9081c9dfe000001c8000001
1084014000021c9cc8000001c800000100000000000000000000000000000000
1802020001041c9c80000000c800000118060100c80c1c9dc8000001c8000001
060003005c0c1c9d5c040001c800000114001706c8001c9dc8000001c8000001
0a000300a00c1c9cf4040001c8000001ee040100c8011c9dc8000001c8003fe1
06860440ce001c9d00020000aa020000000040000000bf800000000000000000
0e061704d0001c9dc8000001c800000110060500c8081c9dc8080001c8000001
1e041702d0001c9dc8000001c8000001ce803940c8011c9dc8000029c800bfe1
02820240fe081c9d00020000c800000100000000000000000000000000000000
10800240ab0c1c9cab0c0000c80000010e060100c80c1c9dc8000001c8000001
0e82024001041c9cc80c0001c800000110800440010c1c9e010c0000c9000003
10800340c9001c9d00020000c800000100003f80000000000000000000000000
0e060340c9081c9dc9000001c800000108863b40ff003c9dff000001c8000001
06000100c8001c9dc8000001c800000108800540c90c1c9dc9000001c8000001
02001708fe0c1c9dc8000001c80000010e843940c80c1c9dc8000029c8000001
10800540c90c1c9dc9080001c800000102020900ff001c9d00020000c8000001
000000000000000000000000000000001080090055001c9d00020000c8000001
0000000000000000000000000000000010021d00c8041c9dc8000001c8000001
02860240ff081c9d00020000c800000100004200000000000000000000000000
10020200c8041c9d010c0000c80000010e840240c8081c9dc8020001c8000001
000000000000000000000000000000000e840240c9081c9dc8020001c8000001
0000000000000000000000000000000008021c00fe041c9dc8000001c8000001
0e86020054041c9dc9040001c80000010e840240c9081c9dff000001c8000001
1080014000021c9cc8000001c800000100000000000000000000000000000000
0e820440c90c1c9dc8020001c908000100000000000000000000000000000000
0e81024000001c9cc9041001c8000001
"
}

SubProgram "d3d11 " {
Keywords { "POINT" }
ConstBuffer "$Globals" 176 // 140 used size, 10 vars
Vector 16 [_LightColor0] 4
Vector 112 [_Color] 4
Float 128 [_Shininess]
Float 132 [_Gloss]
Float 136 [_Parallax]
BindCB "$Globals" 0
SetTexture 0 [_ParallaxMap] 2D 4
SetTexture 1 [_MainTex] 2D 1
SetTexture 2 [_SpecMap] 2D 3
SetTexture 3 [_BumpMap] 2D 2
SetTexture 4 [_LightTexture0] 2D 0
// 42 instructions, 5 temp regs, 0 temp arrays:
// ALU 27 float, 0 int, 0 uint
// TEX 5 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0
eefiecedfegjkaicnmkeocngignojijggmhldofjabaaaaaapeagaaaaadaaaaaa
cmaaaaaammaaaaaaaaabaaaaejfdeheojiaaaaaaafaaaaaaaiaaaaaaiaaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaaimaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaahahaaaaimaaaaaa
adaaaaaaaaaaaaaaadaaaaaaaeaaaaaaahahaaaafdfgfpfaepfdejfeejepeoaa
feeffiedepepfceeaaklklklepfdeheocmaaaaaaabaaaaaaaiaaaaaacaaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaafdfgfpfegbhcghgfheaaklkl
fdeieefcomafaaaaeaaaaaaahlabaaaafjaaaaaeegiocaaaaaaaaaaaajaaaaaa
fkaaaaadaagabaaaaaaaaaaafkaaaaadaagabaaaabaaaaaafkaaaaadaagabaaa
acaaaaaafkaaaaadaagabaaaadaaaaaafkaaaaadaagabaaaaeaaaaaafibiaaae
aahabaaaaaaaaaaaffffaaaafibiaaaeaahabaaaabaaaaaaffffaaaafibiaaae
aahabaaaacaaaaaaffffaaaafibiaaaeaahabaaaadaaaaaaffffaaaafibiaaae
aahabaaaaeaaaaaaffffaaaagcbaaaadpcbabaaaabaaaaaagcbaaaadhcbabaaa
acaaaaaagcbaaaadhcbabaaaadaaaaaagcbaaaadhcbabaaaaeaaaaaagfaaaaad
pccabaaaaaaaaaaagiaaaaacafaaaaaabaaaaaahbcaabaaaaaaaaaaaegbcbaaa
adaaaaaaegbcbaaaadaaaaaaeeaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaa
baaaaaahccaabaaaaaaaaaaaegbcbaaaacaaaaaaegbcbaaaacaaaaaaeeaaaaaf
ccaabaaaaaaaaaaabkaabaaaaaaaaaaadiaaaaahhcaabaaaabaaaaaafgafbaaa
aaaaaaaaegbcbaaaacaaaaaadcaaaaajccaabaaaaaaaaaaackbabaaaacaaaaaa
bkaabaaaaaaaaaaaabeaaaaadnaknhdoaoaaaaahpcaabaaaacaaaaaaegaebaaa
abaaaaaafgafbaaaaaaaaaaadcaaaaajocaabaaaaaaaaaaaagbjbaaaadaaaaaa
agaabaaaaaaaaaaaagajbaaaabaaaaaadiaaaaahhcaabaaaabaaaaaaagaabaaa
aaaaaaaaegbcbaaaadaaaaaabaaaaaahbcaabaaaaaaaaaaajgahbaaaaaaaaaaa
jgahbaaaaaaaaaaaeeaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaadiaaaaah
hcaabaaaaaaaaaaaagaabaaaaaaaaaaajgahbaaaaaaaaaaaefaaaaajpcaabaaa
adaaaaaaogbkbaaaabaaaaaaeghobaaaaaaaaaaaaagabaaaaeaaaaaadiaaaaal
dcaabaaaadaaaaaacgikcaaaaaaaaaaaaiaaaaaaaceaaaaaaaaaaadpaaaaaaec
aaaaaaaaaaaaaaaadcaaaaalicaabaaaaaaaaaaadkaabaaaadaaaaaackiacaaa
aaaaaaaaaiaaaaaaakaabaiaebaaaaaaadaaaaaadcaaaaajpcaabaaaacaaaaaa
pgapbaaaaaaaaaaaegaobaaaacaaaaaaegbobaaaabaaaaaaefaaaaajpcaabaaa
aeaaaaaaogakbaaaacaaaaaaeghobaaaadaaaaaaaagabaaaacaaaaaadcaaaaap
dcaabaaaaeaaaaaahgapbaaaaeaaaaaaaceaaaaaaaaaaaeaaaaaaaeaaaaaaaaa
aaaaaaaaaceaaaaaaaaaialpaaaaialpaaaaaaaaaaaaaaaadcaaaaakicaabaaa
aaaaaaaaakaabaiaebaaaaaaaeaaaaaaakaabaaaaeaaaaaaabeaaaaaaaaaiadp
dcaaaaakicaabaaaaaaaaaaabkaabaiaebaaaaaaaeaaaaaabkaabaaaaeaaaaaa
dkaabaaaaaaaaaaaelaaaaafecaabaaaaeaaaaaadkaabaaaaaaaaaaabaaaaaah
bcaabaaaaaaaaaaaegacbaaaaeaaaaaaegacbaaaaaaaaaaabaaaaaahccaabaaa
aaaaaaaaegacbaaaaeaaaaaaegacbaaaabaaaaaadeaaaaakdcaabaaaaaaaaaaa
egaabaaaaaaaaaaaaceaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaacpaaaaaf
bcaabaaaaaaaaaaaakaabaaaaaaaaaaadiaaaaahbcaabaaaaaaaaaaaakaabaaa
aaaaaaaabkaabaaaadaaaaaabjaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaa
efaaaaajpcaabaaaabaaaaaaegaabaaaacaaaaaaeghobaaaacaaaaaaaagabaaa
adaaaaaaefaaaaajpcaabaaaacaaaaaaegaabaaaacaaaaaaeghobaaaabaaaaaa
aagabaaaabaaaaaadiaaaaahhcaabaaaabaaaaaaegacbaaaabaaaaaapgapbaaa
acaaaaaadiaaaaaihcaabaaaacaaaaaaegacbaaaacaaaaaaegiccaaaaaaaaaaa
ahaaaaaadiaaaaaihcaabaaaacaaaaaaegacbaaaacaaaaaaegiccaaaaaaaaaaa
abaaaaaadiaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaafgifcaaaaaaaaaaa
aiaaaaaadiaaaaahncaabaaaaaaaaaaaagaabaaaaaaaaaaaagajbaaaabaaaaaa
diaaaaaincaabaaaaaaaaaaaagaobaaaaaaaaaaaagijcaaaaaaaaaaaabaaaaaa
dcaaaaajhcaabaaaaaaaaaaaegacbaaaacaaaaaafgafbaaaaaaaaaaaigadbaaa
aaaaaaaabaaaaaahicaabaaaaaaaaaaaegbcbaaaaeaaaaaaegbcbaaaaeaaaaaa
efaaaaajpcaabaaaabaaaaaapgapbaaaaaaaaaaaeghobaaaaeaaaaaaaagabaaa
aaaaaaaaaaaaaaahicaabaaaaaaaaaaaakaabaaaabaaaaaaakaabaaaabaaaaaa
diaaaaahhccabaaaaaaaaaaapgapbaaaaaaaaaaaegacbaaaaaaaaaaadgaaaaaf
iccabaaaaaaaaaaaabeaaaaaaaaaaaaadoaaaaab"
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
Float 4 [_Parallax]
SetTexture 0 [_ParallaxMap] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 2 [_SpecMap] 2D
SetTexture 3 [_BumpMap] 2D
SetTexture 4 [_LightTexture0] 2D
"agal_ps
c5 0.5 0.42 2.0 -1.0
c6 1.0 0.0 32.0 0.0
[bc]
aaaaaaaaabaaabacafaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r1.x, c5
aaaaaaaaacaaacacaaaaaappaeaaaaaaaaaaaaaaaaaaaaaa mov r2.y, v0.w
aaaaaaaaaaaaacacaaaaaappaeaaaaaaaaaaaaaaaaaaaaaa mov r0.y, v0.w
aaaaaaaaaaaaabacaaaaaakkaeaaaaaaaaaaaaaaaaaaaaaa mov r0.x, v0.z
adaaaaaaabaaabacaeaaaaoeabaaaaaaabaaaaaaacaaaaaa mul r1.x, c4, r1.x
ciaaaaaaaaaaapacaaaaaafeacaaaaaaaaaaaaaaafaababb tex r0, r0.xyyy, s0 <2d wrap linear point>
bcaaaaaaaaaaabacabaaaaoeaeaaaaaaabaaaaoeaeaaaaaa dp3 r0.x, v1, v1
akaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r0.x, r0.x
adaaaaaaadaaahacaaaaaaaaacaaaaaaabaaaaoeaeaaaaaa mul r3.xyz, r0.x, v1
adaaaaaaadaaaiacaaaaaappacaaaaaaaeaaaaoeabaaaaaa mul r3.w, r0.w, c4
acaaaaaaabaaabacadaaaappacaaaaaaabaaaaaaacaaaaaa sub r1.x, r3.w, r1.x
abaaaaaaacaaabacadaaaakkacaaaaaaafaaaaffabaaaaaa add r2.x, r3.z, c5.y
afaaaaaaacaaabacacaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rcp r2.x, r2.x
adaaaaaaadaaadacadaaaafeacaaaaaaacaaaaaaacaaaaaa mul r3.xy, r3.xyyy, r2.x
aaaaaaaaacaaabacaaaaaakkaeaaaaaaaaaaaaaaaaaaaaaa mov r2.x, v0.z
adaaaaaaaeaaadacabaaaaaaacaaaaaaadaaaafeacaaaaaa mul r4.xy, r1.x, r3.xyyy
abaaaaaaaeaaadacaeaaaafeacaaaaaaacaaaafeacaaaaaa add r4.xy, r4.xyyy, r2.xyyy
adaaaaaaadaaadacabaaaaaaacaaaaaaadaaaafeacaaaaaa mul r3.xy, r1.x, r3.xyyy
abaaaaaaadaaadacadaaaafeacaaaaaaaaaaaaoeaeaaaaaa add r3.xy, r3.xyyy, v0
bcaaaaaaacaaabacadaaaaoeaeaaaaaaadaaaaoeaeaaaaaa dp3 r2.x, v3, v3
aaaaaaaaabaaadacacaaaaaaacaaaaaaaaaaaaaaaaaaaaaa mov r1.xy, r2.x
aaaaaaaaaaaaaiacagaaaaffabaaaaaaaaaaaaaaaaaaaaaa mov r0.w, c6.y
ciaaaaaaacaaapacadaaaafeacaaaaaaabaaaaaaafaababb tex r2, r3.xyyy, s1 <2d wrap linear point>
ciaaaaaaaeaaapacaeaaaafeacaaaaaaadaaaaaaafaababb tex r4, r4.xyyy, s3 <2d wrap linear point>
ciaaaaaaabaaapacabaaaafeacaaaaaaaeaaaaaaafaababb tex r1, r1.xyyy, s4 <2d wrap linear point>
ciaaaaaaadaaapacadaaaafeacaaaaaaacaaaaaaafaababb tex r3, r3.xyyy, s2 <2d wrap linear point>
aaaaaaaaaeaaabacaeaaaappacaaaaaaaaaaaaaaaaaaaaaa mov r4.x, r4.w
adaaaaaaacaaahacacaaaakeacaaaaaaabaaaaoeabaaaaaa mul r2.xyz, r2.xyzz, c1
adaaaaaaagaaadacaeaaaafeacaaaaaaafaaaakkabaaaaaa mul r6.xy, r4.xyyy, c5.z
abaaaaaaagaaadacagaaaafeacaaaaaaafaaaappabaaaaaa add r6.xy, r6.xyyy, c5.w
bcaaaaaaabaaabacacaaaaoeaeaaaaaaacaaaaoeaeaaaaaa dp3 r1.x, v2, v2
akaaaaaaaeaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r4.x, r1.x
adaaaaaaaeaaahacaeaaaaaaacaaaaaaacaaaaoeaeaaaaaa mul r4.xyz, r4.x, v2
adaaaaaaabaaabacagaaaaffacaaaaaaagaaaaffacaaaaaa mul r1.x, r6.y, r6.y
adaaaaaaafaaahacaaaaaaaaacaaaaaaabaaaaoeaeaaaaaa mul r5.xyz, r0.x, v1
abaaaaaaafaaahacafaaaakeacaaaaaaaeaaaakeacaaaaaa add r5.xyz, r5.xyzz, r4.xyzz
bfaaaaaaafaaaiacagaaaaaaacaaaaaaaaaaaaaaaaaaaaaa neg r5.w, r6.x
adaaaaaaafaaaiacafaaaappacaaaaaaagaaaaaaacaaaaaa mul r5.w, r5.w, r6.x
acaaaaaaabaaabacafaaaappacaaaaaaabaaaaaaacaaaaaa sub r1.x, r5.w, r1.x
abaaaaaaaaaaabacabaaaaaaacaaaaaaagaaaaoeabaaaaaa add r0.x, r1.x, c6
bcaaaaaaabaaabacafaaaakeacaaaaaaafaaaakeacaaaaaa dp3 r1.x, r5.xyzz, r5.xyzz
akaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r0.x, r0.x
afaaaaaaagaaaeacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rcp r6.z, r0.x
akaaaaaaabaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r1.x, r1.x
adaaaaaaabaaahacabaaaaaaacaaaaaaafaaaakeacaaaaaa mul r1.xyz, r1.x, r5.xyzz
bcaaaaaaabaaabacagaaaakeacaaaaaaabaaaakeacaaaaaa dp3 r1.x, r6.xyzz, r1.xyzz
aaaaaaaaaaaaabacacaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0.x, c2
adaaaaaaaaaaabacagaaaakkabaaaaaaaaaaaaaaacaaaaaa mul r0.x, c6.z, r0.x
ahaaaaaaabaaabacabaaaaaaacaaaaaaagaaaaffabaaaaaa max r1.x, r1.x, c6.y
alaaaaaaafaaapacabaaaaaaacaaaaaaaaaaaaaaacaaaaaa pow r5, r1.x, r0.x
adaaaaaaaaaaahacacaaaappacaaaaaaadaaaakeacaaaaaa mul r0.xyz, r2.w, r3.xyzz
adaaaaaaabaaahacaaaaaakeacaaaaaaadaaaaaaabaaaaaa mul r1.xyz, r0.xyzz, c3.x
aaaaaaaaaaaaabacafaaaaaaacaaaaaaaaaaaaaaaaaaaaaa mov r0.x, r5.x
adaaaaaaaaaaahacaaaaaaaaacaaaaaaabaaaakeacaaaaaa mul r0.xyz, r0.x, r1.xyzz
adaaaaaaabaaahacaaaaaakeacaaaaaaaaaaaaoeabaaaaaa mul r1.xyz, r0.xyzz, c0
bcaaaaaaaaaaabacagaaaakeacaaaaaaaeaaaakeacaaaaaa dp3 r0.x, r6.xyzz, r4.xyzz
ahaaaaaaaaaaabacaaaaaaaaacaaaaaaagaaaaffabaaaaaa max r0.x, r0.x, c6.y
adaaaaaaacaaahacacaaaakeacaaaaaaaaaaaaoeabaaaaaa mul r2.xyz, r2.xyzz, c0
adaaaaaaaaaaahacacaaaakeacaaaaaaaaaaaaaaacaaaaaa mul r0.xyz, r2.xyzz, r0.x
abaaaaaaaaaaahacaaaaaakeacaaaaaaabaaaakeacaaaaaa add r0.xyz, r0.xyzz, r1.xyzz
adaaaaaaaaaaahacabaaaappacaaaaaaaaaaaakeacaaaaaa mul r0.xyz, r1.w, r0.xyzz
adaaaaaaaaaaahacaaaaaakeacaaaaaaafaaaakkabaaaaaa mul r0.xyz, r0.xyzz, c5.z
aaaaaaaaaaaaapadaaaaaaoeacaaaaaaaaaaaaaaaaaaaaaa mov o0, r0
"
}

SubProgram "d3d11_9x " {
Keywords { "POINT" }
ConstBuffer "$Globals" 176 // 140 used size, 10 vars
Vector 16 [_LightColor0] 4
Vector 112 [_Color] 4
Float 128 [_Shininess]
Float 132 [_Gloss]
Float 136 [_Parallax]
BindCB "$Globals" 0
SetTexture 0 [_ParallaxMap] 2D 4
SetTexture 1 [_MainTex] 2D 1
SetTexture 2 [_SpecMap] 2D 3
SetTexture 3 [_BumpMap] 2D 2
SetTexture 4 [_LightTexture0] 2D 0
// 42 instructions, 5 temp regs, 0 temp arrays:
// ALU 27 float, 0 int, 0 uint
// TEX 5 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0_level_9_3
eefiecednbiopjciceakcafpndngnmfahboloakhabaaaaaalmakaaaaaeaaaaaa
daaaaaaapeadaaaaoiajaaaaiiakaaaaebgpgodjlmadaaaalmadaaaaaaacpppp
gmadaaaafaaaaaaaacaadiaaaaaafaaaaaaafaaaafaaceaaaaaafaaaaeaaaaaa
abababaaadacacaaacadadaaaaaeaeaaaaaaabaaabaaaaaaaaaaaaaaaaaaahaa
acaaabaaaaaaaaaaabacppppfbaaaaafadaaapkaaaaaaadpaaaaaaecdnaknhdo
aaaaiadpfbaaaaafaeaaapkaaaaaaaeaaaaaialpaaaaaaaaaaaaaaaabpaaaaac
aaaaaaiaaaaaaplabpaaaaacaaaaaaiaabaachlabpaaaaacaaaaaaiaacaachla
bpaaaaacaaaaaaiaadaaahlabpaaaaacaaaaaajaaaaiapkabpaaaaacaaaaaaja
abaiapkabpaaaaacaaaaaajaacaiapkabpaaaaacaaaaaajaadaiapkabpaaaaac
aaaaaajaaeaiapkaaiaaaaadaaaaciiaabaaoelaabaaoelaahaaaaacaaaacbia
aaaappiaaeaaaaaeaaaaaciaabaakklaaaaaaaiaadaakkkaafaaaaadaaaaania
aaaaaaiaabaajelaagaaaaacaaaaaciaaaaaffiaafaaaaadabaaadiaaaaaffia
aaaaoiiaabaaaaacacaaadiaaaaaoolaecaaaaadacaacpiaacaaoeiaaeaioeka
abaaaaacacaaafiaacaaoekaafaaaaadabaaamiaacaaceiaadaaeekaaeaaaaae
aaaacciaacaappiaacaakkkaabaakkibaeaaaaaeacaaadiaaaaaffiaabaaoeia
aaaaoolaaeaaaaaeabaaadiaaaaaffiaabaaoeiaaaaaoelaaiaaaaadadaaaiia
adaaoelaadaaoelaabaaaaacadaaadiaadaappiaecaaaaadacaacpiaacaaoeia
acaioekaecaaaaadadaacpiaadaaoeiaaaaioekaaeaaaaaeacaacdiaacaaohia
aeaaaakaaeaaffkaaeaaaaaeacaaciiaacaaaaiaacaaaaibadaappkaaeaaaaae
acaaciiaacaaffiaacaaffibacaappiaahaaaaacacaaciiaacaappiaagaaaaac
acaaceiaacaappiaaiaaaaadacaaciiaacaaoelaacaaoelaahaaaaacacaaciia
acaappiaaeaaaaaeaaaachiaacaaoelaacaappiaaaaapiiaafaaaaadadaacoia
acaappiaacaajalaaiaaaaadaaaaciiaacaaoeiaadaapjiaalaaaaadacaaciia
aaaappiaaeaakkkaceaaaaacaeaachiaaaaaoeiaaiaaaaadaaaacbiaacaaoeia
aeaaoeiaalaaaaadabaaaeiaaaaaaaiaaeaakkkacaaaaaadaaaaabiaabaakkia
abaappiaecaaaaadaeaacpiaabaaoeiaabaioekaecaaaaadabaacpiaabaaoeia
adaioekaafaaaaadaaaacoiaabaajaiaaeaappiaafaaaaadabaachiaaeaaoeia
abaaoekaafaaaaadabaachiaabaaoeiaaaaaoekaafaaaaadaaaacoiaaaaaoeia
acaaffkaafaaaaadaaaachiaaaaapjiaaaaaaaiaafaaaaadaaaachiaaaaaoeia
aaaaoekaaeaaaaaeaaaachiaabaaoeiaacaappiaaaaaoeiaacaaaaadaaaaciia
adaaaaiaadaaaaiaafaaaaadaaaachiaaaaappiaaaaaoeiaabaaaaacaaaaaiia
aeaakkkaabaaaaacaaaicpiaaaaaoeiappppaaaafdeieefcomafaaaaeaaaaaaa
hlabaaaafjaaaaaeegiocaaaaaaaaaaaajaaaaaafkaaaaadaagabaaaaaaaaaaa
fkaaaaadaagabaaaabaaaaaafkaaaaadaagabaaaacaaaaaafkaaaaadaagabaaa
adaaaaaafkaaaaadaagabaaaaeaaaaaafibiaaaeaahabaaaaaaaaaaaffffaaaa
fibiaaaeaahabaaaabaaaaaaffffaaaafibiaaaeaahabaaaacaaaaaaffffaaaa
fibiaaaeaahabaaaadaaaaaaffffaaaafibiaaaeaahabaaaaeaaaaaaffffaaaa
gcbaaaadpcbabaaaabaaaaaagcbaaaadhcbabaaaacaaaaaagcbaaaadhcbabaaa
adaaaaaagcbaaaadhcbabaaaaeaaaaaagfaaaaadpccabaaaaaaaaaaagiaaaaac
afaaaaaabaaaaaahbcaabaaaaaaaaaaaegbcbaaaadaaaaaaegbcbaaaadaaaaaa
eeaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaabaaaaaahccaabaaaaaaaaaaa
egbcbaaaacaaaaaaegbcbaaaacaaaaaaeeaaaaafccaabaaaaaaaaaaabkaabaaa
aaaaaaaadiaaaaahhcaabaaaabaaaaaafgafbaaaaaaaaaaaegbcbaaaacaaaaaa
dcaaaaajccaabaaaaaaaaaaackbabaaaacaaaaaabkaabaaaaaaaaaaaabeaaaaa
dnaknhdoaoaaaaahpcaabaaaacaaaaaaegaebaaaabaaaaaafgafbaaaaaaaaaaa
dcaaaaajocaabaaaaaaaaaaaagbjbaaaadaaaaaaagaabaaaaaaaaaaaagajbaaa
abaaaaaadiaaaaahhcaabaaaabaaaaaaagaabaaaaaaaaaaaegbcbaaaadaaaaaa
baaaaaahbcaabaaaaaaaaaaajgahbaaaaaaaaaaajgahbaaaaaaaaaaaeeaaaaaf
bcaabaaaaaaaaaaaakaabaaaaaaaaaaadiaaaaahhcaabaaaaaaaaaaaagaabaaa
aaaaaaaajgahbaaaaaaaaaaaefaaaaajpcaabaaaadaaaaaaogbkbaaaabaaaaaa
eghobaaaaaaaaaaaaagabaaaaeaaaaaadiaaaaaldcaabaaaadaaaaaacgikcaaa
aaaaaaaaaiaaaaaaaceaaaaaaaaaaadpaaaaaaecaaaaaaaaaaaaaaaadcaaaaal
icaabaaaaaaaaaaadkaabaaaadaaaaaackiacaaaaaaaaaaaaiaaaaaaakaabaia
ebaaaaaaadaaaaaadcaaaaajpcaabaaaacaaaaaapgapbaaaaaaaaaaaegaobaaa
acaaaaaaegbobaaaabaaaaaaefaaaaajpcaabaaaaeaaaaaaogakbaaaacaaaaaa
eghobaaaadaaaaaaaagabaaaacaaaaaadcaaaaapdcaabaaaaeaaaaaahgapbaaa
aeaaaaaaaceaaaaaaaaaaaeaaaaaaaeaaaaaaaaaaaaaaaaaaceaaaaaaaaaialp
aaaaialpaaaaaaaaaaaaaaaadcaaaaakicaabaaaaaaaaaaaakaabaiaebaaaaaa
aeaaaaaaakaabaaaaeaaaaaaabeaaaaaaaaaiadpdcaaaaakicaabaaaaaaaaaaa
bkaabaiaebaaaaaaaeaaaaaabkaabaaaaeaaaaaadkaabaaaaaaaaaaaelaaaaaf
ecaabaaaaeaaaaaadkaabaaaaaaaaaaabaaaaaahbcaabaaaaaaaaaaaegacbaaa
aeaaaaaaegacbaaaaaaaaaaabaaaaaahccaabaaaaaaaaaaaegacbaaaaeaaaaaa
egacbaaaabaaaaaadeaaaaakdcaabaaaaaaaaaaaegaabaaaaaaaaaaaaceaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaacpaaaaafbcaabaaaaaaaaaaaakaabaaa
aaaaaaaadiaaaaahbcaabaaaaaaaaaaaakaabaaaaaaaaaaabkaabaaaadaaaaaa
bjaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaaefaaaaajpcaabaaaabaaaaaa
egaabaaaacaaaaaaeghobaaaacaaaaaaaagabaaaadaaaaaaefaaaaajpcaabaaa
acaaaaaaegaabaaaacaaaaaaeghobaaaabaaaaaaaagabaaaabaaaaaadiaaaaah
hcaabaaaabaaaaaaegacbaaaabaaaaaapgapbaaaacaaaaaadiaaaaaihcaabaaa
acaaaaaaegacbaaaacaaaaaaegiccaaaaaaaaaaaahaaaaaadiaaaaaihcaabaaa
acaaaaaaegacbaaaacaaaaaaegiccaaaaaaaaaaaabaaaaaadiaaaaaihcaabaaa
abaaaaaaegacbaaaabaaaaaafgifcaaaaaaaaaaaaiaaaaaadiaaaaahncaabaaa
aaaaaaaaagaabaaaaaaaaaaaagajbaaaabaaaaaadiaaaaaincaabaaaaaaaaaaa
agaobaaaaaaaaaaaagijcaaaaaaaaaaaabaaaaaadcaaaaajhcaabaaaaaaaaaaa
egacbaaaacaaaaaafgafbaaaaaaaaaaaigadbaaaaaaaaaaabaaaaaahicaabaaa
aaaaaaaaegbcbaaaaeaaaaaaegbcbaaaaeaaaaaaefaaaaajpcaabaaaabaaaaaa
pgapbaaaaaaaaaaaeghobaaaaeaaaaaaaagabaaaaaaaaaaaaaaaaaahicaabaaa
aaaaaaaaakaabaaaabaaaaaaakaabaaaabaaaaaadiaaaaahhccabaaaaaaaaaaa
pgapbaaaaaaaaaaaegacbaaaaaaaaaaadgaaaaaficcabaaaaaaaaaaaabeaaaaa
aaaaaaaadoaaaaabejfdeheojiaaaaaaafaaaaaaaiaaaaaaiaaaaaaaaaaaaaaa
abaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaaaaaaaaaaaaaaaaaaadaaaaaa
abaaaaaaapapaaaaimaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaaahahaaaa
imaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaahahaaaaimaaaaaaadaaaaaa
aaaaaaaaadaaaaaaaeaaaaaaahahaaaafdfgfpfaepfdejfeejepeoaafeeffied
epepfceeaaklklklepfdeheocmaaaaaaabaaaaaaaiaaaaaacaaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaaaaaaaaaapaaaaaafdfgfpfegbhcghgfheaaklkl"
}

SubProgram "opengl " {
Keywords { "DIRECTIONAL" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
Float 4 [_Parallax]
SetTexture 0 [_ParallaxMap] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 2 [_SpecMap] 2D
SetTexture 3 [_BumpMap] 2D
"!!ARBfp1.0
OPTION ARB_precision_hint_fastest;
# 42 ALU, 4 TEX
PARAM c[7] = { program.local[0..4],
		{ 0, 0.5, 0.41999999, 2 },
		{ 1, 32 } };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
TEX R0.w, fragment.texcoord[0].zwzw, texture[0], 2D;
DP3 R0.x, fragment.texcoord[1], fragment.texcoord[1];
RSQ R1.w, R0.x;
MUL R1.xyz, R1.w, fragment.texcoord[1];
ADD R0.y, R1.z, c[5].z;
RCP R0.y, R0.y;
MOV R0.x, c[4];
MUL R0.x, R0, c[5].y;
MUL R2.xy, R1, R0.y;
MAD R0.x, R0.w, c[4], -R0;
MAD R1.xy, R0.x, R2, fragment.texcoord[0];
MAD R2.xy, R0.x, R2, fragment.texcoord[0].zwzw;
MOV R2.zw, c[6].xyxy;
MOV result.color.w, c[5].x;
TEX R0, R1, texture[1], 2D;
TEX R3.yw, R2, texture[3], 2D;
TEX R1.xyz, R1, texture[2], 2D;
MAD R2.xy, R3.wyzw, c[5].w, -R2.z;
MUL R1.xyz, R0.w, R1;
MOV R3.xyz, fragment.texcoord[2];
MUL R0.xyz, R0, c[1];
MAD R3.xyz, R1.w, fragment.texcoord[1], R3;
MUL R2.z, R2.y, R2.y;
MAD R1.w, -R2.x, R2.x, -R2.z;
DP3 R2.z, R3, R3;
RSQ R2.z, R2.z;
ADD R1.w, R1, c[6].x;
RSQ R1.w, R1.w;
MUL R3.xyz, R2.z, R3;
RCP R2.z, R1.w;
DP3 R1.w, R2, R3;
MAX R1.w, R1, c[5].x;
MUL R0.w, R2, c[2].x;
POW R0.w, R1.w, R0.w;
MUL R1.xyz, R1, c[3].x;
MUL R1.xyz, R0.w, R1;
DP3 R0.w, R2, fragment.texcoord[2];
MUL R1.xyz, R1, c[0];
MAX R0.w, R0, c[5].x;
MUL R0.xyz, R0, c[0];
MAD R0.xyz, R0, R0.w, R1;
MUL result.color.xyz, R0, c[5].w;
END
# 42 instructions, 4 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "DIRECTIONAL" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
Float 4 [_Parallax]
SetTexture 0 [_ParallaxMap] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 2 [_SpecMap] 2D
SetTexture 3 [_BumpMap] 2D
"ps_2_0
; 47 ALU, 4 TEX
dcl_2d s0
dcl_2d s1
dcl_2d s2
dcl_2d s3
def c5, 0.50000000, 0.41999999, 2.00000000, -1.00000000
def c6, 1.00000000, 0.00000000, 32.00000000, 0
dcl t0
dcl t1.xyz
dcl t2.xyz
mov r4.y, t0.w
mov r4.x, t0.z
mov r0.y, t0.w
mov r0.x, t0.z
texld r0, r0, s0
dp3_pp r0.x, t1, t1
rsq_pp r0.x, r0.x
mul_pp r3.xyz, r0.x, t1
add r1.x, r3.z, c5.y
rcp r2.x, r1.x
mov_pp r1.x, c5
mul r2.xy, r3, r2.x
mul_pp r1.x, c4, r1
mad_pp r1.x, r0.w, c4, -r1
mad r3.xy, r1.x, r2, t0
mad r1.xy, r1.x, r2, r4
mov_pp r4.xyz, t2
mad_pp r4.xyz, r0.x, t1, r4
mov_pp r0.w, c6.y
texld r2, r3, s1
texld r1, r1, s3
texld r3, r3, s2
mov r1.x, r1.w
mad_pp r5.xy, r1, c5.z, c5.w
mul_pp r1.x, r5.y, r5.y
mad_pp r1.x, -r5, r5, -r1
add_pp r0.x, r1, c6
dp3_pp r1.x, r4, r4
rsq_pp r0.x, r0.x
rcp_pp r5.z, r0.x
rsq_pp r1.x, r1.x
mul_pp r1.xyz, r1.x, r4
dp3_pp r1.x, r5, r1
mov_pp r0.x, c2
mul_pp r2.xyz, r2, c1
mul_pp r0.x, c6.z, r0
max_pp r1.x, r1, c6.y
pow r4.x, r1.x, r0.x
mul_pp r0.xyz, r2.w, r3
mul_pp r1.xyz, r0, c3.x
mov r0.x, r4.x
mul r0.xyz, r0.x, r1
mul_pp r1.xyz, r0, c0
dp3_pp r0.x, r5, t2
max_pp r0.x, r0, c6.y
mul_pp r2.xyz, r2, c0
mad_pp r0.xyz, r2, r0.x, r1
mul_pp r0.xyz, r0, c5.z
mov_pp oC0, r0
"
}

SubProgram "xbox360 " {
Keywords { "DIRECTIONAL" }
Vector 1 [_Color]
Float 3 [_Gloss]
Vector 0 [_LightColor0]
Float 4 [_Parallax]
Float 2 [_Shininess]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_BumpMap] 2D
SetTexture 2 [_SpecMap] 2D
SetTexture 3 [_ParallaxMap] 2D
// Shader Timing Estimate, in Cycles/64 pixel vector:
// ALU: 30.67 (23 instructions), vertex: 0, texture: 16,
//   sequencer: 12, interpolator: 12;    6 GPRs, 30 threads,
// Performance (if enough threads): ~30 cycles per vector
// * Texture cycle estimates are assuming an 8bit/component texture with no
//     aniso or trilinear filtering.

"ps_360
backbbaaaaaaabomaaaaableaaaaaaaaaaaaaaceaaaaabjiaaaaabmaaaaaaaaa
aaaaaaaaaaaaabhaaaaaaabmaaaaabgeppppadaaaaaaaaajaaaaaabmaaaaaaaa
aaaaabfnaaaaaanaaaadaaabaaabaaaaaaaaaanmaaaaaaaaaaaaaaomaaacaaab
aaabaaaaaaaaaapeaaaaaaaaaaaaabaeaaacaaadaaabaaaaaaaaabamaaaaaaaa
aaaaabbmaaacaaaaaaabaaaaaaaaaapeaaaaaaaaaaaaabcjaaadaaaaaaabaaaa
aaaaaanmaaaaaaaaaaaaabdcaaacaaaeaaabaaaaaaaaabamaaaaaaaaaaaaabdm
aaadaaadaaabaaaaaaaaaanmaaaaaaaaaaaaabejaaacaaacaaabaaaaaaaaabam
aaaaaaaaaaaaabfeaaadaaacaaabaaaaaaaaaanmaaaaaaaafpechfgnhaengbha
aaklklklaaaeaaamaaabaaabaaabaaaaaaaaaaaafpedgpgmgphcaaklaaabaaad
aaabaaaeaaabaaaaaaaaaaaafpehgmgphdhdaaklaaaaaaadaaabaaabaaabaaaa
aaaaaaaafpemgjghgiheedgpgmgphcdaaafpengbgjgofegfhiaafpfagbhcgbgm
gmgbhiaafpfagbhcgbgmgmgbhiengbhaaafpfdgigjgogjgogfhdhdaafpfdhagf
gdengbhaaahahdfpddfpdaaadccodacodcdadddfddcodaaaaaaaaaaaaaaaaaab
aaaaaaaaaaaaaaaaaaaaaabeabpmaabaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaeaaaaaabhebaaaafaaaaaaaaaeaaaaaaaaaaaacigdaaahaaahaaaaaaab
aaaapafaaaaahbfbaaaahcfcaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaeaaaaaaadpiaaaaaecaaaaaaaaaaaaaadpaaaaaa
aaaaaaaalpiaaaaadonhakdnaaajgaadgaajbcaabcaaafeaaaaaaaaagaapmeaa
bcaaaaaaaaaagabfdablbcaaccaaaaaadidadaabbpbpppplaaaaeaaamiaiaaac
aaloloaapaababaafiiiacabaagmgmblcbaeppicmiaiaaababgmgmblkladaeab
miahaaabaablloaaobacabaaleibacaeaagmmgaaabacpoppemiaacaaaaaaaabl
ocaaaaacmiadaaadaamfblaaobabacaamiapaaaaaakablaaoladabaalibieaab
bpbppompaaaaeaaabacidaabbpbppoiiaaaaeaaabaaiaaabbpbppgecaaaaeaaa
miahaaadaablmaaaobaaadaamiagaaaeaalggmmgilaepoppmiaiaaabaelclclb
nbaeaepobechaaafaagfmalboaabacaakaibaeabaaloloblpaafafibfibcabab
aamdlogmpaaeacibkibnacabaapagmebmbafababkicbacabaampmdicnaabaeab
kiegacabaalmlbmaicabppabeaboabaeaapmpmlbkbacaaibmiapaaaaaaaameaa
obaeabaadiboababaapmgmgmkbadadaamiahaaabaabfgmaaobababaamiahaaaa
aamamabfklabaaaamiahmaaaaamamaaaoaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
"
}

SubProgram "ps3 " {
Keywords { "DIRECTIONAL" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
Float 4 [_Parallax]
SetTexture 0 [_ParallaxMap] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 2 [_SpecMap] 2D
SetTexture 3 [_BumpMap] 2D
"sce_fp_rsx // 55 instructions using 4 registers
[Configuration]
24
ffffffff0001c0200007fff8000000000000840004000000
[Offsets]
5
_LightColor0 2 0
00000360000002c0
_Color 1 0
000001c0
_Shininess 1 0
000001e0
_Gloss 1 0
00000190
_Parallax 2 0
0000004000000020
[Microcode]
880
900017005c011c9dc8000001c8003fe102800240fe001c9d00020000c8000001
000000000000000000000000000000001080014000021c9cc8000001c8000001
00000000000000000000000000000000ae843940c8011c9dc8000029c800bfe1
08020300c9081c9d00020000c80000010a3d3ed7000000000000000000000000
02820440ff001c9daa020000c9000001000000000000bf000000000000000000
06003a00c9081c9d54040001c80000010606020001041c9cc8000001c8000001
9e040100c8011c9dc8000001c8003fe11806030080081c9c800c0000c8000001
1e7e7d00c8001c9dc8000001c8000001060003005c081c9dc80c0001c8000001
14001706c8001c9dc8000001c800000106860440ce001c9d00020000aa020000
000040000000bf800000000000000000ce800140c8011c9dc8000001c8003fe1
1602034049001c9d49080001c80000011e0417025c0c1c9dc8000001c8000001
0e84394068041c9dc8000029c800000110840240ab0c1c9cab0c0000c8000001
088a0240fe081c9d00020000c800000100000000000000000000000000000000
108a0440010c1c9e010c0000c90800030e880240c8081c9dc8020001c8000001
000000000000000000000000000000001084014000021c9cc8000001c8000001
000000000000000000000000000000000e0617045c0c1c9dc8000001c8000001
0e82024055141c9dc80c0001c800000110800340c9141c9d00020000c8000001
00003f8000000000000000000000000008863b40ff003c9dff000001c8000001
10800540c90c1c9dc9080001c800000110840240c9081c9d00020000c8000001
0000420000000000000000000000000002840540c90c1c9dc9000001c8000001
10020900c9001c9d00020000c800000100000000000000000000000000000000
08021d00fe041c9dc8000001c80000010e800240c9101c9dc8020001c8000001
000000000000000000000000000000000204020054041c9dff080001c8000001
1080090001081c9c00020000c800000100000000000000000000000000000000
0e800240c9001c9dff000001c800000102041c00c8081c9dc8000001c8000001
0e82020000081c9cc9040001c80000011080014000021c9cc8000001c8000001
000000000000000000000000000000000e810440c9041c9dc8021001c9000001
00000000000000000000000000000000
"
}

SubProgram "d3d11 " {
Keywords { "DIRECTIONAL" }
ConstBuffer "$Globals" 112 // 76 used size, 9 vars
Vector 16 [_LightColor0] 4
Vector 48 [_Color] 4
Float 64 [_Shininess]
Float 68 [_Gloss]
Float 72 [_Parallax]
BindCB "$Globals" 0
SetTexture 0 [_ParallaxMap] 2D 3
SetTexture 1 [_MainTex] 2D 0
SetTexture 2 [_SpecMap] 2D 2
SetTexture 3 [_BumpMap] 2D 1
// 36 instructions, 4 temp regs, 0 temp arrays:
// ALU 22 float, 0 int, 0 uint
// TEX 4 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0
eefiecedalmhejbimabplbaljhjahpbibglmlodkabaaaaaaamagaaaaadaaaaaa
cmaaaaaaleaaaaaaoiaaaaaaejfdeheoiaaaaaaaaeaaaaaaaiaaaaaagiaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaheaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaaheaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaaheaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaahahaaaafdfgfpfa
epfdejfeejepeoaafeeffiedepepfceeaaklklklepfdeheocmaaaaaaabaaaaaa
aiaaaaaacaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaafdfgfpfe
gbhcghgfheaaklklfdeieefcbmafaaaaeaaaaaaaehabaaaafjaaaaaeegiocaaa
aaaaaaaaafaaaaaafkaaaaadaagabaaaaaaaaaaafkaaaaadaagabaaaabaaaaaa
fkaaaaadaagabaaaacaaaaaafkaaaaadaagabaaaadaaaaaafibiaaaeaahabaaa
aaaaaaaaffffaaaafibiaaaeaahabaaaabaaaaaaffffaaaafibiaaaeaahabaaa
acaaaaaaffffaaaafibiaaaeaahabaaaadaaaaaaffffaaaagcbaaaadpcbabaaa
abaaaaaagcbaaaadhcbabaaaacaaaaaagcbaaaadhcbabaaaadaaaaaagfaaaaad
pccabaaaaaaaaaaagiaaaaacaeaaaaaabaaaaaahbcaabaaaaaaaaaaaegbcbaaa
acaaaaaaegbcbaaaacaaaaaaeeaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaa
dcaaaaajocaabaaaaaaaaaaaagbjbaaaacaaaaaaagaabaaaaaaaaaaaagbjbaaa
adaaaaaabaaaaaahbcaabaaaabaaaaaajgahbaaaaaaaaaaajgahbaaaaaaaaaaa
eeaaaaafbcaabaaaabaaaaaaakaabaaaabaaaaaadiaaaaahocaabaaaaaaaaaaa
fgaobaaaaaaaaaaaagaabaaaabaaaaaadiaaaaahdcaabaaaabaaaaaaagaabaaa
aaaaaaaaegbabaaaacaaaaaadcaaaaajbcaabaaaaaaaaaaackbabaaaacaaaaaa
akaabaaaaaaaaaaaabeaaaaadnaknhdoaoaaaaahpcaabaaaabaaaaaaegaebaaa
abaaaaaaagaabaaaaaaaaaaaefaaaaajpcaabaaaacaaaaaaogbkbaaaabaaaaaa
eghobaaaaaaaaaaaaagabaaaadaaaaaadiaaaaaldcaabaaaacaaaaaacgikcaaa
aaaaaaaaaeaaaaaaaceaaaaaaaaaaadpaaaaaaecaaaaaaaaaaaaaaaadcaaaaal
bcaabaaaaaaaaaaadkaabaaaacaaaaaackiacaaaaaaaaaaaaeaaaaaaakaabaia
ebaaaaaaacaaaaaadcaaaaajpcaabaaaabaaaaaaagaabaaaaaaaaaaaegaobaaa
abaaaaaaegbobaaaabaaaaaaefaaaaajpcaabaaaadaaaaaaogakbaaaabaaaaaa
eghobaaaadaaaaaaaagabaaaabaaaaaadcaaaaapdcaabaaaadaaaaaahgapbaaa
adaaaaaaaceaaaaaaaaaaaeaaaaaaaeaaaaaaaaaaaaaaaaaaceaaaaaaaaaialp
aaaaialpaaaaaaaaaaaaaaaadcaaaaakbcaabaaaaaaaaaaaakaabaiaebaaaaaa
adaaaaaaakaabaaaadaaaaaaabeaaaaaaaaaiadpdcaaaaakbcaabaaaaaaaaaaa
bkaabaiaebaaaaaaadaaaaaabkaabaaaadaaaaaaakaabaaaaaaaaaaaelaaaaaf
ecaabaaaadaaaaaaakaabaaaaaaaaaaabaaaaaahbcaabaaaaaaaaaaaegacbaaa
adaaaaaajgahbaaaaaaaaaaabaaaaaahccaabaaaaaaaaaaaegacbaaaadaaaaaa
egbcbaaaadaaaaaadeaaaaakdcaabaaaaaaaaaaaegaabaaaaaaaaaaaaceaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaacpaaaaafbcaabaaaaaaaaaaaakaabaaa
aaaaaaaadiaaaaahbcaabaaaaaaaaaaaakaabaaaaaaaaaaabkaabaaaacaaaaaa
bjaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaaefaaaaajpcaabaaaacaaaaaa
egaabaaaabaaaaaaeghobaaaacaaaaaaaagabaaaacaaaaaaefaaaaajpcaabaaa
abaaaaaaegaabaaaabaaaaaaeghobaaaabaaaaaaaagabaaaaaaaaaaadiaaaaah
hcaabaaaacaaaaaaegacbaaaacaaaaaapgapbaaaabaaaaaadiaaaaaihcaabaaa
abaaaaaaegacbaaaabaaaaaaegiccaaaaaaaaaaaadaaaaaadiaaaaaihcaabaaa
abaaaaaaegacbaaaabaaaaaaegiccaaaaaaaaaaaabaaaaaadiaaaaaihcaabaaa
acaaaaaaegacbaaaacaaaaaafgifcaaaaaaaaaaaaeaaaaaadiaaaaahncaabaaa
aaaaaaaaagaabaaaaaaaaaaaagajbaaaacaaaaaadiaaaaaincaabaaaaaaaaaaa
agaobaaaaaaaaaaaagijcaaaaaaaaaaaabaaaaaadcaaaaajhcaabaaaaaaaaaaa
egacbaaaabaaaaaafgafbaaaaaaaaaaaigadbaaaaaaaaaaaaaaaaaahhccabaaa
aaaaaaaaegacbaaaaaaaaaaaegacbaaaaaaaaaaadgaaaaaficcabaaaaaaaaaaa
abeaaaaaaaaaaaaadoaaaaab"
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
Float 4 [_Parallax]
SetTexture 0 [_ParallaxMap] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 2 [_SpecMap] 2D
SetTexture 3 [_BumpMap] 2D
"agal_ps
c5 0.5 0.42 2.0 -1.0
c6 1.0 0.0 32.0 0.0
[bc]
aaaaaaaaaeaaacacaaaaaappaeaaaaaaaaaaaaaaaaaaaaaa mov r4.y, v0.w
aaaaaaaaaeaaabacaaaaaakkaeaaaaaaaaaaaaaaaaaaaaaa mov r4.x, v0.z
aaaaaaaaaaaaacacaaaaaappaeaaaaaaaaaaaaaaaaaaaaaa mov r0.y, v0.w
aaaaaaaaaaaaabacaaaaaakkaeaaaaaaaaaaaaaaaaaaaaaa mov r0.x, v0.z
ciaaaaaaaaaaapacaaaaaafeacaaaaaaaaaaaaaaafaababb tex r0, r0.xyyy, s0 <2d wrap linear point>
bcaaaaaaaaaaabacabaaaaoeaeaaaaaaabaaaaoeaeaaaaaa dp3 r0.x, v1, v1
akaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r0.x, r0.x
adaaaaaaadaaahacaaaaaaaaacaaaaaaabaaaaoeaeaaaaaa mul r3.xyz, r0.x, v1
abaaaaaaabaaabacadaaaakkacaaaaaaafaaaaffabaaaaaa add r1.x, r3.z, c5.y
afaaaaaaacaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rcp r2.x, r1.x
aaaaaaaaabaaabacafaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r1.x, c5
adaaaaaaacaaadacadaaaafeacaaaaaaacaaaaaaacaaaaaa mul r2.xy, r3.xyyy, r2.x
adaaaaaaabaaabacaeaaaaoeabaaaaaaabaaaaaaacaaaaaa mul r1.x, c4, r1.x
adaaaaaaadaaaiacaaaaaappacaaaaaaaeaaaaoeabaaaaaa mul r3.w, r0.w, c4
acaaaaaaabaaabacadaaaappacaaaaaaabaaaaaaacaaaaaa sub r1.x, r3.w, r1.x
adaaaaaaadaaadacabaaaaaaacaaaaaaacaaaafeacaaaaaa mul r3.xy, r1.x, r2.xyyy
abaaaaaaadaaadacadaaaafeacaaaaaaaaaaaaoeaeaaaaaa add r3.xy, r3.xyyy, v0
adaaaaaaabaaadacabaaaaaaacaaaaaaacaaaafeacaaaaaa mul r1.xy, r1.x, r2.xyyy
abaaaaaaabaaadacabaaaafeacaaaaaaaeaaaafeacaaaaaa add r1.xy, r1.xyyy, r4.xyyy
aaaaaaaaaeaaahacacaaaaoeaeaaaaaaaaaaaaaaaaaaaaaa mov r4.xyz, v2
adaaaaaaafaaahacaaaaaaaaacaaaaaaabaaaaoeaeaaaaaa mul r5.xyz, r0.x, v1
abaaaaaaaeaaahacafaaaakeacaaaaaaaeaaaakeacaaaaaa add r4.xyz, r5.xyzz, r4.xyzz
aaaaaaaaaaaaaiacagaaaaffabaaaaaaaaaaaaaaaaaaaaaa mov r0.w, c6.y
ciaaaaaaacaaapacadaaaafeacaaaaaaabaaaaaaafaababb tex r2, r3.xyyy, s1 <2d wrap linear point>
ciaaaaaaabaaapacabaaaafeacaaaaaaadaaaaaaafaababb tex r1, r1.xyyy, s3 <2d wrap linear point>
ciaaaaaaadaaapacadaaaafeacaaaaaaacaaaaaaafaababb tex r3, r3.xyyy, s2 <2d wrap linear point>
aaaaaaaaabaaabacabaaaappacaaaaaaaaaaaaaaaaaaaaaa mov r1.x, r1.w
adaaaaaaafaaadacabaaaafeacaaaaaaafaaaakkabaaaaaa mul r5.xy, r1.xyyy, c5.z
abaaaaaaafaaadacafaaaafeacaaaaaaafaaaappabaaaaaa add r5.xy, r5.xyyy, c5.w
adaaaaaaabaaabacafaaaaffacaaaaaaafaaaaffacaaaaaa mul r1.x, r5.y, r5.y
bfaaaaaaaeaaaiacafaaaaaaacaaaaaaaaaaaaaaaaaaaaaa neg r4.w, r5.x
adaaaaaaaeaaaiacaeaaaappacaaaaaaafaaaaaaacaaaaaa mul r4.w, r4.w, r5.x
acaaaaaaabaaabacaeaaaappacaaaaaaabaaaaaaacaaaaaa sub r1.x, r4.w, r1.x
abaaaaaaaaaaabacabaaaaaaacaaaaaaagaaaaoeabaaaaaa add r0.x, r1.x, c6
bcaaaaaaabaaabacaeaaaakeacaaaaaaaeaaaakeacaaaaaa dp3 r1.x, r4.xyzz, r4.xyzz
akaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r0.x, r0.x
afaaaaaaafaaaeacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rcp r5.z, r0.x
akaaaaaaabaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r1.x, r1.x
adaaaaaaabaaahacabaaaaaaacaaaaaaaeaaaakeacaaaaaa mul r1.xyz, r1.x, r4.xyzz
bcaaaaaaabaaabacafaaaakeacaaaaaaabaaaakeacaaaaaa dp3 r1.x, r5.xyzz, r1.xyzz
aaaaaaaaaaaaabacacaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0.x, c2
adaaaaaaacaaahacacaaaakeacaaaaaaabaaaaoeabaaaaaa mul r2.xyz, r2.xyzz, c1
adaaaaaaaaaaabacagaaaakkabaaaaaaaaaaaaaaacaaaaaa mul r0.x, c6.z, r0.x
ahaaaaaaabaaabacabaaaaaaacaaaaaaagaaaaffabaaaaaa max r1.x, r1.x, c6.y
alaaaaaaaeaaapacabaaaaaaacaaaaaaaaaaaaaaacaaaaaa pow r4, r1.x, r0.x
adaaaaaaaaaaahacacaaaappacaaaaaaadaaaakeacaaaaaa mul r0.xyz, r2.w, r3.xyzz
adaaaaaaabaaahacaaaaaakeacaaaaaaadaaaaaaabaaaaaa mul r1.xyz, r0.xyzz, c3.x
aaaaaaaaaaaaabacaeaaaaaaacaaaaaaaaaaaaaaaaaaaaaa mov r0.x, r4.x
adaaaaaaaaaaahacaaaaaaaaacaaaaaaabaaaakeacaaaaaa mul r0.xyz, r0.x, r1.xyzz
adaaaaaaabaaahacaaaaaakeacaaaaaaaaaaaaoeabaaaaaa mul r1.xyz, r0.xyzz, c0
bcaaaaaaaaaaabacafaaaakeacaaaaaaacaaaaoeaeaaaaaa dp3 r0.x, r5.xyzz, v2
ahaaaaaaaaaaabacaaaaaaaaacaaaaaaagaaaaffabaaaaaa max r0.x, r0.x, c6.y
adaaaaaaacaaahacacaaaakeacaaaaaaaaaaaaoeabaaaaaa mul r2.xyz, r2.xyzz, c0
adaaaaaaaaaaahacacaaaakeacaaaaaaaaaaaaaaacaaaaaa mul r0.xyz, r2.xyzz, r0.x
abaaaaaaaaaaahacaaaaaakeacaaaaaaabaaaakeacaaaaaa add r0.xyz, r0.xyzz, r1.xyzz
adaaaaaaaaaaahacaaaaaakeacaaaaaaafaaaakkabaaaaaa mul r0.xyz, r0.xyzz, c5.z
aaaaaaaaaaaaapadaaaaaaoeacaaaaaaaaaaaaaaaaaaaaaa mov o0, r0
"
}

SubProgram "d3d11_9x " {
Keywords { "DIRECTIONAL" }
ConstBuffer "$Globals" 112 // 76 used size, 9 vars
Vector 16 [_LightColor0] 4
Vector 48 [_Color] 4
Float 64 [_Shininess]
Float 68 [_Gloss]
Float 72 [_Parallax]
BindCB "$Globals" 0
SetTexture 0 [_ParallaxMap] 2D 3
SetTexture 1 [_MainTex] 2D 0
SetTexture 2 [_SpecMap] 2D 2
SetTexture 3 [_BumpMap] 2D 1
// 36 instructions, 4 temp regs, 0 temp arrays:
// ALU 22 float, 0 int, 0 uint
// TEX 4 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0_level_9_3
eefiecedppmpinpblainkjbpafppakeifdccdllcabaaaaaafmajaaaaaeaaaaaa
daaaaaaahmadaaaakaaiaaaaciajaaaaebgpgodjeeadaaaaeeadaaaaaaacpppp
piacaaaaemaaaaaaacaadeaaaaaaemaaaaaaemaaaeaaceaaaaaaemaaabaaaaaa
adababaaacacacaaaaadadaaaaaaabaaabaaaaaaaaaaaaaaaaaaadaaacaaabaa
aaaaaaaaabacppppfbaaaaafadaaapkaaaaaaadpaaaaaaecdnaknhdoaaaaiadp
fbaaaaafaeaaapkaaaaaaaeaaaaaialpaaaaaaaaaaaaaaaabpaaaaacaaaaaaia
aaaaaplabpaaaaacaaaaaaiaabaachlabpaaaaacaaaaaaiaacaachlabpaaaaac
aaaaaajaaaaiapkabpaaaaacaaaaaajaabaiapkabpaaaaacaaaaaajaacaiapka
bpaaaaacaaaaaajaadaiapkaaiaaaaadaaaaciiaabaaoelaabaaoelaahaaaaac
aaaacbiaaaaappiaaeaaaaaeaaaaaciaabaakklaaaaaaaiaadaakkkaagaaaaac
aaaaaciaaaaaffiaafaaaaadaaaaamiaaaaaaaiaabaaeelaabaaaaacabaaahia
abaaoelaaeaaaaaeabaachiaabaaoeiaaaaaaaiaacaaoelaceaaaaacacaachia
abaaoeiaafaaaaadaaaaadiaaaaaffiaaaaaooiaabaaaaacabaaadiaaaaaoola
ecaaaaadabaacpiaabaaoeiaadaioekaabaaaaacabaaafiaacaaoekaafaaaaad
aaaaamiaabaaceiaadaaeekaaeaaaaaeacaaciiaabaappiaacaakkkaaaaakkib
aeaaaaaeabaaadiaacaappiaaaaaoeiaaaaaoolaaeaaaaaeaaaaadiaacaappia
aaaaoeiaaaaaoelaecaaaaadabaacpiaabaaoeiaabaioekaaeaaaaaeabaacdia
abaaohiaaeaaaakaaeaaffkaaeaaaaaeabaaciiaabaaaaiaabaaaaibadaappka
aeaaaaaeabaaciiaabaaffiaabaaffibabaappiaahaaaaacabaaciiaabaappia
agaaaaacabaaceiaabaappiaaiaaaaadabaaciiaabaaoeiaacaaoeiaaiaaaaad
aaaaceiaabaaoeiaacaaoelaalaaaaadabaacbiaaaaakkiaaeaakkkaalaaaaad
aaaaaeiaabaappiaaeaakkkacaaaaaadabaaaciaaaaakkiaaaaappiaecaaaaad
acaacpiaaaaaoeiaaaaioekaecaaaaadaaaacpiaaaaaoeiaacaioekaafaaaaad
aaaachiaaaaaoeiaacaappiaafaaaaadacaachiaacaaoeiaabaaoekaafaaaaad
acaachiaacaaoeiaaaaaoekaafaaaaadaaaachiaaaaaoeiaacaaffkaafaaaaad
aaaachiaaaaaoeiaabaaffiaafaaaaadaaaachiaaaaaoeiaaaaaoekaaeaaaaae
aaaachiaacaaoeiaabaaaaiaaaaaoeiaacaaaaadaaaachiaaaaaoeiaaaaaoeia
abaaaaacaaaaciiaaeaakkkaabaaaaacaaaicpiaaaaaoeiappppaaaafdeieefc
bmafaaaaeaaaaaaaehabaaaafjaaaaaeegiocaaaaaaaaaaaafaaaaaafkaaaaad
aagabaaaaaaaaaaafkaaaaadaagabaaaabaaaaaafkaaaaadaagabaaaacaaaaaa
fkaaaaadaagabaaaadaaaaaafibiaaaeaahabaaaaaaaaaaaffffaaaafibiaaae
aahabaaaabaaaaaaffffaaaafibiaaaeaahabaaaacaaaaaaffffaaaafibiaaae
aahabaaaadaaaaaaffffaaaagcbaaaadpcbabaaaabaaaaaagcbaaaadhcbabaaa
acaaaaaagcbaaaadhcbabaaaadaaaaaagfaaaaadpccabaaaaaaaaaaagiaaaaac
aeaaaaaabaaaaaahbcaabaaaaaaaaaaaegbcbaaaacaaaaaaegbcbaaaacaaaaaa
eeaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaadcaaaaajocaabaaaaaaaaaaa
agbjbaaaacaaaaaaagaabaaaaaaaaaaaagbjbaaaadaaaaaabaaaaaahbcaabaaa
abaaaaaajgahbaaaaaaaaaaajgahbaaaaaaaaaaaeeaaaaafbcaabaaaabaaaaaa
akaabaaaabaaaaaadiaaaaahocaabaaaaaaaaaaafgaobaaaaaaaaaaaagaabaaa
abaaaaaadiaaaaahdcaabaaaabaaaaaaagaabaaaaaaaaaaaegbabaaaacaaaaaa
dcaaaaajbcaabaaaaaaaaaaackbabaaaacaaaaaaakaabaaaaaaaaaaaabeaaaaa
dnaknhdoaoaaaaahpcaabaaaabaaaaaaegaebaaaabaaaaaaagaabaaaaaaaaaaa
efaaaaajpcaabaaaacaaaaaaogbkbaaaabaaaaaaeghobaaaaaaaaaaaaagabaaa
adaaaaaadiaaaaaldcaabaaaacaaaaaacgikcaaaaaaaaaaaaeaaaaaaaceaaaaa
aaaaaadpaaaaaaecaaaaaaaaaaaaaaaadcaaaaalbcaabaaaaaaaaaaadkaabaaa
acaaaaaackiacaaaaaaaaaaaaeaaaaaaakaabaiaebaaaaaaacaaaaaadcaaaaaj
pcaabaaaabaaaaaaagaabaaaaaaaaaaaegaobaaaabaaaaaaegbobaaaabaaaaaa
efaaaaajpcaabaaaadaaaaaaogakbaaaabaaaaaaeghobaaaadaaaaaaaagabaaa
abaaaaaadcaaaaapdcaabaaaadaaaaaahgapbaaaadaaaaaaaceaaaaaaaaaaaea
aaaaaaeaaaaaaaaaaaaaaaaaaceaaaaaaaaaialpaaaaialpaaaaaaaaaaaaaaaa
dcaaaaakbcaabaaaaaaaaaaaakaabaiaebaaaaaaadaaaaaaakaabaaaadaaaaaa
abeaaaaaaaaaiadpdcaaaaakbcaabaaaaaaaaaaabkaabaiaebaaaaaaadaaaaaa
bkaabaaaadaaaaaaakaabaaaaaaaaaaaelaaaaafecaabaaaadaaaaaaakaabaaa
aaaaaaaabaaaaaahbcaabaaaaaaaaaaaegacbaaaadaaaaaajgahbaaaaaaaaaaa
baaaaaahccaabaaaaaaaaaaaegacbaaaadaaaaaaegbcbaaaadaaaaaadeaaaaak
dcaabaaaaaaaaaaaegaabaaaaaaaaaaaaceaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaacpaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaadiaaaaahbcaabaaa
aaaaaaaaakaabaaaaaaaaaaabkaabaaaacaaaaaabjaaaaafbcaabaaaaaaaaaaa
akaabaaaaaaaaaaaefaaaaajpcaabaaaacaaaaaaegaabaaaabaaaaaaeghobaaa
acaaaaaaaagabaaaacaaaaaaefaaaaajpcaabaaaabaaaaaaegaabaaaabaaaaaa
eghobaaaabaaaaaaaagabaaaaaaaaaaadiaaaaahhcaabaaaacaaaaaaegacbaaa
acaaaaaapgapbaaaabaaaaaadiaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaa
egiccaaaaaaaaaaaadaaaaaadiaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaa
egiccaaaaaaaaaaaabaaaaaadiaaaaaihcaabaaaacaaaaaaegacbaaaacaaaaaa
fgifcaaaaaaaaaaaaeaaaaaadiaaaaahncaabaaaaaaaaaaaagaabaaaaaaaaaaa
agajbaaaacaaaaaadiaaaaaincaabaaaaaaaaaaaagaobaaaaaaaaaaaagijcaaa
aaaaaaaaabaaaaaadcaaaaajhcaabaaaaaaaaaaaegacbaaaabaaaaaafgafbaaa
aaaaaaaaigadbaaaaaaaaaaaaaaaaaahhccabaaaaaaaaaaaegacbaaaaaaaaaaa
egacbaaaaaaaaaaadgaaaaaficcabaaaaaaaaaaaabeaaaaaaaaaaaaadoaaaaab
ejfdeheoiaaaaaaaaeaaaaaaaiaaaaaagiaaaaaaaaaaaaaaabaaaaaaadaaaaaa
aaaaaaaaapaaaaaaheaaaaaaaaaaaaaaaaaaaaaaadaaaaaaabaaaaaaapapaaaa
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
Float 4 [_Parallax]
SetTexture 0 [_ParallaxMap] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 2 [_SpecMap] 2D
SetTexture 3 [_BumpMap] 2D
SetTexture 4 [_LightTexture0] 2D
SetTexture 5 [_LightTextureB0] 2D
"!!ARBfp1.0
OPTION ARB_precision_hint_fastest;
# 53 ALU, 6 TEX
PARAM c[7] = { program.local[0..4],
		{ 0, 0.5, 0.41999999, 2 },
		{ 1, 32 } };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
TEMP R4;
TEX R0.w, fragment.texcoord[0].zwzw, texture[0], 2D;
DP3 R0.x, fragment.texcoord[1], fragment.texcoord[1];
RSQ R1.z, R0.x;
MUL R2.xyz, R1.z, fragment.texcoord[1];
ADD R0.y, R2.z, c[5].z;
RCP R0.y, R0.y;
MOV R0.x, c[4];
MUL R0.x, R0, c[5].y;
DP3 R1.w, fragment.texcoord[3], fragment.texcoord[3];
MUL R1.xy, R2, R0.y;
MAD R0.x, R0.w, c[4], -R0;
MAD R3.xy, R0.x, R1, fragment.texcoord[0].zwzw;
MAD R0.xy, R0.x, R1, fragment.texcoord[0];
RCP R0.z, fragment.texcoord[3].w;
MAD R1.xy, fragment.texcoord[3], R0.z, c[5].y;
MOV R3.zw, c[6].xyxy;
MOV result.color.w, c[5].x;
TEX R2, R0, texture[1], 2D;
TEX R0.w, R1, texture[4], 2D;
TEX R0.xyz, R0, texture[2], 2D;
TEX R4.yw, R3, texture[3], 2D;
TEX R1.w, R1.w, texture[5], 2D;
MUL R0.xyz, R2.w, R0;
DP3 R1.x, fragment.texcoord[2], fragment.texcoord[2];
RSQ R3.x, R1.x;
MAD R1.xy, R4.wyzw, c[5].w, -R3.z;
MUL R3.xyz, R3.x, fragment.texcoord[2];
MUL R4.w, R1.y, R1.y;
MAD R4.xyz, R1.z, fragment.texcoord[1], R3;
MAD R1.z, -R1.x, R1.x, -R4.w;
DP3 R4.w, R4, R4;
ADD R1.z, R1, c[6].x;
RSQ R4.w, R4.w;
RSQ R1.z, R1.z;
RCP R1.z, R1.z;
MUL R4.xyz, R4.w, R4;
DP3 R4.x, R1, R4;
MAX R4.x, R4, c[5];
MUL R2.w, R3, c[2].x;
POW R2.w, R4.x, R2.w;
MUL R0.xyz, R0, c[3].x;
MUL R0.xyz, R2.w, R0;
DP3 R2.w, R1, R3;
MUL R1.xyz, R2, c[1];
MUL R0.xyz, R0, c[0];
MAX R2.x, R2.w, c[5];
MUL R1.xyz, R1, c[0];
MAD R1.xyz, R1, R2.x, R0;
SLT R0.x, c[5], fragment.texcoord[3].z;
MUL R0.x, R0, R0.w;
MUL R0.x, R0, R1.w;
MUL R0.xyz, R0.x, R1;
MUL result.color.xyz, R0, c[5].w;
END
# 53 instructions, 5 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "SPOT" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
Float 4 [_Parallax]
SetTexture 0 [_ParallaxMap] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 2 [_SpecMap] 2D
SetTexture 3 [_BumpMap] 2D
SetTexture 4 [_LightTexture0] 2D
SetTexture 5 [_LightTextureB0] 2D
"ps_2_0
; 57 ALU, 6 TEX
dcl_2d s0
dcl_2d s1
dcl_2d s2
dcl_2d s3
dcl_2d s4
dcl_2d s5
def c5, 0.00000000, 1.00000000, 0.50000000, 0.41999999
def c6, 2.00000000, -1.00000000, 32.00000000, 0
dcl t0
dcl t1.xyz
dcl t2.xyz
dcl t3
mov r0.y, t0.w
mov r0.x, t0.z
texld r0, r0, s0
dp3_pp r0.x, t1, t1
rsq_pp r0.x, r0.x
mul_pp r3.xyz, r0.x, t1
mov_pp r0.z, c5
add r2.x, r3.z, c5.w
mul_pp r1.x, c4, r0.z
mad_pp r1.x, r0.w, c4, -r1
rcp r2.x, r2.x
mul r2.xy, r3, r2.x
mov r3.y, t0.w
mov r3.x, t0.z
mad r4.xy, r1.x, r2, r3
mad r3.xy, r1.x, r2, t0
dp3 r2.x, t3, t3
mov r5.xy, r2.x
rcp r1.x, t3.w
mad r1.xy, t3, r1.x, c5.z
mov_pp r0.w, c5.x
texld r2, r3, s1
texld r4, r4, s3
texld r1, r1, s4
texld r7, r5, s5
texld r3, r3, s2
mov r4.x, r4.w
mul_pp r2.xyz, r2, c1
mad_pp r5.xy, r4, c6.x, c6.y
dp3_pp r1.x, t2, t2
rsq_pp r4.x, r1.x
mul_pp r4.xyz, r4.x, t2
mul_pp r1.x, r5.y, r5.y
mad_pp r6.xyz, r0.x, t1, r4
mad_pp r1.x, -r5, r5, -r1
add_pp r0.x, r1, c5.y
dp3_pp r1.x, r6, r6
rsq_pp r0.x, r0.x
rcp_pp r5.z, r0.x
rsq_pp r1.x, r1.x
mul_pp r1.xyz, r1.x, r6
dp3_pp r1.x, r5, r1
mov_pp r0.x, c2
mul_pp r0.x, c6.z, r0
max_pp r1.x, r1, c5
pow r6.x, r1.x, r0.x
mul_pp r0.xyz, r2.w, r3
mul_pp r1.xyz, r0, c3.x
mov r0.x, r6.x
mul r0.xyz, r0.x, r1
mul_pp r1.xyz, r0, c0
dp3_pp r0.x, r5, r4
max_pp r0.x, r0, c5
mul_pp r2.xyz, r2, c0
mad_pp r1.xyz, r2, r0.x, r1
cmp r0.x, -t3.z, c5, c5.y
mul_pp r0.x, r0, r1.w
mul_pp r0.x, r0, r7
mul_pp r0.xyz, r0.x, r1
mul_pp r0.xyz, r0, c6.x
mov_pp oC0, r0
"
}

SubProgram "xbox360 " {
Keywords { "SPOT" }
Vector 1 [_Color]
Float 3 [_Gloss]
Vector 0 [_LightColor0]
Float 4 [_Parallax]
Float 2 [_Shininess]
SetTexture 0 [_LightTexture0] 2D
SetTexture 1 [_LightTextureB0] 2D
SetTexture 2 [_MainTex] 2D
SetTexture 3 [_BumpMap] 2D
SetTexture 4 [_SpecMap] 2D
SetTexture 5 [_ParallaxMap] 2D
// Shader Timing Estimate, in Cycles/64 pixel vector:
// ALU: 38.67 (29 instructions), vertex: 0, texture: 24,
//   sequencer: 14, interpolator: 16;    7 GPRs, 27 threads,
// Performance (if enough threads): ~38 cycles per vector
// * Texture cycle estimates are assuming an 8bit/component texture with no
//     aniso or trilinear filtering.

"ps_360
backbbaaaaaaacdiaaaaaccaaaaaaaaaaaaaaaceaaaaaboaaaaaacaiaaaaaaaa
aaaaaaaaaaaaabliaaaaaabmaaaaabklppppadaaaaaaaaalaaaaaabmaaaaaaaa
aaaaabkeaaaaaapiaaadaaadaaabaaaaaaaaabaeaaaaaaaaaaaaabbeaaacaaab
aaabaaaaaaaaabbmaaaaaaaaaaaaabcmaaacaaadaaabaaaaaaaaabdeaaaaaaaa
aaaaabeeaaacaaaaaaabaaaaaaaaabbmaaaaaaaaaaaaabfbaaadaaaaaaabaaaa
aaaaabaeaaaaaaaaaaaaabgaaaadaaabaaabaaaaaaaaabaeaaaaaaaaaaaaabha
aaadaaacaaabaaaaaaaaabaeaaaaaaaaaaaaabhjaaacaaaeaaabaaaaaaaaabde
aaaaaaaaaaaaabidaaadaaafaaabaaaaaaaaabaeaaaaaaaaaaaaabjaaaacaaac
aaabaaaaaaaaabdeaaaaaaaaaaaaabjlaaadaaaeaaabaaaaaaaaabaeaaaaaaaa
fpechfgnhaengbhaaaklklklaaaeaaamaaabaaabaaabaaaaaaaaaaaafpedgpgm
gphcaaklaaabaaadaaabaaaeaaabaaaaaaaaaaaafpehgmgphdhdaaklaaaaaaad
aaabaaabaaabaaaaaaaaaaaafpemgjghgiheedgpgmgphcdaaafpemgjghgihefe
gfhihehfhcgfdaaafpemgjghgihefegfhihehfhcgfecdaaafpengbgjgofegfhi
aafpfagbhcgbgmgmgbhiaafpfagbhcgbgmgmgbhiengbhaaafpfdgigjgogjgogf
hdhdaafpfdhagfgdengbhaaahahdfpddfpdaaadccodacodcdadddfddcodaaakl
aaaaaaaaaaaaaaabaaaaaaaaaaaaaaaaaaaaaabeabpmaabaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaeaaaaaaboabaaaagaaaaaaaaaeaaaaaaaaaaaadiie
aaapaaapaaaaaaabaaaapafaaaaahbfbaaaahcfcaaaapdfdaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaalpiaaaaaecaaaaaa
aaaaaaaaaaaaaaaadpaaaaaaaaaaaaaadonhakdndpiaaaaaacfagaaegaakbcaa
bcaaaaaaabfefabaaaaabcaameaaaaaaaaaagabfgablbcaabcaaaaaaaaaagacb
aaaaccaaaaaaaaaaemibabaeaaloloblpaacacadmiagaaaeaabllmgmmlabadpp
geaabaibbpbpphppaaaaeaaadifaeaabbpbpppnpaaaaeaaamiaeaaaeaaloloaa
paababaamiaiaaacaagmgmaacbaeppaamiacaaaeablbgmblklaeaeacfieiaeac
aalolomgpaadadiemiahaaabaamgloaaobaeabaaleeaaeaaaaaaaamamcaaaapp
embcadadaamglbmgkfadppaemialaaadaabflmaaobabadaamiapaaaaaakalbaa
oladaeaalidigaabbpbppompaaaaeaaapmbicaebbpbppbppaaaaeaaabaeieaab
bpbppeehaaaaeaaabaciaaabbpbppgecaaaaeaaafibhaeadaablbfgmobaaaeie
miaoaaaeaagmpmaaobaeacaamiahaaafaabfgfaaoaaeabaabeaiaaabaalologm
naafafacambbaeagaablbllbmbadacpobechaaabaalelelboaagagaafibgacac
aambgmblkaabpoibmiaiaaabaelclcblnbacacppkaihacafaamagmblobafacib
kicbaeacaamdmdebnaaeacabkiecaeacaalomdicnaafacabkiigaeacaalmlbma
icacppabeaboacaeaaabpmmgkbaeaaicmiapaaaaaaaalaaaobaeacaadiihabac
aamagmgmkbadadaamiahaaacaamablaaobacabaamiahaaaaaamamabfklacaaaa
miahmaaaaagmmaaaobabaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "SPOT" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
Float 4 [_Parallax]
SetTexture 0 [_ParallaxMap] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 2 [_SpecMap] 2D
SetTexture 3 [_BumpMap] 2D
SetTexture 4 [_LightTexture0] 2D
SetTexture 5 [_LightTextureB0] 2D
"sce_fp_rsx // 70 instructions using 5 registers
[Configuration]
24
ffffffff0003c020000ffff0000000000000840005000000
[Offsets]
5
_LightColor0 2 0
0000042000000290
_Color 1 0
00000250
_Shininess 1 0
00000210
_Gloss 1 0
00000140
_Parallax 2 0
0000004000000020
[Microcode]
1120
900017005c011c9dc8000001c8003fe102800240fe001c9d00020000c8000001
000000000000000000000000000000001088014000021c9cc8000001c8000001
00000000000000000000000000000000ae883940c8011c9dc8000029c800bfe1
08040300c9101c9d00020000c80000010a3d3ed7000000000000000000000000
02820440ff101c9daa020000c9000001000000000000bf000000000000000000
06003a00c9101c9d54080001c80000011804020001041c9c80000000c8000001
9e020100c8011c9dc8000001c8003fe1060003005c041c9d5c080001c8000001
14001706c8001c9dc8000001c80000011804030080041c9cc8080001c8000001
1e0217025c081c9dc8000001c800000106900440ce001c9daa02000054020001
00000000000040000000bf800000000008860240fe041c9d00020000c8000001
00000000000000000000000000000000fe000100c8011c9dc8000001c8003fe1
10900240ab201c9cab200000c80000011086044001201c9e01200000c9200003
06063a00c8001c9dfe000001c800000106060300c80c1c9d00020000c8000001
00003f0000000000000000000000000010001708c80c1c9dc8000001c8000001
ce8c3940c8011c9dc8000029c800bfe1108c0340c90c1c9d00020000c8000001
00003f8000000000000000000000000002000500c8001c9dc8000001c8000001
1086014000021c9cc8000001c800000100000000000000000000000000000000
10860240c90c1c9dc8020001c800000100000000000000000000000000004200
0e840240c8041c9dc8020001c800000100000000000000000000000000000000
08903b40ff183c9dff180001c800000106020100c8041c9dc8000001c8000001
0e840240c9081c9dc8020001c800000100000000000000000000000000000000
08800540c9201c9dc9180001c80000010e060340c9101c9dc9180001c8000001
0e883940c80c1c9dc8000029c800000110800540c9201c9dc9100001c8000001
1e7e7d00c8001c9dc8000001c80000011084090055001c9d00020000c8000001
0000000000000000000000000000000008020900ff001c9d00020000c8000001
0000000000000000000000000000000008021d0054041c9dc8000001c8000001
0e840240c9081c9dff080001c800000108000100c8001c9dc8000001c8000001
0200170a00001c9cc8000001c80000010206020054041c9dff0c0001c8000001
10800d0054001c9d00020000c800000100000000000000000000000000000000
0e0417045c081c9dc8000001c800000108021c00c80c1c9dc8000001c8000001
0e880240550c1c9dc8080001c800000108020100c8041c9dc8000001c8000001
10800240c9001c9dc8000001c80000010e82020054041c9dc9100001c8000001
10800240c9001c9d00000000c80000010e800440c9041c9dc8020001c9080001
000000000000000000000000000000000e800240ff001c9dc9001001c8000001
1081014000021c9cc8000001c800000100000000000000000000000000000000
"
}

SubProgram "d3d11 " {
Keywords { "SPOT" }
ConstBuffer "$Globals" 176 // 140 used size, 10 vars
Vector 16 [_LightColor0] 4
Vector 112 [_Color] 4
Float 128 [_Shininess]
Float 132 [_Gloss]
Float 136 [_Parallax]
BindCB "$Globals" 0
SetTexture 0 [_ParallaxMap] 2D 5
SetTexture 1 [_MainTex] 2D 2
SetTexture 2 [_SpecMap] 2D 4
SetTexture 3 [_BumpMap] 2D 3
SetTexture 4 [_LightTexture0] 2D 0
SetTexture 5 [_LightTextureB0] 2D 1
// 48 instructions, 5 temp regs, 0 temp arrays:
// ALU 31 float, 0 int, 1 uint
// TEX 6 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0
eefieceddajnglaejpoemdgkkehjhmlhaddgcbeeabaaaaaammahaaaaadaaaaaa
cmaaaaaammaaaaaaaaabaaaaejfdeheojiaaaaaaafaaaaaaaiaaaaaaiaaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaaimaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaahahaaaaimaaaaaa
adaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapapaaaafdfgfpfaepfdejfeejepeoaa
feeffiedepepfceeaaklklklepfdeheocmaaaaaaabaaaaaaaiaaaaaacaaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaafdfgfpfegbhcghgfheaaklkl
fdeieefcmeagaaaaeaaaaaaalbabaaaafjaaaaaeegiocaaaaaaaaaaaajaaaaaa
fkaaaaadaagabaaaaaaaaaaafkaaaaadaagabaaaabaaaaaafkaaaaadaagabaaa
acaaaaaafkaaaaadaagabaaaadaaaaaafkaaaaadaagabaaaaeaaaaaafkaaaaad
aagabaaaafaaaaaafibiaaaeaahabaaaaaaaaaaaffffaaaafibiaaaeaahabaaa
abaaaaaaffffaaaafibiaaaeaahabaaaacaaaaaaffffaaaafibiaaaeaahabaaa
adaaaaaaffffaaaafibiaaaeaahabaaaaeaaaaaaffffaaaafibiaaaeaahabaaa
afaaaaaaffffaaaagcbaaaadpcbabaaaabaaaaaagcbaaaadhcbabaaaacaaaaaa
gcbaaaadhcbabaaaadaaaaaagcbaaaadpcbabaaaaeaaaaaagfaaaaadpccabaaa
aaaaaaaagiaaaaacafaaaaaabaaaaaahbcaabaaaaaaaaaaaegbcbaaaadaaaaaa
egbcbaaaadaaaaaaeeaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaabaaaaaah
ccaabaaaaaaaaaaaegbcbaaaacaaaaaaegbcbaaaacaaaaaaeeaaaaafccaabaaa
aaaaaaaabkaabaaaaaaaaaaadiaaaaahhcaabaaaabaaaaaafgafbaaaaaaaaaaa
egbcbaaaacaaaaaadcaaaaajccaabaaaaaaaaaaackbabaaaacaaaaaabkaabaaa
aaaaaaaaabeaaaaadnaknhdoaoaaaaahpcaabaaaacaaaaaaegaebaaaabaaaaaa
fgafbaaaaaaaaaaadcaaaaajocaabaaaaaaaaaaaagbjbaaaadaaaaaaagaabaaa
aaaaaaaaagajbaaaabaaaaaadiaaaaahhcaabaaaabaaaaaaagaabaaaaaaaaaaa
egbcbaaaadaaaaaabaaaaaahbcaabaaaaaaaaaaajgahbaaaaaaaaaaajgahbaaa
aaaaaaaaeeaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaadiaaaaahhcaabaaa
aaaaaaaaagaabaaaaaaaaaaajgahbaaaaaaaaaaaefaaaaajpcaabaaaadaaaaaa
ogbkbaaaabaaaaaaeghobaaaaaaaaaaaaagabaaaafaaaaaadiaaaaaldcaabaaa
adaaaaaacgikcaaaaaaaaaaaaiaaaaaaaceaaaaaaaaaaadpaaaaaaecaaaaaaaa
aaaaaaaadcaaaaalicaabaaaaaaaaaaadkaabaaaadaaaaaackiacaaaaaaaaaaa
aiaaaaaaakaabaiaebaaaaaaadaaaaaadcaaaaajpcaabaaaacaaaaaapgapbaaa
aaaaaaaaegaobaaaacaaaaaaegbobaaaabaaaaaaefaaaaajpcaabaaaaeaaaaaa
ogakbaaaacaaaaaaeghobaaaadaaaaaaaagabaaaadaaaaaadcaaaaapdcaabaaa
aeaaaaaahgapbaaaaeaaaaaaaceaaaaaaaaaaaeaaaaaaaeaaaaaaaaaaaaaaaaa
aceaaaaaaaaaialpaaaaialpaaaaaaaaaaaaaaaadcaaaaakicaabaaaaaaaaaaa
akaabaiaebaaaaaaaeaaaaaaakaabaaaaeaaaaaaabeaaaaaaaaaiadpdcaaaaak
icaabaaaaaaaaaaabkaabaiaebaaaaaaaeaaaaaabkaabaaaaeaaaaaadkaabaaa
aaaaaaaaelaaaaafecaabaaaaeaaaaaadkaabaaaaaaaaaaabaaaaaahbcaabaaa
aaaaaaaaegacbaaaaeaaaaaaegacbaaaaaaaaaaabaaaaaahccaabaaaaaaaaaaa
egacbaaaaeaaaaaaegacbaaaabaaaaaadeaaaaakdcaabaaaaaaaaaaaegaabaaa
aaaaaaaaaceaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaacpaaaaafbcaabaaa
aaaaaaaaakaabaaaaaaaaaaadiaaaaahbcaabaaaaaaaaaaaakaabaaaaaaaaaaa
bkaabaaaadaaaaaabjaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaaefaaaaaj
pcaabaaaabaaaaaaegaabaaaacaaaaaaeghobaaaacaaaaaaaagabaaaaeaaaaaa
efaaaaajpcaabaaaacaaaaaaegaabaaaacaaaaaaeghobaaaabaaaaaaaagabaaa
acaaaaaadiaaaaahhcaabaaaabaaaaaaegacbaaaabaaaaaapgapbaaaacaaaaaa
diaaaaaihcaabaaaacaaaaaaegacbaaaacaaaaaaegiccaaaaaaaaaaaahaaaaaa
diaaaaaihcaabaaaacaaaaaaegacbaaaacaaaaaaegiccaaaaaaaaaaaabaaaaaa
diaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaafgifcaaaaaaaaaaaaiaaaaaa
diaaaaahncaabaaaaaaaaaaaagaabaaaaaaaaaaaagajbaaaabaaaaaadiaaaaai
ncaabaaaaaaaaaaaagaobaaaaaaaaaaaagijcaaaaaaaaaaaabaaaaaadcaaaaaj
hcaabaaaaaaaaaaaegacbaaaacaaaaaafgafbaaaaaaaaaaaigadbaaaaaaaaaaa
aoaaaaahdcaabaaaabaaaaaaegbabaaaaeaaaaaapgbpbaaaaeaaaaaaaaaaaaak
dcaabaaaabaaaaaaegaabaaaabaaaaaaaceaaaaaaaaaaadpaaaaaadpaaaaaaaa
aaaaaaaaefaaaaajpcaabaaaabaaaaaaegaabaaaabaaaaaaeghobaaaaeaaaaaa
aagabaaaaaaaaaaadbaaaaahicaabaaaaaaaaaaaabeaaaaaaaaaaaaackbabaaa
aeaaaaaaabaaaaahicaabaaaaaaaaaaadkaabaaaaaaaaaaaabeaaaaaaaaaiadp
diaaaaahicaabaaaaaaaaaaadkaabaaaabaaaaaadkaabaaaaaaaaaaabaaaaaah
bcaabaaaabaaaaaaegbcbaaaaeaaaaaaegbcbaaaaeaaaaaaefaaaaajpcaabaaa
abaaaaaaagaabaaaabaaaaaaeghobaaaafaaaaaaaagabaaaabaaaaaaapaaaaah
icaabaaaaaaaaaaapgapbaaaaaaaaaaaagaabaaaabaaaaaadiaaaaahhccabaaa
aaaaaaaapgapbaaaaaaaaaaaegacbaaaaaaaaaaadgaaaaaficcabaaaaaaaaaaa
abeaaaaaaaaaaaaadoaaaaab"
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
Float 4 [_Parallax]
SetTexture 0 [_ParallaxMap] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 2 [_SpecMap] 2D
SetTexture 3 [_BumpMap] 2D
SetTexture 4 [_LightTexture0] 2D
SetTexture 5 [_LightTextureB0] 2D
"agal_ps
c5 0.0 1.0 0.5 0.42
c6 2.0 -1.0 32.0 0.0
[bc]
aaaaaaaaaaaaacacaaaaaappaeaaaaaaaaaaaaaaaaaaaaaa mov r0.y, v0.w
aaaaaaaaaaaaabacaaaaaakkaeaaaaaaaaaaaaaaaaaaaaaa mov r0.x, v0.z
ciaaaaaaaaaaapacaaaaaafeacaaaaaaaaaaaaaaafaababb tex r0, r0.xyyy, s0 <2d wrap linear point>
bcaaaaaaaaaaabacabaaaaoeaeaaaaaaabaaaaoeaeaaaaaa dp3 r0.x, v1, v1
akaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r0.x, r0.x
adaaaaaaadaaahacaaaaaaaaacaaaaaaabaaaaoeaeaaaaaa mul r3.xyz, r0.x, v1
aaaaaaaaaaaaaeacafaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0.z, c5
abaaaaaaacaaabacadaaaakkacaaaaaaafaaaappabaaaaaa add r2.x, r3.z, c5.w
adaaaaaaabaaabacaeaaaaoeabaaaaaaaaaaaakkacaaaaaa mul r1.x, c4, r0.z
adaaaaaaadaaaiacaaaaaappacaaaaaaaeaaaaoeabaaaaaa mul r3.w, r0.w, c4
acaaaaaaabaaabacadaaaappacaaaaaaabaaaaaaacaaaaaa sub r1.x, r3.w, r1.x
afaaaaaaacaaabacacaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rcp r2.x, r2.x
adaaaaaaacaaadacadaaaafeacaaaaaaacaaaaaaacaaaaaa mul r2.xy, r3.xyyy, r2.x
aaaaaaaaadaaacacaaaaaappaeaaaaaaaaaaaaaaaaaaaaaa mov r3.y, v0.w
aaaaaaaaadaaabacaaaaaakkaeaaaaaaaaaaaaaaaaaaaaaa mov r3.x, v0.z
adaaaaaaafaaadacabaaaaaaacaaaaaaacaaaafeacaaaaaa mul r5.xy, r1.x, r2.xyyy
abaaaaaaafaaadacafaaaafeacaaaaaaadaaaafeacaaaaaa add r5.xy, r5.xyyy, r3.xyyy
adaaaaaaadaaadacabaaaaaaacaaaaaaacaaaafeacaaaaaa mul r3.xy, r1.x, r2.xyyy
abaaaaaaadaaadacadaaaafeacaaaaaaaaaaaaoeaeaaaaaa add r3.xy, r3.xyyy, v0
afaaaaaaacaaabacadaaaappaeaaaaaaaaaaaaaaaaaaaaaa rcp r2.x, v3.w
adaaaaaaaeaaadacadaaaaoeaeaaaaaaacaaaaaaacaaaaaa mul r4.xy, v3, r2.x
abaaaaaaaeaaadacaeaaaafeacaaaaaaafaaaakkabaaaaaa add r4.xy, r4.xyyy, c5.z
bcaaaaaaabaaabacadaaaaoeaeaaaaaaadaaaaoeaeaaaaaa dp3 r1.x, v3, v3
aaaaaaaaabaaadacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa mov r1.xy, r1.x
aaaaaaaaaaaaaiacafaaaaaaabaaaaaaaaaaaaaaaaaaaaaa mov r0.w, c5.x
ciaaaaaaacaaapacadaaaafeacaaaaaaabaaaaaaafaababb tex r2, r3.xyyy, s1 <2d wrap linear point>
ciaaaaaaafaaapacafaaaafeacaaaaaaadaaaaaaafaababb tex r5, r5.xyyy, s3 <2d wrap linear point>
ciaaaaaaabaaapacabaaaafeacaaaaaaafaaaaaaafaababb tex r1, r1.xyyy, s5 <2d wrap linear point>
ciaaaaaaadaaapacadaaaafeacaaaaaaacaaaaaaafaababb tex r3, r3.xyyy, s2 <2d wrap linear point>
ciaaaaaaaeaaapacaeaaaafeacaaaaaaaeaaaaaaafaababb tex r4, r4.xyyy, s4 <2d wrap linear point>
adaaaaaaacaaahacacaaaakeacaaaaaaabaaaaoeabaaaaaa mul r2.xyz, r2.xyzz, c1
aaaaaaaaaeaaacacafaaaaffacaaaaaaaaaaaaaaaaaaaaaa mov r4.y, r5.y
aaaaaaaaaeaaabacafaaaappacaaaaaaaaaaaaaaaaaaaaaa mov r4.x, r5.w
adaaaaaaafaaadacaeaaaafeacaaaaaaagaaaaaaabaaaaaa mul r5.xy, r4.xyyy, c6.x
abaaaaaaafaaadacafaaaafeacaaaaaaagaaaaffabaaaaaa add r5.xy, r5.xyyy, c6.y
bcaaaaaaabaaabacacaaaaoeaeaaaaaaacaaaaoeaeaaaaaa dp3 r1.x, v2, v2
akaaaaaaaeaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r4.x, r1.x
adaaaaaaaeaaahacaeaaaaaaacaaaaaaacaaaaoeaeaaaaaa mul r4.xyz, r4.x, v2
adaaaaaaabaaabacafaaaaffacaaaaaaafaaaaffacaaaaaa mul r1.x, r5.y, r5.y
adaaaaaaagaaahacaaaaaaaaacaaaaaaabaaaaoeaeaaaaaa mul r6.xyz, r0.x, v1
abaaaaaaagaaahacagaaaakeacaaaaaaaeaaaakeacaaaaaa add r6.xyz, r6.xyzz, r4.xyzz
bfaaaaaaagaaaiacafaaaaaaacaaaaaaaaaaaaaaaaaaaaaa neg r6.w, r5.x
adaaaaaaagaaaiacagaaaappacaaaaaaafaaaaaaacaaaaaa mul r6.w, r6.w, r5.x
acaaaaaaabaaabacagaaaappacaaaaaaabaaaaaaacaaaaaa sub r1.x, r6.w, r1.x
abaaaaaaaaaaabacabaaaaaaacaaaaaaafaaaaffabaaaaaa add r0.x, r1.x, c5.y
bcaaaaaaabaaabacagaaaakeacaaaaaaagaaaakeacaaaaaa dp3 r1.x, r6.xyzz, r6.xyzz
akaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r0.x, r0.x
afaaaaaaafaaaeacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rcp r5.z, r0.x
akaaaaaaabaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r1.x, r1.x
adaaaaaaabaaahacabaaaaaaacaaaaaaagaaaakeacaaaaaa mul r1.xyz, r1.x, r6.xyzz
bcaaaaaaabaaabacafaaaakeacaaaaaaabaaaakeacaaaaaa dp3 r1.x, r5.xyzz, r1.xyzz
aaaaaaaaaaaaabacacaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0.x, c2
adaaaaaaaaaaabacagaaaakkabaaaaaaaaaaaaaaacaaaaaa mul r0.x, c6.z, r0.x
ahaaaaaaabaaabacabaaaaaaacaaaaaaafaaaaoeabaaaaaa max r1.x, r1.x, c5
alaaaaaaagaaapacabaaaaaaacaaaaaaaaaaaaaaacaaaaaa pow r6, r1.x, r0.x
adaaaaaaaaaaahacacaaaappacaaaaaaadaaaakeacaaaaaa mul r0.xyz, r2.w, r3.xyzz
adaaaaaaabaaahacaaaaaakeacaaaaaaadaaaaaaabaaaaaa mul r1.xyz, r0.xyzz, c3.x
aaaaaaaaaaaaabacagaaaaaaacaaaaaaaaaaaaaaaaaaaaaa mov r0.x, r6.x
adaaaaaaaaaaahacaaaaaaaaacaaaaaaabaaaakeacaaaaaa mul r0.xyz, r0.x, r1.xyzz
adaaaaaaabaaahacaaaaaakeacaaaaaaaaaaaaoeabaaaaaa mul r1.xyz, r0.xyzz, c0
bcaaaaaaaaaaabacafaaaakeacaaaaaaaeaaaakeacaaaaaa dp3 r0.x, r5.xyzz, r4.xyzz
ahaaaaaaaaaaabacaaaaaaaaacaaaaaaafaaaaoeabaaaaaa max r0.x, r0.x, c5
adaaaaaaacaaahacacaaaakeacaaaaaaaaaaaaoeabaaaaaa mul r2.xyz, r2.xyzz, c0
adaaaaaaacaaahacacaaaakeacaaaaaaaaaaaaaaacaaaaaa mul r2.xyz, r2.xyzz, r0.x
abaaaaaaabaaahacacaaaakeacaaaaaaabaaaakeacaaaaaa add r1.xyz, r2.xyzz, r1.xyzz
bfaaaaaaacaaaeacadaaaakkaeaaaaaaaaaaaaaaaaaaaaaa neg r2.z, v3.z
ckaaaaaaaaaaabacacaaaakkacaaaaaaagaaaappabaaaaaa slt r0.x, r2.z, c6.w
adaaaaaaaaaaabacaaaaaaaaacaaaaaaaeaaaappacaaaaaa mul r0.x, r0.x, r4.w
adaaaaaaaaaaabacaaaaaaaaacaaaaaaabaaaappacaaaaaa mul r0.x, r0.x, r1.w
adaaaaaaaaaaahacaaaaaaaaacaaaaaaabaaaakeacaaaaaa mul r0.xyz, r0.x, r1.xyzz
adaaaaaaaaaaahacaaaaaakeacaaaaaaagaaaaaaabaaaaaa mul r0.xyz, r0.xyzz, c6.x
aaaaaaaaaaaaapadaaaaaaoeacaaaaaaaaaaaaaaaaaaaaaa mov o0, r0
"
}

SubProgram "d3d11_9x " {
Keywords { "SPOT" }
ConstBuffer "$Globals" 176 // 140 used size, 10 vars
Vector 16 [_LightColor0] 4
Vector 112 [_Color] 4
Float 128 [_Shininess]
Float 132 [_Gloss]
Float 136 [_Parallax]
BindCB "$Globals" 0
SetTexture 0 [_ParallaxMap] 2D 5
SetTexture 1 [_MainTex] 2D 2
SetTexture 2 [_SpecMap] 2D 4
SetTexture 3 [_BumpMap] 2D 3
SetTexture 4 [_LightTexture0] 2D 0
SetTexture 5 [_LightTextureB0] 2D 1
// 48 instructions, 5 temp regs, 0 temp arrays:
// ALU 31 float, 0 int, 1 uint
// TEX 6 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0_level_9_3
eefiecedpohjfffmlkcdomdhpddnlmjmjmlmdlaoabaaaaaaomalaaaaaeaaaaaa
daaaaaaaemaeaaaabialaaaalialaaaaebgpgodjbeaeaaaabeaeaaaaaaacpppp
maadaaaafeaaaaaaacaadmaaaaaafeaaaaaafeaaagaaceaaaaaafeaaaeaaaaaa
afababaaabacacaaadadadaaacaeaeaaaaafafaaaaaaabaaabaaaaaaaaaaaaaa
aaaaahaaacaaabaaaaaaaaaaabacppppfbaaaaafadaaapkaaaaaaadpaaaaaaec
dnaknhdoaaaaiadpfbaaaaafaeaaapkaaaaaaaeaaaaaialpaaaaaaaaaaaaaaaa
bpaaaaacaaaaaaiaaaaaaplabpaaaaacaaaaaaiaabaachlabpaaaaacaaaaaaia
acaachlabpaaaaacaaaaaaiaadaaaplabpaaaaacaaaaaajaaaaiapkabpaaaaac
aaaaaajaabaiapkabpaaaaacaaaaaajaacaiapkabpaaaaacaaaaaajaadaiapka
bpaaaaacaaaaaajaaeaiapkabpaaaaacaaaaaajaafaiapkaaiaaaaadaaaaciia
abaaoelaabaaoelaahaaaaacaaaacbiaaaaappiaaeaaaaaeaaaaaciaabaakkla
aaaaaaiaadaakkkaafaaaaadaaaaaniaaaaaaaiaabaajelaagaaaaacaaaaacia
aaaaffiaafaaaaadabaaadiaaaaaffiaaaaaoiiaabaaaaacacaaadiaaaaaoola
ecaaaaadacaacpiaacaaoeiaafaioekaabaaaaacacaaafiaacaaoekaafaaaaad
abaaamiaacaaceiaadaaeekaaeaaaaaeaaaacciaacaappiaacaakkkaabaakkib
aeaaaaaeacaaadiaaaaaffiaabaaoeiaaaaaoolaaeaaaaaeabaaadiaaaaaffia
abaaoeiaaaaaoelaagaaaaacaaaaaciaadaapplaaeaaaaaeadaaadiaadaaoela
aaaaffiaadaaaakaecaaaaadacaacpiaacaaoeiaadaioekaecaaaaadadaacpia
adaaoeiaaaaioekaaeaaaaaeacaacdiaacaaohiaaeaaaakaaeaaffkaaeaaaaae
acaaciiaacaaaaiaacaaaaibadaappkaaeaaaaaeacaaciiaacaaffiaacaaffib
acaappiaahaaaaacacaaciiaacaappiaagaaaaacacaaceiaacaappiaaiaaaaad
acaaciiaacaaoelaacaaoelaahaaaaacacaaciiaacaappiaaeaaaaaeaaaachia
acaaoelaacaappiaaaaapiiaafaaaaadadaachiaacaappiaacaaoelaaiaaaaad
aaaaciiaacaaoeiaadaaoeiaalaaaaadacaaciiaaaaappiaaeaakkkaceaaaaac
adaachiaaaaaoeiaaiaaaaadaaaacbiaacaaoeiaadaaoeiaalaaaaadabaaaeia
aaaaaaiaaeaakkkacaaaaaadaaaaabiaabaakkiaabaappiaecaaaaadaeaacpia
abaaoeiaacaioekaecaaaaadabaacpiaabaaoeiaaeaioekaafaaaaadaaaacoia
abaajaiaaeaappiaafaaaaadabaachiaaeaaoeiaabaaoekaafaaaaadabaachia
abaaoeiaaaaaoekaafaaaaadaaaacoiaaaaaoeiaacaaffkaafaaaaadaaaachia
aaaapjiaaaaaaaiaafaaaaadaaaachiaaaaaoeiaaaaaoekaaeaaaaaeaaaachia
abaaoeiaacaappiaaaaaoeiaaiaaaaadabaaadiaadaaoelaadaaoelaecaaaaad
abaacpiaabaaoeiaabaioekaafaaaaadaaaaciiaabaaaaiaadaappiafiaaaaae
aaaaciiaadaakklbaeaakkkaaaaappiaacaaaaadaaaaciiaaaaappiaaaaappia
afaaaaadaaaachiaaaaappiaaaaaoeiaabaaaaacaaaaaiiaaeaakkkaabaaaaac
aaaicpiaaaaaoeiappppaaaafdeieefcmeagaaaaeaaaaaaalbabaaaafjaaaaae
egiocaaaaaaaaaaaajaaaaaafkaaaaadaagabaaaaaaaaaaafkaaaaadaagabaaa
abaaaaaafkaaaaadaagabaaaacaaaaaafkaaaaadaagabaaaadaaaaaafkaaaaad
aagabaaaaeaaaaaafkaaaaadaagabaaaafaaaaaafibiaaaeaahabaaaaaaaaaaa
ffffaaaafibiaaaeaahabaaaabaaaaaaffffaaaafibiaaaeaahabaaaacaaaaaa
ffffaaaafibiaaaeaahabaaaadaaaaaaffffaaaafibiaaaeaahabaaaaeaaaaaa
ffffaaaafibiaaaeaahabaaaafaaaaaaffffaaaagcbaaaadpcbabaaaabaaaaaa
gcbaaaadhcbabaaaacaaaaaagcbaaaadhcbabaaaadaaaaaagcbaaaadpcbabaaa
aeaaaaaagfaaaaadpccabaaaaaaaaaaagiaaaaacafaaaaaabaaaaaahbcaabaaa
aaaaaaaaegbcbaaaadaaaaaaegbcbaaaadaaaaaaeeaaaaafbcaabaaaaaaaaaaa
akaabaaaaaaaaaaabaaaaaahccaabaaaaaaaaaaaegbcbaaaacaaaaaaegbcbaaa
acaaaaaaeeaaaaafccaabaaaaaaaaaaabkaabaaaaaaaaaaadiaaaaahhcaabaaa
abaaaaaafgafbaaaaaaaaaaaegbcbaaaacaaaaaadcaaaaajccaabaaaaaaaaaaa
ckbabaaaacaaaaaabkaabaaaaaaaaaaaabeaaaaadnaknhdoaoaaaaahpcaabaaa
acaaaaaaegaebaaaabaaaaaafgafbaaaaaaaaaaadcaaaaajocaabaaaaaaaaaaa
agbjbaaaadaaaaaaagaabaaaaaaaaaaaagajbaaaabaaaaaadiaaaaahhcaabaaa
abaaaaaaagaabaaaaaaaaaaaegbcbaaaadaaaaaabaaaaaahbcaabaaaaaaaaaaa
jgahbaaaaaaaaaaajgahbaaaaaaaaaaaeeaaaaafbcaabaaaaaaaaaaaakaabaaa
aaaaaaaadiaaaaahhcaabaaaaaaaaaaaagaabaaaaaaaaaaajgahbaaaaaaaaaaa
efaaaaajpcaabaaaadaaaaaaogbkbaaaabaaaaaaeghobaaaaaaaaaaaaagabaaa
afaaaaaadiaaaaaldcaabaaaadaaaaaacgikcaaaaaaaaaaaaiaaaaaaaceaaaaa
aaaaaadpaaaaaaecaaaaaaaaaaaaaaaadcaaaaalicaabaaaaaaaaaaadkaabaaa
adaaaaaackiacaaaaaaaaaaaaiaaaaaaakaabaiaebaaaaaaadaaaaaadcaaaaaj
pcaabaaaacaaaaaapgapbaaaaaaaaaaaegaobaaaacaaaaaaegbobaaaabaaaaaa
efaaaaajpcaabaaaaeaaaaaaogakbaaaacaaaaaaeghobaaaadaaaaaaaagabaaa
adaaaaaadcaaaaapdcaabaaaaeaaaaaahgapbaaaaeaaaaaaaceaaaaaaaaaaaea
aaaaaaeaaaaaaaaaaaaaaaaaaceaaaaaaaaaialpaaaaialpaaaaaaaaaaaaaaaa
dcaaaaakicaabaaaaaaaaaaaakaabaiaebaaaaaaaeaaaaaaakaabaaaaeaaaaaa
abeaaaaaaaaaiadpdcaaaaakicaabaaaaaaaaaaabkaabaiaebaaaaaaaeaaaaaa
bkaabaaaaeaaaaaadkaabaaaaaaaaaaaelaaaaafecaabaaaaeaaaaaadkaabaaa
aaaaaaaabaaaaaahbcaabaaaaaaaaaaaegacbaaaaeaaaaaaegacbaaaaaaaaaaa
baaaaaahccaabaaaaaaaaaaaegacbaaaaeaaaaaaegacbaaaabaaaaaadeaaaaak
dcaabaaaaaaaaaaaegaabaaaaaaaaaaaaceaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaacpaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaadiaaaaahbcaabaaa
aaaaaaaaakaabaaaaaaaaaaabkaabaaaadaaaaaabjaaaaafbcaabaaaaaaaaaaa
akaabaaaaaaaaaaaefaaaaajpcaabaaaabaaaaaaegaabaaaacaaaaaaeghobaaa
acaaaaaaaagabaaaaeaaaaaaefaaaaajpcaabaaaacaaaaaaegaabaaaacaaaaaa
eghobaaaabaaaaaaaagabaaaacaaaaaadiaaaaahhcaabaaaabaaaaaaegacbaaa
abaaaaaapgapbaaaacaaaaaadiaaaaaihcaabaaaacaaaaaaegacbaaaacaaaaaa
egiccaaaaaaaaaaaahaaaaaadiaaaaaihcaabaaaacaaaaaaegacbaaaacaaaaaa
egiccaaaaaaaaaaaabaaaaaadiaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaa
fgifcaaaaaaaaaaaaiaaaaaadiaaaaahncaabaaaaaaaaaaaagaabaaaaaaaaaaa
agajbaaaabaaaaaadiaaaaaincaabaaaaaaaaaaaagaobaaaaaaaaaaaagijcaaa
aaaaaaaaabaaaaaadcaaaaajhcaabaaaaaaaaaaaegacbaaaacaaaaaafgafbaaa
aaaaaaaaigadbaaaaaaaaaaaaoaaaaahdcaabaaaabaaaaaaegbabaaaaeaaaaaa
pgbpbaaaaeaaaaaaaaaaaaakdcaabaaaabaaaaaaegaabaaaabaaaaaaaceaaaaa
aaaaaadpaaaaaadpaaaaaaaaaaaaaaaaefaaaaajpcaabaaaabaaaaaaegaabaaa
abaaaaaaeghobaaaaeaaaaaaaagabaaaaaaaaaaadbaaaaahicaabaaaaaaaaaaa
abeaaaaaaaaaaaaackbabaaaaeaaaaaaabaaaaahicaabaaaaaaaaaaadkaabaaa
aaaaaaaaabeaaaaaaaaaiadpdiaaaaahicaabaaaaaaaaaaadkaabaaaabaaaaaa
dkaabaaaaaaaaaaabaaaaaahbcaabaaaabaaaaaaegbcbaaaaeaaaaaaegbcbaaa
aeaaaaaaefaaaaajpcaabaaaabaaaaaaagaabaaaabaaaaaaeghobaaaafaaaaaa
aagabaaaabaaaaaaapaaaaahicaabaaaaaaaaaaapgapbaaaaaaaaaaaagaabaaa
abaaaaaadiaaaaahhccabaaaaaaaaaaapgapbaaaaaaaaaaaegacbaaaaaaaaaaa
dgaaaaaficcabaaaaaaaaaaaabeaaaaaaaaaaaaadoaaaaabejfdeheojiaaaaaa
afaaaaaaaiaaaaaaiaaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaa
imaaaaaaaaaaaaaaaaaaaaaaadaaaaaaabaaaaaaapapaaaaimaaaaaaabaaaaaa
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
Float 4 [_Parallax]
SetTexture 0 [_ParallaxMap] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 2 [_SpecMap] 2D
SetTexture 3 [_BumpMap] 2D
SetTexture 4 [_LightTextureB0] 2D
SetTexture 5 [_LightTexture0] CUBE
"!!ARBfp1.0
OPTION ARB_precision_hint_fastest;
# 49 ALU, 6 TEX
PARAM c[7] = { program.local[0..4],
		{ 0, 0.5, 0.41999999, 2 },
		{ 1, 32 } };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
TEMP R4;
TEX R0.w, fragment.texcoord[0].zwzw, texture[0], 2D;
TEX R1.w, fragment.texcoord[3], texture[5], CUBE;
DP3 R0.x, fragment.texcoord[1], fragment.texcoord[1];
RSQ R1.z, R0.x;
MUL R2.xyz, R1.z, fragment.texcoord[1];
ADD R0.y, R2.z, c[5].z;
RCP R0.y, R0.y;
MOV R0.x, c[4];
MUL R0.x, R0, c[5].y;
MAD R0.x, R0.w, c[4], -R0;
MUL R2.xy, R2, R0.y;
MAD R1.xy, R0.x, R2, fragment.texcoord[0].zwzw;
MAD R0.xy, R0.x, R2, fragment.texcoord[0];
DP3 R0.w, fragment.texcoord[3], fragment.texcoord[3];
MOV R3.zw, c[6].xyxy;
MOV result.color.w, c[5].x;
TEX R2, R0, texture[1], 2D;
TEX R4.yw, R1, texture[3], 2D;
TEX R0.xyz, R0, texture[2], 2D;
TEX R0.w, R0.w, texture[4], 2D;
MUL R0.xyz, R2.w, R0;
DP3 R1.x, fragment.texcoord[2], fragment.texcoord[2];
RSQ R3.x, R1.x;
MAD R1.xy, R4.wyzw, c[5].w, -R3.z;
MUL R3.xyz, R3.x, fragment.texcoord[2];
MUL R4.w, R1.y, R1.y;
MAD R4.xyz, R1.z, fragment.texcoord[1], R3;
MAD R1.z, -R1.x, R1.x, -R4.w;
DP3 R4.w, R4, R4;
ADD R1.z, R1, c[6].x;
RSQ R4.w, R4.w;
RSQ R1.z, R1.z;
RCP R1.z, R1.z;
MUL R4.xyz, R4.w, R4;
DP3 R4.x, R1, R4;
MAX R4.x, R4, c[5];
MUL R2.w, R3, c[2].x;
POW R2.w, R4.x, R2.w;
MUL R0.xyz, R0, c[3].x;
MUL R0.xyz, R2.w, R0;
DP3 R2.w, R1, R3;
MUL R1.xyz, R2, c[1];
MUL R0.xyz, R0, c[0];
MAX R2.x, R2.w, c[5];
MUL R1.xyz, R1, c[0];
MAD R1.xyz, R1, R2.x, R0;
MUL R0.x, R0.w, R1.w;
MUL R0.xyz, R0.x, R1;
MUL result.color.xyz, R0, c[5].w;
END
# 49 instructions, 5 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "POINT_COOKIE" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
Float 4 [_Parallax]
SetTexture 0 [_ParallaxMap] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 2 [_SpecMap] 2D
SetTexture 3 [_BumpMap] 2D
SetTexture 4 [_LightTextureB0] 2D
SetTexture 5 [_LightTexture0] CUBE
"ps_2_0
; 53 ALU, 6 TEX
dcl_2d s0
dcl_2d s1
dcl_2d s2
dcl_2d s3
dcl_2d s4
dcl_cube s5
def c5, 0.50000000, 0.41999999, 2.00000000, -1.00000000
def c6, 1.00000000, 0.00000000, 32.00000000, 0
dcl t0
dcl t1.xyz
dcl t2.xyz
dcl t3.xyz
mov_pp r1.x, c5
mov r2.y, t0.w
mov r0.y, t0.w
mov r0.x, t0.z
mul_pp r1.x, c4, r1
texld r0, r0, s0
dp3_pp r0.x, t1, t1
rsq_pp r0.x, r0.x
mul_pp r3.xyz, r0.x, t1
mad_pp r1.x, r0.w, c4, -r1
add r2.x, r3.z, c5.y
rcp r2.x, r2.x
mul r3.xy, r3, r2.x
mov r2.x, t0.z
mad r4.xy, r1.x, r3, r2
mad r3.xy, r1.x, r3, t0
dp3 r2.x, t3, t3
mov r1.xy, r2.x
mov_pp r0.w, c6.y
texld r2, r3, s1
texld r7, r1, s4
texld r4, r4, s3
texld r1, t3, s5
texld r3, r3, s2
mov r4.x, r4.w
mul_pp r2.xyz, r2, c1
mad_pp r5.xy, r4, c5.z, c5.w
dp3_pp r1.x, t2, t2
rsq_pp r4.x, r1.x
mul_pp r4.xyz, r4.x, t2
mul_pp r1.x, r5.y, r5.y
mad_pp r6.xyz, r0.x, t1, r4
mad_pp r1.x, -r5, r5, -r1
add_pp r0.x, r1, c6
dp3_pp r1.x, r6, r6
rsq_pp r0.x, r0.x
rcp_pp r5.z, r0.x
rsq_pp r1.x, r1.x
mul_pp r1.xyz, r1.x, r6
dp3_pp r1.x, r5, r1
mov_pp r0.x, c2
mul_pp r0.x, c6.z, r0
max_pp r1.x, r1, c6.y
pow r6.x, r1.x, r0.x
mul_pp r0.xyz, r2.w, r3
mul_pp r1.xyz, r0, c3.x
mov r0.x, r6.x
mul r0.xyz, r0.x, r1
mul_pp r1.xyz, r0, c0
dp3_pp r0.x, r5, r4
max_pp r0.x, r0, c6.y
mul_pp r2.xyz, r2, c0
mad_pp r1.xyz, r2, r0.x, r1
mul r0.x, r7, r1.w
mul_pp r0.xyz, r0.x, r1
mul_pp r0.xyz, r0, c5.z
mov_pp oC0, r0
"
}

SubProgram "xbox360 " {
Keywords { "POINT_COOKIE" }
Vector 1 [_Color]
Float 3 [_Gloss]
Vector 0 [_LightColor0]
Float 4 [_Parallax]
Float 2 [_Shininess]
SetTexture 0 [_LightTexture0] CUBE
SetTexture 1 [_LightTextureB0] 2D
SetTexture 2 [_MainTex] 2D
SetTexture 3 [_BumpMap] 2D
SetTexture 4 [_SpecMap] 2D
SetTexture 5 [_ParallaxMap] 2D
// Shader Timing Estimate, in Cycles/64 pixel vector:
// ALU: 38.67 (29 instructions), vertex: 0, texture: 24,
//   sequencer: 14, interpolator: 16;    7 GPRs, 27 threads,
// Performance (if enough threads): ~38 cycles per vector
// * Texture cycle estimates are assuming an 8bit/component texture with no
//     aniso or trilinear filtering.

"ps_360
backbbaaaaaaaceiaaaaaccaaaaaaaaaaaaaaaceaaaaabpaaaaaacbiaaaaaaaa
aaaaaaaaaaaaabmiaaaaaabmaaaaabllppppadaaaaaaaaalaaaaaabmaaaaaaaa
aaaaableaaaaaapiaaadaaadaaabaaaaaaaaabaeaaaaaaaaaaaaabbeaaacaaab
aaabaaaaaaaaabbmaaaaaaaaaaaaabcmaaacaaadaaabaaaaaaaaabdeaaaaaaaa
aaaaabeeaaacaaaaaaabaaaaaaaaabbmaaaaaaaaaaaaabfbaaadaaaaaaabaaaa
aaaaabgaaaaaaaaaaaaaabhaaaadaaabaaabaaaaaaaaabaeaaaaaaaaaaaaabia
aaadaaacaaabaaaaaaaaabaeaaaaaaaaaaaaabijaaacaaaeaaabaaaaaaaaabde
aaaaaaaaaaaaabjdaaadaaafaaabaaaaaaaaabaeaaaaaaaaaaaaabkaaaacaaac
aaabaaaaaaaaabdeaaaaaaaaaaaaabklaaadaaaeaaabaaaaaaaaabaeaaaaaaaa
fpechfgnhaengbhaaaklklklaaaeaaamaaabaaabaaabaaaaaaaaaaaafpedgpgm
gphcaaklaaabaaadaaabaaaeaaabaaaaaaaaaaaafpehgmgphdhdaaklaaaaaaad
aaabaaabaaabaaaaaaaaaaaafpemgjghgiheedgpgmgphcdaaafpemgjghgihefe
gfhihehfhcgfdaaaaaaeaaaoaaabaaabaaabaaaaaaaaaaaafpemgjghgihefegf
hihehfhcgfecdaaafpengbgjgofegfhiaafpfagbhcgbgmgmgbhiaafpfagbhcgb
gmgmgbhiengbhaaafpfdgigjgogjgogfhdhdaafpfdhagfgdengbhaaahahdfpdd
fpdaaadccodacodcdadddfddcodaaaklaaaaaaaaaaaaaaabaaaaaaaaaaaaaaaa
aaaaaabeabpmaabaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaeaaaaaaboa
baaaagaaaaaaaaaeaaaaaaaaaaaadeieaaapaaapaaaaaaabaaaapafaaaaahbfb
aaaahcfcaaaahdfdaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaalpiaaaaadpiaaaaaecaaaaaaaaaaaaaadpaaaaaaaaaaaaaa
donhakdndpmaaaaaaaajgaaegaakbcaabcaaaaaaabfffabaaaaabcaameaaaaaa
aaaagabfgablbcaabcaaaaaaaaaagacbaaaaccaaaaaaaaaadifadaabbpbpphpp
aaaaeaaamiaiaaabaaloloaapaababaamiaiaaacaagmgmaacbaeppaamiaiaaad
abblgmblkladaeacfiipabaeaakgmnblpcadadibembiadacaalolomgpaadadie
miadaaadaagngmblmlaeadppmiahaaabaablloaaobababaalebiafabaaloloma
naacacppembeaeadaablblgmocaeaeafmiadaaaeaamfgmaaobabaeaamiapaaaa
aakablaaolaeadaalidigaabbpbppompaaaaeaaajaaidagbbpbpphppaaaamaaa
pmbicaebbpbppbppaaaaeaaabaeidaabbpbppoiiaaaaeaaabaciaaabbpbppgec
aaaaeaaamiabaaaeaagmmgaacbacpoaabechaaadaablmalbobaaadaafiibabag
aablblblobadacibaaboacaeaablpmgmobabacagaachacafaabfgfmgoaaeabag
aaebacabaalololbpaafafagfiigababaambgmgmkaacpoibmiabaaabaelclclb
nbababpokaihabafaamablgmobafabibkicbaeabaamdmdebnaaeababkiecaeab
aalomdicnaafababkiigaeabaalmlbmaicabppabeaboabaeaaabpmmgkbaeaaib
miapaaaaaaaalaaaobaeabaadiboababaapmgmgmkbadadaamiahaaabaabfgmaa
obababaamiahaaaaaamamabfklabaaaamiahmaaaaagmmaaaobacaaaaaaaaaaaa
aaaaaaaaaaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "POINT_COOKIE" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
Float 4 [_Parallax]
SetTexture 0 [_ParallaxMap] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 2 [_SpecMap] 2D
SetTexture 3 [_BumpMap] 2D
SetTexture 4 [_LightTextureB0] 2D
SetTexture 5 [_LightTexture0] CUBE
"sce_fp_rsx // 62 instructions using 5 registers
[Configuration]
24
ffffffff0003c020000ffff0000000000000840005000000
[Offsets]
5
_LightColor0 2 0
000003a000000320
_Color 1 0
00000300
_Shininess 1 0
00000230
_Gloss 1 0
00000170
_Parallax 2 0
0000005000000020
[Microcode]
992
ee080100c8011c9dc8000001c8003fe10286014000021c9cc8000001c8000001
00000000000000000000000000000000900017005c011c9dc8000001c8003fe1
10840240c8001c9d00020000c800000100000000000000000000000000000000
08000500c8101c9dc8100001c8000001ae843940c8011c9dc8000029c800bfe1
1002030055081c9d00020000c80000010a3d3ed7000000000000000000000000
18063a0081081c9cfe040001c800000102800440c90c1c9d00020000ff080001
0000bf000000000000000000000000000606020001001c9c5c0c0001c8000001
9e040100c8011c9dc8000001c8003fe11806030080081c9c800c0000c8000001
1e7e7d00c8001c9dc8000001c8000001060003005c081c9dc80c0001c8000001
14001706c8001c9dc8000001c800000106860440ce001c9d00020000aa020000
000040000000bf8000000000000000001e0417025c0c1c9dc8000001c8000001
02900240fe081c9d00020000c800000100000000000000000000000000000000
0e0617045c0c1c9dc8000001c800000110840240ab0c1c9cab0c0000c8000001
0e8c024001201c9cc80c0001c8000001f000170ac8011c9dc8000001c8003fe1
108a0440010c1c9e010c0000c9080003ce803940c8011c9dc8000029c800bfe1
10800340c9141c9d00020000c800000100003f80000000000000000000000000
08863b40ff003c9dff000001c8000001108c0540c90c1c9dc9000001c8000001
1084014000021c9cc8000001c800000100000000000000000000000000000000
0e080340c9081c9dc9000001c80000010200170854001c9dc8000001c8000001
0e843940c8101c9dc8000029c800000108800540c90c1c9dc9080001c8000001
0400090055001c9d00020000c800000100000000000000000000000000000000
10840100c9081c9dc8000001c800000104001d00aa001c9cc8000001c8000001
02820240ff081c9d00020000c800000100004200000000000000000000000000
10020200aa001c9c01040000c80000010e840240c8081c9dc8020001c8000001
000000000000000000000000000000000e840240c9081c9dc8020001c8000001
0000000000000000000000000000000008021c00fe041c9dc8000001c8000001
0e86020054041c9dc9180001c800000110840900c9181c9d00020000c8000001
000000000000000000000000000000000e840240c9081c9dff080001c8000001
1080020000001c9cc8000001c80000010e800440c90c1c9dc8020001c9080001
000000000000000000000000000000000e800240ff001c9dc9001001c8000001
1081014000021c9cc8000001c800000100000000000000000000000000000000
"
}

SubProgram "d3d11 " {
Keywords { "POINT_COOKIE" }
ConstBuffer "$Globals" 176 // 140 used size, 10 vars
Vector 16 [_LightColor0] 4
Vector 112 [_Color] 4
Float 128 [_Shininess]
Float 132 [_Gloss]
Float 136 [_Parallax]
BindCB "$Globals" 0
SetTexture 0 [_ParallaxMap] 2D 5
SetTexture 1 [_MainTex] 2D 2
SetTexture 2 [_SpecMap] 2D 4
SetTexture 3 [_BumpMap] 2D 3
SetTexture 4 [_LightTextureB0] 2D 1
SetTexture 5 [_LightTexture0] CUBE 0
// 43 instructions, 5 temp regs, 0 temp arrays:
// ALU 27 float, 0 int, 0 uint
// TEX 6 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0
eefiecedgbmibgileenjbacnpgihpffhenpfjgliabaaaaaadeahaaaaadaaaaaa
cmaaaaaammaaaaaaaaabaaaaejfdeheojiaaaaaaafaaaaaaaiaaaaaaiaaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaaimaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaahahaaaaimaaaaaa
adaaaaaaaaaaaaaaadaaaaaaaeaaaaaaahahaaaafdfgfpfaepfdejfeejepeoaa
feeffiedepepfceeaaklklklepfdeheocmaaaaaaabaaaaaaaiaaaaaacaaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaafdfgfpfegbhcghgfheaaklkl
fdeieefccmagaaaaeaaaaaaailabaaaafjaaaaaeegiocaaaaaaaaaaaajaaaaaa
fkaaaaadaagabaaaaaaaaaaafkaaaaadaagabaaaabaaaaaafkaaaaadaagabaaa
acaaaaaafkaaaaadaagabaaaadaaaaaafkaaaaadaagabaaaaeaaaaaafkaaaaad
aagabaaaafaaaaaafibiaaaeaahabaaaaaaaaaaaffffaaaafibiaaaeaahabaaa
abaaaaaaffffaaaafibiaaaeaahabaaaacaaaaaaffffaaaafibiaaaeaahabaaa
adaaaaaaffffaaaafibiaaaeaahabaaaaeaaaaaaffffaaaafidaaaaeaahabaaa
afaaaaaaffffaaaagcbaaaadpcbabaaaabaaaaaagcbaaaadhcbabaaaacaaaaaa
gcbaaaadhcbabaaaadaaaaaagcbaaaadhcbabaaaaeaaaaaagfaaaaadpccabaaa
aaaaaaaagiaaaaacafaaaaaabaaaaaahbcaabaaaaaaaaaaaegbcbaaaadaaaaaa
egbcbaaaadaaaaaaeeaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaabaaaaaah
ccaabaaaaaaaaaaaegbcbaaaacaaaaaaegbcbaaaacaaaaaaeeaaaaafccaabaaa
aaaaaaaabkaabaaaaaaaaaaadiaaaaahhcaabaaaabaaaaaafgafbaaaaaaaaaaa
egbcbaaaacaaaaaadcaaaaajccaabaaaaaaaaaaackbabaaaacaaaaaabkaabaaa
aaaaaaaaabeaaaaadnaknhdoaoaaaaahpcaabaaaacaaaaaaegaebaaaabaaaaaa
fgafbaaaaaaaaaaadcaaaaajocaabaaaaaaaaaaaagbjbaaaadaaaaaaagaabaaa
aaaaaaaaagajbaaaabaaaaaadiaaaaahhcaabaaaabaaaaaaagaabaaaaaaaaaaa
egbcbaaaadaaaaaabaaaaaahbcaabaaaaaaaaaaajgahbaaaaaaaaaaajgahbaaa
aaaaaaaaeeaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaadiaaaaahhcaabaaa
aaaaaaaaagaabaaaaaaaaaaajgahbaaaaaaaaaaaefaaaaajpcaabaaaadaaaaaa
ogbkbaaaabaaaaaaeghobaaaaaaaaaaaaagabaaaafaaaaaadiaaaaaldcaabaaa
adaaaaaacgikcaaaaaaaaaaaaiaaaaaaaceaaaaaaaaaaadpaaaaaaecaaaaaaaa
aaaaaaaadcaaaaalicaabaaaaaaaaaaadkaabaaaadaaaaaackiacaaaaaaaaaaa
aiaaaaaaakaabaiaebaaaaaaadaaaaaadcaaaaajpcaabaaaacaaaaaapgapbaaa
aaaaaaaaegaobaaaacaaaaaaegbobaaaabaaaaaaefaaaaajpcaabaaaaeaaaaaa
ogakbaaaacaaaaaaeghobaaaadaaaaaaaagabaaaadaaaaaadcaaaaapdcaabaaa
aeaaaaaahgapbaaaaeaaaaaaaceaaaaaaaaaaaeaaaaaaaeaaaaaaaaaaaaaaaaa
aceaaaaaaaaaialpaaaaialpaaaaaaaaaaaaaaaadcaaaaakicaabaaaaaaaaaaa
akaabaiaebaaaaaaaeaaaaaaakaabaaaaeaaaaaaabeaaaaaaaaaiadpdcaaaaak
icaabaaaaaaaaaaabkaabaiaebaaaaaaaeaaaaaabkaabaaaaeaaaaaadkaabaaa
aaaaaaaaelaaaaafecaabaaaaeaaaaaadkaabaaaaaaaaaaabaaaaaahbcaabaaa
aaaaaaaaegacbaaaaeaaaaaaegacbaaaaaaaaaaabaaaaaahccaabaaaaaaaaaaa
egacbaaaaeaaaaaaegacbaaaabaaaaaadeaaaaakdcaabaaaaaaaaaaaegaabaaa
aaaaaaaaaceaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaacpaaaaafbcaabaaa
aaaaaaaaakaabaaaaaaaaaaadiaaaaahbcaabaaaaaaaaaaaakaabaaaaaaaaaaa
bkaabaaaadaaaaaabjaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaaefaaaaaj
pcaabaaaabaaaaaaegaabaaaacaaaaaaeghobaaaacaaaaaaaagabaaaaeaaaaaa
efaaaaajpcaabaaaacaaaaaaegaabaaaacaaaaaaeghobaaaabaaaaaaaagabaaa
acaaaaaadiaaaaahhcaabaaaabaaaaaaegacbaaaabaaaaaapgapbaaaacaaaaaa
diaaaaaihcaabaaaacaaaaaaegacbaaaacaaaaaaegiccaaaaaaaaaaaahaaaaaa
diaaaaaihcaabaaaacaaaaaaegacbaaaacaaaaaaegiccaaaaaaaaaaaabaaaaaa
diaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaafgifcaaaaaaaaaaaaiaaaaaa
diaaaaahncaabaaaaaaaaaaaagaabaaaaaaaaaaaagajbaaaabaaaaaadiaaaaai
ncaabaaaaaaaaaaaagaobaaaaaaaaaaaagijcaaaaaaaaaaaabaaaaaadcaaaaaj
hcaabaaaaaaaaaaaegacbaaaacaaaaaafgafbaaaaaaaaaaaigadbaaaaaaaaaaa
baaaaaahicaabaaaaaaaaaaaegbcbaaaaeaaaaaaegbcbaaaaeaaaaaaefaaaaaj
pcaabaaaabaaaaaapgapbaaaaaaaaaaaeghobaaaaeaaaaaaaagabaaaabaaaaaa
efaaaaajpcaabaaaacaaaaaaegbcbaaaaeaaaaaaeghobaaaafaaaaaaaagabaaa
aaaaaaaaapaaaaahicaabaaaaaaaaaaaagaabaaaabaaaaaapgapbaaaacaaaaaa
diaaaaahhccabaaaaaaaaaaapgapbaaaaaaaaaaaegacbaaaaaaaaaaadgaaaaaf
iccabaaaaaaaaaaaabeaaaaaaaaaaaaadoaaaaab"
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
Float 4 [_Parallax]
SetTexture 0 [_ParallaxMap] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 2 [_SpecMap] 2D
SetTexture 3 [_BumpMap] 2D
SetTexture 4 [_LightTextureB0] 2D
SetTexture 5 [_LightTexture0] CUBE
"agal_ps
c5 0.5 0.42 2.0 -1.0
c6 1.0 0.0 32.0 0.0
[bc]
aaaaaaaaabaaabacafaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r1.x, c5
aaaaaaaaacaaacacaaaaaappaeaaaaaaaaaaaaaaaaaaaaaa mov r2.y, v0.w
aaaaaaaaaaaaacacaaaaaappaeaaaaaaaaaaaaaaaaaaaaaa mov r0.y, v0.w
aaaaaaaaaaaaabacaaaaaakkaeaaaaaaaaaaaaaaaaaaaaaa mov r0.x, v0.z
adaaaaaaabaaabacaeaaaaoeabaaaaaaabaaaaaaacaaaaaa mul r1.x, c4, r1.x
ciaaaaaaaaaaapacaaaaaafeacaaaaaaaaaaaaaaafaababb tex r0, r0.xyyy, s0 <2d wrap linear point>
bcaaaaaaaaaaabacabaaaaoeaeaaaaaaabaaaaoeaeaaaaaa dp3 r0.x, v1, v1
akaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r0.x, r0.x
adaaaaaaadaaahacaaaaaaaaacaaaaaaabaaaaoeaeaaaaaa mul r3.xyz, r0.x, v1
adaaaaaaadaaaiacaaaaaappacaaaaaaaeaaaaoeabaaaaaa mul r3.w, r0.w, c4
acaaaaaaabaaabacadaaaappacaaaaaaabaaaaaaacaaaaaa sub r1.x, r3.w, r1.x
abaaaaaaacaaabacadaaaakkacaaaaaaafaaaaffabaaaaaa add r2.x, r3.z, c5.y
afaaaaaaacaaabacacaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rcp r2.x, r2.x
adaaaaaaadaaadacadaaaafeacaaaaaaacaaaaaaacaaaaaa mul r3.xy, r3.xyyy, r2.x
aaaaaaaaacaaabacaaaaaakkaeaaaaaaaaaaaaaaaaaaaaaa mov r2.x, v0.z
adaaaaaaaeaaadacabaaaaaaacaaaaaaadaaaafeacaaaaaa mul r4.xy, r1.x, r3.xyyy
abaaaaaaaeaaadacaeaaaafeacaaaaaaacaaaafeacaaaaaa add r4.xy, r4.xyyy, r2.xyyy
adaaaaaaadaaadacabaaaaaaacaaaaaaadaaaafeacaaaaaa mul r3.xy, r1.x, r3.xyyy
abaaaaaaadaaadacadaaaafeacaaaaaaaaaaaaoeaeaaaaaa add r3.xy, r3.xyyy, v0
bcaaaaaaacaaabacadaaaaoeaeaaaaaaadaaaaoeaeaaaaaa dp3 r2.x, v3, v3
aaaaaaaaabaaadacacaaaaaaacaaaaaaaaaaaaaaaaaaaaaa mov r1.xy, r2.x
aaaaaaaaaaaaaiacagaaaaffabaaaaaaaaaaaaaaaaaaaaaa mov r0.w, c6.y
ciaaaaaaafaaapacaeaaaafeacaaaaaaadaaaaaaafaababb tex r5, r4.xyyy, s3 <2d wrap linear point>
ciaaaaaaacaaapacadaaaafeacaaaaaaabaaaaaaafaababb tex r2, r3.xyyy, s1 <2d wrap linear point>
ciaaaaaaaeaaapacabaaaafeacaaaaaaaeaaaaaaafaababb tex r4, r1.xyyy, s4 <2d wrap linear point>
ciaaaaaaabaaapacadaaaaoeaeaaaaaaafaaaaaaafbababb tex r1, v3, s5 <cube wrap linear point>
ciaaaaaaadaaapacadaaaafeacaaaaaaacaaaaaaafaababb tex r3, r3.xyyy, s2 <2d wrap linear point>
adaaaaaaacaaahacacaaaakeacaaaaaaabaaaaoeabaaaaaa mul r2.xyz, r2.xyzz, c1
aaaaaaaaaeaaacacafaaaaffacaaaaaaaaaaaaaaaaaaaaaa mov r4.y, r5.y
aaaaaaaaaeaaabacafaaaappacaaaaaaaaaaaaaaaaaaaaaa mov r4.x, r5.w
adaaaaaaafaaadacaeaaaafeacaaaaaaafaaaakkabaaaaaa mul r5.xy, r4.xyyy, c5.z
abaaaaaaafaaadacafaaaafeacaaaaaaafaaaappabaaaaaa add r5.xy, r5.xyyy, c5.w
bcaaaaaaabaaabacacaaaaoeaeaaaaaaacaaaaoeaeaaaaaa dp3 r1.x, v2, v2
akaaaaaaaeaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r4.x, r1.x
adaaaaaaaeaaahacaeaaaaaaacaaaaaaacaaaaoeaeaaaaaa mul r4.xyz, r4.x, v2
adaaaaaaabaaabacafaaaaffacaaaaaaafaaaaffacaaaaaa mul r1.x, r5.y, r5.y
adaaaaaaagaaahacaaaaaaaaacaaaaaaabaaaaoeaeaaaaaa mul r6.xyz, r0.x, v1
abaaaaaaagaaahacagaaaakeacaaaaaaaeaaaakeacaaaaaa add r6.xyz, r6.xyzz, r4.xyzz
bfaaaaaaagaaaiacafaaaaaaacaaaaaaaaaaaaaaaaaaaaaa neg r6.w, r5.x
adaaaaaaagaaaiacagaaaappacaaaaaaafaaaaaaacaaaaaa mul r6.w, r6.w, r5.x
acaaaaaaabaaabacagaaaappacaaaaaaabaaaaaaacaaaaaa sub r1.x, r6.w, r1.x
abaaaaaaaaaaabacabaaaaaaacaaaaaaagaaaaoeabaaaaaa add r0.x, r1.x, c6
bcaaaaaaabaaabacagaaaakeacaaaaaaagaaaakeacaaaaaa dp3 r1.x, r6.xyzz, r6.xyzz
akaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r0.x, r0.x
afaaaaaaafaaaeacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rcp r5.z, r0.x
akaaaaaaabaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r1.x, r1.x
adaaaaaaabaaahacabaaaaaaacaaaaaaagaaaakeacaaaaaa mul r1.xyz, r1.x, r6.xyzz
bcaaaaaaabaaabacafaaaakeacaaaaaaabaaaakeacaaaaaa dp3 r1.x, r5.xyzz, r1.xyzz
aaaaaaaaaaaaabacacaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0.x, c2
adaaaaaaaaaaabacagaaaakkabaaaaaaaaaaaaaaacaaaaaa mul r0.x, c6.z, r0.x
ahaaaaaaabaaabacabaaaaaaacaaaaaaagaaaaffabaaaaaa max r1.x, r1.x, c6.y
alaaaaaaagaaapacabaaaaaaacaaaaaaaaaaaaaaacaaaaaa pow r6, r1.x, r0.x
adaaaaaaaaaaahacacaaaappacaaaaaaadaaaakeacaaaaaa mul r0.xyz, r2.w, r3.xyzz
adaaaaaaabaaahacaaaaaakeacaaaaaaadaaaaaaabaaaaaa mul r1.xyz, r0.xyzz, c3.x
aaaaaaaaaaaaabacagaaaaaaacaaaaaaaaaaaaaaaaaaaaaa mov r0.x, r6.x
adaaaaaaaaaaahacaaaaaaaaacaaaaaaabaaaakeacaaaaaa mul r0.xyz, r0.x, r1.xyzz
adaaaaaaabaaahacaaaaaakeacaaaaaaaaaaaaoeabaaaaaa mul r1.xyz, r0.xyzz, c0
bcaaaaaaaaaaabacafaaaakeacaaaaaaaeaaaakeacaaaaaa dp3 r0.x, r5.xyzz, r4.xyzz
ahaaaaaaaaaaabacaaaaaaaaacaaaaaaagaaaaffabaaaaaa max r0.x, r0.x, c6.y
adaaaaaaacaaahacacaaaakeacaaaaaaaaaaaaoeabaaaaaa mul r2.xyz, r2.xyzz, c0
adaaaaaaacaaahacacaaaakeacaaaaaaaaaaaaaaacaaaaaa mul r2.xyz, r2.xyzz, r0.x
abaaaaaaabaaahacacaaaakeacaaaaaaabaaaakeacaaaaaa add r1.xyz, r2.xyzz, r1.xyzz
adaaaaaaaaaaabacaeaaaappacaaaaaaabaaaappacaaaaaa mul r0.x, r4.w, r1.w
adaaaaaaaaaaahacaaaaaaaaacaaaaaaabaaaakeacaaaaaa mul r0.xyz, r0.x, r1.xyzz
adaaaaaaaaaaahacaaaaaakeacaaaaaaafaaaakkabaaaaaa mul r0.xyz, r0.xyzz, c5.z
aaaaaaaaaaaaapadaaaaaaoeacaaaaaaaaaaaaaaaaaaaaaa mov o0, r0
"
}

SubProgram "d3d11_9x " {
Keywords { "POINT_COOKIE" }
ConstBuffer "$Globals" 176 // 140 used size, 10 vars
Vector 16 [_LightColor0] 4
Vector 112 [_Color] 4
Float 128 [_Shininess]
Float 132 [_Gloss]
Float 136 [_Parallax]
BindCB "$Globals" 0
SetTexture 0 [_ParallaxMap] 2D 5
SetTexture 1 [_MainTex] 2D 2
SetTexture 2 [_SpecMap] 2D 4
SetTexture 3 [_BumpMap] 2D 3
SetTexture 4 [_LightTextureB0] 2D 1
SetTexture 5 [_LightTexture0] CUBE 0
// 43 instructions, 5 temp regs, 0 temp arrays:
// ALU 27 float, 0 int, 0 uint
// TEX 6 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0_level_9_3
eefiecedphnabnfhfdaigobglacmogpbangfejfcabaaaaaabealaaaaaeaaaaaa
daaaaaaaamaeaaaaeaakaaaaoaakaaaaebgpgodjneadaaaaneadaaaaaaacpppp
iaadaaaafeaaaaaaacaadmaaaaaafeaaaaaafeaaagaaceaaaaaafeaaafaaaaaa
aeababaaabacacaaadadadaaacaeaeaaaaafafaaaaaaabaaabaaaaaaaaaaaaaa
aaaaahaaacaaabaaaaaaaaaaabacppppfbaaaaafadaaapkaaaaaaadpaaaaaaec
dnaknhdoaaaaiadpfbaaaaafaeaaapkaaaaaaaeaaaaaialpaaaaaaaaaaaaaaaa
bpaaaaacaaaaaaiaaaaaaplabpaaaaacaaaaaaiaabaachlabpaaaaacaaaaaaia
acaachlabpaaaaacaaaaaaiaadaaahlabpaaaaacaaaaaajiaaaiapkabpaaaaac
aaaaaajaabaiapkabpaaaaacaaaaaajaacaiapkabpaaaaacaaaaaajaadaiapka
bpaaaaacaaaaaajaaeaiapkabpaaaaacaaaaaajaafaiapkaaiaaaaadaaaaciia
abaaoelaabaaoelaahaaaaacaaaacbiaaaaappiaaeaaaaaeaaaaaciaabaakkla
aaaaaaiaadaakkkaafaaaaadaaaaaniaaaaaaaiaabaajelaagaaaaacaaaaacia
aaaaffiaafaaaaadabaaadiaaaaaffiaaaaaoiiaabaaaaacacaaadiaaaaaoola
ecaaaaadacaacpiaacaaoeiaafaioekaabaaaaacacaaafiaacaaoekaafaaaaad
abaaamiaacaaceiaadaaeekaaeaaaaaeaaaacciaacaappiaacaakkkaabaakkib
aeaaaaaeacaaadiaaaaaffiaabaaoeiaaaaaoolaaeaaaaaeabaaadiaaaaaffia
abaaoeiaaaaaoelaecaaaaadacaacpiaacaaoeiaadaioekaaeaaaaaeacaacdia
acaaohiaaeaaaakaaeaaffkaaeaaaaaeacaaciiaacaaaaiaacaaaaibadaappka
aeaaaaaeacaaciiaacaaffiaacaaffibacaappiaahaaaaacacaaciiaacaappia
agaaaaacacaaceiaacaappiaaiaaaaadacaaciiaacaaoelaacaaoelaahaaaaac
acaaciiaacaappiaaeaaaaaeaaaachiaacaaoelaacaappiaaaaapiiaafaaaaad
adaachiaacaappiaacaaoelaaiaaaaadaaaaciiaacaaoeiaadaaoeiaalaaaaad
acaaciiaaaaappiaaeaakkkaceaaaaacadaachiaaaaaoeiaaiaaaaadaaaacbia
acaaoeiaadaaoeiaalaaaaadabaaaeiaaaaaaaiaaeaakkkacaaaaaadaaaaabia
abaakkiaabaappiaecaaaaadadaacpiaabaaoeiaacaioekaecaaaaadabaacpia
abaaoeiaaeaioekaafaaaaadaaaacoiaabaajaiaadaappiaafaaaaadabaachia
adaaoeiaabaaoekaafaaaaadabaachiaabaaoeiaaaaaoekaafaaaaadaaaacoia
aaaaoeiaacaaffkaafaaaaadaaaachiaaaaapjiaaaaaaaiaafaaaaadaaaachia
aaaaoeiaaaaaoekaaeaaaaaeaaaachiaabaaoeiaacaappiaaaaaoeiaaiaaaaad
abaaadiaadaaoelaadaaoelaecaaaaadacaaapiaadaaoelaaaaioekaecaaaaad
abaaapiaabaaoeiaabaioekafkaaaaaeaaaaciiaabaaaaiaacaappiaaeaakkka
afaaaaadaaaachiaaaaappiaaaaaoeiaabaaaaacaaaaaiiaaeaakkkaabaaaaac
aaaicpiaaaaaoeiappppaaaafdeieefccmagaaaaeaaaaaaailabaaaafjaaaaae
egiocaaaaaaaaaaaajaaaaaafkaaaaadaagabaaaaaaaaaaafkaaaaadaagabaaa
abaaaaaafkaaaaadaagabaaaacaaaaaafkaaaaadaagabaaaadaaaaaafkaaaaad
aagabaaaaeaaaaaafkaaaaadaagabaaaafaaaaaafibiaaaeaahabaaaaaaaaaaa
ffffaaaafibiaaaeaahabaaaabaaaaaaffffaaaafibiaaaeaahabaaaacaaaaaa
ffffaaaafibiaaaeaahabaaaadaaaaaaffffaaaafibiaaaeaahabaaaaeaaaaaa
ffffaaaafidaaaaeaahabaaaafaaaaaaffffaaaagcbaaaadpcbabaaaabaaaaaa
gcbaaaadhcbabaaaacaaaaaagcbaaaadhcbabaaaadaaaaaagcbaaaadhcbabaaa
aeaaaaaagfaaaaadpccabaaaaaaaaaaagiaaaaacafaaaaaabaaaaaahbcaabaaa
aaaaaaaaegbcbaaaadaaaaaaegbcbaaaadaaaaaaeeaaaaafbcaabaaaaaaaaaaa
akaabaaaaaaaaaaabaaaaaahccaabaaaaaaaaaaaegbcbaaaacaaaaaaegbcbaaa
acaaaaaaeeaaaaafccaabaaaaaaaaaaabkaabaaaaaaaaaaadiaaaaahhcaabaaa
abaaaaaafgafbaaaaaaaaaaaegbcbaaaacaaaaaadcaaaaajccaabaaaaaaaaaaa
ckbabaaaacaaaaaabkaabaaaaaaaaaaaabeaaaaadnaknhdoaoaaaaahpcaabaaa
acaaaaaaegaebaaaabaaaaaafgafbaaaaaaaaaaadcaaaaajocaabaaaaaaaaaaa
agbjbaaaadaaaaaaagaabaaaaaaaaaaaagajbaaaabaaaaaadiaaaaahhcaabaaa
abaaaaaaagaabaaaaaaaaaaaegbcbaaaadaaaaaabaaaaaahbcaabaaaaaaaaaaa
jgahbaaaaaaaaaaajgahbaaaaaaaaaaaeeaaaaafbcaabaaaaaaaaaaaakaabaaa
aaaaaaaadiaaaaahhcaabaaaaaaaaaaaagaabaaaaaaaaaaajgahbaaaaaaaaaaa
efaaaaajpcaabaaaadaaaaaaogbkbaaaabaaaaaaeghobaaaaaaaaaaaaagabaaa
afaaaaaadiaaaaaldcaabaaaadaaaaaacgikcaaaaaaaaaaaaiaaaaaaaceaaaaa
aaaaaadpaaaaaaecaaaaaaaaaaaaaaaadcaaaaalicaabaaaaaaaaaaadkaabaaa
adaaaaaackiacaaaaaaaaaaaaiaaaaaaakaabaiaebaaaaaaadaaaaaadcaaaaaj
pcaabaaaacaaaaaapgapbaaaaaaaaaaaegaobaaaacaaaaaaegbobaaaabaaaaaa
efaaaaajpcaabaaaaeaaaaaaogakbaaaacaaaaaaeghobaaaadaaaaaaaagabaaa
adaaaaaadcaaaaapdcaabaaaaeaaaaaahgapbaaaaeaaaaaaaceaaaaaaaaaaaea
aaaaaaeaaaaaaaaaaaaaaaaaaceaaaaaaaaaialpaaaaialpaaaaaaaaaaaaaaaa
dcaaaaakicaabaaaaaaaaaaaakaabaiaebaaaaaaaeaaaaaaakaabaaaaeaaaaaa
abeaaaaaaaaaiadpdcaaaaakicaabaaaaaaaaaaabkaabaiaebaaaaaaaeaaaaaa
bkaabaaaaeaaaaaadkaabaaaaaaaaaaaelaaaaafecaabaaaaeaaaaaadkaabaaa
aaaaaaaabaaaaaahbcaabaaaaaaaaaaaegacbaaaaeaaaaaaegacbaaaaaaaaaaa
baaaaaahccaabaaaaaaaaaaaegacbaaaaeaaaaaaegacbaaaabaaaaaadeaaaaak
dcaabaaaaaaaaaaaegaabaaaaaaaaaaaaceaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaacpaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaadiaaaaahbcaabaaa
aaaaaaaaakaabaaaaaaaaaaabkaabaaaadaaaaaabjaaaaafbcaabaaaaaaaaaaa
akaabaaaaaaaaaaaefaaaaajpcaabaaaabaaaaaaegaabaaaacaaaaaaeghobaaa
acaaaaaaaagabaaaaeaaaaaaefaaaaajpcaabaaaacaaaaaaegaabaaaacaaaaaa
eghobaaaabaaaaaaaagabaaaacaaaaaadiaaaaahhcaabaaaabaaaaaaegacbaaa
abaaaaaapgapbaaaacaaaaaadiaaaaaihcaabaaaacaaaaaaegacbaaaacaaaaaa
egiccaaaaaaaaaaaahaaaaaadiaaaaaihcaabaaaacaaaaaaegacbaaaacaaaaaa
egiccaaaaaaaaaaaabaaaaaadiaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaa
fgifcaaaaaaaaaaaaiaaaaaadiaaaaahncaabaaaaaaaaaaaagaabaaaaaaaaaaa
agajbaaaabaaaaaadiaaaaaincaabaaaaaaaaaaaagaobaaaaaaaaaaaagijcaaa
aaaaaaaaabaaaaaadcaaaaajhcaabaaaaaaaaaaaegacbaaaacaaaaaafgafbaaa
aaaaaaaaigadbaaaaaaaaaaabaaaaaahicaabaaaaaaaaaaaegbcbaaaaeaaaaaa
egbcbaaaaeaaaaaaefaaaaajpcaabaaaabaaaaaapgapbaaaaaaaaaaaeghobaaa
aeaaaaaaaagabaaaabaaaaaaefaaaaajpcaabaaaacaaaaaaegbcbaaaaeaaaaaa
eghobaaaafaaaaaaaagabaaaaaaaaaaaapaaaaahicaabaaaaaaaaaaaagaabaaa
abaaaaaapgapbaaaacaaaaaadiaaaaahhccabaaaaaaaaaaapgapbaaaaaaaaaaa
egacbaaaaaaaaaaadgaaaaaficcabaaaaaaaaaaaabeaaaaaaaaaaaaadoaaaaab
ejfdeheojiaaaaaaafaaaaaaaiaaaaaaiaaaaaaaaaaaaaaaabaaaaaaadaaaaaa
aaaaaaaaapaaaaaaimaaaaaaaaaaaaaaaaaaaaaaadaaaaaaabaaaaaaapapaaaa
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
Float 4 [_Parallax]
SetTexture 0 [_ParallaxMap] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 2 [_SpecMap] 2D
SetTexture 3 [_BumpMap] 2D
SetTexture 4 [_LightTexture0] 2D
"!!ARBfp1.0
OPTION ARB_precision_hint_fastest;
# 44 ALU, 5 TEX
PARAM c[7] = { program.local[0..4],
		{ 0, 0.5, 0.41999999, 2 },
		{ 1, 32 } };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
TEX R0.w, fragment.texcoord[0].zwzw, texture[0], 2D;
TEX R1.w, fragment.texcoord[3], texture[4], 2D;
DP3 R0.x, fragment.texcoord[1], fragment.texcoord[1];
RSQ R2.z, R0.x;
MUL R1.xyz, R2.z, fragment.texcoord[1];
ADD R0.x, R1.z, c[5].z;
MOV R0.y, c[4].x;
MUL R0.z, R0.y, c[5].y;
RCP R0.x, R0.x;
MUL R0.xy, R1, R0.x;
MAD R0.z, R0.w, c[4].x, -R0;
MAD R2.xy, R0.z, R0, fragment.texcoord[0].zwzw;
MAD R1.xy, R0.z, R0, fragment.texcoord[0];
MOV R3.xy, c[6];
MOV result.color.w, c[5].x;
TEX R0, R1, texture[1], 2D;
TEX R2.yw, R2, texture[3], 2D;
TEX R1.xyz, R1, texture[2], 2D;
MAD R3.zw, R2.xywy, c[5].w, -R3.x;
MUL R1.xyz, R0.w, R1;
MOV R2.xyw, fragment.texcoord[2].xyzz;
MUL R0.xyz, R0, c[1];
MAD R2.xyz, R2.z, fragment.texcoord[1], R2.xyww;
MUL R3.x, R3.w, R3.w;
MAD R2.w, -R3.z, R3.z, -R3.x;
DP3 R3.x, R2, R2;
RSQ R3.x, R3.x;
ADD R2.w, R2, c[6].x;
MUL R2.xyz, R3.x, R2;
RSQ R2.w, R2.w;
RCP R3.x, R2.w;
DP3 R2.x, R3.zwxw, R2;
MAX R2.x, R2, c[5];
MUL R0.w, R3.y, c[2].x;
POW R0.w, R2.x, R0.w;
MUL R1.xyz, R1, c[3].x;
MUL R1.xyz, R0.w, R1;
DP3 R0.w, R3.zwxw, fragment.texcoord[2];
MUL R1.xyz, R1, c[0];
MAX R0.w, R0, c[5].x;
MUL R0.xyz, R0, c[0];
MAD R0.xyz, R0, R0.w, R1;
MUL R0.xyz, R1.w, R0;
MUL result.color.xyz, R0, c[5].w;
END
# 44 instructions, 4 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "DIRECTIONAL_COOKIE" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
Float 4 [_Parallax]
SetTexture 0 [_ParallaxMap] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 2 [_SpecMap] 2D
SetTexture 3 [_BumpMap] 2D
SetTexture 4 [_LightTexture0] 2D
"ps_2_0
; 49 ALU, 5 TEX
dcl_2d s0
dcl_2d s1
dcl_2d s2
dcl_2d s3
dcl_2d s4
def c5, 0.50000000, 0.41999999, 2.00000000, -1.00000000
def c6, 1.00000000, 0.00000000, 32.00000000, 0
dcl t0
dcl t1.xyz
dcl t2.xyz
dcl t3.xy
mov r4.y, t0.w
mov r4.x, t0.z
mov r0.y, t0.w
mov r0.x, t0.z
mov_pp r5.xyz, t2
texld r0, r0, s0
dp3_pp r0.x, t1, t1
rsq_pp r0.x, r0.x
mul_pp r3.xyz, r0.x, t1
add r1.x, r3.z, c5.y
rcp r2.x, r1.x
mov_pp r1.x, c5
mul r2.xy, r3, r2.x
mul_pp r1.x, c4, r1
mad_pp r1.x, r0.w, c4, -r1
mad r3.xy, r1.x, r2, t0
mad r1.xy, r1.x, r2, r4
mad_pp r5.xyz, r0.x, t1, r5
mov_pp r0.w, c6.y
texld r4, r1, s3
texld r2, r3, s1
texld r3, r3, s2
texld r1, t3, s4
mul_pp r2.xyz, r2, c1
mov r1.y, r4
mov r1.x, r4.w
mad_pp r4.xy, r1, c5.z, c5.w
mul_pp r1.x, r4.y, r4.y
mad_pp r1.x, -r4, r4, -r1
add_pp r0.x, r1, c6
dp3_pp r1.x, r5, r5
rsq_pp r0.x, r0.x
rcp_pp r4.z, r0.x
rsq_pp r1.x, r1.x
mul_pp r1.xyz, r1.x, r5
dp3_pp r1.x, r4, r1
mov_pp r0.x, c2
mul_pp r0.x, c6.z, r0
max_pp r1.x, r1, c6.y
pow r5.x, r1.x, r0.x
mul_pp r0.xyz, r2.w, r3
mul_pp r1.xyz, r0, c3.x
mov r0.x, r5.x
mul r0.xyz, r0.x, r1
mul_pp r1.xyz, r0, c0
dp3_pp r0.x, r4, t2
max_pp r0.x, r0, c6.y
mul_pp r2.xyz, r2, c0
mad_pp r0.xyz, r2, r0.x, r1
mul_pp r0.xyz, r1.w, r0
mul_pp r0.xyz, r0, c5.z
mov_pp oC0, r0
"
}

SubProgram "xbox360 " {
Keywords { "DIRECTIONAL_COOKIE" }
Vector 1 [_Color]
Float 3 [_Gloss]
Vector 0 [_LightColor0]
Float 4 [_Parallax]
Float 2 [_Shininess]
SetTexture 0 [_LightTexture0] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_SpecMap] 2D
SetTexture 4 [_ParallaxMap] 2D
// Shader Timing Estimate, in Cycles/64 pixel vector:
// ALU: 32.00 (24 instructions), vertex: 0, texture: 20,
//   sequencer: 14, interpolator: 16;    7 GPRs, 27 threads,
// Performance (if enough threads): ~32 cycles per vector
// * Texture cycle estimates are assuming an 8bit/component texture with no
//     aniso or trilinear filtering.

"ps_360
backbbaaaaaaacbeaaaaabniaaaaaaaaaaaaaaceaaaaablmaaaaaboeaaaaaaaa
aaaaaaaaaaaaabjeaaaaaabmaaaaabihppppadaaaaaaaaakaaaaaabmaaaaaaaa
aaaaabiaaaaaaaoeaaadaaacaaabaaaaaaaaaapaaaaaaaaaaaaaabaaaaacaaab
aaabaaaaaaaaabaiaaaaaaaaaaaaabbiaaacaaadaaabaaaaaaaaabcaaaaaaaaa
aaaaabdaaaacaaaaaaabaaaaaaaaabaiaaaaaaaaaaaaabdnaaadaaaaaaabaaaa
aaaaaapaaaaaaaaaaaaaabemaaadaaabaaabaaaaaaaaaapaaaaaaaaaaaaaabff
aaacaaaeaaabaaaaaaaaabcaaaaaaaaaaaaaabfpaaadaaaeaaabaaaaaaaaaapa
aaaaaaaaaaaaabgmaaacaaacaaabaaaaaaaaabcaaaaaaaaaaaaaabhhaaadaaad
aaabaaaaaaaaaapaaaaaaaaafpechfgnhaengbhaaaklklklaaaeaaamaaabaaab
aaabaaaaaaaaaaaafpedgpgmgphcaaklaaabaaadaaabaaaeaaabaaaaaaaaaaaa
fpehgmgphdhdaaklaaaaaaadaaabaaabaaabaaaaaaaaaaaafpemgjghgiheedgp
gmgphcdaaafpemgjghgihefegfhihehfhcgfdaaafpengbgjgofegfhiaafpfagb
hcgbgmgmgbhiaafpfagbhcgbgmgmgbhiengbhaaafpfdgigjgogjgogfhdhdaafp
fdhagfgdengbhaaahahdfpddfpdaaadccodacodcdadddfddcodaaaklaaaaaaaa
aaaaaaabaaaaaaaaaaaaaaaaaaaaaabeabpmaabaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaeaaaaaabjibaaaagaaaaaaaaaeaaaaaaaaaaaadaieaaapaaap
aaaaaaabaaaapafaaaaahbfbaaaahcfcaaaaddfdaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaalpiaaaaaecaaaaaaaaaaaaaa
aaaaaaaadpaaaaaaaaaaaaaadonhakdndpiaaaaaaaajgaaegaakbcaabcaaafea
aaabbabaaaaabcaameaaaaaaaaaagabbgabhbcaabcaaaaaaaaaaeabnaaaaccaa
aaaaaaaadieadaabbpbppoppaaaaeaaamiaiaaacaaloloaapaababaafiiiacab
aagmgmblcbaeppicmiaiaaababmggmblkladaeabmiahaaabaablloaaobacabaa
leiaacaaaaaaaamamcaaaappemiaacaaaaaaaablocaaaaacmiamaaadaapbblaa
obabacaamiapaaaaaaakblaaoladabaabadigaabbpbppoiiaaaaeaaabaaidagb
bpbpppplaaaaeaaalicidaabbpbppompaaaaeaaababiaaabbpbppgecaaaaeaaa
miahaaadaaleleaaoaadadaabeahaaaeaagfmagmmaabacacamidaeafaamfgmlb
iaadpopomiaiaaabaegngnblnbafafppbeciaaacaalololbpaaeaeaafiihacab
aablmablobaaagickaehafagaamablblobaeacibkibbaeacaaloloebnaafacab
kiccaeacaaloloicnaagafabkiedaeacaalalbmaicacppabeaehacaeaamamalb
kbaeaaicmiapaaaaaaaaomaaobaeacaadiboababaapmgmblkbabadaamiahaaab
aabfgmaaobababaamiahaaaaaamamamaklabaaaamiahmaaaaagmmaaaobadaaaa
aaaaaaaaaaaaaaaaaaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "DIRECTIONAL_COOKIE" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
Float 4 [_Parallax]
SetTexture 0 [_ParallaxMap] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 2 [_SpecMap] 2D
SetTexture 3 [_BumpMap] 2D
SetTexture 4 [_LightTexture0] 2D
"sce_fp_rsx // 56 instructions using 4 registers
[Configuration]
24
ffffffff0003c020000ffff8000000000000840004000000
[Offsets]
5
_LightColor0 2 0
00000360000002e0
_Color 1 0
000002b0
_Shininess 1 0
00000200
_Gloss 1 0
00000140
_Parallax 2 0
0000004000000020
[Microcode]
896
900017005c011c9dc8000001c8003fe102800240fe001c9d00020000c8000001
000000000000000000000000000000001080014000021c9cc8000001c8000001
00000000000000000000000000000000ae843940c8011c9dc8000029c800bfe1
08020300c9081c9d00020000c80000010a3d3ed7000000000000000000000000
02820440ff001c9daa020000c9000001000000000000bf000000000000000000
06003a00c9081c9d54040001c80000011802020001041c9c80000000c8000001
9e040100c8011c9dc8000001c8003fe1060003005c081c9d5c040001c8000001
1804030080081c9cc8040001c800000114001706c8001c9dc8000001c8000001
06880440ce001c9d00020000aa020000000040000000bf800000000000000000
1e0017025c081c9dc8000001c800000108880240fe001c9d00020000c8000001
000000000000000000000000000000000e0617045c081c9dc8000001c8000001
0e86024055101c9dc80c0001c800000110840240ab101c9cab100000c8000001
1088044001101c9e01100000c9080003ce8a0140c8011c9dc8000001c8003fe1
0e060340c9141c9dc9080001c800000110880340c9101c9d00020000c8000001
00003f8000000000000000000000000008883b40ff103c9dff100001c8000001
10840540c9101c9dc9140001c80000011086014000021c9cc8000001c8000001
00000000000000000000000000000000f0001708c8011c9dc8000001c8003fe1
0e843940c80c1c9dc8000029c800000102840540c9101c9dc9080001c8000001
02040900c9081c9d00020000c800000100000000000000000000000000000000
02041d00c8081c9dc8000001c800000102840240ff0c1c9d00020000c8000001
000042000000000000000000000000001004020000081c9c01080000c8000001
0e800240c8001c9dc8020001c800000100000000000000000000000000000000
08001c00fe081c9dc8000001c80000010e800240c9001c9dc8020001c8000001
0000000000000000000000000000000010800900c9081c9dc8020001c8000001
000000000000000000000000000000000e84020054001c9dc90c0001c8000001
0e860240c9001c9dff000001c80000011080014000021c9cc8000001c8000001
000000000000000000000000000000000e800440c9081c9dc8020001c90c0001
000000000000000000000000000000000e810240fe001c9dc9001001c8000001
"
}

SubProgram "d3d11 " {
Keywords { "DIRECTIONAL_COOKIE" }
ConstBuffer "$Globals" 176 // 140 used size, 10 vars
Vector 16 [_LightColor0] 4
Vector 112 [_Color] 4
Float 128 [_Shininess]
Float 132 [_Gloss]
Float 136 [_Parallax]
BindCB "$Globals" 0
SetTexture 0 [_ParallaxMap] 2D 4
SetTexture 1 [_MainTex] 2D 1
SetTexture 2 [_SpecMap] 2D 3
SetTexture 3 [_BumpMap] 2D 2
SetTexture 4 [_LightTexture0] 2D 0
// 38 instructions, 4 temp regs, 0 temp arrays:
// ALU 23 float, 0 int, 0 uint
// TEX 5 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0
eefiecedjimkganfgmlblagpiagapokampfhneoeabaaaaaaimagaaaaadaaaaaa
cmaaaaaammaaaaaaaaabaaaaejfdeheojiaaaaaaafaaaaaaaiaaaaaaiaaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaaimaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaahahaaaaimaaaaaa
adaaaaaaaaaaaaaaadaaaaaaaeaaaaaaadadaaaafdfgfpfaepfdejfeejepeoaa
feeffiedepepfceeaaklklklepfdeheocmaaaaaaabaaaaaaaiaaaaaacaaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaafdfgfpfegbhcghgfheaaklkl
fdeieefcieafaaaaeaaaaaaagbabaaaafjaaaaaeegiocaaaaaaaaaaaajaaaaaa
fkaaaaadaagabaaaaaaaaaaafkaaaaadaagabaaaabaaaaaafkaaaaadaagabaaa
acaaaaaafkaaaaadaagabaaaadaaaaaafkaaaaadaagabaaaaeaaaaaafibiaaae
aahabaaaaaaaaaaaffffaaaafibiaaaeaahabaaaabaaaaaaffffaaaafibiaaae
aahabaaaacaaaaaaffffaaaafibiaaaeaahabaaaadaaaaaaffffaaaafibiaaae
aahabaaaaeaaaaaaffffaaaagcbaaaadpcbabaaaabaaaaaagcbaaaadhcbabaaa
acaaaaaagcbaaaadhcbabaaaadaaaaaagcbaaaaddcbabaaaaeaaaaaagfaaaaad
pccabaaaaaaaaaaagiaaaaacaeaaaaaabaaaaaahbcaabaaaaaaaaaaaegbcbaaa
acaaaaaaegbcbaaaacaaaaaaeeaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaa
dcaaaaajocaabaaaaaaaaaaaagbjbaaaacaaaaaaagaabaaaaaaaaaaaagbjbaaa
adaaaaaabaaaaaahbcaabaaaabaaaaaajgahbaaaaaaaaaaajgahbaaaaaaaaaaa
eeaaaaafbcaabaaaabaaaaaaakaabaaaabaaaaaadiaaaaahocaabaaaaaaaaaaa
fgaobaaaaaaaaaaaagaabaaaabaaaaaadiaaaaahdcaabaaaabaaaaaaagaabaaa
aaaaaaaaegbabaaaacaaaaaadcaaaaajbcaabaaaaaaaaaaackbabaaaacaaaaaa
akaabaaaaaaaaaaaabeaaaaadnaknhdoaoaaaaahpcaabaaaabaaaaaaegaebaaa
abaaaaaaagaabaaaaaaaaaaaefaaaaajpcaabaaaacaaaaaaogbkbaaaabaaaaaa
eghobaaaaaaaaaaaaagabaaaaeaaaaaadiaaaaaldcaabaaaacaaaaaacgikcaaa
aaaaaaaaaiaaaaaaaceaaaaaaaaaaadpaaaaaaecaaaaaaaaaaaaaaaadcaaaaal
bcaabaaaaaaaaaaadkaabaaaacaaaaaackiacaaaaaaaaaaaaiaaaaaaakaabaia
ebaaaaaaacaaaaaadcaaaaajpcaabaaaabaaaaaaagaabaaaaaaaaaaaegaobaaa
abaaaaaaegbobaaaabaaaaaaefaaaaajpcaabaaaadaaaaaaogakbaaaabaaaaaa
eghobaaaadaaaaaaaagabaaaacaaaaaadcaaaaapdcaabaaaadaaaaaahgapbaaa
adaaaaaaaceaaaaaaaaaaaeaaaaaaaeaaaaaaaaaaaaaaaaaaceaaaaaaaaaialp
aaaaialpaaaaaaaaaaaaaaaadcaaaaakbcaabaaaaaaaaaaaakaabaiaebaaaaaa
adaaaaaaakaabaaaadaaaaaaabeaaaaaaaaaiadpdcaaaaakbcaabaaaaaaaaaaa
bkaabaiaebaaaaaaadaaaaaabkaabaaaadaaaaaaakaabaaaaaaaaaaaelaaaaaf
ecaabaaaadaaaaaaakaabaaaaaaaaaaabaaaaaahbcaabaaaaaaaaaaaegacbaaa
adaaaaaajgahbaaaaaaaaaaabaaaaaahccaabaaaaaaaaaaaegacbaaaadaaaaaa
egbcbaaaadaaaaaadeaaaaakdcaabaaaaaaaaaaaegaabaaaaaaaaaaaaceaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaacpaaaaafbcaabaaaaaaaaaaaakaabaaa
aaaaaaaadiaaaaahbcaabaaaaaaaaaaaakaabaaaaaaaaaaabkaabaaaacaaaaaa
bjaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaaefaaaaajpcaabaaaacaaaaaa
egaabaaaabaaaaaaeghobaaaacaaaaaaaagabaaaadaaaaaaefaaaaajpcaabaaa
abaaaaaaegaabaaaabaaaaaaeghobaaaabaaaaaaaagabaaaabaaaaaadiaaaaah
hcaabaaaacaaaaaaegacbaaaacaaaaaapgapbaaaabaaaaaadiaaaaaihcaabaaa
abaaaaaaegacbaaaabaaaaaaegiccaaaaaaaaaaaahaaaaaadiaaaaaihcaabaaa
abaaaaaaegacbaaaabaaaaaaegiccaaaaaaaaaaaabaaaaaadiaaaaaihcaabaaa
acaaaaaaegacbaaaacaaaaaafgifcaaaaaaaaaaaaiaaaaaadiaaaaahncaabaaa
aaaaaaaaagaabaaaaaaaaaaaagajbaaaacaaaaaadiaaaaaincaabaaaaaaaaaaa
agaobaaaaaaaaaaaagijcaaaaaaaaaaaabaaaaaadcaaaaajhcaabaaaaaaaaaaa
egacbaaaabaaaaaafgafbaaaaaaaaaaaigadbaaaaaaaaaaaefaaaaajpcaabaaa
abaaaaaaegbabaaaaeaaaaaaeghobaaaaeaaaaaaaagabaaaaaaaaaaaaaaaaaah
icaabaaaaaaaaaaadkaabaaaabaaaaaadkaabaaaabaaaaaadiaaaaahhccabaaa
aaaaaaaapgapbaaaaaaaaaaaegacbaaaaaaaaaaadgaaaaaficcabaaaaaaaaaaa
abeaaaaaaaaaaaaadoaaaaab"
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
Float 4 [_Parallax]
SetTexture 0 [_ParallaxMap] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 2 [_SpecMap] 2D
SetTexture 3 [_BumpMap] 2D
SetTexture 4 [_LightTexture0] 2D
"agal_ps
c5 0.5 0.42 2.0 -1.0
c6 1.0 0.0 32.0 0.0
[bc]
aaaaaaaaaeaaacacaaaaaappaeaaaaaaaaaaaaaaaaaaaaaa mov r4.y, v0.w
aaaaaaaaaeaaabacaaaaaakkaeaaaaaaaaaaaaaaaaaaaaaa mov r4.x, v0.z
aaaaaaaaaaaaacacaaaaaappaeaaaaaaaaaaaaaaaaaaaaaa mov r0.y, v0.w
aaaaaaaaaaaaabacaaaaaakkaeaaaaaaaaaaaaaaaaaaaaaa mov r0.x, v0.z
aaaaaaaaafaaahacacaaaaoeaeaaaaaaaaaaaaaaaaaaaaaa mov r5.xyz, v2
ciaaaaaaaaaaapacaaaaaafeacaaaaaaaaaaaaaaafaababb tex r0, r0.xyyy, s0 <2d wrap linear point>
bcaaaaaaaaaaabacabaaaaoeaeaaaaaaabaaaaoeaeaaaaaa dp3 r0.x, v1, v1
akaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r0.x, r0.x
adaaaaaaadaaahacaaaaaaaaacaaaaaaabaaaaoeaeaaaaaa mul r3.xyz, r0.x, v1
abaaaaaaabaaabacadaaaakkacaaaaaaafaaaaffabaaaaaa add r1.x, r3.z, c5.y
afaaaaaaacaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rcp r2.x, r1.x
aaaaaaaaabaaabacafaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r1.x, c5
adaaaaaaacaaadacadaaaafeacaaaaaaacaaaaaaacaaaaaa mul r2.xy, r3.xyyy, r2.x
adaaaaaaabaaabacaeaaaaoeabaaaaaaabaaaaaaacaaaaaa mul r1.x, c4, r1.x
adaaaaaaadaaaiacaaaaaappacaaaaaaaeaaaaoeabaaaaaa mul r3.w, r0.w, c4
acaaaaaaabaaabacadaaaappacaaaaaaabaaaaaaacaaaaaa sub r1.x, r3.w, r1.x
adaaaaaaadaaadacabaaaaaaacaaaaaaacaaaafeacaaaaaa mul r3.xy, r1.x, r2.xyyy
abaaaaaaadaaadacadaaaafeacaaaaaaaaaaaaoeaeaaaaaa add r3.xy, r3.xyyy, v0
adaaaaaaabaaadacabaaaaaaacaaaaaaacaaaafeacaaaaaa mul r1.xy, r1.x, r2.xyyy
abaaaaaaabaaadacabaaaafeacaaaaaaaeaaaafeacaaaaaa add r1.xy, r1.xyyy, r4.xyyy
adaaaaaaagaaahacaaaaaaaaacaaaaaaabaaaaoeaeaaaaaa mul r6.xyz, r0.x, v1
abaaaaaaafaaahacagaaaakeacaaaaaaafaaaakeacaaaaaa add r5.xyz, r6.xyzz, r5.xyzz
aaaaaaaaaaaaaiacagaaaaffabaaaaaaaaaaaaaaaaaaaaaa mov r0.w, c6.y
ciaaaaaaaeaaapacabaaaafeacaaaaaaadaaaaaaafaababb tex r4, r1.xyyy, s3 <2d wrap linear point>
ciaaaaaaacaaapacadaaaafeacaaaaaaabaaaaaaafaababb tex r2, r3.xyyy, s1 <2d wrap linear point>
ciaaaaaaadaaapacadaaaafeacaaaaaaacaaaaaaafaababb tex r3, r3.xyyy, s2 <2d wrap linear point>
ciaaaaaaabaaapacadaaaaoeaeaaaaaaaeaaaaaaafaababb tex r1, v3, s4 <2d wrap linear point>
adaaaaaaacaaahacacaaaakeacaaaaaaabaaaaoeabaaaaaa mul r2.xyz, r2.xyzz, c1
aaaaaaaaabaaacacaeaaaaffacaaaaaaaaaaaaaaaaaaaaaa mov r1.y, r4.y
aaaaaaaaabaaabacaeaaaappacaaaaaaaaaaaaaaaaaaaaaa mov r1.x, r4.w
adaaaaaaaeaaadacabaaaafeacaaaaaaafaaaakkabaaaaaa mul r4.xy, r1.xyyy, c5.z
abaaaaaaaeaaadacaeaaaafeacaaaaaaafaaaappabaaaaaa add r4.xy, r4.xyyy, c5.w
adaaaaaaabaaabacaeaaaaffacaaaaaaaeaaaaffacaaaaaa mul r1.x, r4.y, r4.y
bfaaaaaaagaaabacaeaaaaaaacaaaaaaaaaaaaaaaaaaaaaa neg r6.x, r4.x
adaaaaaaagaaabacagaaaaaaacaaaaaaaeaaaaaaacaaaaaa mul r6.x, r6.x, r4.x
acaaaaaaabaaabacagaaaaaaacaaaaaaabaaaaaaacaaaaaa sub r1.x, r6.x, r1.x
abaaaaaaaaaaabacabaaaaaaacaaaaaaagaaaaoeabaaaaaa add r0.x, r1.x, c6
bcaaaaaaabaaabacafaaaakeacaaaaaaafaaaakeacaaaaaa dp3 r1.x, r5.xyzz, r5.xyzz
akaaaaaaaaaaabacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r0.x, r0.x
afaaaaaaaeaaaeacaaaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rcp r4.z, r0.x
akaaaaaaabaaabacabaaaaaaacaaaaaaaaaaaaaaaaaaaaaa rsq r1.x, r1.x
adaaaaaaabaaahacabaaaaaaacaaaaaaafaaaakeacaaaaaa mul r1.xyz, r1.x, r5.xyzz
bcaaaaaaabaaabacaeaaaakeacaaaaaaabaaaakeacaaaaaa dp3 r1.x, r4.xyzz, r1.xyzz
aaaaaaaaaaaaabacacaaaaoeabaaaaaaaaaaaaaaaaaaaaaa mov r0.x, c2
adaaaaaaaaaaabacagaaaakkabaaaaaaaaaaaaaaacaaaaaa mul r0.x, c6.z, r0.x
ahaaaaaaabaaabacabaaaaaaacaaaaaaagaaaaffabaaaaaa max r1.x, r1.x, c6.y
alaaaaaaafaaapacabaaaaaaacaaaaaaaaaaaaaaacaaaaaa pow r5, r1.x, r0.x
adaaaaaaaaaaahacacaaaappacaaaaaaadaaaakeacaaaaaa mul r0.xyz, r2.w, r3.xyzz
adaaaaaaabaaahacaaaaaakeacaaaaaaadaaaaaaabaaaaaa mul r1.xyz, r0.xyzz, c3.x
aaaaaaaaaaaaabacafaaaaaaacaaaaaaaaaaaaaaaaaaaaaa mov r0.x, r5.x
adaaaaaaaaaaahacaaaaaaaaacaaaaaaabaaaakeacaaaaaa mul r0.xyz, r0.x, r1.xyzz
adaaaaaaabaaahacaaaaaakeacaaaaaaaaaaaaoeabaaaaaa mul r1.xyz, r0.xyzz, c0
bcaaaaaaaaaaabacaeaaaakeacaaaaaaacaaaaoeaeaaaaaa dp3 r0.x, r4.xyzz, v2
ahaaaaaaaaaaabacaaaaaaaaacaaaaaaagaaaaffabaaaaaa max r0.x, r0.x, c6.y
adaaaaaaacaaahacacaaaakeacaaaaaaaaaaaaoeabaaaaaa mul r2.xyz, r2.xyzz, c0
adaaaaaaaaaaahacacaaaakeacaaaaaaaaaaaaaaacaaaaaa mul r0.xyz, r2.xyzz, r0.x
abaaaaaaaaaaahacaaaaaakeacaaaaaaabaaaakeacaaaaaa add r0.xyz, r0.xyzz, r1.xyzz
adaaaaaaaaaaahacabaaaappacaaaaaaaaaaaakeacaaaaaa mul r0.xyz, r1.w, r0.xyzz
adaaaaaaaaaaahacaaaaaakeacaaaaaaafaaaakkabaaaaaa mul r0.xyz, r0.xyzz, c5.z
aaaaaaaaaaaaapadaaaaaaoeacaaaaaaaaaaaaaaaaaaaaaa mov o0, r0
"
}

SubProgram "d3d11_9x " {
Keywords { "DIRECTIONAL_COOKIE" }
ConstBuffer "$Globals" 176 // 140 used size, 10 vars
Vector 16 [_LightColor0] 4
Vector 112 [_Color] 4
Float 128 [_Shininess]
Float 132 [_Gloss]
Float 136 [_Parallax]
BindCB "$Globals" 0
SetTexture 0 [_ParallaxMap] 2D 4
SetTexture 1 [_MainTex] 2D 1
SetTexture 2 [_SpecMap] 2D 3
SetTexture 3 [_BumpMap] 2D 2
SetTexture 4 [_LightTexture0] 2D 0
// 38 instructions, 4 temp regs, 0 temp arrays:
// ALU 23 float, 0 int, 0 uint
// TEX 5 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0_level_9_3
eefiecednfkbeangjildbponpejhgkknhgfajaanabaaaaaabiakaaaaaeaaaaaa
daaaaaaaliadaaaaeeajaaaaoeajaaaaebgpgodjiaadaaaaiaadaaaaaaacpppp
daadaaaafaaaaaaaacaadiaaaaaafaaaaaaafaaaafaaceaaaaaafaaaaeaaaaaa
abababaaadacacaaacadadaaaaaeaeaaaaaaabaaabaaaaaaaaaaaaaaaaaaahaa
acaaabaaaaaaaaaaabacppppfbaaaaafadaaapkaaaaaaadpaaaaaaecdnaknhdo
aaaaiadpfbaaaaafaeaaapkaaaaaaaeaaaaaialpaaaaaaaaaaaaaaaabpaaaaac
aaaaaaiaaaaaaplabpaaaaacaaaaaaiaabaachlabpaaaaacaaaaaaiaacaachla
bpaaaaacaaaaaaiaadaaadlabpaaaaacaaaaaajaaaaiapkabpaaaaacaaaaaaja
abaiapkabpaaaaacaaaaaajaacaiapkabpaaaaacaaaaaajaadaiapkabpaaaaac
aaaaaajaaeaiapkaaiaaaaadaaaaciiaabaaoelaabaaoelaahaaaaacaaaacbia
aaaappiaaeaaaaaeaaaaaciaabaakklaaaaaaaiaadaakkkaagaaaaacaaaaacia
aaaaffiaafaaaaadaaaaamiaaaaaaaiaabaaeelaabaaaaacabaaahiaabaaoela
aeaaaaaeabaachiaabaaoeiaaaaaaaiaacaaoelaceaaaaacacaachiaabaaoeia
afaaaaadaaaaadiaaaaaffiaaaaaooiaabaaaaacabaaadiaaaaaoolaecaaaaad
abaacpiaabaaoeiaaeaioekaabaaaaacabaaafiaacaaoekaafaaaaadaaaaamia
abaaceiaadaaeekaaeaaaaaeacaaciiaabaappiaacaakkkaaaaakkibaeaaaaae
abaaadiaacaappiaaaaaoeiaaaaaoolaaeaaaaaeaaaaadiaacaappiaaaaaoeia
aaaaoelaecaaaaadadaacpiaadaaoelaaaaioekaecaaaaadabaacpiaabaaoeia
acaioekaaeaaaaaeabaacdiaabaaohiaaeaaaakaaeaaffkaaeaaaaaeabaaciia
abaaaaiaabaaaaibadaappkaaeaaaaaeabaaciiaabaaffiaabaaffibabaappia
ahaaaaacabaaciiaabaappiaagaaaaacabaaceiaabaappiaaiaaaaadabaaciia
abaaoeiaacaaoeiaaiaaaaadaaaaceiaabaaoeiaacaaoelaalaaaaadabaacbia
aaaakkiaaeaakkkaalaaaaadaaaaaeiaabaappiaaeaakkkacaaaaaadabaaacia
aaaakkiaaaaappiaecaaaaadacaacpiaaaaaoeiaabaioekaecaaaaadaaaacpia
aaaaoeiaadaioekaafaaaaadaaaachiaaaaaoeiaacaappiaafaaaaadacaachia
acaaoeiaabaaoekaafaaaaadacaachiaacaaoeiaaaaaoekaafaaaaadaaaachia
aaaaoeiaacaaffkaafaaaaadaaaachiaaaaaoeiaabaaffiaafaaaaadaaaachia
aaaaoeiaaaaaoekaaeaaaaaeaaaachiaacaaoeiaabaaaaiaaaaaoeiaacaaaaad
aaaaciiaadaappiaadaappiaafaaaaadaaaachiaaaaappiaaaaaoeiaabaaaaac
aaaaaiiaaeaakkkaabaaaaacaaaicpiaaaaaoeiappppaaaafdeieefcieafaaaa
eaaaaaaagbabaaaafjaaaaaeegiocaaaaaaaaaaaajaaaaaafkaaaaadaagabaaa
aaaaaaaafkaaaaadaagabaaaabaaaaaafkaaaaadaagabaaaacaaaaaafkaaaaad
aagabaaaadaaaaaafkaaaaadaagabaaaaeaaaaaafibiaaaeaahabaaaaaaaaaaa
ffffaaaafibiaaaeaahabaaaabaaaaaaffffaaaafibiaaaeaahabaaaacaaaaaa
ffffaaaafibiaaaeaahabaaaadaaaaaaffffaaaafibiaaaeaahabaaaaeaaaaaa
ffffaaaagcbaaaadpcbabaaaabaaaaaagcbaaaadhcbabaaaacaaaaaagcbaaaad
hcbabaaaadaaaaaagcbaaaaddcbabaaaaeaaaaaagfaaaaadpccabaaaaaaaaaaa
giaaaaacaeaaaaaabaaaaaahbcaabaaaaaaaaaaaegbcbaaaacaaaaaaegbcbaaa
acaaaaaaeeaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaadcaaaaajocaabaaa
aaaaaaaaagbjbaaaacaaaaaaagaabaaaaaaaaaaaagbjbaaaadaaaaaabaaaaaah
bcaabaaaabaaaaaajgahbaaaaaaaaaaajgahbaaaaaaaaaaaeeaaaaafbcaabaaa
abaaaaaaakaabaaaabaaaaaadiaaaaahocaabaaaaaaaaaaafgaobaaaaaaaaaaa
agaabaaaabaaaaaadiaaaaahdcaabaaaabaaaaaaagaabaaaaaaaaaaaegbabaaa
acaaaaaadcaaaaajbcaabaaaaaaaaaaackbabaaaacaaaaaaakaabaaaaaaaaaaa
abeaaaaadnaknhdoaoaaaaahpcaabaaaabaaaaaaegaebaaaabaaaaaaagaabaaa
aaaaaaaaefaaaaajpcaabaaaacaaaaaaogbkbaaaabaaaaaaeghobaaaaaaaaaaa
aagabaaaaeaaaaaadiaaaaaldcaabaaaacaaaaaacgikcaaaaaaaaaaaaiaaaaaa
aceaaaaaaaaaaadpaaaaaaecaaaaaaaaaaaaaaaadcaaaaalbcaabaaaaaaaaaaa
dkaabaaaacaaaaaackiacaaaaaaaaaaaaiaaaaaaakaabaiaebaaaaaaacaaaaaa
dcaaaaajpcaabaaaabaaaaaaagaabaaaaaaaaaaaegaobaaaabaaaaaaegbobaaa
abaaaaaaefaaaaajpcaabaaaadaaaaaaogakbaaaabaaaaaaeghobaaaadaaaaaa
aagabaaaacaaaaaadcaaaaapdcaabaaaadaaaaaahgapbaaaadaaaaaaaceaaaaa
aaaaaaeaaaaaaaeaaaaaaaaaaaaaaaaaaceaaaaaaaaaialpaaaaialpaaaaaaaa
aaaaaaaadcaaaaakbcaabaaaaaaaaaaaakaabaiaebaaaaaaadaaaaaaakaabaaa
adaaaaaaabeaaaaaaaaaiadpdcaaaaakbcaabaaaaaaaaaaabkaabaiaebaaaaaa
adaaaaaabkaabaaaadaaaaaaakaabaaaaaaaaaaaelaaaaafecaabaaaadaaaaaa
akaabaaaaaaaaaaabaaaaaahbcaabaaaaaaaaaaaegacbaaaadaaaaaajgahbaaa
aaaaaaaabaaaaaahccaabaaaaaaaaaaaegacbaaaadaaaaaaegbcbaaaadaaaaaa
deaaaaakdcaabaaaaaaaaaaaegaabaaaaaaaaaaaaceaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaacpaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaadiaaaaah
bcaabaaaaaaaaaaaakaabaaaaaaaaaaabkaabaaaacaaaaaabjaaaaafbcaabaaa
aaaaaaaaakaabaaaaaaaaaaaefaaaaajpcaabaaaacaaaaaaegaabaaaabaaaaaa
eghobaaaacaaaaaaaagabaaaadaaaaaaefaaaaajpcaabaaaabaaaaaaegaabaaa
abaaaaaaeghobaaaabaaaaaaaagabaaaabaaaaaadiaaaaahhcaabaaaacaaaaaa
egacbaaaacaaaaaapgapbaaaabaaaaaadiaaaaaihcaabaaaabaaaaaaegacbaaa
abaaaaaaegiccaaaaaaaaaaaahaaaaaadiaaaaaihcaabaaaabaaaaaaegacbaaa
abaaaaaaegiccaaaaaaaaaaaabaaaaaadiaaaaaihcaabaaaacaaaaaaegacbaaa
acaaaaaafgifcaaaaaaaaaaaaiaaaaaadiaaaaahncaabaaaaaaaaaaaagaabaaa
aaaaaaaaagajbaaaacaaaaaadiaaaaaincaabaaaaaaaaaaaagaobaaaaaaaaaaa
agijcaaaaaaaaaaaabaaaaaadcaaaaajhcaabaaaaaaaaaaaegacbaaaabaaaaaa
fgafbaaaaaaaaaaaigadbaaaaaaaaaaaefaaaaajpcaabaaaabaaaaaaegbabaaa
aeaaaaaaeghobaaaaeaaaaaaaagabaaaaaaaaaaaaaaaaaahicaabaaaaaaaaaaa
dkaabaaaabaaaaaadkaabaaaabaaaaaadiaaaaahhccabaaaaaaaaaaapgapbaaa
aaaaaaaaegacbaaaaaaaaaaadgaaaaaficcabaaaaaaaaaaaabeaaaaaaaaaaaaa
doaaaaabejfdeheojiaaaaaaafaaaaaaaiaaaaaaiaaaaaaaaaaaaaaaabaaaaaa
adaaaaaaaaaaaaaaapaaaaaaimaaaaaaaaaaaaaaaaaaaaaaadaaaaaaabaaaaaa
apapaaaaimaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaaahahaaaaimaaaaaa
acaaaaaaaaaaaaaaadaaaaaaadaaaaaaahahaaaaimaaaaaaadaaaaaaaaaaaaaa
adaaaaaaaeaaaaaaadadaaaafdfgfpfaepfdejfeejepeoaafeeffiedepepfcee
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
//   opengl - ALU: 30 to 30
//   d3d9 - ALU: 31 to 31
//   d3d11 - ALU: 20 to 20, TEX: 0 to 0, FLOW: 1 to 1
//   d3d11_9x - ALU: 20 to 20, TEX: 0 to 0, FLOW: 1 to 1
SubProgram "opengl " {
Keywords { }
Bind "vertex" Vertex
Bind "tangent" ATTR14
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 13 [_WorldSpaceCameraPos]
Matrix 5 [_Object2World]
Matrix 9 [_World2Object]
Vector 14 [unity_Scale]
Vector 15 [_BumpMap_ST]
"!!ARBvp1.0
# 30 ALU
PARAM c[16] = { { 1 },
		state.matrix.mvp,
		program.local[5..15] };
TEMP R0;
TEMP R1;
TEMP R2;
MOV R0.xyz, vertex.attrib[14];
MUL R1.xyz, vertex.normal.zxyw, R0.yzxw;
MAD R0.xyz, vertex.normal.yzxw, R0.zxyw, -R1;
MUL R1.xyz, R0, vertex.attrib[14].w;
MOV R0.xyz, c[13];
MOV R0.w, c[0].x;
DP4 R2.z, R0, c[11];
DP4 R2.x, R0, c[9];
DP4 R2.y, R0, c[10];
MAD R0.xyz, R2, c[14].w, -vertex.position;
DP3 result.texcoord[1].y, R0, R1;
DP3 result.texcoord[1].z, vertex.normal, R0;
DP3 result.texcoord[1].x, R0, vertex.attrib[14];
DP3 R0.y, R1, c[5];
DP3 R0.x, vertex.attrib[14], c[5];
DP3 R0.z, vertex.normal, c[5];
MUL result.texcoord[2].xyz, R0, c[14].w;
DP3 R0.y, R1, c[6];
DP3 R0.x, vertex.attrib[14], c[6];
DP3 R0.z, vertex.normal, c[6];
MUL result.texcoord[3].xyz, R0, c[14].w;
DP3 R0.y, R1, c[7];
DP3 R0.x, vertex.attrib[14], c[7];
DP3 R0.z, vertex.normal, c[7];
MUL result.texcoord[4].xyz, R0, c[14].w;
MAD result.texcoord[0].xy, vertex.texcoord[0], c[15], c[15].zwzw;
DP4 result.position.w, vertex.position, c[4];
DP4 result.position.z, vertex.position, c[3];
DP4 result.position.y, vertex.position, c[2];
DP4 result.position.x, vertex.position, c[1];
END
# 30 instructions, 3 R-regs
"
}

SubProgram "d3d9 " {
Keywords { }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Matrix 0 [glstate_matrix_mvp]
Vector 12 [_WorldSpaceCameraPos]
Matrix 4 [_Object2World]
Matrix 8 [_World2Object]
Vector 13 [unity_Scale]
Vector 14 [_BumpMap_ST]
"vs_2_0
; 31 ALU
def c15, 1.00000000, 0, 0, 0
dcl_position0 v0
dcl_tangent0 v1
dcl_normal0 v2
dcl_texcoord0 v3
mov r0.xyz, v1
mul r1.xyz, v2.zxyw, r0.yzxw
mov r0.xyz, v1
mad r0.xyz, v2.yzxw, r0.zxyw, -r1
mul r1.xyz, r0, v1.w
mov r0.xyz, c12
mov r0.w, c15.x
dp4 r2.z, r0, c10
dp4 r2.x, r0, c8
dp4 r2.y, r0, c9
mad r0.xyz, r2, c13.w, -v0
dp3 oT1.y, r0, r1
dp3 oT1.z, v2, r0
dp3 oT1.x, r0, v1
dp3 r0.y, r1, c4
dp3 r0.x, v1, c4
dp3 r0.z, v2, c4
mul oT2.xyz, r0, c13.w
dp3 r0.y, r1, c5
dp3 r0.x, v1, c5
dp3 r0.z, v2, c5
mul oT3.xyz, r0, c13.w
dp3 r0.y, r1, c6
dp3 r0.x, v1, c6
dp3 r0.z, v2, c6
mul oT4.xyz, r0, c13.w
mad oT0.xy, v3, c14, c14.zwzw
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
Vector 13 [_BumpMap_ST]
Matrix 5 [_Object2World] 3
Matrix 8 [_World2Object] 4
Vector 0 [_WorldSpaceCameraPos]
Matrix 1 [glstate_matrix_mvp] 4
Vector 12 [unity_Scale]
// Shader Timing Estimate, in Cycles/64 vertex vector:
// ALU: 42.67 (32 instructions), vertex: 32, texture: 0,
//   sequencer: 18,  11 GPRs, 15 threads,
// Performance (if enough threads): ~42 cycles per vector
// * Vertex cycle estimates are assuming 3 vfetch_minis for every vfetch_full,
//     with <= 32 bytes per vfetch_full group.

"vs_360
backbbabaaaaabmiaaaaabpiaaaaaaaaaaaaaaceaaaaaaaaaaaaabgaaaaaaaaa
aaaaaaaaaaaaabdiaaaaaabmaaaaabckpppoadaaaaaaaaagaaaaaabmaaaaaaaa
aaaaabcdaaaaaajeaaacaaanaaabaaaaaaaaaakaaaaaaaaaaaaaaalaaaacaaaf
aaadaaaaaaaaaamaaaaaaaaaaaaaaanaaaacaaaiaaaeaaaaaaaaaamaaaaaaaaa
aaaaaanoaaacaaaaaaabaaaaaaaaaapeaaaaaaaaaaaaabaeaaacaaabaaaeaaaa
aaaaaamaaaaaaaaaaaaaabbhaaacaaamaaabaaaaaaaaaakaaaaaaaaafpechfgn
haengbhafpfdfeaaaaabaaadaaabaaaeaaabaaaaaaaaaaaafpepgcgkgfgdhedc
fhgphcgmgeaaklklaaadaaadaaaeaaaeaaabaaaaaaaaaaaafpfhgphcgmgedcep
gcgkgfgdheaafpfhgphcgmgefdhagbgdgfedgbgngfhcgbfagphdaaklaaabaaad
aaabaaadaaabaaaaaaaaaaaaghgmhdhegbhegffpgngbhehcgjhifpgnhghaaahf
gogjhehjfpfdgdgbgmgfaahghdfpddfpdaaadccodacodcdadddfddcodaaaklkl
aaaaaaaaaaaaabpiaaebaaakaaaaaaaaaaaaaaaaaaaadikfaaaaaaabaaaaaaae
aaaaaaahaaaaacjaaabaaaafaaaagaagaaaadaahaacafaaiaaaadafaaaabhbfb
aaaehcfcaaafhdfdaaaghefeaaaabacdaaaaaabpaaaaaacaaaaabacbaaaabacc
aaaabachaaaabacipaffeaafaaaabcaamcaaaaaaaaaaeaajaaaabcaameaaaaaa
aaaagaangabdbcaabcaaaaaaaaaagabjgabpbcaabcaaaaaaaaaaeacfaaaaccaa
aaaaaaaaafpigaaaaaaaagiiaaaaaaaaafpihaaaaaaaagiiaaaaaaaaafpiaaaa
aaaaacihaaaaaaaaafpieaaaaaaaapmiaaaaaaaakmbpaiabaabliiehkbagaeaf
miapaaabaamgiiaaklagadabmiapaaabaalbdejeklagacabmiapiadoaagmaade
klagababmiahaaadaamamgmaalakaaalbeceabaiaablgmblkbahahahkmcnaiab
aamgkoebibaaahagkibhacafaamgmamdibahahagkichacakaalbgcedibahagag
miahaaajaalelbleclajaaadkiehacadaabcgfidmbaaahagmiaoaaadabilkgpm
olaaahadmiahaaajaamagmleclaiaaajmialaaacaalblomaklaaafacmiahaaak
aagmloleklahafakbeaeaaacaagmgmgmoaacabakaebbacadaamglbmgoaakafaf
beaeaaafaalbmglboaacabakmiahaaagabmablmaklajamagaebhafabaamdblgm
obadahafkmccadafaalomdmanaaiadahkmbmaaadaamgigiaibabagahmiabiaab
aaloloaapaagahaamiaciaabaamaloaapaabagaamiaeiaabaalobcaapaagaaaa
miahiaacaamablaakbafamaamiadiaaaaalalabkilaeananmiagaaaaaalblgbg
klabafadbeacaaacaalblbbloaaaadacaeecadadaamggmbloaaaaaabmiahiaad
aamablaakbadamaamiahiaaeaamablaakbacamaaaaaaaaaaaaaaaaaaaaaaaaaa
"
}

SubProgram "ps3 " {
Keywords { }
Matrix 256 [glstate_matrix_mvp]
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 467 [_WorldSpaceCameraPos]
Matrix 260 [_Object2World]
Matrix 264 [_World2Object]
Vector 466 [unity_Scale]
Vector 465 [_BumpMap_ST]
"sce_vp_rsx // 29 instructions using 6 registers
[Configuration]
8
0000001d41050600
[Microcode]
464
00021c6c00400e0c0106c0836041dffc00029c6c005d300c0186c0836041dffc
401f9c6c011d1808010400d740619f9c401f9c6c01d0300d8106c0c360403f80
401f9c6c01d0200d8106c0c360405f80401f9c6c01d0100d8106c0c360409f80
401f9c6c01d0000d8106c0c360411f8000001c6c0150620c010600c360405ffc
00001c6c01506e0c010600c360411ffc00009c6c0150520c010600c360405ffc
00009c6c01505e0c010600c360411ffc00011c6c0150420c010600c360405ffc
00011c6c01504e0c010600c360411ffc00019c6c0190a00c0a86c0c360405ffc
00019c6c0190900c0a86c0c360409ffc00019c6c0190800c0a86c0c360411ffc
00029c6c00800243011844436041dffc00021c6c010002308121846302a1dffc
00019c6c011d200c06bfc0e30041dffc00021c6c00800e0c08bfc0836041dffc
401f9c6c0140020c0106034360405fa0401f9c6c01400e0c0686008360411fa0
401f9c6c0140000c0686044360409fa000001c6c0150600c088600c360409ffc
00011c6c0150400c088600c360409ffc00009c6c0150500c088600c360409ffc
401f9c6c009d200c04bfc0c36041dfa4401f9c6c009d200c02bfc0c36041dfa8
401f9c6c009d200c00bfc0c36041dfad
"
}

SubProgram "d3d11 " {
Keywords { }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 96 // 96 used size, 8 vars
Vector 80 [_BumpMap_ST] 4
ConstBuffer "UnityPerCamera" 128 // 76 used size, 8 vars
Vector 64 [_WorldSpaceCameraPos] 3
ConstBuffer "UnityPerDraw" 336 // 336 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
Matrix 192 [_Object2World] 4
Matrix 256 [_World2Object] 4
Vector 320 [unity_Scale] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityPerDraw" 2
// 38 instructions, 3 temp regs, 0 temp arrays:
// ALU 20 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0
eefiecedlidkopochmfpkkgepgnfnmjejcijhkccabaaaaaaoiagaaaaadaaaaaa
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
aeaaaaaaaaaaaaaaadaaaaaaafaaaaaaahaiaaaafdfgfpfaepfdejfeejepeoaa
feeffiedepepfceeaaklklklfdeieefcdeafaaaaeaaaabaaenabaaaafjaaaaae
egiocaaaaaaaaaaaagaaaaaafjaaaaaeegiocaaaabaaaaaaafaaaaaafjaaaaae
egiocaaaacaaaaaabfaaaaaafpaaaaadpcbabaaaaaaaaaaafpaaaaadpcbabaaa
abaaaaaafpaaaaadhcbabaaaacaaaaaafpaaaaaddcbabaaaadaaaaaaghaaaaae
pccabaaaaaaaaaaaabaaaaaagfaaaaaddccabaaaabaaaaaagfaaaaadhccabaaa
acaaaaaagfaaaaadhccabaaaadaaaaaagfaaaaadhccabaaaaeaaaaaagfaaaaad
hccabaaaafaaaaaagiaaaaacadaaaaaadiaaaaaipcaabaaaaaaaaaaafgbfbaaa
aaaaaaaaegiocaaaacaaaaaaabaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaa
acaaaaaaaaaaaaaaagbabaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaa
aaaaaaaaegiocaaaacaaaaaaacaaaaaakgbkbaaaaaaaaaaaegaobaaaaaaaaaaa
dcaaaaakpccabaaaaaaaaaaaegiocaaaacaaaaaaadaaaaaapgbpbaaaaaaaaaaa
egaobaaaaaaaaaaadcaaaaaldccabaaaabaaaaaaegbabaaaadaaaaaaegiacaaa
aaaaaaaaafaaaaaaogikcaaaaaaaaaaaafaaaaaadiaaaaajhcaabaaaaaaaaaaa
fgifcaaaabaaaaaaaeaaaaaaegiccaaaacaaaaaabbaaaaaadcaaaaalhcaabaaa
aaaaaaaaegiccaaaacaaaaaabaaaaaaaagiacaaaabaaaaaaaeaaaaaaegacbaaa
aaaaaaaadcaaaaalhcaabaaaaaaaaaaaegiccaaaacaaaaaabcaaaaaakgikcaaa
abaaaaaaaeaaaaaaegacbaaaaaaaaaaaaaaaaaaihcaabaaaaaaaaaaaegacbaaa
aaaaaaaaegiccaaaacaaaaaabdaaaaaadcaaaaalhcaabaaaaaaaaaaaegacbaaa
aaaaaaaapgipcaaaacaaaaaabeaaaaaaegbcbaiaebaaaaaaaaaaaaaabaaaaaah
bccabaaaacaaaaaaegbcbaaaabaaaaaaegacbaaaaaaaaaaabaaaaaaheccabaaa
acaaaaaaegbcbaaaacaaaaaaegacbaaaaaaaaaaadiaaaaahhcaabaaaabaaaaaa
jgbebaaaabaaaaaacgbjbaaaacaaaaaadcaaaaakhcaabaaaabaaaaaajgbebaaa
acaaaaaacgbjbaaaabaaaaaaegacbaiaebaaaaaaabaaaaaadiaaaaahhcaabaaa
abaaaaaaegacbaaaabaaaaaapgbpbaaaabaaaaaabaaaaaahcccabaaaacaaaaaa
egacbaaaabaaaaaaegacbaaaaaaaaaaadgaaaaagbcaabaaaaaaaaaaaakiacaaa
acaaaaaaamaaaaaadgaaaaagccaabaaaaaaaaaaaakiacaaaacaaaaaaanaaaaaa
dgaaaaagecaabaaaaaaaaaaaakiacaaaacaaaaaaaoaaaaaabaaaaaahccaabaaa
acaaaaaaegacbaaaabaaaaaaegacbaaaaaaaaaaabaaaaaahbcaabaaaacaaaaaa
egbcbaaaabaaaaaaegacbaaaaaaaaaaabaaaaaahecaabaaaacaaaaaaegbcbaaa
acaaaaaaegacbaaaaaaaaaaadiaaaaaihccabaaaadaaaaaaegacbaaaacaaaaaa
pgipcaaaacaaaaaabeaaaaaadgaaaaagbcaabaaaaaaaaaaabkiacaaaacaaaaaa
amaaaaaadgaaaaagccaabaaaaaaaaaaabkiacaaaacaaaaaaanaaaaaadgaaaaag
ecaabaaaaaaaaaaabkiacaaaacaaaaaaaoaaaaaabaaaaaahccaabaaaacaaaaaa
egacbaaaabaaaaaaegacbaaaaaaaaaaabaaaaaahbcaabaaaacaaaaaaegbcbaaa
abaaaaaaegacbaaaaaaaaaaabaaaaaahecaabaaaacaaaaaaegbcbaaaacaaaaaa
egacbaaaaaaaaaaadiaaaaaihccabaaaaeaaaaaaegacbaaaacaaaaaapgipcaaa
acaaaaaabeaaaaaadgaaaaagbcaabaaaaaaaaaaackiacaaaacaaaaaaamaaaaaa
dgaaaaagccaabaaaaaaaaaaackiacaaaacaaaaaaanaaaaaadgaaaaagecaabaaa
aaaaaaaackiacaaaacaaaaaaaoaaaaaabaaaaaahccaabaaaabaaaaaaegacbaaa
abaaaaaaegacbaaaaaaaaaaabaaaaaahbcaabaaaabaaaaaaegbcbaaaabaaaaaa
egacbaaaaaaaaaaabaaaaaahecaabaaaabaaaaaaegbcbaaaacaaaaaaegacbaaa
aaaaaaaadiaaaaaihccabaaaafaaaaaaegacbaaaabaaaaaapgipcaaaacaaaaaa
beaaaaaadoaaaaab"
}

SubProgram "gles " {
Keywords { }
"!!GLES
#define SHADER_API_GLES 1
#define tex2D texture2D


#ifdef VERTEX
#define gl_ModelViewProjectionMatrix glstate_matrix_mvp
uniform mat4 glstate_matrix_mvp;

varying highp vec3 xlv_TEXCOORD4;
varying highp vec3 xlv_TEXCOORD3;
varying highp vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp vec4 _BumpMap_ST;
uniform highp vec4 unity_Scale;
uniform highp mat4 _World2Object;
uniform highp mat4 _Object2World;

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
  highp vec4 tmpvar_9;
  tmpvar_9.w = 1.0;
  tmpvar_9.xyz = _WorldSpaceCameraPos;
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
  xlv_TEXCOORD1 = (tmpvar_5 * (((_World2Object * tmpvar_9).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = ((tmpvar_5 * v_6) * unity_Scale.w);
  xlv_TEXCOORD3 = ((tmpvar_5 * v_7) * unity_Scale.w);
  xlv_TEXCOORD4 = ((tmpvar_5 * v_8) * unity_Scale.w);
}



#endif
#ifdef FRAGMENT

varying highp vec3 xlv_TEXCOORD4;
varying highp vec3 xlv_TEXCOORD3;
varying highp vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp float _Parallax;
uniform mediump float _Shininess;
uniform sampler2D _ParallaxMap;
uniform sampler2D _BumpMap;
void main ()
{
  lowp vec4 res_1;
  lowp vec3 worldN_2;
  mediump vec3 tmpvar_3;
  mediump vec3 tmpvar_4;
  mediump float h_5;
  lowp float tmpvar_6;
  tmpvar_6 = texture2D (_ParallaxMap, xlv_TEXCOORD0).w;
  h_5 = tmpvar_6;
  mediump float height_7;
  height_7 = _Parallax;
  mediump vec3 viewDir_8;
  viewDir_8 = xlv_TEXCOORD1;
  highp vec3 v_9;
  mediump float tmpvar_10;
  tmpvar_10 = ((h_5 * height_7) - (height_7 / 2.0));
  mediump vec3 tmpvar_11;
  tmpvar_11 = normalize(viewDir_8);
  v_9 = tmpvar_11;
  v_9.z = (v_9.z + 0.42);
  highp vec2 tmpvar_12;
  tmpvar_12 = (xlv_TEXCOORD0 + (tmpvar_10 * (v_9.xy / v_9.z)));
  lowp vec3 tmpvar_13;
  tmpvar_13 = ((texture2D (_BumpMap, tmpvar_12).xyz * 2.0) - 1.0);
  tmpvar_4 = tmpvar_13;
  highp float tmpvar_14;
  tmpvar_14 = dot (xlv_TEXCOORD2, tmpvar_4);
  worldN_2.x = tmpvar_14;
  highp float tmpvar_15;
  tmpvar_15 = dot (xlv_TEXCOORD3, tmpvar_4);
  worldN_2.y = tmpvar_15;
  highp float tmpvar_16;
  tmpvar_16 = dot (xlv_TEXCOORD4, tmpvar_4);
  worldN_2.z = tmpvar_16;
  tmpvar_3 = worldN_2;
  mediump vec3 tmpvar_17;
  tmpvar_17 = ((tmpvar_3 * 0.5) + 0.5);
  res_1.xyz = tmpvar_17;
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

varying highp vec3 xlv_TEXCOORD4;
varying highp vec3 xlv_TEXCOORD3;
varying highp vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp vec4 _BumpMap_ST;
uniform highp vec4 unity_Scale;
uniform highp mat4 _World2Object;
uniform highp mat4 _Object2World;

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
  highp vec4 tmpvar_9;
  tmpvar_9.w = 1.0;
  tmpvar_9.xyz = _WorldSpaceCameraPos;
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
  xlv_TEXCOORD1 = (tmpvar_5 * (((_World2Object * tmpvar_9).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = ((tmpvar_5 * v_6) * unity_Scale.w);
  xlv_TEXCOORD3 = ((tmpvar_5 * v_7) * unity_Scale.w);
  xlv_TEXCOORD4 = ((tmpvar_5 * v_8) * unity_Scale.w);
}



#endif
#ifdef FRAGMENT

varying highp vec3 xlv_TEXCOORD4;
varying highp vec3 xlv_TEXCOORD3;
varying highp vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp float _Parallax;
uniform mediump float _Shininess;
uniform sampler2D _ParallaxMap;
uniform sampler2D _BumpMap;
void main ()
{
  lowp vec4 res_1;
  lowp vec3 worldN_2;
  mediump vec3 tmpvar_3;
  mediump vec3 tmpvar_4;
  mediump float h_5;
  lowp float tmpvar_6;
  tmpvar_6 = texture2D (_ParallaxMap, xlv_TEXCOORD0).w;
  h_5 = tmpvar_6;
  mediump float height_7;
  height_7 = _Parallax;
  mediump vec3 viewDir_8;
  viewDir_8 = xlv_TEXCOORD1;
  highp vec3 v_9;
  mediump float tmpvar_10;
  tmpvar_10 = ((h_5 * height_7) - (height_7 / 2.0));
  mediump vec3 tmpvar_11;
  tmpvar_11 = normalize(viewDir_8);
  v_9 = tmpvar_11;
  v_9.z = (v_9.z + 0.42);
  highp vec2 tmpvar_12;
  tmpvar_12 = (xlv_TEXCOORD0 + (tmpvar_10 * (v_9.xy / v_9.z)));
  lowp vec3 normal_13;
  normal_13.xy = ((texture2D (_BumpMap, tmpvar_12).wy * 2.0) - 1.0);
  normal_13.z = sqrt(((1.0 - (normal_13.x * normal_13.x)) - (normal_13.y * normal_13.y)));
  tmpvar_4 = normal_13;
  highp float tmpvar_14;
  tmpvar_14 = dot (xlv_TEXCOORD2, tmpvar_4);
  worldN_2.x = tmpvar_14;
  highp float tmpvar_15;
  tmpvar_15 = dot (xlv_TEXCOORD3, tmpvar_4);
  worldN_2.y = tmpvar_15;
  highp float tmpvar_16;
  tmpvar_16 = dot (xlv_TEXCOORD4, tmpvar_4);
  worldN_2.z = tmpvar_16;
  tmpvar_3 = worldN_2;
  mediump vec3 tmpvar_17;
  tmpvar_17 = ((tmpvar_3 * 0.5) + 0.5);
  res_1.xyz = tmpvar_17;
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
ConstBuffer "$Globals" 96 // 96 used size, 8 vars
Vector 80 [_BumpMap_ST] 4
ConstBuffer "UnityPerCamera" 128 // 76 used size, 8 vars
Vector 64 [_WorldSpaceCameraPos] 3
ConstBuffer "UnityPerDraw" 336 // 336 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
Matrix 192 [_Object2World] 4
Matrix 256 [_World2Object] 4
Vector 320 [unity_Scale] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityPerDraw" 2
// 38 instructions, 3 temp regs, 0 temp arrays:
// ALU 20 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0_level_9_3
eefiecedpnnbpoobljddbffaaiimhmnaddhldcaiabaaaaaabeakaaaaaeaaaaaa
daaaaaaafiadaaaajeaiaaaafmajaaaaebgpgodjcaadaaaacaadaaaaaaacpopp
lmacaaaageaaaaaaafaaceaaaaaagaaaaaaagaaaaaaaceaaabaagaaaaaaaafaa
abaaabaaaaaaaaaaabaaaeaaabaaacaaaaaaaaaaacaaaaaaaeaaadaaaaaaaaaa
acaaamaaadaaahaaaaaaaaaaacaabaaaafaaakaaaaaaaaaaaaaaaaaaabacpopp
bpaaaaacafaaaaiaaaaaapjabpaaaaacafaaabiaabaaapjabpaaaaacafaaacia
acaaapjabpaaaaacafaaadiaadaaapjaaeaaaaaeaaaaadoaadaaoejaabaaoeka
abaaookaabaaaaacaaaaahiaabaaoejaafaaaaadabaaahiaaaaamjiaacaancja
aeaaaaaeaaaaahiaacaamjjaaaaanciaabaaoeibafaaaaadaaaaahiaaaaaoeia
abaappjaabaaaaacabaaabiaahaaaakaabaaaaacabaaaciaaiaaaakaabaaaaac
abaaaeiaajaaaakaaiaaaaadacaaaciaaaaaoeiaabaaoeiaaiaaaaadacaaabia
abaaoejaabaaoeiaaiaaaaadacaaaeiaacaaoejaabaaoeiaafaaaaadacaaahoa
acaaoeiaaoaappkaabaaaaacabaaabiaahaaffkaabaaaaacabaaaciaaiaaffka
abaaaaacabaaaeiaajaaffkaaiaaaaadacaaaciaaaaaoeiaabaaoeiaaiaaaaad
acaaabiaabaaoejaabaaoeiaaiaaaaadacaaaeiaacaaoejaabaaoeiaafaaaaad
adaaahoaacaaoeiaaoaappkaabaaaaacabaaabiaahaakkkaabaaaaacabaaacia
aiaakkkaabaaaaacabaaaeiaajaakkkaaiaaaaadacaaaciaaaaaoeiaabaaoeia
aiaaaaadacaaabiaabaaoejaabaaoeiaaiaaaaadacaaaeiaacaaoejaabaaoeia
afaaaaadaeaaahoaacaaoeiaaoaappkaabaaaaacabaaahiaacaaoekaafaaaaad
acaaahiaabaaffiaalaaoekaaeaaaaaeabaaaliaakaakekaabaaaaiaacaakeia
aeaaaaaeabaaahiaamaaoekaabaakkiaabaapeiaacaaaaadabaaahiaabaaoeia
anaaoekaaeaaaaaeabaaahiaabaaoeiaaoaappkaaaaaoejbaiaaaaadabaaaboa
abaaoejaabaaoeiaaiaaaaadabaaacoaaaaaoeiaabaaoeiaaiaaaaadabaaaeoa
acaaoejaabaaoeiaafaaaaadaaaaapiaaaaaffjaaeaaoekaaeaaaaaeaaaaapia
adaaoekaaaaaaajaaaaaoeiaaeaaaaaeaaaaapiaafaaoekaaaaakkjaaaaaoeia
aeaaaaaeaaaaapiaagaaoekaaaaappjaaaaaoeiaaeaaaaaeaaaaadmaaaaappia
aaaaoekaaaaaoeiaabaaaaacaaaaammaaaaaoeiappppaaaafdeieefcdeafaaaa
eaaaabaaenabaaaafjaaaaaeegiocaaaaaaaaaaaagaaaaaafjaaaaaeegiocaaa
abaaaaaaafaaaaaafjaaaaaeegiocaaaacaaaaaabfaaaaaafpaaaaadpcbabaaa
aaaaaaaafpaaaaadpcbabaaaabaaaaaafpaaaaadhcbabaaaacaaaaaafpaaaaad
dcbabaaaadaaaaaaghaaaaaepccabaaaaaaaaaaaabaaaaaagfaaaaaddccabaaa
abaaaaaagfaaaaadhccabaaaacaaaaaagfaaaaadhccabaaaadaaaaaagfaaaaad
hccabaaaaeaaaaaagfaaaaadhccabaaaafaaaaaagiaaaaacadaaaaaadiaaaaai
pcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaacaaaaaaabaaaaaadcaaaaak
pcaabaaaaaaaaaaaegiocaaaacaaaaaaaaaaaaaaagbabaaaaaaaaaaaegaobaaa
aaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaacaaaaaaacaaaaaakgbkbaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaakpccabaaaaaaaaaaaegiocaaaacaaaaaa
adaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaaldccabaaaabaaaaaa
egbabaaaadaaaaaaegiacaaaaaaaaaaaafaaaaaaogikcaaaaaaaaaaaafaaaaaa
diaaaaajhcaabaaaaaaaaaaafgifcaaaabaaaaaaaeaaaaaaegiccaaaacaaaaaa
bbaaaaaadcaaaaalhcaabaaaaaaaaaaaegiccaaaacaaaaaabaaaaaaaagiacaaa
abaaaaaaaeaaaaaaegacbaaaaaaaaaaadcaaaaalhcaabaaaaaaaaaaaegiccaaa
acaaaaaabcaaaaaakgikcaaaabaaaaaaaeaaaaaaegacbaaaaaaaaaaaaaaaaaai
hcaabaaaaaaaaaaaegacbaaaaaaaaaaaegiccaaaacaaaaaabdaaaaaadcaaaaal
hcaabaaaaaaaaaaaegacbaaaaaaaaaaapgipcaaaacaaaaaabeaaaaaaegbcbaia
ebaaaaaaaaaaaaaabaaaaaahbccabaaaacaaaaaaegbcbaaaabaaaaaaegacbaaa
aaaaaaaabaaaaaaheccabaaaacaaaaaaegbcbaaaacaaaaaaegacbaaaaaaaaaaa
diaaaaahhcaabaaaabaaaaaajgbebaaaabaaaaaacgbjbaaaacaaaaaadcaaaaak
hcaabaaaabaaaaaajgbebaaaacaaaaaacgbjbaaaabaaaaaaegacbaiaebaaaaaa
abaaaaaadiaaaaahhcaabaaaabaaaaaaegacbaaaabaaaaaapgbpbaaaabaaaaaa
baaaaaahcccabaaaacaaaaaaegacbaaaabaaaaaaegacbaaaaaaaaaaadgaaaaag
bcaabaaaaaaaaaaaakiacaaaacaaaaaaamaaaaaadgaaaaagccaabaaaaaaaaaaa
akiacaaaacaaaaaaanaaaaaadgaaaaagecaabaaaaaaaaaaaakiacaaaacaaaaaa
aoaaaaaabaaaaaahccaabaaaacaaaaaaegacbaaaabaaaaaaegacbaaaaaaaaaaa
baaaaaahbcaabaaaacaaaaaaegbcbaaaabaaaaaaegacbaaaaaaaaaaabaaaaaah
ecaabaaaacaaaaaaegbcbaaaacaaaaaaegacbaaaaaaaaaaadiaaaaaihccabaaa
adaaaaaaegacbaaaacaaaaaapgipcaaaacaaaaaabeaaaaaadgaaaaagbcaabaaa
aaaaaaaabkiacaaaacaaaaaaamaaaaaadgaaaaagccaabaaaaaaaaaaabkiacaaa
acaaaaaaanaaaaaadgaaaaagecaabaaaaaaaaaaabkiacaaaacaaaaaaaoaaaaaa
baaaaaahccaabaaaacaaaaaaegacbaaaabaaaaaaegacbaaaaaaaaaaabaaaaaah
bcaabaaaacaaaaaaegbcbaaaabaaaaaaegacbaaaaaaaaaaabaaaaaahecaabaaa
acaaaaaaegbcbaaaacaaaaaaegacbaaaaaaaaaaadiaaaaaihccabaaaaeaaaaaa
egacbaaaacaaaaaapgipcaaaacaaaaaabeaaaaaadgaaaaagbcaabaaaaaaaaaaa
ckiacaaaacaaaaaaamaaaaaadgaaaaagccaabaaaaaaaaaaackiacaaaacaaaaaa
anaaaaaadgaaaaagecaabaaaaaaaaaaackiacaaaacaaaaaaaoaaaaaabaaaaaah
ccaabaaaabaaaaaaegacbaaaabaaaaaaegacbaaaaaaaaaaabaaaaaahbcaabaaa
abaaaaaaegbcbaaaabaaaaaaegacbaaaaaaaaaaabaaaaaahecaabaaaabaaaaaa
egbcbaaaacaaaaaaegacbaaaaaaaaaaadiaaaaaihccabaaaafaaaaaaegacbaaa
abaaaaaapgipcaaaacaaaaaabeaaaaaadoaaaaabejfdeheomaaaaaaaagaaaaaa
aiaaaaaajiaaaaaaaaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaacaaaaaaahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaa
apadaaaalaaaaaaaabaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaljaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeo
ehefeofeaaeoepfcenebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheo
laaaaaaaagaaaaaaaiaaaaaajiaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaa
apaaaaaakeaaaaaaaaaaaaaaaaaaaaaaadaaaaaaabaaaaaaadamaaaakeaaaaaa
abaaaaaaaaaaaaaaadaaaaaaacaaaaaaahaiaaaakeaaaaaaacaaaaaaaaaaaaaa
adaaaaaaadaaaaaaahaiaaaakeaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaa
ahaiaaaakeaaaaaaaeaaaaaaaaaaaaaaadaaaaaaafaaaaaaahaiaaaafdfgfpfa
epfdejfeejepeoaafeeffiedepepfceeaaklklkl"
}

}
Program "fp" {
// Fragment combos: 1
//   opengl - ALU: 23 to 23, TEX: 2 to 2
//   d3d9 - ALU: 23 to 23, TEX: 2 to 2
//   d3d11 - ALU: 9 to 9, TEX: 2 to 2, FLOW: 1 to 1
//   d3d11_9x - ALU: 9 to 9, TEX: 2 to 2, FLOW: 1 to 1
SubProgram "opengl " {
Keywords { }
Float 0 [_Shininess]
Float 1 [_Parallax]
SetTexture 0 [_ParallaxMap] 2D
SetTexture 1 [_BumpMap] 2D
"!!ARBfp1.0
OPTION ARB_precision_hint_fastest;
# 23 ALU, 2 TEX
PARAM c[3] = { program.local[0..1],
		{ 0.5, 0.41999999, 2, 1 } };
TEMP R0;
TEMP R1;
TEX R0.w, fragment.texcoord[0], texture[0], 2D;
DP3 R0.x, fragment.texcoord[1], fragment.texcoord[1];
RSQ R0.x, R0.x;
MUL R0.xyz, R0.x, fragment.texcoord[1];
ADD R1.x, R0.z, c[2].y;
MOV R0.z, c[1].x;
RCP R1.x, R1.x;
MUL R1.xy, R0, R1.x;
MUL R0.z, R0, c[2].x;
MAD R0.x, R0.w, c[1], -R0.z;
MAD R0.xy, R0.x, R1, fragment.texcoord[0];
MOV result.color.w, c[0].x;
TEX R0.yw, R0, texture[1], 2D;
MAD R0.xy, R0.wyzw, c[2].z, -c[2].w;
MUL R0.z, R0.y, R0.y;
MAD R0.z, -R0.x, R0.x, -R0;
ADD R0.z, R0, c[2].w;
RSQ R0.z, R0.z;
RCP R0.z, R0.z;
DP3 R1.z, fragment.texcoord[4], R0;
DP3 R1.x, R0, fragment.texcoord[2];
DP3 R1.y, R0, fragment.texcoord[3];
MAD result.color.xyz, R1, c[2].x, c[2].x;
END
# 23 instructions, 2 R-regs
"
}

SubProgram "d3d9 " {
Keywords { }
Float 0 [_Shininess]
Float 1 [_Parallax]
SetTexture 0 [_ParallaxMap] 2D
SetTexture 1 [_BumpMap] 2D
"ps_2_0
; 23 ALU, 2 TEX
dcl_2d s0
dcl_2d s1
def c2, 0.50000000, 0.41999999, 2.00000000, -1.00000000
def c3, 1.00000000, 0, 0, 0
dcl t0.xy
dcl t1.xyz
dcl t2.xyz
dcl t3.xyz
dcl t4.xyz
texld r0, t0, s0
dp3_pp r0.x, t1, t1
rsq_pp r0.x, r0.x
mul_pp r2.xyz, r0.x, t1
add r0.x, r2.z, c2.y
rcp r1.x, r0.x
mov_pp r0.x, c2
mul_pp r0.x, c1, r0
mul r1.xy, r2, r1.x
mad_pp r0.x, r0.w, c1, -r0
mad r0.xy, r0.x, r1, t0
texld r0, r0, s1
mov r0.x, r0.w
mad_pp r1.xy, r0, c2.z, c2.w
mul_pp r0.x, r1.y, r1.y
mad_pp r0.x, -r1, r1, -r0
add_pp r0.x, r0, c3
rsq_pp r0.x, r0.x
rcp_pp r1.z, r0.x
dp3 r0.z, t4, r1
dp3 r0.x, r1, t2
dp3 r0.y, r1, t3
mad_pp r0.xyz, r0, c2.x, c2.x
mov_pp r0.w, c0.x
mov_pp oC0, r0
"
}

SubProgram "xbox360 " {
Keywords { }
Float 1 [_Parallax]
Float 0 [_Shininess]
SetTexture 0 [_BumpMap] 2D
SetTexture 1 [_ParallaxMap] 2D
// Shader Timing Estimate, in Cycles/64 pixel vector:
// ALU: 21.33 (16 instructions), vertex: 0, texture: 8,
//   sequencer: 10, interpolator: 20;    5 GPRs, 36 threads,
// Performance (if enough threads): ~21 cycles per vector
// * Texture cycle estimates are assuming an 8bit/component texture with no
//     aniso or trilinear filtering.

"ps_360
backbbaaaaaaabfeaaaaabeiaaaaaaaaaaaaaaceaaaaaapiaaaaabcaaaaaaaaa
aaaaaaaaaaaaaanaaaaaaabmaaaaaamdppppadaaaaaaaaaeaaaaaabmaaaaaaaa
aaaaaalmaaaaaagmaaadaaaaaaabaaaaaaaaaahiaaaaaaaaaaaaaaiiaaacaaab
aaabaaaaaaaaaajeaaaaaaaaaaaaaakeaaadaaabaaabaaaaaaaaaahiaaaaaaaa
aaaaaalbaaacaaaaaaabaaaaaaaaaajeaaaaaaaafpechfgnhaengbhaaaklklkl
aaaeaaamaaabaaabaaabaaaaaaaaaaaafpfagbhcgbgmgmgbhiaaklklaaaaaaad
aaabaaabaaabaaaaaaaaaaaafpfagbhcgbgmgmgbhiengbhaaafpfdgigjgogjgo
gfhdhdaahahdfpddfpdaaadccodacodcdadddfddcodaaaklaaaaaaaaaaaaaaab
aaaaaaaaaaaaaaaaaaaaaabeabpmaabaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaeaaaaaabaibaaaaeaaaaaaaaaeaaaaaaaaaaaadikfaabpaabpaaaaaaab
aaaadafaaaaahbfbaaaahcfcaaaahdfdaaaahefeaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaadpaaaaaalpiaaaaadonhakdn
dpiaaaaaeaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaajgaadeaajbcaabcaaaaea
aaaaaaaagaanmeaabcaaaaaaaaaacabdaaaaccaaaaaaaaaabababaabbpbpphpp
aaaaeaaamiaiaaaaaaloloaapaababaafiieaaaaaagmgmblcbabpoiamiaeaaaa
abblgmmgklababaamiahaaabaablloaaobaaabaaleiaaaaaaaaaaamamcaaaapo
emiaaaaaaaaaaablocaaaaaamiadaaabaamfblaaobabaaaamiadaaaaaalamgla
olabaaaabaaiaaabbpbpppnjaaaaeaaabeiaiaaaaaaaaagmmcaaaaaamiadaaab
aagngmlbilaapppomiabaaaaaegngnblnbababpokaeaabaaaaaaaagmocaaaaia
miabaaaaaaloloaapaabacaamiacaaaaaaloloaapaabadaamiaeaaaaaaloloaa
paabaeaamiahiaaaaamagmgmilaapopoaaaaaaaaaaaaaaaaaaaaaaaa"
}

SubProgram "ps3 " {
Keywords { }
Float 0 [_Shininess]
Float 1 [_Parallax]
SetTexture 0 [_ParallaxMap] 2D
SetTexture 1 [_BumpMap] 2D
"sce_fp_rsx // 28 instructions using 2 registers
[Configuration]
24
ffffffff0007c020001fffe1000000000000840002000000
[Offsets]
2
_Shininess 1 0
00000110
_Parallax 2 0
0000007000000050
[Microcode]
448
90021700c8011c9dc8000001c8003fe1ae803940c8011c9dc8000029c800bfe1
0202030055001c9daa020000c8000001000000000a3d3ed70000000000000000
08800240fe041c9d00020000c800000100000000000000000000000000000000
1086014000021c9cc8000001c800000100000000000000000000000000000000
06023a00c9001c9dc8040001c800000102800440ff0c1c9d0002000055000001
0000bf000000000000000000000000009800010080011c9cc8000001c8003fe1
0600040001001c9cc80400015c00000114001702c8001c9dc8000001c8000001
06820440ce001c9daa0200005402000100000000000040000000bf8000000000
1080014000021c9cc8000001c800000100000000000000000000000000000000
02800240ab041c9cab040000c800000102800440c9041c9fc9040001c9000003
02800340c9001c9d00020000c800000100003f80000000000000000000000000
08823b4001003c9cc9000001c800000108800501c8011c9dc9040001c8003fe1
e4800500c9041c9dc8010001c8003fe1c2800500c9041c9dc8010001c8003fe1
0e810440c9001c9d000200000002000000003f00000000000000000000000000
"
}

SubProgram "d3d11 " {
Keywords { }
ConstBuffer "$Globals" 96 // 76 used size, 8 vars
Float 64 [_Shininess]
Float 72 [_Parallax]
BindCB "$Globals" 0
SetTexture 0 [_ParallaxMap] 2D 1
SetTexture 1 [_BumpMap] 2D 0
// 20 instructions, 2 temp regs, 0 temp arrays:
// ALU 9 float, 0 int, 0 uint
// TEX 2 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0
eefiecedmoooaojjfbkincacekjfdghmfnegodagabaaaaaafaaeaaaaadaaaaaa
cmaaaaaaoeaaaaaabiabaaaaejfdeheolaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaakeaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaadadaaaakeaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaakeaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaahahaaaakeaaaaaa
adaaaaaaaaaaaaaaadaaaaaaaeaaaaaaahahaaaakeaaaaaaaeaaaaaaaaaaaaaa
adaaaaaaafaaaaaaahahaaaafdfgfpfaepfdejfeejepeoaafeeffiedepepfcee
aaklklklepfdeheocmaaaaaaabaaaaaaaiaaaaaacaaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaaaaaaaaaapaaaaaafdfgfpfegbhcghgfheaaklklfdeieefcdaadaaaa
eaaaaaaammaaaaaafjaaaaaeegiocaaaaaaaaaaaafaaaaaafkaaaaadaagabaaa
aaaaaaaafkaaaaadaagabaaaabaaaaaafibiaaaeaahabaaaaaaaaaaaffffaaaa
fibiaaaeaahabaaaabaaaaaaffffaaaagcbaaaaddcbabaaaabaaaaaagcbaaaad
hcbabaaaacaaaaaagcbaaaadhcbabaaaadaaaaaagcbaaaadhcbabaaaaeaaaaaa
gcbaaaadhcbabaaaafaaaaaagfaaaaadpccabaaaaaaaaaaagiaaaaacacaaaaaa
baaaaaahbcaabaaaaaaaaaaaegbcbaaaacaaaaaaegbcbaaaacaaaaaaeeaaaaaf
bcaabaaaaaaaaaaaakaabaaaaaaaaaaadiaaaaahgcaabaaaaaaaaaaaagaabaaa
aaaaaaaaagbbbaaaacaaaaaadcaaaaajbcaabaaaaaaaaaaackbabaaaacaaaaaa
akaabaaaaaaaaaaaabeaaaaadnaknhdoaoaaaaahdcaabaaaaaaaaaaajgafbaaa
aaaaaaaaagaabaaaaaaaaaaaefaaaaajpcaabaaaabaaaaaaegbabaaaabaaaaaa
eghobaaaaaaaaaaaaagabaaaabaaaaaadiaaaaaiecaabaaaaaaaaaaackiacaaa
aaaaaaaaaeaaaaaaabeaaaaaaaaaaadpdcaaaaalecaabaaaaaaaaaaadkaabaaa
abaaaaaackiacaaaaaaaaaaaaeaaaaaackaabaiaebaaaaaaaaaaaaaadcaaaaaj
dcaabaaaaaaaaaaakgakbaaaaaaaaaaaegaabaaaaaaaaaaaegbabaaaabaaaaaa
efaaaaajpcaabaaaaaaaaaaaegaabaaaaaaaaaaaeghobaaaabaaaaaaaagabaaa
aaaaaaaadcaaaaapdcaabaaaaaaaaaaahgapbaaaaaaaaaaaaceaaaaaaaaaaaea
aaaaaaeaaaaaaaaaaaaaaaaaaceaaaaaaaaaialpaaaaialpaaaaaaaaaaaaaaaa
dcaaaaakicaabaaaaaaaaaaaakaabaiaebaaaaaaaaaaaaaaakaabaaaaaaaaaaa
abeaaaaaaaaaiadpdcaaaaakicaabaaaaaaaaaaabkaabaiaebaaaaaaaaaaaaaa
bkaabaaaaaaaaaaadkaabaaaaaaaaaaaelaaaaafecaabaaaaaaaaaaadkaabaaa
aaaaaaaabaaaaaahbcaabaaaabaaaaaaegbcbaaaadaaaaaaegacbaaaaaaaaaaa
baaaaaahccaabaaaabaaaaaaegbcbaaaaeaaaaaaegacbaaaaaaaaaaabaaaaaah
ecaabaaaabaaaaaaegbcbaaaafaaaaaaegacbaaaaaaaaaaadcaaaaaphccabaaa
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
ConstBuffer "$Globals" 96 // 76 used size, 8 vars
Float 64 [_Shininess]
Float 72 [_Parallax]
BindCB "$Globals" 0
SetTexture 0 [_ParallaxMap] 2D 1
SetTexture 1 [_BumpMap] 2D 0
// 20 instructions, 2 temp regs, 0 temp arrays:
// ALU 9 float, 0 int, 0 uint
// TEX 2 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0_level_9_3
eefiecedcpfmabdbbaoljempmjphcjcdbikkgmhnabaaaaaahiagaaaaaeaaaaaa
daaaaaaafeacaaaaimafaaaaeeagaaaaebgpgodjbmacaaaabmacaaaaaaacpppp
oeabaaaadiaaaaaaabaacmaaaaaadiaaaaaadiaaacaaceaaaaaadiaaabaaaaaa
aaababaaaaaaaeaaabaaaaaaaaaaaaaaabacppppfbaaaaafabaaapkaaaaaaadp
dnaknhdoaaaaaaeaaaaaialpbpaaaaacaaaaaaiaaaaaadlabpaaaaacaaaaaaia
abaaahlabpaaaaacaaaaaaiaacaaahlabpaaaaacaaaaaaiaadaaahlabpaaaaac
aaaaaaiaaeaaahlabpaaaaacaaaaaajaaaaiapkabpaaaaacaaaaaajaabaiapka
aiaaaaadaaaaciiaabaaoelaabaaoelaahaaaaacaaaacbiaaaaappiaaeaaaaae
aaaaaciaabaakklaaaaaaaiaabaaffkaafaaaaadaaaaafiaaaaaaaiaabaanela
agaaaaacaaaaaciaaaaaffiaafaaaaadaaaaadiaaaaaffiaaaaaoiiaecaaaaad
abaacpiaaaaaoelaabaioekaabaaaaacabaaabiaabaaaakaafaaaaadaaaaceia
abaaaaiaaaaakkkaaeaaaaaeaaaaceiaabaappiaaaaakkkaaaaakkibaeaaaaae
aaaaadiaaaaakkiaaaaaoeiaaaaaoelaecaaaaadaaaacpiaaaaaoeiaaaaioeka
aeaaaaaeaaaacdiaaaaaohiaabaakkkaabaappkaaeaaaaaeaaaaciiaaaaaaaia
aaaaaaibabaappkbaeaaaaaeaaaaciiaaaaaffiaaaaaffibaaaappiaahaaaaac
aaaaciiaaaaappiaagaaaaacaaaaceiaaaaappiaaiaaaaadabaacbiaacaaoela
aaaaoeiaaiaaaaadabaacciaadaaoelaaaaaoeiaaiaaaaadabaaceiaaeaaoela
aaaaoeiaaeaaaaaeaaaachiaabaaoeiaabaaaakaabaaaakaabaaaaacaaaaciia
aaaaaakaabaaaaacaaaicpiaaaaaoeiappppaaaafdeieefcdaadaaaaeaaaaaaa
mmaaaaaafjaaaaaeegiocaaaaaaaaaaaafaaaaaafkaaaaadaagabaaaaaaaaaaa
fkaaaaadaagabaaaabaaaaaafibiaaaeaahabaaaaaaaaaaaffffaaaafibiaaae
aahabaaaabaaaaaaffffaaaagcbaaaaddcbabaaaabaaaaaagcbaaaadhcbabaaa
acaaaaaagcbaaaadhcbabaaaadaaaaaagcbaaaadhcbabaaaaeaaaaaagcbaaaad
hcbabaaaafaaaaaagfaaaaadpccabaaaaaaaaaaagiaaaaacacaaaaaabaaaaaah
bcaabaaaaaaaaaaaegbcbaaaacaaaaaaegbcbaaaacaaaaaaeeaaaaafbcaabaaa
aaaaaaaaakaabaaaaaaaaaaadiaaaaahgcaabaaaaaaaaaaaagaabaaaaaaaaaaa
agbbbaaaacaaaaaadcaaaaajbcaabaaaaaaaaaaackbabaaaacaaaaaaakaabaaa
aaaaaaaaabeaaaaadnaknhdoaoaaaaahdcaabaaaaaaaaaaajgafbaaaaaaaaaaa
agaabaaaaaaaaaaaefaaaaajpcaabaaaabaaaaaaegbabaaaabaaaaaaeghobaaa
aaaaaaaaaagabaaaabaaaaaadiaaaaaiecaabaaaaaaaaaaackiacaaaaaaaaaaa
aeaaaaaaabeaaaaaaaaaaadpdcaaaaalecaabaaaaaaaaaaadkaabaaaabaaaaaa
ckiacaaaaaaaaaaaaeaaaaaackaabaiaebaaaaaaaaaaaaaadcaaaaajdcaabaaa
aaaaaaaakgakbaaaaaaaaaaaegaabaaaaaaaaaaaegbabaaaabaaaaaaefaaaaaj
pcaabaaaaaaaaaaaegaabaaaaaaaaaaaeghobaaaabaaaaaaaagabaaaaaaaaaaa
dcaaaaapdcaabaaaaaaaaaaahgapbaaaaaaaaaaaaceaaaaaaaaaaaeaaaaaaaea
aaaaaaaaaaaaaaaaaceaaaaaaaaaialpaaaaialpaaaaaaaaaaaaaaaadcaaaaak
icaabaaaaaaaaaaaakaabaiaebaaaaaaaaaaaaaaakaabaaaaaaaaaaaabeaaaaa
aaaaiadpdcaaaaakicaabaaaaaaaaaaabkaabaiaebaaaaaaaaaaaaaabkaabaaa
aaaaaaaadkaabaaaaaaaaaaaelaaaaafecaabaaaaaaaaaaadkaabaaaaaaaaaaa
baaaaaahbcaabaaaabaaaaaaegbcbaaaadaaaaaaegacbaaaaaaaaaaabaaaaaah
ccaabaaaabaaaaaaegbcbaaaaeaaaaaaegacbaaaaaaaaaaabaaaaaahecaabaaa
abaaaaaaegbcbaaaafaaaaaaegacbaaaaaaaaaaadcaaaaaphccabaaaaaaaaaaa
egacbaaaabaaaaaaaceaaaaaaaaaaadpaaaaaadpaaaaaadpaaaaaaaaaceaaaaa
aaaaaadpaaaaaadpaaaaaadpaaaaaaaadgaaaaagiccabaaaaaaaaaaaakiacaaa
aaaaaaaaaeaaaaaadoaaaaabejfdeheolaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaakeaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaadadaaaakeaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaakeaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaahahaaaakeaaaaaa
adaaaaaaaaaaaaaaadaaaaaaaeaaaaaaahahaaaakeaaaaaaaeaaaaaaaaaaaaaa
adaaaaaaafaaaaaaahahaaaafdfgfpfaepfdejfeejepeoaafeeffiedepepfcee
aaklklklepfdeheocmaaaaaaabaaaaaaaiaaaaaacaaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaaaaaaaaaapaaaaaafdfgfpfegbhcghgfheaaklkl"
}

}
	}
	Pass {
		Name "PREPASS"
		Tags { "LightMode" = "PrePassFinal" }
		ZWrite Off
Program "vp" {
// Vertex combos: 4
//   opengl - ALU: 34 to 42
//   d3d9 - ALU: 35 to 43
//   d3d11 - ALU: 17 to 22, TEX: 0 to 0, FLOW: 1 to 1
//   d3d11_9x - ALU: 17 to 22, TEX: 0 to 0, FLOW: 1 to 1
SubProgram "opengl " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
Bind "vertex" Vertex
Bind "tangent" ATTR14
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 13 [_WorldSpaceCameraPos]
Vector 14 [_ProjectionParams]
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
Vector 24 [_BumpMap_ST]
"!!ARBvp1.0
# 42 ALU
PARAM c[25] = { { 1, 0.5 },
		state.matrix.mvp,
		program.local[5..24] };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
MUL R1.xyz, vertex.normal, c[22].w;
DP3 R2.w, R1, c[6];
DP3 R0.x, R1, c[5];
DP3 R0.z, R1, c[7];
MOV R0.y, R2.w;
MUL R1, R0.xyzz, R0.yzzx;
MOV R0.w, c[0].x;
DP4 R2.z, R0, c[17];
DP4 R2.y, R0, c[16];
DP4 R2.x, R0, c[15];
MUL R0.y, R2.w, R2.w;
DP4 R3.z, R1, c[20];
DP4 R3.x, R1, c[18];
DP4 R3.y, R1, c[19];
ADD R2.xyz, R2, R3;
MAD R0.w, R0.x, R0.x, -R0.y;
MUL R3.xyz, R0.w, c[21];
MOV R1.xyz, vertex.attrib[14];
MUL R0.xyz, vertex.normal.zxyw, R1.yzxw;
MAD R0.xyz, vertex.normal.yzxw, R1.zxyw, -R0;
MUL R0.xyz, R0, vertex.attrib[14].w;
MOV R1.xyz, c[13];
ADD result.texcoord[3].xyz, R2, R3;
MOV R1.w, c[0].x;
DP4 R0.w, vertex.position, c[4];
DP4 R2.z, R1, c[11];
DP4 R2.x, R1, c[9];
DP4 R2.y, R1, c[10];
MAD R2.xyz, R2, c[22].w, -vertex.position;
DP3 result.texcoord[1].y, R2, R0;
DP4 R0.z, vertex.position, c[3];
DP4 R0.x, vertex.position, c[1];
DP4 R0.y, vertex.position, c[2];
MUL R1.xyz, R0.xyww, c[0].y;
MUL R1.y, R1, c[14].x;
DP3 result.texcoord[1].z, vertex.normal, R2;
DP3 result.texcoord[1].x, R2, vertex.attrib[14];
ADD result.texcoord[2].xy, R1, R1.z;
MOV result.position, R0;
MOV result.texcoord[2].zw, R0;
MAD result.texcoord[0].zw, vertex.texcoord[0].xyxy, c[24].xyxy, c[24];
MAD result.texcoord[0].xy, vertex.texcoord[0], c[23], c[23].zwzw;
END
# 42 instructions, 4 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Matrix 0 [glstate_matrix_mvp]
Vector 12 [_WorldSpaceCameraPos]
Vector 13 [_ProjectionParams]
Vector 14 [_ScreenParams]
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
Vector 23 [_MainTex_ST]
Vector 24 [_BumpMap_ST]
"vs_2_0
; 43 ALU
def c25, 1.00000000, 0.50000000, 0, 0
dcl_position0 v0
dcl_tangent0 v1
dcl_normal0 v2
dcl_texcoord0 v3
mul r1.xyz, v2, c22.w
dp3 r2.w, r1, c5
dp3 r0.x, r1, c4
dp3 r0.z, r1, c6
mov r0.y, r2.w
mul r1, r0.xyzz, r0.yzzx
mov r0.w, c25.x
dp4 r2.z, r0, c17
dp4 r2.y, r0, c16
dp4 r2.x, r0, c15
mul r0.y, r2.w, r2.w
mad r0.w, r0.x, r0.x, -r0.y
dp4 r3.z, r1, c20
dp4 r3.y, r1, c19
dp4 r3.x, r1, c18
add r2.xyz, r2, r3
mul r3.xyz, r0.w, c21
mov r0.xyz, v1
mul r1.xyz, v2.zxyw, r0.yzxw
mov r0.xyz, v1
mad r0.xyz, v2.yzxw, r0.zxyw, -r1
mul r0.xyz, r0, v1.w
mov r1.xyz, c12
add oT3.xyz, r2, r3
mov r1.w, c25.x
dp4 r0.w, v0, c3
dp4 r2.z, r1, c10
dp4 r2.x, r1, c8
dp4 r2.y, r1, c9
mad r2.xyz, r2, c22.w, -v0
dp3 oT1.y, r2, r0
dp4 r0.z, v0, c2
dp4 r0.x, v0, c0
dp4 r0.y, v0, c1
mul r1.xyz, r0.xyww, c25.y
mul r1.y, r1, c13.x
dp3 oT1.z, v2, r2
dp3 oT1.x, r2, v1
mad oT2.xy, r1.z, c14.zwzw, r1
mov oPos, r0
mov oT2.zw, r0
mad oT0.zw, v3.xyxy, c24.xyxy, c24
mad oT0.xy, v3, c23, c23.zwzw
"
}

SubProgram "xbox360 " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 23 [_BumpMap_ST]
Vector 22 [_MainTex_ST]
Matrix 14 [_Object2World] 3
Vector 1 [_ProjectionParams]
Vector 2 [_ScreenParams]
Matrix 17 [_World2Object] 4
Vector 0 [_WorldSpaceCameraPos]
Matrix 10 [glstate_matrix_mvp] 4
Vector 5 [unity_SHAb]
Vector 4 [unity_SHAg]
Vector 3 [unity_SHAr]
Vector 8 [unity_SHBb]
Vector 7 [unity_SHBg]
Vector 6 [unity_SHBr]
Vector 9 [unity_SHC]
Vector 21 [unity_Scale]
// Shader Timing Estimate, in Cycles/64 vertex vector:
// ALU: 44.00 (33 instructions), vertex: 32, texture: 0,
//   sequencer: 18,  9 GPRs, 21 threads,
// Performance (if enough threads): ~44 cycles per vector
// * Vertex cycle estimates are assuming 3 vfetch_minis for every vfetch_full,
//     with <= 32 bytes per vfetch_full group.

"vs_360
backbbabaaaaaddaaaaaaceeaaaaaaaaaaaaaaceaaaaackaaaaaacmiaaaaaaaa
aaaaaaaaaaaaachiaaaaaabmaaaaacgkpppoadaaaaaaaabaaaaaaabmaaaaaaaa
aaaaacgdaaaaabfmaaacaabhaaabaaaaaaaaabgiaaaaaaaaaaaaabhiaaacaabg
aaabaaaaaaaaabgiaaaaaaaaaaaaabieaaacaaaoaaadaaaaaaaaabjeaaaaaaaa
aaaaabkeaaacaaabaaabaaaaaaaaabgiaaaaaaaaaaaaablgaaacaaacaaabaaaa
aaaaabgiaaaaaaaaaaaaabmeaaacaabbaaaeaaaaaaaaabjeaaaaaaaaaaaaabnc
aaacaaaaaaabaaaaaaaaaboiaaaaaaaaaaaaabpiaaacaaakaaaeaaaaaaaaabje
aaaaaaaaaaaaacalaaacaaafaaabaaaaaaaaabgiaaaaaaaaaaaaacbgaaacaaae
aaabaaaaaaaaabgiaaaaaaaaaaaaaccbaaacaaadaaabaaaaaaaaabgiaaaaaaaa
aaaaaccmaaacaaaiaaabaaaaaaaaabgiaaaaaaaaaaaaacdhaaacaaahaaabaaaa
aaaaabgiaaaaaaaaaaaaacecaaacaaagaaabaaaaaaaaabgiaaaaaaaaaaaaacen
aaacaaajaaabaaaaaaaaabgiaaaaaaaaaaaaacfhaaacaabfaaabaaaaaaaaabgi
aaaaaaaafpechfgnhaengbhafpfdfeaaaaabaaadaaabaaaeaaabaaaaaaaaaaaa
fpengbgjgofegfhifpfdfeaafpepgcgkgfgdhedcfhgphcgmgeaaklklaaadaaad
aaaeaaaeaaabaaaaaaaaaaaafpfahcgpgkgfgdhegjgpgofagbhcgbgnhdaafpfd
gdhcgfgfgofagbhcgbgnhdaafpfhgphcgmgedcepgcgkgfgdheaafpfhgphcgmge
fdhagbgdgfedgbgngfhcgbfagphdaaklaaabaaadaaabaaadaaabaaaaaaaaaaaa
ghgmhdhegbhegffpgngbhehcgjhifpgnhghaaahfgogjhehjfpfdeiebgcaahfgo
gjhehjfpfdeiebghaahfgogjhehjfpfdeiebhcaahfgogjhehjfpfdeiecgcaahf
gogjhehjfpfdeiecghaahfgogjhehjfpfdeiechcaahfgogjhehjfpfdeiedaahf
gogjhehjfpfdgdgbgmgfaahghdfpddfpdaaadccodacodcdadddfddcodaaaklkl
aaaaaaaaaaaaaaabaaaaaaaaaaaaaaaaaaaaaabeaapmaabaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaeaaaaaacaeaadbaaaiaaaaaaaaaaaaaaaaaaaadiie
aaaaaaabaaaaaaaeaaaaaaaiaaaaacjaaabaaaafaaaagaagaaaadaahaadafaai
aaaapafaaaachbfbaaafpcfcaaahhdfdaaaaaaboaaaababpaaaaaablaaaaaabm
aaaababnaaaaaabkaaaabachaaaabacjaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
dpaaaaaaaaaaaaaaaaaaaaaaaaaaaaaapaffeaafaaaabcaamcaaaaaaaaaafaaj
aaaabcaameaaaaaaaaaagaaogabebcaabcaaaaaaaaaagabkgacabcaabcaaaaaa
aaaaeacgaaaaccaaaaaaaaaaafpicaaaaaaaagiiaaaaaaaaafpigaaaaaaaagii
aaaaaaaaafpibaaaaaaaaeehaaaaaaaaafpiaaaaaaaaapmiaaaaaaaamiapaaad
aabliiaakbacanaamiapaaadaamgnapiklacamadmiapaaadaalbdepiklacalad
miapaaahaagmnajeklacakadmiapiadoaananaaaocahahaamiahaaadaamamgma
albdaabemiahaaafaamdgfaaobabagaamiahaaaiaalelbleclbcaaadmialaaad
aalkblaakbabbfaamiahaaaeaalbleaakbadbaaamiahaaaiaamagmleclbbaaai
miahaaafablklomaolabagafceihaeafaamablgmobafagiamiahaaacabmablma
klaibfacmiahaaadaagmlemakladapaemiahaaaeaabllemakladaoadaibhabad
aamagmggkbahppaemiamiaacaanlnlaaocahahaamiabiaabaaloloaapaacagaa
miaciaabaaloloaapaafacaamiaeiaabaalomdaapaacabaamiadiaaaaalalabk
ilaabgbgmiamiaaaaakmkmagilaabhbhaicbabacaadoanmbgpadaeaeaiecabac
aadoanlbgpaeaeaeaiieabacaadoanlmgpafaeaemiabaaaaaakhkhaakpabagaa
miacaaaaaakhkhaakpabahaaaibeabaaaakhkhgmkpabaiaeaiciabadaalbgmmg
kbadabaemiadiaacaamgbkbikladacadgeihaaaaaalologboaacaaabmiahiaad
aablmagfklaaajaaaaaaaaaaaaaaaaaaaaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
Matrix 256 [glstate_matrix_mvp]
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 467 [_WorldSpaceCameraPos]
Vector 466 [_ProjectionParams]
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
Vector 456 [_BumpMap_ST]
"sce_vp_rsx // 40 instructions using 5 registers
[Configuration]
8
0000002841050500
[Defaults]
1
455 1
3f000000
[Microcode]
640
00011c6c00400e0c0106c0836041dffc00019c6c005d300c0186c0836041dffc
00001c6c009ca20c013fc0c36041dffc401f9c6c011c8800810040d560607f9c
401f9c6c011c9808010400d740619f9c00009c6c01d0300d8106c0c360403ffc
00009c6c01d0200d8106c0c360405ffc00009c6c01d0100d8106c0c360409ffc
00009c6c01d0000d8106c0c360411ffc00021c6c0150400c008600c360411ffc
00021c6c0150600c008600c360405ffc00001c6c0150500c008600c360403ffc
00001c6c0190a00c0686c0c360405ffc00001c6c0190900c0686c0c360409ffc
00001c6c0190800c0686c0c360411ffc00019c6c00800243011842436041dffc
00011c6c010002308121826301a1dffc401f9c6c0040000d8286c0836041ff80
401f9c6c004000558286c08360407fa400001c6c011ca00c00bfc0e30041dffc
00009c6c009c700e028000c36041dffc00009c6c009d202a828000c360409ffc
00009c6c0080007f80bfc04360403ffc00021c6c0040007f8086c08360409ffc
401f9c6c00c000080286c09540a19fa400011c6c00800e0c04bfc0836041dffc
401f9c6c0140020c0106004360405fa0401f9c6c01400e0c0086008360411fa0
00009c6c019cf00c0886c0c360405ffc00009c6c019d000c0886c0c360409ffc
00009c6c019d100c0886c0c360411ffc00001c6c010000000880047fe0a03ffc
00019c6c0080000d089a04436041fffc401f9c6c0140000c0086024360409fa0
00001c6c01dcc00d8686c0c360405ffc00001c6c01dcd00d8686c0c360409ffc
00001c6c01dce00d8686c0c360411ffc00001c6c00c0000c0286c0830021dffc
00009c6c009cb07f808600c36041dffc401f9c6c00c0000c0286c0830021dfa9
"
}

SubProgram "d3d11 " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 128 // 112 used size, 10 vars
Vector 80 [_MainTex_ST] 4
Vector 96 [_BumpMap_ST] 4
ConstBuffer "UnityPerCamera" 128 // 96 used size, 8 vars
Vector 64 [_WorldSpaceCameraPos] 3
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
Matrix 256 [_World2Object] 4
Vector 320 [unity_Scale] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityLighting" 2
BindCB "UnityPerDraw" 3
// 39 instructions, 4 temp regs, 0 temp arrays:
// ALU 22 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0
eefiecedjglbapihppffmfjgdankmlkbcgeampgpabaaaaaagaahaaaaadaaaaaa
cmaaaaaapeaaaaaajeabaaaaejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaa
abaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaljaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfc
enebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheojiaaaaaaafaaaaaa
aiaaaaaaiaaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaabaaaaaaapaaaaaaimaaaaaaabaaaaaaaaaaaaaa
adaaaaaaacaaaaaaahaiaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaa
apaaaaaaimaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaahaiaaaafdfgfpfa
epfdejfeejepeoaafeeffiedepepfceeaaklklklfdeieefcmeafaaaaeaaaabaa
hbabaaaafjaaaaaeegiocaaaaaaaaaaaahaaaaaafjaaaaaeegiocaaaabaaaaaa
agaaaaaafjaaaaaeegiocaaaacaaaaaabjaaaaaafjaaaaaeegiocaaaadaaaaaa
bfaaaaaafpaaaaadpcbabaaaaaaaaaaafpaaaaadpcbabaaaabaaaaaafpaaaaad
hcbabaaaacaaaaaafpaaaaaddcbabaaaadaaaaaaghaaaaaepccabaaaaaaaaaaa
abaaaaaagfaaaaadpccabaaaabaaaaaagfaaaaadhccabaaaacaaaaaagfaaaaad
pccabaaaadaaaaaagfaaaaadhccabaaaaeaaaaaagiaaaaacaeaaaaaadiaaaaai
pcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaadaaaaaaabaaaaaadcaaaaak
pcaabaaaaaaaaaaaegiocaaaadaaaaaaaaaaaaaaagbabaaaaaaaaaaaegaobaaa
aaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaacaaaaaakgbkbaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaa
adaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadgaaaaafpccabaaaaaaaaaaa
egaobaaaaaaaaaaadcaaaaaldccabaaaabaaaaaaegbabaaaadaaaaaaegiacaaa
aaaaaaaaafaaaaaaogikcaaaaaaaaaaaafaaaaaadcaaaaalmccabaaaabaaaaaa
agbebaaaadaaaaaaagiecaaaaaaaaaaaagaaaaaakgiocaaaaaaaaaaaagaaaaaa
diaaaaahhcaabaaaabaaaaaajgbebaaaabaaaaaacgbjbaaaacaaaaaadcaaaaak
hcaabaaaabaaaaaajgbebaaaacaaaaaacgbjbaaaabaaaaaaegacbaiaebaaaaaa
abaaaaaadiaaaaahhcaabaaaabaaaaaaegacbaaaabaaaaaapgbpbaaaabaaaaaa
diaaaaajhcaabaaaacaaaaaafgifcaaaabaaaaaaaeaaaaaaegiccaaaadaaaaaa
bbaaaaaadcaaaaalhcaabaaaacaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaa
abaaaaaaaeaaaaaaegacbaaaacaaaaaadcaaaaalhcaabaaaacaaaaaaegiccaaa
adaaaaaabcaaaaaakgikcaaaabaaaaaaaeaaaaaaegacbaaaacaaaaaaaaaaaaai
hcaabaaaacaaaaaaegacbaaaacaaaaaaegiccaaaadaaaaaabdaaaaaadcaaaaal
hcaabaaaacaaaaaaegacbaaaacaaaaaapgipcaaaadaaaaaabeaaaaaaegbcbaia
ebaaaaaaaaaaaaaabaaaaaahcccabaaaacaaaaaaegacbaaaabaaaaaaegacbaaa
acaaaaaabaaaaaahbccabaaaacaaaaaaegbcbaaaabaaaaaaegacbaaaacaaaaaa
baaaaaaheccabaaaacaaaaaaegbcbaaaacaaaaaaegacbaaaacaaaaaadiaaaaai
ccaabaaaaaaaaaaabkaabaaaaaaaaaaaakiacaaaabaaaaaaafaaaaaadiaaaaak
ncaabaaaabaaaaaaagahbaaaaaaaaaaaaceaaaaaaaaaaadpaaaaaaaaaaaaaadp
aaaaaadpdgaaaaafmccabaaaadaaaaaakgaobaaaaaaaaaaaaaaaaaahdccabaaa
adaaaaaakgakbaaaabaaaaaamgaabaaaabaaaaaadiaaaaaihcaabaaaaaaaaaaa
egbcbaaaacaaaaaapgipcaaaadaaaaaabeaaaaaadiaaaaaihcaabaaaabaaaaaa
fgafbaaaaaaaaaaaegiccaaaadaaaaaaanaaaaaadcaaaaaklcaabaaaaaaaaaaa
egiicaaaadaaaaaaamaaaaaaagaabaaaaaaaaaaaegaibaaaabaaaaaadcaaaaak
hcaabaaaaaaaaaaaegiccaaaadaaaaaaaoaaaaaakgakbaaaaaaaaaaaegadbaaa
aaaaaaaadgaaaaaficaabaaaaaaaaaaaabeaaaaaaaaaiadpbbaaaaaibcaabaaa
abaaaaaaegiocaaaacaaaaaabcaaaaaaegaobaaaaaaaaaaabbaaaaaiccaabaaa
abaaaaaaegiocaaaacaaaaaabdaaaaaaegaobaaaaaaaaaaabbaaaaaiecaabaaa
abaaaaaaegiocaaaacaaaaaabeaaaaaaegaobaaaaaaaaaaadiaaaaahpcaabaaa
acaaaaaajgacbaaaaaaaaaaaegakbaaaaaaaaaaabbaaaaaibcaabaaaadaaaaaa
egiocaaaacaaaaaabfaaaaaaegaobaaaacaaaaaabbaaaaaiccaabaaaadaaaaaa
egiocaaaacaaaaaabgaaaaaaegaobaaaacaaaaaabbaaaaaiecaabaaaadaaaaaa
egiocaaaacaaaaaabhaaaaaaegaobaaaacaaaaaaaaaaaaahhcaabaaaabaaaaaa
egacbaaaabaaaaaaegacbaaaadaaaaaadiaaaaahccaabaaaaaaaaaaabkaabaaa
aaaaaaaabkaabaaaaaaaaaaadcaaaaakbcaabaaaaaaaaaaaakaabaaaaaaaaaaa
akaabaaaaaaaaaaabkaabaiaebaaaaaaaaaaaaaadcaaaaakhccabaaaaeaaaaaa
egiccaaaacaaaaaabiaaaaaaagaabaaaaaaaaaaaegacbaaaabaaaaaadoaaaaab
"
}

SubProgram "gles " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
"!!GLES
#define SHADER_API_GLES 1
#define tex2D texture2D


#ifdef VERTEX
#define gl_ModelViewProjectionMatrix glstate_matrix_mvp
uniform mat4 glstate_matrix_mvp;

varying highp vec3 xlv_TEXCOORD3;
varying highp vec4 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 _BumpMap_ST;
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
  highp vec4 tmpvar_3;
  highp vec3 tmpvar_4;
  highp vec4 tmpvar_5;
  tmpvar_5 = (gl_ModelViewProjectionMatrix * _glesVertex);
  tmpvar_3.xy = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  tmpvar_3.zw = ((_glesMultiTexCoord0.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
  highp vec4 o_6;
  highp vec4 tmpvar_7;
  tmpvar_7 = (tmpvar_5 * 0.5);
  highp vec2 tmpvar_8;
  tmpvar_8.x = tmpvar_7.x;
  tmpvar_8.y = (tmpvar_7.y * _ProjectionParams.x);
  o_6.xy = (tmpvar_8 + tmpvar_7.w);
  o_6.zw = tmpvar_5.zw;
  mat3 tmpvar_9;
  tmpvar_9[0] = _Object2World[0].xyz;
  tmpvar_9[1] = _Object2World[1].xyz;
  tmpvar_9[2] = _Object2World[2].xyz;
  highp vec4 tmpvar_10;
  tmpvar_10.w = 1.0;
  tmpvar_10.xyz = (tmpvar_9 * (tmpvar_2 * unity_Scale.w));
  mediump vec3 tmpvar_11;
  mediump vec4 normal_12;
  normal_12 = tmpvar_10;
  highp float vC_13;
  mediump vec3 x3_14;
  mediump vec3 x2_15;
  mediump vec3 x1_16;
  highp float tmpvar_17;
  tmpvar_17 = dot (unity_SHAr, normal_12);
  x1_16.x = tmpvar_17;
  highp float tmpvar_18;
  tmpvar_18 = dot (unity_SHAg, normal_12);
  x1_16.y = tmpvar_18;
  highp float tmpvar_19;
  tmpvar_19 = dot (unity_SHAb, normal_12);
  x1_16.z = tmpvar_19;
  mediump vec4 tmpvar_20;
  tmpvar_20 = (normal_12.xyzz * normal_12.yzzx);
  highp float tmpvar_21;
  tmpvar_21 = dot (unity_SHBr, tmpvar_20);
  x2_15.x = tmpvar_21;
  highp float tmpvar_22;
  tmpvar_22 = dot (unity_SHBg, tmpvar_20);
  x2_15.y = tmpvar_22;
  highp float tmpvar_23;
  tmpvar_23 = dot (unity_SHBb, tmpvar_20);
  x2_15.z = tmpvar_23;
  mediump float tmpvar_24;
  tmpvar_24 = ((normal_12.x * normal_12.x) - (normal_12.y * normal_12.y));
  vC_13 = tmpvar_24;
  highp vec3 tmpvar_25;
  tmpvar_25 = (unity_SHC.xyz * vC_13);
  x3_14 = tmpvar_25;
  tmpvar_11 = ((x1_16 + x2_15) + x3_14);
  tmpvar_4 = tmpvar_11;
  highp vec3 tmpvar_26;
  highp vec3 tmpvar_27;
  tmpvar_26 = tmpvar_1.xyz;
  tmpvar_27 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_28;
  tmpvar_28[0].x = tmpvar_26.x;
  tmpvar_28[0].y = tmpvar_27.x;
  tmpvar_28[0].z = tmpvar_2.x;
  tmpvar_28[1].x = tmpvar_26.y;
  tmpvar_28[1].y = tmpvar_27.y;
  tmpvar_28[1].z = tmpvar_2.y;
  tmpvar_28[2].x = tmpvar_26.z;
  tmpvar_28[2].y = tmpvar_27.z;
  tmpvar_28[2].z = tmpvar_2.z;
  highp vec4 tmpvar_29;
  tmpvar_29.w = 1.0;
  tmpvar_29.xyz = _WorldSpaceCameraPos;
  gl_Position = tmpvar_5;
  xlv_TEXCOORD0 = tmpvar_3;
  xlv_TEXCOORD1 = (tmpvar_28 * (((_World2Object * tmpvar_29).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = o_6;
  xlv_TEXCOORD3 = tmpvar_4;
}



#endif
#ifdef FRAGMENT

varying highp vec3 xlv_TEXCOORD3;
varying highp vec4 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform sampler2D _LightBuffer;
uniform highp float _Parallax;
uniform mediump float _Gloss;
uniform lowp vec4 _Color;
uniform sampler2D _ParallaxMap;
uniform sampler2D _SpecMap;
uniform sampler2D _MainTex;
uniform lowp vec4 _SpecColor;
void main ()
{
  lowp vec4 tmpvar_1;
  mediump vec4 light_2;
  mediump vec3 tmpvar_3;
  mediump vec4 spec_4;
  mediump float h_5;
  lowp float tmpvar_6;
  tmpvar_6 = texture2D (_ParallaxMap, xlv_TEXCOORD0.zw).w;
  h_5 = tmpvar_6;
  mediump float height_7;
  height_7 = _Parallax;
  mediump vec3 viewDir_8;
  viewDir_8 = xlv_TEXCOORD1;
  highp vec3 v_9;
  mediump float tmpvar_10;
  tmpvar_10 = ((h_5 * height_7) - (height_7 / 2.0));
  mediump vec3 tmpvar_11;
  tmpvar_11 = normalize(viewDir_8);
  v_9 = tmpvar_11;
  v_9.z = (v_9.z + 0.42);
  highp vec2 tmpvar_12;
  tmpvar_12 = (xlv_TEXCOORD0.xy + (tmpvar_10 * (v_9.xy / v_9.z)));
  lowp vec4 tmpvar_13;
  tmpvar_13 = texture2D (_MainTex, tmpvar_12);
  lowp vec3 tmpvar_14;
  tmpvar_14 = (tmpvar_13.xyz * _Color.xyz);
  tmpvar_3 = tmpvar_14;
  lowp vec4 tmpvar_15;
  tmpvar_15 = texture2D (_SpecMap, tmpvar_12);
  spec_4 = tmpvar_15;
  lowp vec4 tmpvar_16;
  tmpvar_16 = texture2DProj (_LightBuffer, xlv_TEXCOORD2);
  light_2 = tmpvar_16;
  mediump vec4 tmpvar_17;
  tmpvar_17 = -(log2(max (light_2, vec4(0.001, 0.001, 0.001, 0.001))));
  light_2.w = tmpvar_17.w;
  highp vec3 tmpvar_18;
  tmpvar_18 = (tmpvar_17.xyz + xlv_TEXCOORD3);
  light_2.xyz = tmpvar_18;
  mediump vec4 c_19;
  mediump vec3 tmpvar_20;
  tmpvar_20 = (tmpvar_17.w * ((tmpvar_13.w * spec_4.xyz) * _Gloss));
  c_19.xyz = ((tmpvar_3 * light_2.xyz) + (light_2.xyz * tmpvar_20));
  c_19.w = (tmpvar_20 * _SpecColor.w).x;
  tmpvar_1 = c_19;
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

varying highp vec3 xlv_TEXCOORD3;
varying highp vec4 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 _BumpMap_ST;
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
  highp vec4 tmpvar_3;
  highp vec3 tmpvar_4;
  highp vec4 tmpvar_5;
  tmpvar_5 = (gl_ModelViewProjectionMatrix * _glesVertex);
  tmpvar_3.xy = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  tmpvar_3.zw = ((_glesMultiTexCoord0.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
  highp vec4 o_6;
  highp vec4 tmpvar_7;
  tmpvar_7 = (tmpvar_5 * 0.5);
  highp vec2 tmpvar_8;
  tmpvar_8.x = tmpvar_7.x;
  tmpvar_8.y = (tmpvar_7.y * _ProjectionParams.x);
  o_6.xy = (tmpvar_8 + tmpvar_7.w);
  o_6.zw = tmpvar_5.zw;
  mat3 tmpvar_9;
  tmpvar_9[0] = _Object2World[0].xyz;
  tmpvar_9[1] = _Object2World[1].xyz;
  tmpvar_9[2] = _Object2World[2].xyz;
  highp vec4 tmpvar_10;
  tmpvar_10.w = 1.0;
  tmpvar_10.xyz = (tmpvar_9 * (tmpvar_2 * unity_Scale.w));
  mediump vec3 tmpvar_11;
  mediump vec4 normal_12;
  normal_12 = tmpvar_10;
  highp float vC_13;
  mediump vec3 x3_14;
  mediump vec3 x2_15;
  mediump vec3 x1_16;
  highp float tmpvar_17;
  tmpvar_17 = dot (unity_SHAr, normal_12);
  x1_16.x = tmpvar_17;
  highp float tmpvar_18;
  tmpvar_18 = dot (unity_SHAg, normal_12);
  x1_16.y = tmpvar_18;
  highp float tmpvar_19;
  tmpvar_19 = dot (unity_SHAb, normal_12);
  x1_16.z = tmpvar_19;
  mediump vec4 tmpvar_20;
  tmpvar_20 = (normal_12.xyzz * normal_12.yzzx);
  highp float tmpvar_21;
  tmpvar_21 = dot (unity_SHBr, tmpvar_20);
  x2_15.x = tmpvar_21;
  highp float tmpvar_22;
  tmpvar_22 = dot (unity_SHBg, tmpvar_20);
  x2_15.y = tmpvar_22;
  highp float tmpvar_23;
  tmpvar_23 = dot (unity_SHBb, tmpvar_20);
  x2_15.z = tmpvar_23;
  mediump float tmpvar_24;
  tmpvar_24 = ((normal_12.x * normal_12.x) - (normal_12.y * normal_12.y));
  vC_13 = tmpvar_24;
  highp vec3 tmpvar_25;
  tmpvar_25 = (unity_SHC.xyz * vC_13);
  x3_14 = tmpvar_25;
  tmpvar_11 = ((x1_16 + x2_15) + x3_14);
  tmpvar_4 = tmpvar_11;
  highp vec3 tmpvar_26;
  highp vec3 tmpvar_27;
  tmpvar_26 = tmpvar_1.xyz;
  tmpvar_27 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_28;
  tmpvar_28[0].x = tmpvar_26.x;
  tmpvar_28[0].y = tmpvar_27.x;
  tmpvar_28[0].z = tmpvar_2.x;
  tmpvar_28[1].x = tmpvar_26.y;
  tmpvar_28[1].y = tmpvar_27.y;
  tmpvar_28[1].z = tmpvar_2.y;
  tmpvar_28[2].x = tmpvar_26.z;
  tmpvar_28[2].y = tmpvar_27.z;
  tmpvar_28[2].z = tmpvar_2.z;
  highp vec4 tmpvar_29;
  tmpvar_29.w = 1.0;
  tmpvar_29.xyz = _WorldSpaceCameraPos;
  gl_Position = tmpvar_5;
  xlv_TEXCOORD0 = tmpvar_3;
  xlv_TEXCOORD1 = (tmpvar_28 * (((_World2Object * tmpvar_29).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = o_6;
  xlv_TEXCOORD3 = tmpvar_4;
}



#endif
#ifdef FRAGMENT

varying highp vec3 xlv_TEXCOORD3;
varying highp vec4 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform sampler2D _LightBuffer;
uniform highp float _Parallax;
uniform mediump float _Gloss;
uniform lowp vec4 _Color;
uniform sampler2D _ParallaxMap;
uniform sampler2D _SpecMap;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform lowp vec4 _SpecColor;
void main ()
{
  lowp vec4 tmpvar_1;
  mediump vec4 light_2;
  mediump vec3 tmpvar_3;
  mediump vec4 spec_4;
  mediump float h_5;
  lowp float tmpvar_6;
  tmpvar_6 = texture2D (_ParallaxMap, xlv_TEXCOORD0.zw).w;
  h_5 = tmpvar_6;
  highp vec2 tmpvar_7;
  mediump float height_8;
  height_8 = _Parallax;
  mediump vec3 viewDir_9;
  viewDir_9 = xlv_TEXCOORD1;
  highp vec3 v_10;
  mediump float tmpvar_11;
  tmpvar_11 = ((h_5 * height_8) - (height_8 / 2.0));
  mediump vec3 tmpvar_12;
  tmpvar_12 = normalize(viewDir_9);
  v_10 = tmpvar_12;
  v_10.z = (v_10.z + 0.42);
  tmpvar_7 = (tmpvar_11 * (v_10.xy / v_10.z));
  highp vec2 tmpvar_13;
  tmpvar_13 = (xlv_TEXCOORD0.xy + tmpvar_7);
  highp vec2 tmpvar_14;
  tmpvar_14 = (xlv_TEXCOORD0.zw + tmpvar_7);
  lowp vec4 tmpvar_15;
  tmpvar_15 = texture2D (_MainTex, tmpvar_13);
  lowp vec3 tmpvar_16;
  tmpvar_16 = (tmpvar_15.xyz * _Color.xyz);
  tmpvar_3 = tmpvar_16;
  lowp vec4 tmpvar_17;
  tmpvar_17 = texture2D (_SpecMap, tmpvar_13);
  spec_4 = tmpvar_17;
  lowp vec3 normal_18;
  normal_18.xy = ((texture2D (_BumpMap, tmpvar_14).wy * 2.0) - 1.0);
  normal_18.z = sqrt(((1.0 - (normal_18.x * normal_18.x)) - (normal_18.y * normal_18.y)));
  lowp vec4 tmpvar_19;
  tmpvar_19 = texture2DProj (_LightBuffer, xlv_TEXCOORD2);
  light_2 = tmpvar_19;
  mediump vec4 tmpvar_20;
  tmpvar_20 = -(log2(max (light_2, vec4(0.001, 0.001, 0.001, 0.001))));
  light_2.w = tmpvar_20.w;
  highp vec3 tmpvar_21;
  tmpvar_21 = (tmpvar_20.xyz + xlv_TEXCOORD3);
  light_2.xyz = tmpvar_21;
  mediump vec4 c_22;
  mediump vec3 tmpvar_23;
  tmpvar_23 = (tmpvar_20.w * ((tmpvar_15.w * spec_4.xyz) * _Gloss));
  c_22.xyz = ((tmpvar_3 * light_2.xyz) + (light_2.xyz * tmpvar_23));
  c_22.w = (tmpvar_23 * _SpecColor.w).x;
  tmpvar_1 = c_22;
  gl_FragData[0] = tmpvar_1;
}



#endif"
}

SubProgram "d3d11_9x " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 128 // 112 used size, 10 vars
Vector 80 [_MainTex_ST] 4
Vector 96 [_BumpMap_ST] 4
ConstBuffer "UnityPerCamera" 128 // 96 used size, 8 vars
Vector 64 [_WorldSpaceCameraPos] 3
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
Matrix 256 [_World2Object] 4
Vector 320 [unity_Scale] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityLighting" 2
BindCB "UnityPerDraw" 3
// 39 instructions, 4 temp regs, 0 temp arrays:
// ALU 22 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0_level_9_3
eefiecedejebmalhedjmpniplopmdohjpieonibkabaaaaaapaakaaaaaeaaaaaa
daaaaaaalmadaaaaiiajaaaafaakaaaaebgpgodjieadaaaaieadaaaaaaacpopp
beadaaaahaaaaaaaagaaceaaaaaagmaaaaaagmaaaaaaceaaabaagmaaaaaaafaa
acaaabaaaaaaaaaaabaaaeaaacaaadaaaaaaaaaaacaabcaaahaaafaaaaaaaaaa
adaaaaaaaeaaamaaaaaaaaaaadaaamaaadaabaaaaaaaaaaaadaabaaaafaabdaa
aaaaaaaaaaaaaaaaabacpoppfbaaaaafbiaaapkaaaaaaadpaaaaiadpaaaaaaaa
aaaaaaaabpaaaaacafaaaaiaaaaaapjabpaaaaacafaaabiaabaaapjabpaaaaac
afaaaciaacaaapjabpaaaaacafaaadiaadaaapjaaeaaaaaeaaaaadoaadaaoeja
abaaoekaabaaookaaeaaaaaeaaaaamoaadaaeejaacaaeekaacaaoekaafaaaaad
aaaaapiaaaaaffjaanaaoekaaeaaaaaeaaaaapiaamaaoekaaaaaaajaaaaaoeia
aeaaaaaeaaaaapiaaoaaoekaaaaakkjaaaaaoeiaaeaaaaaeaaaaapiaapaaoeka
aaaappjaaaaaoeiaafaaaaadabaaabiaaaaaffiaaeaaaakaafaaaaadabaaaiia
abaaaaiabiaaaakaafaaaaadabaaafiaaaaapeiabiaaaakaacaaaaadacaaadoa
abaakkiaabaaomiaafaaaaadabaaahiaacaaoejabhaappkaafaaaaadacaaahia
abaaffiabbaaoekaaeaaaaaeabaaaliabaaakekaabaaaaiaacaakeiaaeaaaaae
abaaahiabcaaoekaabaakkiaabaapeiaabaaaaacabaaaiiabiaaffkaajaaaaad
acaaabiaafaaoekaabaaoeiaajaaaaadacaaaciaagaaoekaabaaoeiaajaaaaad
acaaaeiaahaaoekaabaaoeiaafaaaaadadaaapiaabaacjiaabaakeiaajaaaaad
aeaaabiaaiaaoekaadaaoeiaajaaaaadaeaaaciaajaaoekaadaaoeiaajaaaaad
aeaaaeiaakaaoekaadaaoeiaacaaaaadacaaahiaacaaoeiaaeaaoeiaafaaaaad
abaaaciaabaaffiaabaaffiaaeaaaaaeabaaabiaabaaaaiaabaaaaiaabaaffib
aeaaaaaeadaaahoaalaaoekaabaaaaiaacaaoeiaabaaaaacabaaahiaadaaoeka
afaaaaadacaaahiaabaaffiabeaaoekaaeaaaaaeabaaaliabdaakekaabaaaaia
acaakeiaaeaaaaaeabaaahiabfaaoekaabaakkiaabaapeiaacaaaaadabaaahia
abaaoeiabgaaoekaaeaaaaaeabaaahiaabaaoeiabhaappkaaaaaoejbaiaaaaad
abaaaboaabaaoejaabaaoeiaabaaaaacacaaahiaacaaoejaafaaaaadadaaahia
acaanciaabaamjjaaeaaaaaeacaaahiaacaamjiaabaancjaadaaoeibafaaaaad
acaaahiaacaaoeiaabaappjaaiaaaaadabaaacoaacaaoeiaabaaoeiaaiaaaaad
abaaaeoaacaaoejaabaaoeiaaeaaaaaeaaaaadmaaaaappiaaaaaoekaaaaaoeia
abaaaaacaaaaammaaaaaoeiaabaaaaacacaaamoaaaaaoeiappppaaaafdeieefc
meafaaaaeaaaabaahbabaaaafjaaaaaeegiocaaaaaaaaaaaahaaaaaafjaaaaae
egiocaaaabaaaaaaagaaaaaafjaaaaaeegiocaaaacaaaaaabjaaaaaafjaaaaae
egiocaaaadaaaaaabfaaaaaafpaaaaadpcbabaaaaaaaaaaafpaaaaadpcbabaaa
abaaaaaafpaaaaadhcbabaaaacaaaaaafpaaaaaddcbabaaaadaaaaaaghaaaaae
pccabaaaaaaaaaaaabaaaaaagfaaaaadpccabaaaabaaaaaagfaaaaadhccabaaa
acaaaaaagfaaaaadpccabaaaadaaaaaagfaaaaadhccabaaaaeaaaaaagiaaaaac
aeaaaaaadiaaaaaipcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaadaaaaaa
abaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaaaaaaaaaagbabaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaa
acaaaaaakgbkbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaa
egiocaaaadaaaaaaadaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadgaaaaaf
pccabaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaaldccabaaaabaaaaaaegbabaaa
adaaaaaaegiacaaaaaaaaaaaafaaaaaaogikcaaaaaaaaaaaafaaaaaadcaaaaal
mccabaaaabaaaaaaagbebaaaadaaaaaaagiecaaaaaaaaaaaagaaaaaakgiocaaa
aaaaaaaaagaaaaaadiaaaaahhcaabaaaabaaaaaajgbebaaaabaaaaaacgbjbaaa
acaaaaaadcaaaaakhcaabaaaabaaaaaajgbebaaaacaaaaaacgbjbaaaabaaaaaa
egacbaiaebaaaaaaabaaaaaadiaaaaahhcaabaaaabaaaaaaegacbaaaabaaaaaa
pgbpbaaaabaaaaaadiaaaaajhcaabaaaacaaaaaafgifcaaaabaaaaaaaeaaaaaa
egiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaaacaaaaaaegiccaaaadaaaaaa
baaaaaaaagiacaaaabaaaaaaaeaaaaaaegacbaaaacaaaaaadcaaaaalhcaabaaa
acaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaaabaaaaaaaeaaaaaaegacbaaa
acaaaaaaaaaaaaaihcaabaaaacaaaaaaegacbaaaacaaaaaaegiccaaaadaaaaaa
bdaaaaaadcaaaaalhcaabaaaacaaaaaaegacbaaaacaaaaaapgipcaaaadaaaaaa
beaaaaaaegbcbaiaebaaaaaaaaaaaaaabaaaaaahcccabaaaacaaaaaaegacbaaa
abaaaaaaegacbaaaacaaaaaabaaaaaahbccabaaaacaaaaaaegbcbaaaabaaaaaa
egacbaaaacaaaaaabaaaaaaheccabaaaacaaaaaaegbcbaaaacaaaaaaegacbaaa
acaaaaaadiaaaaaiccaabaaaaaaaaaaabkaabaaaaaaaaaaaakiacaaaabaaaaaa
afaaaaaadiaaaaakncaabaaaabaaaaaaagahbaaaaaaaaaaaaceaaaaaaaaaaadp
aaaaaaaaaaaaaadpaaaaaadpdgaaaaafmccabaaaadaaaaaakgaobaaaaaaaaaaa
aaaaaaahdccabaaaadaaaaaakgakbaaaabaaaaaamgaabaaaabaaaaaadiaaaaai
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
hccabaaaaeaaaaaaegiccaaaacaaaaaabiaaaaaaagaabaaaaaaaaaaaegacbaaa
abaaaaaadoaaaaabejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaaadaaaaaa
abaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaaahahaaaa
laaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaaabaaaaaa
aaaaaaaaadaaaaaaaeaaaaaaapaaaaaaljaaaaaaaaaaaaaaaaaaaaaaadaaaaaa
afaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfcenebemaa
feeffiedepepfceeaaedepemepfcaaklepfdeheojiaaaaaaafaaaaaaaiaaaaaa
iaaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaabaaaaaaapaaaaaaimaaaaaaabaaaaaaaaaaaaaaadaaaaaa
acaaaaaaahaiaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaapaaaaaa
imaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaahaiaaaafdfgfpfaepfdejfe
ejepeoaafeeffiedepepfceeaaklklkl"
}

SubProgram "opengl " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
Bind "vertex" Vertex
Bind "tangent" ATTR14
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Vector 17 [_WorldSpaceCameraPos]
Vector 18 [_ProjectionParams]
Vector 19 [unity_ShadowFadeCenterAndType]
Matrix 9 [_Object2World]
Matrix 13 [_World2Object]
Vector 20 [unity_Scale]
Vector 21 [unity_LightmapST]
Vector 22 [_MainTex_ST]
Vector 23 [_BumpMap_ST]
"!!ARBvp1.0
# 34 ALU
PARAM c[24] = { { 1, 0.5 },
		state.matrix.modelview[0],
		state.matrix.mvp,
		program.local[9..23] };
TEMP R0;
TEMP R1;
TEMP R2;
MOV R0.xyz, vertex.attrib[14];
MUL R1.xyz, vertex.normal.zxyw, R0.yzxw;
MAD R0.xyz, vertex.normal.yzxw, R0.zxyw, -R1;
MUL R0.xyz, R0, vertex.attrib[14].w;
MOV R1.xyz, c[17];
MOV R1.w, c[0].x;
DP4 R0.w, vertex.position, c[8];
DP4 R2.z, R1, c[15];
DP4 R2.x, R1, c[13];
DP4 R2.y, R1, c[14];
MAD R2.xyz, R2, c[20].w, -vertex.position;
DP3 result.texcoord[1].y, R2, R0;
DP4 R0.z, vertex.position, c[7];
DP4 R0.x, vertex.position, c[5];
DP4 R0.y, vertex.position, c[6];
MUL R1.xyz, R0.xyww, c[0].y;
MUL R1.y, R1, c[18].x;
ADD result.texcoord[2].xy, R1, R1.z;
MOV result.position, R0;
MOV R0.x, c[0];
ADD R0.y, R0.x, -c[19].w;
DP4 R0.x, vertex.position, c[3];
DP4 R1.z, vertex.position, c[11];
DP4 R1.x, vertex.position, c[9];
DP4 R1.y, vertex.position, c[10];
ADD R1.xyz, R1, -c[19];
DP3 result.texcoord[1].z, vertex.normal, R2;
DP3 result.texcoord[1].x, R2, vertex.attrib[14];
MOV result.texcoord[2].zw, R0;
MUL result.texcoord[4].xyz, R1, c[19].w;
MAD result.texcoord[0].zw, vertex.texcoord[0].xyxy, c[23].xyxy, c[23];
MAD result.texcoord[0].xy, vertex.texcoord[0], c[22], c[22].zwzw;
MAD result.texcoord[3].xy, vertex.texcoord[1], c[21], c[21].zwzw;
MUL result.texcoord[4].w, -R0.x, R0.y;
END
# 34 instructions, 3 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Matrix 0 [glstate_matrix_modelview0]
Matrix 4 [glstate_matrix_mvp]
Vector 16 [_WorldSpaceCameraPos]
Vector 17 [_ProjectionParams]
Vector 18 [_ScreenParams]
Vector 19 [unity_ShadowFadeCenterAndType]
Matrix 8 [_Object2World]
Matrix 12 [_World2Object]
Vector 20 [unity_Scale]
Vector 21 [unity_LightmapST]
Vector 22 [_MainTex_ST]
Vector 23 [_BumpMap_ST]
"vs_2_0
; 35 ALU
def c24, 1.00000000, 0.50000000, 0, 0
dcl_position0 v0
dcl_tangent0 v1
dcl_normal0 v2
dcl_texcoord0 v3
dcl_texcoord1 v4
mov r0.xyz, v1
mul r1.xyz, v2.zxyw, r0.yzxw
mov r0.xyz, v1
mad r0.xyz, v2.yzxw, r0.zxyw, -r1
mul r0.xyz, r0, v1.w
mov r1.xyz, c16
mov r1.w, c24.x
dp4 r0.w, v0, c7
dp4 r2.z, r1, c14
dp4 r2.x, r1, c12
dp4 r2.y, r1, c13
mad r2.xyz, r2, c20.w, -v0
dp3 oT1.y, r2, r0
dp4 r0.z, v0, c6
dp4 r0.x, v0, c4
dp4 r0.y, v0, c5
mul r1.xyz, r0.xyww, c24.y
mul r1.y, r1, c17.x
mad oT2.xy, r1.z, c18.zwzw, r1
mov oPos, r0
mov r0.x, c19.w
add r0.y, c24.x, -r0.x
dp4 r0.x, v0, c2
dp4 r1.z, v0, c10
dp4 r1.x, v0, c8
dp4 r1.y, v0, c9
add r1.xyz, r1, -c19
dp3 oT1.z, v2, r2
dp3 oT1.x, r2, v1
mov oT2.zw, r0
mul oT4.xyz, r1, c19.w
mad oT0.zw, v3.xyxy, c23.xyxy, c23
mad oT0.xy, v3, c22, c22.zwzw
mad oT3.xy, v4, c21, c21.zwzw
mul oT4.w, -r0.x, r0.y
"
}

SubProgram "xbox360 " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Vector 23 [_BumpMap_ST]
Vector 22 [_MainTex_ST]
Matrix 12 [_Object2World] 4
Vector 1 [_ProjectionParams]
Vector 2 [_ScreenParams]
Matrix 16 [_World2Object] 4
Vector 0 [_WorldSpaceCameraPos]
Matrix 8 [glstate_matrix_modelview0] 4
Matrix 4 [glstate_matrix_mvp] 4
Vector 21 [unity_LightmapST]
Vector 20 [unity_Scale]
Vector 3 [unity_ShadowFadeCenterAndType]
// Shader Timing Estimate, in Cycles/64 vertex vector:
// ALU: 44.00 (33 instructions), vertex: 64, texture: 0,
//   sequencer: 18,  11 GPRs, 15 threads,
// Performance (if enough threads): ~64 cycles per vector
// * Vertex cycle estimates are assuming 3 vfetch_minis for every vfetch_full,
//     with <= 32 bytes per vfetch_full group.

"vs_360
backbbabaaaaacomaaaaacfaaaaaaaaaaaaaaaceaaaaacemaaaaacheaaaaaaaa
aaaaaaaaaaaaacceaaaaaabmaaaaacbhpppoadaaaaaaaaamaaaaaabmaaaaaaaa
aaaaacbaaaaaabamaaacaabhaaabaaaaaaaaabbiaaaaaaaaaaaaabciaaacaabg
aaabaaaaaaaaabbiaaaaaaaaaaaaabdeaaacaaamaaaeaaaaaaaaabeeaaaaaaaa
aaaaabfeaaacaaabaaabaaaaaaaaabbiaaaaaaaaaaaaabggaaacaaacaaabaaaa
aaaaabbiaaaaaaaaaaaaabheaaacaabaaaaeaaaaaaaaabeeaaaaaaaaaaaaabic
aaacaaaaaaabaaaaaaaaabjiaaaaaaaaaaaaabkiaaacaaaiaaaeaaaaaaaaabee
aaaaaaaaaaaaabmcaaacaaaeaaaeaaaaaaaaabeeaaaaaaaaaaaaabnfaaacaabf
aaabaaaaaaaaabbiaaaaaaaaaaaaabogaaacaabeaaabaaaaaaaaabbiaaaaaaaa
aaaaabpcaaacaaadaaabaaaaaaaaabbiaaaaaaaafpechfgnhaengbhafpfdfeaa
aaabaaadaaabaaaeaaabaaaaaaaaaaaafpengbgjgofegfhifpfdfeaafpepgcgk
gfgdhedcfhgphcgmgeaaklklaaadaaadaaaeaaaeaaabaaaaaaaaaaaafpfahcgp
gkgfgdhegjgpgofagbhcgbgnhdaafpfdgdhcgfgfgofagbhcgbgnhdaafpfhgphc
gmgedcepgcgkgfgdheaafpfhgphcgmgefdhagbgdgfedgbgngfhcgbfagphdaakl
aaabaaadaaabaaadaaabaaaaaaaaaaaaghgmhdhegbhegffpgngbhehcgjhifpgn
gpgegfgmhggjgfhhdaaaghgmhdhegbhegffpgngbhehcgjhifpgnhghaaahfgogj
hehjfpemgjghgihegngbhafdfeaahfgogjhehjfpfdgdgbgmgfaahfgogjhehjfp
fdgigbgegphheggbgegfedgfgohegfhcebgogefehjhagfaahghdfpddfpdaaadc
codacodcdadddfddcodaaaklaaaaaaaaaaaaaaabaaaaaaaaaaaaaaaaaaaaaabe
aapmaabaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaeaaaaaacbaaaebaaak
aaaaaaaaaaaaaaaaaaaaeekfaaaaaaabaaaaaaafaaaaaaakaaaaacjaaabaaaaf
aaaagaagaaaadaahaaaafaaiaacbfaajaaaapafaaaachbfbaaafpcfcaaahddfd
aaaipefeaaaaaaboaaaababpaaaaaaciaaaaaacjaaaabackaaaaaabmaaaabacd
aaaababnaaaaaaccaaaabacfaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaadpiaaaaa
dpaaaaaaaaaaaaaaaaaaaaaapbfffaafaaaabcabmcaaaaaaaaaafaakaaaabcaa
meaaaaaaaaaagaapgabfbcaabcaaaaaaaaaagablgacbbcaabcaaaaaaaaaaeach
aaaaccaaaaaaaaaaafpibaaaaaaaaeedaaaaaaaaafpieaaaaaaaagiiaaaaaaaa
afpicaaaaaaaaoiiaaaaaaaaafpiiaaaaaaaapmiaaaaaaaaafpiiaaaaaaaacdp
aaaaaaaakmipafaaaagmiimaibabahalmiapaaaaaablnapiklabagaamiapaaaa
aamgdepiklabafaamiapaaajaalbnajeklabaeaamiapiadoaananaaaocajajaa
miaiaaacaeblgmaacaadppaamiahaaaaaalelbaacbbbaaaamiahaaagaamagmle
clbaaaaabebhaaafaalemglbcbbcaaabkiiiakahaamgmgmaibabajaibecnaaaa
aabliegmkbabaoabkibhahadaalogfebmbacaeapmiahaaaaaamglebeklabanaa
miahaaadabgflomaolacaeadkmchahadaamablmambadaeapmiahaaakaalblema
klabamaakmehahaaaamalbiaibajppapmiapaaahaadedeaaoaakahaamiamiaac
aanlnlaaocajajaamiadiaadaabklabkilaibfbfmiadiaaaaalalabkilaibgbg
miamiaaaaakmkmagilaibhbhmiaiaaagaablmgblklabakahkiihaaahacmamaeb
iaahadabmiahiaaeaamablaakbahadaamiadiaacaamgbkbiklaaacaamiapaaaa
aadeaaaaoaagafaamiaiiaaeaeblblaaobaaacaamiahaaaaaalemaaakaaabdaa
miahaaaaabmablbfklaabeabmiabiaabaaloloaapaaaaeaamiaciaabaaloloaa
paadaaaamiaeiaabaaloloaapaaaacaaaaaaaaaaaaaaaaaaaaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
Matrix 256 [glstate_matrix_modelview0]
Matrix 260 [glstate_matrix_mvp]
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Vector 467 [_WorldSpaceCameraPos]
Vector 466 [_ProjectionParams]
Vector 465 [unity_ShadowFadeCenterAndType]
Matrix 264 [_Object2World]
Matrix 268 [_World2Object]
Vector 464 [unity_Scale]
Vector 463 [unity_LightmapST]
Vector 462 [_MainTex_ST]
Vector 461 [_BumpMap_ST]
"sce_vp_rsx // 33 instructions using 5 registers
[Configuration]
8
0000002143050500
[Defaults]
1
460 2
3f0000003f800000
[Microcode]
528
00011c6c00400e0c0106c0836041dffc00021c6c005d300c0186c0836041dffc
401f9c6c011cd800810040d560607f9c401f9c6c011ce808010400d740619f9c
401f9c6c011cf908010400d740619fa800001c6c01d0200d8106c0c360403ffc
00009c6c01d0700d8106c0c360403ffc00009c6c01d0600d8106c0c360405ffc
00009c6c01d0500d8106c0c360409ffc00009c6c01d0400d8106c0c360411ffc
00011c6c005d107f8186c08360403ffc00019c6c01d0a00d8106c0c360405ffc
00019c6c01d0900d8106c0c360409ffc00019c6c01d0800d8106c0c360411ffc
00001c6c0190e00c0886c0c360405ffc00001c6c0190d00c0886c0c360409ffc
00001c6c0190c00c0886c0c360411ffc00021c6c00dd108c0186c08301a1dffc
00019c6c00800243011842436041dffc00011c6c00dcc02a8186c0bfe1203ffc
00011c6c010002308121826301a1dffc401f9c6c0040000d8286c0836041ff80
401f9c6c004000558286c08360407fa400001c6c011d000c00bfc0e30041dffc
00009c6c009cc00e028000c36041dffc401f9c6c009d100c08bfc0c36041dfac
401f9c6c008000ff80bfc24360403fac00009c6c009d202a828000c360409ffc
401f9c6c00c000080286c09540a19fa400009c6c00800e0c04bfc0836041dffc
401f9c6c0140020c0106004360405fa0401f9c6c01400e0c0086008360411fa0
401f9c6c0140000c0086014360409fa1
"
}

SubProgram "d3d11 " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Bind "color" Color
ConstBuffer "$Globals" 160 // 128 used size, 12 vars
Vector 80 [unity_LightmapST] 4
Vector 96 [_MainTex_ST] 4
Vector 112 [_BumpMap_ST] 4
ConstBuffer "UnityPerCamera" 128 // 96 used size, 8 vars
Vector 64 [_WorldSpaceCameraPos] 3
Vector 80 [_ProjectionParams] 4
ConstBuffer "UnityShadows" 416 // 416 used size, 8 vars
Vector 400 [unity_ShadowFadeCenterAndType] 4
ConstBuffer "UnityPerDraw" 336 // 336 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
Matrix 64 [glstate_matrix_modelview0] 4
Matrix 192 [_Object2World] 4
Matrix 256 [_World2Object] 4
Vector 320 [unity_Scale] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityShadows" 2
BindCB "UnityPerDraw" 3
// 36 instructions, 3 temp regs, 0 temp arrays:
// ALU 17 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0
eefiecedhpgnfackdfhjkcdnfikgkcikiajkponcabaaaaaagmahaaaaadaaaaaa
cmaaaaaapeaaaaaakmabaaaaejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaa
abaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapadaaaaljaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfc
enebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheolaaaaaaaagaaaaaa
aiaaaaaajiaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaakeaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaabaaaaaaapaaaaaakeaaaaaaabaaaaaaaaaaaaaa
adaaaaaaacaaaaaaahaiaaaakeaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaa
apaaaaaakeaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaadamaaaakeaaaaaa
aeaaaaaaaaaaaaaaadaaaaaaafaaaaaaapaaaaaafdfgfpfaepfdejfeejepeoaa
feeffiedepepfceeaaklklklfdeieefcliafaaaaeaaaabaagoabaaaafjaaaaae
egiocaaaaaaaaaaaaiaaaaaafjaaaaaeegiocaaaabaaaaaaagaaaaaafjaaaaae
egiocaaaacaaaaaabkaaaaaafjaaaaaeegiocaaaadaaaaaabfaaaaaafpaaaaad
pcbabaaaaaaaaaaafpaaaaadpcbabaaaabaaaaaafpaaaaadhcbabaaaacaaaaaa
fpaaaaaddcbabaaaadaaaaaafpaaaaaddcbabaaaaeaaaaaaghaaaaaepccabaaa
aaaaaaaaabaaaaaagfaaaaadpccabaaaabaaaaaagfaaaaadhccabaaaacaaaaaa
gfaaaaadpccabaaaadaaaaaagfaaaaaddccabaaaaeaaaaaagfaaaaadpccabaaa
afaaaaaagiaaaaacadaaaaaadiaaaaaipcaabaaaaaaaaaaafgbfbaaaaaaaaaaa
egiocaaaadaaaaaaabaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaa
aaaaaaaaagbabaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaa
egiocaaaadaaaaaaacaaaaaakgbkbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaak
pcaabaaaaaaaaaaaegiocaaaadaaaaaaadaaaaaapgbpbaaaaaaaaaaaegaobaaa
aaaaaaaadgaaaaafpccabaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaaldccabaaa
abaaaaaaegbabaaaadaaaaaaegiacaaaaaaaaaaaagaaaaaaogikcaaaaaaaaaaa
agaaaaaadcaaaaalmccabaaaabaaaaaaagbebaaaadaaaaaaagiecaaaaaaaaaaa
ahaaaaaakgiocaaaaaaaaaaaahaaaaaadiaaaaahhcaabaaaabaaaaaajgbebaaa
abaaaaaacgbjbaaaacaaaaaadcaaaaakhcaabaaaabaaaaaajgbebaaaacaaaaaa
cgbjbaaaabaaaaaaegacbaiaebaaaaaaabaaaaaadiaaaaahhcaabaaaabaaaaaa
egacbaaaabaaaaaapgbpbaaaabaaaaaadiaaaaajhcaabaaaacaaaaaafgifcaaa
abaaaaaaaeaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaaacaaaaaa
egiccaaaadaaaaaabaaaaaaaagiacaaaabaaaaaaaeaaaaaaegacbaaaacaaaaaa
dcaaaaalhcaabaaaacaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaaabaaaaaa
aeaaaaaaegacbaaaacaaaaaaaaaaaaaihcaabaaaacaaaaaaegacbaaaacaaaaaa
egiccaaaadaaaaaabdaaaaaadcaaaaalhcaabaaaacaaaaaaegacbaaaacaaaaaa
pgipcaaaadaaaaaabeaaaaaaegbcbaiaebaaaaaaaaaaaaaabaaaaaahcccabaaa
acaaaaaaegacbaaaabaaaaaaegacbaaaacaaaaaabaaaaaahbccabaaaacaaaaaa
egbcbaaaabaaaaaaegacbaaaacaaaaaabaaaaaaheccabaaaacaaaaaaegbcbaaa
acaaaaaaegacbaaaacaaaaaadiaaaaaiccaabaaaaaaaaaaabkaabaaaaaaaaaaa
akiacaaaabaaaaaaafaaaaaadiaaaaakncaabaaaabaaaaaaagahbaaaaaaaaaaa
aceaaaaaaaaaaadpaaaaaaaaaaaaaadpaaaaaadpdgaaaaafmccabaaaadaaaaaa
kgaobaaaaaaaaaaaaaaaaaahdccabaaaadaaaaaakgakbaaaabaaaaaamgaabaaa
abaaaaaadcaaaaaldccabaaaaeaaaaaaegbabaaaaeaaaaaaegiacaaaaaaaaaaa
afaaaaaaogikcaaaaaaaaaaaafaaaaaadiaaaaaihcaabaaaaaaaaaaafgbfbaaa
aaaaaaaaegiccaaaadaaaaaaanaaaaaadcaaaaakhcaabaaaaaaaaaaaegiccaaa
adaaaaaaamaaaaaaagbabaaaaaaaaaaaegacbaaaaaaaaaaadcaaaaakhcaabaaa
aaaaaaaaegiccaaaadaaaaaaaoaaaaaakgbkbaaaaaaaaaaaegacbaaaaaaaaaaa
dcaaaaakhcaabaaaaaaaaaaaegiccaaaadaaaaaaapaaaaaapgbpbaaaaaaaaaaa
egacbaaaaaaaaaaaaaaaaaajhcaabaaaaaaaaaaaegacbaaaaaaaaaaaegiccaia
ebaaaaaaacaaaaaabjaaaaaadiaaaaaihccabaaaafaaaaaaegacbaaaaaaaaaaa
pgipcaaaacaaaaaabjaaaaaadiaaaaaibcaabaaaaaaaaaaabkbabaaaaaaaaaaa
ckiacaaaadaaaaaaafaaaaaadcaaaaakbcaabaaaaaaaaaaackiacaaaadaaaaaa
aeaaaaaaakbabaaaaaaaaaaaakaabaaaaaaaaaaadcaaaaakbcaabaaaaaaaaaaa
ckiacaaaadaaaaaaagaaaaaackbabaaaaaaaaaaaakaabaaaaaaaaaaadcaaaaak
bcaabaaaaaaaaaaackiacaaaadaaaaaaahaaaaaadkbabaaaaaaaaaaaakaabaaa
aaaaaaaaaaaaaaajccaabaaaaaaaaaaadkiacaiaebaaaaaaacaaaaaabjaaaaaa
abeaaaaaaaaaiadpdiaaaaaiiccabaaaafaaaaaabkaabaaaaaaaaaaaakaabaia
ebaaaaaaaaaaaaaadoaaaaab"
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

varying highp vec4 xlv_TEXCOORD4;
varying highp vec2 xlv_TEXCOORD3;
varying highp vec4 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 _BumpMap_ST;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 unity_LightmapST;
uniform highp vec4 unity_Scale;
uniform highp mat4 _World2Object;
uniform highp mat4 _Object2World;


uniform highp vec4 unity_ShadowFadeCenterAndType;
uniform highp vec4 _ProjectionParams;
uniform highp vec3 _WorldSpaceCameraPos;
attribute vec4 _glesTANGENT;
attribute vec4 _glesMultiTexCoord1;
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
  highp vec4 tmpvar_3;
  highp vec4 tmpvar_4;
  highp vec4 tmpvar_5;
  tmpvar_5 = (gl_ModelViewProjectionMatrix * _glesVertex);
  tmpvar_3.xy = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  tmpvar_3.zw = ((_glesMultiTexCoord0.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
  highp vec4 o_6;
  highp vec4 tmpvar_7;
  tmpvar_7 = (tmpvar_5 * 0.5);
  highp vec2 tmpvar_8;
  tmpvar_8.x = tmpvar_7.x;
  tmpvar_8.y = (tmpvar_7.y * _ProjectionParams.x);
  o_6.xy = (tmpvar_8 + tmpvar_7.w);
  o_6.zw = tmpvar_5.zw;
  tmpvar_4.xyz = (((_Object2World * _glesVertex).xyz - unity_ShadowFadeCenterAndType.xyz) * unity_ShadowFadeCenterAndType.w);
  tmpvar_4.w = (-((gl_ModelViewMatrix * _glesVertex).z) * (1.0 - unity_ShadowFadeCenterAndType.w));
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
  highp vec4 tmpvar_12;
  tmpvar_12.w = 1.0;
  tmpvar_12.xyz = _WorldSpaceCameraPos;
  gl_Position = tmpvar_5;
  xlv_TEXCOORD0 = tmpvar_3;
  xlv_TEXCOORD1 = (tmpvar_11 * (((_World2Object * tmpvar_12).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = o_6;
  xlv_TEXCOORD3 = ((_glesMultiTexCoord1.xy * unity_LightmapST.xy) + unity_LightmapST.zw);
  xlv_TEXCOORD4 = tmpvar_4;
}



#endif
#ifdef FRAGMENT

varying highp vec4 xlv_TEXCOORD4;
varying highp vec2 xlv_TEXCOORD3;
varying highp vec4 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 unity_LightmapFade;
uniform sampler2D unity_LightmapInd;
uniform sampler2D unity_Lightmap;
uniform sampler2D _LightBuffer;
uniform highp float _Parallax;
uniform mediump float _Gloss;
uniform lowp vec4 _Color;
uniform sampler2D _ParallaxMap;
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
  mediump vec4 spec_6;
  mediump float h_7;
  lowp float tmpvar_8;
  tmpvar_8 = texture2D (_ParallaxMap, xlv_TEXCOORD0.zw).w;
  h_7 = tmpvar_8;
  mediump float height_9;
  height_9 = _Parallax;
  mediump vec3 viewDir_10;
  viewDir_10 = xlv_TEXCOORD1;
  highp vec3 v_11;
  mediump float tmpvar_12;
  tmpvar_12 = ((h_7 * height_9) - (height_9 / 2.0));
  mediump vec3 tmpvar_13;
  tmpvar_13 = normalize(viewDir_10);
  v_11 = tmpvar_13;
  v_11.z = (v_11.z + 0.42);
  highp vec2 tmpvar_14;
  tmpvar_14 = (xlv_TEXCOORD0.xy + (tmpvar_12 * (v_11.xy / v_11.z)));
  lowp vec4 tmpvar_15;
  tmpvar_15 = texture2D (_MainTex, tmpvar_14);
  lowp vec3 tmpvar_16;
  tmpvar_16 = (tmpvar_15.xyz * _Color.xyz);
  tmpvar_5 = tmpvar_16;
  lowp vec4 tmpvar_17;
  tmpvar_17 = texture2D (_SpecMap, tmpvar_14);
  spec_6 = tmpvar_17;
  lowp vec4 tmpvar_18;
  tmpvar_18 = texture2DProj (_LightBuffer, xlv_TEXCOORD2);
  light_4 = tmpvar_18;
  mediump vec4 tmpvar_19;
  tmpvar_19 = -(log2(max (light_4, vec4(0.001, 0.001, 0.001, 0.001))));
  light_4.w = tmpvar_19.w;
  lowp vec3 tmpvar_20;
  tmpvar_20 = (2.0 * texture2D (unity_Lightmap, xlv_TEXCOORD3).xyz);
  lmFull_3 = tmpvar_20;
  lowp vec3 tmpvar_21;
  tmpvar_21 = (2.0 * texture2D (unity_LightmapInd, xlv_TEXCOORD3).xyz);
  lmIndirect_2 = tmpvar_21;
  highp float tmpvar_22;
  tmpvar_22 = clamp (((sqrt(dot (xlv_TEXCOORD4, xlv_TEXCOORD4)) * unity_LightmapFade.z) + unity_LightmapFade.w), 0.0, 1.0);
  light_4.xyz = (tmpvar_19.xyz + mix (lmIndirect_2, lmFull_3, vec3(tmpvar_22)));
  mediump vec4 c_23;
  mediump vec3 tmpvar_24;
  tmpvar_24 = (tmpvar_19.w * ((tmpvar_15.w * spec_6.xyz) * _Gloss));
  c_23.xyz = ((tmpvar_5 * light_4.xyz) + (light_4.xyz * tmpvar_24));
  c_23.w = (tmpvar_24 * _SpecColor.w).x;
  tmpvar_1 = c_23;
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

varying highp vec4 xlv_TEXCOORD4;
varying highp vec2 xlv_TEXCOORD3;
varying highp vec4 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 _BumpMap_ST;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 unity_LightmapST;
uniform highp vec4 unity_Scale;
uniform highp mat4 _World2Object;
uniform highp mat4 _Object2World;


uniform highp vec4 unity_ShadowFadeCenterAndType;
uniform highp vec4 _ProjectionParams;
uniform highp vec3 _WorldSpaceCameraPos;
attribute vec4 _glesTANGENT;
attribute vec4 _glesMultiTexCoord1;
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
  highp vec4 tmpvar_3;
  highp vec4 tmpvar_4;
  highp vec4 tmpvar_5;
  tmpvar_5 = (gl_ModelViewProjectionMatrix * _glesVertex);
  tmpvar_3.xy = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  tmpvar_3.zw = ((_glesMultiTexCoord0.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
  highp vec4 o_6;
  highp vec4 tmpvar_7;
  tmpvar_7 = (tmpvar_5 * 0.5);
  highp vec2 tmpvar_8;
  tmpvar_8.x = tmpvar_7.x;
  tmpvar_8.y = (tmpvar_7.y * _ProjectionParams.x);
  o_6.xy = (tmpvar_8 + tmpvar_7.w);
  o_6.zw = tmpvar_5.zw;
  tmpvar_4.xyz = (((_Object2World * _glesVertex).xyz - unity_ShadowFadeCenterAndType.xyz) * unity_ShadowFadeCenterAndType.w);
  tmpvar_4.w = (-((gl_ModelViewMatrix * _glesVertex).z) * (1.0 - unity_ShadowFadeCenterAndType.w));
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
  highp vec4 tmpvar_12;
  tmpvar_12.w = 1.0;
  tmpvar_12.xyz = _WorldSpaceCameraPos;
  gl_Position = tmpvar_5;
  xlv_TEXCOORD0 = tmpvar_3;
  xlv_TEXCOORD1 = (tmpvar_11 * (((_World2Object * tmpvar_12).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = o_6;
  xlv_TEXCOORD3 = ((_glesMultiTexCoord1.xy * unity_LightmapST.xy) + unity_LightmapST.zw);
  xlv_TEXCOORD4 = tmpvar_4;
}



#endif
#ifdef FRAGMENT

varying highp vec4 xlv_TEXCOORD4;
varying highp vec2 xlv_TEXCOORD3;
varying highp vec4 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 unity_LightmapFade;
uniform sampler2D unity_LightmapInd;
uniform sampler2D unity_Lightmap;
uniform sampler2D _LightBuffer;
uniform highp float _Parallax;
uniform mediump float _Gloss;
uniform lowp vec4 _Color;
uniform sampler2D _ParallaxMap;
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
  mediump vec4 spec_6;
  mediump float h_7;
  lowp float tmpvar_8;
  tmpvar_8 = texture2D (_ParallaxMap, xlv_TEXCOORD0.zw).w;
  h_7 = tmpvar_8;
  highp vec2 tmpvar_9;
  mediump float height_10;
  height_10 = _Parallax;
  mediump vec3 viewDir_11;
  viewDir_11 = xlv_TEXCOORD1;
  highp vec3 v_12;
  mediump float tmpvar_13;
  tmpvar_13 = ((h_7 * height_10) - (height_10 / 2.0));
  mediump vec3 tmpvar_14;
  tmpvar_14 = normalize(viewDir_11);
  v_12 = tmpvar_14;
  v_12.z = (v_12.z + 0.42);
  tmpvar_9 = (tmpvar_13 * (v_12.xy / v_12.z));
  highp vec2 tmpvar_15;
  tmpvar_15 = (xlv_TEXCOORD0.xy + tmpvar_9);
  highp vec2 tmpvar_16;
  tmpvar_16 = (xlv_TEXCOORD0.zw + tmpvar_9);
  lowp vec4 tmpvar_17;
  tmpvar_17 = texture2D (_MainTex, tmpvar_15);
  lowp vec3 tmpvar_18;
  tmpvar_18 = (tmpvar_17.xyz * _Color.xyz);
  tmpvar_5 = tmpvar_18;
  lowp vec4 tmpvar_19;
  tmpvar_19 = texture2D (_SpecMap, tmpvar_15);
  spec_6 = tmpvar_19;
  lowp vec3 normal_20;
  normal_20.xy = ((texture2D (_BumpMap, tmpvar_16).wy * 2.0) - 1.0);
  normal_20.z = sqrt(((1.0 - (normal_20.x * normal_20.x)) - (normal_20.y * normal_20.y)));
  lowp vec4 tmpvar_21;
  tmpvar_21 = texture2DProj (_LightBuffer, xlv_TEXCOORD2);
  light_4 = tmpvar_21;
  mediump vec4 tmpvar_22;
  tmpvar_22 = -(log2(max (light_4, vec4(0.001, 0.001, 0.001, 0.001))));
  light_4.w = tmpvar_22.w;
  lowp vec4 tmpvar_23;
  tmpvar_23 = texture2D (unity_Lightmap, xlv_TEXCOORD3);
  lowp vec3 tmpvar_24;
  tmpvar_24 = ((8.0 * tmpvar_23.w) * tmpvar_23.xyz);
  lmFull_3 = tmpvar_24;
  lowp vec4 tmpvar_25;
  tmpvar_25 = texture2D (unity_LightmapInd, xlv_TEXCOORD3);
  lowp vec3 tmpvar_26;
  tmpvar_26 = ((8.0 * tmpvar_25.w) * tmpvar_25.xyz);
  lmIndirect_2 = tmpvar_26;
  highp float tmpvar_27;
  tmpvar_27 = clamp (((sqrt(dot (xlv_TEXCOORD4, xlv_TEXCOORD4)) * unity_LightmapFade.z) + unity_LightmapFade.w), 0.0, 1.0);
  light_4.xyz = (tmpvar_22.xyz + mix (lmIndirect_2, lmFull_3, vec3(tmpvar_27)));
  mediump vec4 c_28;
  mediump vec3 tmpvar_29;
  tmpvar_29 = (tmpvar_22.w * ((tmpvar_17.w * spec_6.xyz) * _Gloss));
  c_28.xyz = ((tmpvar_5 * light_4.xyz) + (light_4.xyz * tmpvar_29));
  c_28.w = (tmpvar_29 * _SpecColor.w).x;
  tmpvar_1 = c_28;
  gl_FragData[0] = tmpvar_1;
}



#endif"
}

SubProgram "d3d11_9x " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Bind "color" Color
ConstBuffer "$Globals" 160 // 128 used size, 12 vars
Vector 80 [unity_LightmapST] 4
Vector 96 [_MainTex_ST] 4
Vector 112 [_BumpMap_ST] 4
ConstBuffer "UnityPerCamera" 128 // 96 used size, 8 vars
Vector 64 [_WorldSpaceCameraPos] 3
Vector 80 [_ProjectionParams] 4
ConstBuffer "UnityShadows" 416 // 416 used size, 8 vars
Vector 400 [unity_ShadowFadeCenterAndType] 4
ConstBuffer "UnityPerDraw" 336 // 336 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
Matrix 64 [glstate_matrix_modelview0] 4
Matrix 192 [_Object2World] 4
Matrix 256 [_World2Object] 4
Vector 320 [unity_Scale] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityShadows" 2
BindCB "UnityPerDraw" 3
// 36 instructions, 3 temp regs, 0 temp arrays:
// ALU 17 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0_level_9_3
eefieceddnapboddpcmpibnejgblokeedbocphaiabaaaaaaoiakaaaaaeaaaaaa
daaaaaaakiadaaaagiajaaaadaakaaaaebgpgodjhaadaaaahaadaaaaaaacpopp
amadaaaageaaaaaaafaaceaaaaaagaaaaaaagaaaaaaaceaaabaagaaaaaaaafaa
adaaabaaaaaaaaaaabaaaeaaacaaaeaaaaaaaaaaacaabjaaabaaagaaaaaaaaaa
adaaaaaaaiaaahaaaaaaaaaaadaaamaaajaaapaaaaaaaaaaaaaaaaaaabacpopp
fbaaaaafbiaaapkaaaaaaadpaaaaiadpaaaaaaaaaaaaaaaabpaaaaacafaaaaia
aaaaapjabpaaaaacafaaabiaabaaapjabpaaaaacafaaaciaacaaapjabpaaaaac
afaaadiaadaaapjabpaaaaacafaaaeiaaeaaapjaaeaaaaaeaaaaadoaadaaoeja
acaaoekaacaaookaaeaaaaaeaaaaamoaadaaeejaadaaeekaadaaoekaafaaaaad
aaaaapiaaaaaffjaaiaaoekaaeaaaaaeaaaaapiaahaaoekaaaaaaajaaaaaoeia
aeaaaaaeaaaaapiaajaaoekaaaaakkjaaaaaoeiaaeaaaaaeaaaaapiaakaaoeka
aaaappjaaaaaoeiaafaaaaadabaaabiaaaaaffiaafaaaakaafaaaaadabaaaiia
abaaaaiabiaaaakaafaaaaadabaaafiaaaaapeiabiaaaakaacaaaaadacaaadoa
abaakkiaabaaomiaaeaaaaaeadaaadoaaeaaoejaabaaoekaabaaookaafaaaaad
abaaahiaaaaaffjabaaaoekaaeaaaaaeabaaahiaapaaoekaaaaaaajaabaaoeia
aeaaaaaeabaaahiabbaaoekaaaaakkjaabaaoeiaaeaaaaaeabaaahiabcaaoeka
aaaappjaabaaoeiaacaaaaadabaaahiaabaaoeiaagaaoekbafaaaaadaeaaahoa
abaaoeiaagaappkaafaaaaadabaaabiaaaaaffjaamaakkkaaeaaaaaeabaaabia
alaakkkaaaaaaajaabaaaaiaaeaaaaaeabaaabiaanaakkkaaaaakkjaabaaaaia
aeaaaaaeabaaabiaaoaakkkaaaaappjaabaaaaiaabaaaaacabaaaiiaagaappka
acaaaaadabaaaciaabaappibbiaaffkaafaaaaadaeaaaioaabaaffiaabaaaaib
abaaaaacabaaahiaaeaaoekaafaaaaadacaaahiaabaaffiabeaaoekaaeaaaaae
abaaaliabdaakekaabaaaaiaacaakeiaaeaaaaaeabaaahiabfaaoekaabaakkia
abaapeiaacaaaaadabaaahiaabaaoeiabgaaoekaaeaaaaaeabaaahiaabaaoeia
bhaappkaaaaaoejbaiaaaaadabaaaboaabaaoejaabaaoeiaabaaaaacacaaahia
abaaoejaafaaaaadadaaahiaacaamjiaacaancjaaeaaaaaeacaaahiaacaamjja
acaanciaadaaoeibafaaaaadacaaahiaacaaoeiaabaappjaaiaaaaadabaaacoa
acaaoeiaabaaoeiaaiaaaaadabaaaeoaacaaoejaabaaoeiaaeaaaaaeaaaaadma
aaaappiaaaaaoekaaaaaoeiaabaaaaacaaaaammaaaaaoeiaabaaaaacacaaamoa
aaaaoeiappppaaaafdeieefcliafaaaaeaaaabaagoabaaaafjaaaaaeegiocaaa
aaaaaaaaaiaaaaaafjaaaaaeegiocaaaabaaaaaaagaaaaaafjaaaaaeegiocaaa
acaaaaaabkaaaaaafjaaaaaeegiocaaaadaaaaaabfaaaaaafpaaaaadpcbabaaa
aaaaaaaafpaaaaadpcbabaaaabaaaaaafpaaaaadhcbabaaaacaaaaaafpaaaaad
dcbabaaaadaaaaaafpaaaaaddcbabaaaaeaaaaaaghaaaaaepccabaaaaaaaaaaa
abaaaaaagfaaaaadpccabaaaabaaaaaagfaaaaadhccabaaaacaaaaaagfaaaaad
pccabaaaadaaaaaagfaaaaaddccabaaaaeaaaaaagfaaaaadpccabaaaafaaaaaa
giaaaaacadaaaaaadiaaaaaipcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaa
adaaaaaaabaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaaaaaaaaa
agbabaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaa
adaaaaaaacaaaaaakgbkbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaa
aaaaaaaaegiocaaaadaaaaaaadaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaa
dgaaaaafpccabaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaaldccabaaaabaaaaaa
egbabaaaadaaaaaaegiacaaaaaaaaaaaagaaaaaaogikcaaaaaaaaaaaagaaaaaa
dcaaaaalmccabaaaabaaaaaaagbebaaaadaaaaaaagiecaaaaaaaaaaaahaaaaaa
kgiocaaaaaaaaaaaahaaaaaadiaaaaahhcaabaaaabaaaaaajgbebaaaabaaaaaa
cgbjbaaaacaaaaaadcaaaaakhcaabaaaabaaaaaajgbebaaaacaaaaaacgbjbaaa
abaaaaaaegacbaiaebaaaaaaabaaaaaadiaaaaahhcaabaaaabaaaaaaegacbaaa
abaaaaaapgbpbaaaabaaaaaadiaaaaajhcaabaaaacaaaaaafgifcaaaabaaaaaa
aeaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaaacaaaaaaegiccaaa
adaaaaaabaaaaaaaagiacaaaabaaaaaaaeaaaaaaegacbaaaacaaaaaadcaaaaal
hcaabaaaacaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaaabaaaaaaaeaaaaaa
egacbaaaacaaaaaaaaaaaaaihcaabaaaacaaaaaaegacbaaaacaaaaaaegiccaaa
adaaaaaabdaaaaaadcaaaaalhcaabaaaacaaaaaaegacbaaaacaaaaaapgipcaaa
adaaaaaabeaaaaaaegbcbaiaebaaaaaaaaaaaaaabaaaaaahcccabaaaacaaaaaa
egacbaaaabaaaaaaegacbaaaacaaaaaabaaaaaahbccabaaaacaaaaaaegbcbaaa
abaaaaaaegacbaaaacaaaaaabaaaaaaheccabaaaacaaaaaaegbcbaaaacaaaaaa
egacbaaaacaaaaaadiaaaaaiccaabaaaaaaaaaaabkaabaaaaaaaaaaaakiacaaa
abaaaaaaafaaaaaadiaaaaakncaabaaaabaaaaaaagahbaaaaaaaaaaaaceaaaaa
aaaaaadpaaaaaaaaaaaaaadpaaaaaadpdgaaaaafmccabaaaadaaaaaakgaobaaa
aaaaaaaaaaaaaaahdccabaaaadaaaaaakgakbaaaabaaaaaamgaabaaaabaaaaaa
dcaaaaaldccabaaaaeaaaaaaegbabaaaaeaaaaaaegiacaaaaaaaaaaaafaaaaaa
ogikcaaaaaaaaaaaafaaaaaadiaaaaaihcaabaaaaaaaaaaafgbfbaaaaaaaaaaa
egiccaaaadaaaaaaanaaaaaadcaaaaakhcaabaaaaaaaaaaaegiccaaaadaaaaaa
amaaaaaaagbabaaaaaaaaaaaegacbaaaaaaaaaaadcaaaaakhcaabaaaaaaaaaaa
egiccaaaadaaaaaaaoaaaaaakgbkbaaaaaaaaaaaegacbaaaaaaaaaaadcaaaaak
hcaabaaaaaaaaaaaegiccaaaadaaaaaaapaaaaaapgbpbaaaaaaaaaaaegacbaaa
aaaaaaaaaaaaaaajhcaabaaaaaaaaaaaegacbaaaaaaaaaaaegiccaiaebaaaaaa
acaaaaaabjaaaaaadiaaaaaihccabaaaafaaaaaaegacbaaaaaaaaaaapgipcaaa
acaaaaaabjaaaaaadiaaaaaibcaabaaaaaaaaaaabkbabaaaaaaaaaaackiacaaa
adaaaaaaafaaaaaadcaaaaakbcaabaaaaaaaaaaackiacaaaadaaaaaaaeaaaaaa
akbabaaaaaaaaaaaakaabaaaaaaaaaaadcaaaaakbcaabaaaaaaaaaaackiacaaa
adaaaaaaagaaaaaackbabaaaaaaaaaaaakaabaaaaaaaaaaadcaaaaakbcaabaaa
aaaaaaaackiacaaaadaaaaaaahaaaaaadkbabaaaaaaaaaaaakaabaaaaaaaaaaa
aaaaaaajccaabaaaaaaaaaaadkiacaiaebaaaaaaacaaaaaabjaaaaaaabeaaaaa
aaaaiadpdiaaaaaiiccabaaaafaaaaaabkaabaaaaaaaaaaaakaabaiaebaaaaaa
aaaaaaaadoaaaaabejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaaadaaaaaa
abaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaaahahaaaa
laaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaaabaaaaaa
aaaaaaaaadaaaaaaaeaaaaaaapadaaaaljaaaaaaaaaaaaaaaaaaaaaaadaaaaaa
afaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfcenebemaa
feeffiedepepfceeaaedepemepfcaaklepfdeheolaaaaaaaagaaaaaaaiaaaaaa
jiaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaakeaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaabaaaaaaapaaaaaakeaaaaaaabaaaaaaaaaaaaaaadaaaaaa
acaaaaaaahaiaaaakeaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaapaaaaaa
keaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaadamaaaakeaaaaaaaeaaaaaa
aaaaaaaaadaaaaaaafaaaaaaapaaaaaafdfgfpfaepfdejfeejepeoaafeeffied
epepfceeaaklklkl"
}

SubProgram "opengl " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
Bind "vertex" Vertex
Bind "tangent" ATTR14
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 13 [_WorldSpaceCameraPos]
Vector 14 [_ProjectionParams]
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
Vector 24 [_BumpMap_ST]
"!!ARBvp1.0
# 42 ALU
PARAM c[25] = { { 1, 0.5 },
		state.matrix.mvp,
		program.local[5..24] };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
MUL R1.xyz, vertex.normal, c[22].w;
DP3 R2.w, R1, c[6];
DP3 R0.x, R1, c[5];
DP3 R0.z, R1, c[7];
MOV R0.y, R2.w;
MUL R1, R0.xyzz, R0.yzzx;
MOV R0.w, c[0].x;
DP4 R2.z, R0, c[17];
DP4 R2.y, R0, c[16];
DP4 R2.x, R0, c[15];
MUL R0.y, R2.w, R2.w;
DP4 R3.z, R1, c[20];
DP4 R3.x, R1, c[18];
DP4 R3.y, R1, c[19];
ADD R2.xyz, R2, R3;
MAD R0.w, R0.x, R0.x, -R0.y;
MUL R3.xyz, R0.w, c[21];
MOV R1.xyz, vertex.attrib[14];
MUL R0.xyz, vertex.normal.zxyw, R1.yzxw;
MAD R0.xyz, vertex.normal.yzxw, R1.zxyw, -R0;
MUL R0.xyz, R0, vertex.attrib[14].w;
MOV R1.xyz, c[13];
ADD result.texcoord[3].xyz, R2, R3;
MOV R1.w, c[0].x;
DP4 R0.w, vertex.position, c[4];
DP4 R2.z, R1, c[11];
DP4 R2.x, R1, c[9];
DP4 R2.y, R1, c[10];
MAD R2.xyz, R2, c[22].w, -vertex.position;
DP3 result.texcoord[1].y, R2, R0;
DP4 R0.z, vertex.position, c[3];
DP4 R0.x, vertex.position, c[1];
DP4 R0.y, vertex.position, c[2];
MUL R1.xyz, R0.xyww, c[0].y;
MUL R1.y, R1, c[14].x;
DP3 result.texcoord[1].z, vertex.normal, R2;
DP3 result.texcoord[1].x, R2, vertex.attrib[14];
ADD result.texcoord[2].xy, R1, R1.z;
MOV result.position, R0;
MOV result.texcoord[2].zw, R0;
MAD result.texcoord[0].zw, vertex.texcoord[0].xyxy, c[24].xyxy, c[24];
MAD result.texcoord[0].xy, vertex.texcoord[0], c[23], c[23].zwzw;
END
# 42 instructions, 4 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Matrix 0 [glstate_matrix_mvp]
Vector 12 [_WorldSpaceCameraPos]
Vector 13 [_ProjectionParams]
Vector 14 [_ScreenParams]
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
Vector 23 [_MainTex_ST]
Vector 24 [_BumpMap_ST]
"vs_2_0
; 43 ALU
def c25, 1.00000000, 0.50000000, 0, 0
dcl_position0 v0
dcl_tangent0 v1
dcl_normal0 v2
dcl_texcoord0 v3
mul r1.xyz, v2, c22.w
dp3 r2.w, r1, c5
dp3 r0.x, r1, c4
dp3 r0.z, r1, c6
mov r0.y, r2.w
mul r1, r0.xyzz, r0.yzzx
mov r0.w, c25.x
dp4 r2.z, r0, c17
dp4 r2.y, r0, c16
dp4 r2.x, r0, c15
mul r0.y, r2.w, r2.w
mad r0.w, r0.x, r0.x, -r0.y
dp4 r3.z, r1, c20
dp4 r3.y, r1, c19
dp4 r3.x, r1, c18
add r2.xyz, r2, r3
mul r3.xyz, r0.w, c21
mov r0.xyz, v1
mul r1.xyz, v2.zxyw, r0.yzxw
mov r0.xyz, v1
mad r0.xyz, v2.yzxw, r0.zxyw, -r1
mul r0.xyz, r0, v1.w
mov r1.xyz, c12
add oT3.xyz, r2, r3
mov r1.w, c25.x
dp4 r0.w, v0, c3
dp4 r2.z, r1, c10
dp4 r2.x, r1, c8
dp4 r2.y, r1, c9
mad r2.xyz, r2, c22.w, -v0
dp3 oT1.y, r2, r0
dp4 r0.z, v0, c2
dp4 r0.x, v0, c0
dp4 r0.y, v0, c1
mul r1.xyz, r0.xyww, c25.y
mul r1.y, r1, c13.x
dp3 oT1.z, v2, r2
dp3 oT1.x, r2, v1
mad oT2.xy, r1.z, c14.zwzw, r1
mov oPos, r0
mov oT2.zw, r0
mad oT0.zw, v3.xyxy, c24.xyxy, c24
mad oT0.xy, v3, c23, c23.zwzw
"
}

SubProgram "xbox360 " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 23 [_BumpMap_ST]
Vector 22 [_MainTex_ST]
Matrix 14 [_Object2World] 3
Vector 1 [_ProjectionParams]
Vector 2 [_ScreenParams]
Matrix 17 [_World2Object] 4
Vector 0 [_WorldSpaceCameraPos]
Matrix 10 [glstate_matrix_mvp] 4
Vector 5 [unity_SHAb]
Vector 4 [unity_SHAg]
Vector 3 [unity_SHAr]
Vector 8 [unity_SHBb]
Vector 7 [unity_SHBg]
Vector 6 [unity_SHBr]
Vector 9 [unity_SHC]
Vector 21 [unity_Scale]
// Shader Timing Estimate, in Cycles/64 vertex vector:
// ALU: 44.00 (33 instructions), vertex: 32, texture: 0,
//   sequencer: 18,  9 GPRs, 21 threads,
// Performance (if enough threads): ~44 cycles per vector
// * Vertex cycle estimates are assuming 3 vfetch_minis for every vfetch_full,
//     with <= 32 bytes per vfetch_full group.

"vs_360
backbbabaaaaaddaaaaaaceeaaaaaaaaaaaaaaceaaaaackaaaaaacmiaaaaaaaa
aaaaaaaaaaaaachiaaaaaabmaaaaacgkpppoadaaaaaaaabaaaaaaabmaaaaaaaa
aaaaacgdaaaaabfmaaacaabhaaabaaaaaaaaabgiaaaaaaaaaaaaabhiaaacaabg
aaabaaaaaaaaabgiaaaaaaaaaaaaabieaaacaaaoaaadaaaaaaaaabjeaaaaaaaa
aaaaabkeaaacaaabaaabaaaaaaaaabgiaaaaaaaaaaaaablgaaacaaacaaabaaaa
aaaaabgiaaaaaaaaaaaaabmeaaacaabbaaaeaaaaaaaaabjeaaaaaaaaaaaaabnc
aaacaaaaaaabaaaaaaaaaboiaaaaaaaaaaaaabpiaaacaaakaaaeaaaaaaaaabje
aaaaaaaaaaaaacalaaacaaafaaabaaaaaaaaabgiaaaaaaaaaaaaacbgaaacaaae
aaabaaaaaaaaabgiaaaaaaaaaaaaaccbaaacaaadaaabaaaaaaaaabgiaaaaaaaa
aaaaaccmaaacaaaiaaabaaaaaaaaabgiaaaaaaaaaaaaacdhaaacaaahaaabaaaa
aaaaabgiaaaaaaaaaaaaacecaaacaaagaaabaaaaaaaaabgiaaaaaaaaaaaaacen
aaacaaajaaabaaaaaaaaabgiaaaaaaaaaaaaacfhaaacaabfaaabaaaaaaaaabgi
aaaaaaaafpechfgnhaengbhafpfdfeaaaaabaaadaaabaaaeaaabaaaaaaaaaaaa
fpengbgjgofegfhifpfdfeaafpepgcgkgfgdhedcfhgphcgmgeaaklklaaadaaad
aaaeaaaeaaabaaaaaaaaaaaafpfahcgpgkgfgdhegjgpgofagbhcgbgnhdaafpfd
gdhcgfgfgofagbhcgbgnhdaafpfhgphcgmgedcepgcgkgfgdheaafpfhgphcgmge
fdhagbgdgfedgbgngfhcgbfagphdaaklaaabaaadaaabaaadaaabaaaaaaaaaaaa
ghgmhdhegbhegffpgngbhehcgjhifpgnhghaaahfgogjhehjfpfdeiebgcaahfgo
gjhehjfpfdeiebghaahfgogjhehjfpfdeiebhcaahfgogjhehjfpfdeiecgcaahf
gogjhehjfpfdeiecghaahfgogjhehjfpfdeiechcaahfgogjhehjfpfdeiedaahf
gogjhehjfpfdgdgbgmgfaahghdfpddfpdaaadccodacodcdadddfddcodaaaklkl
aaaaaaaaaaaaaaabaaaaaaaaaaaaaaaaaaaaaabeaapmaabaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaeaaaaaacaeaadbaaaiaaaaaaaaaaaaaaaaaaaadiie
aaaaaaabaaaaaaaeaaaaaaaiaaaaacjaaabaaaafaaaagaagaaaadaahaadafaai
aaaapafaaaachbfbaaafpcfcaaahhdfdaaaaaaboaaaababpaaaaaablaaaaaabm
aaaababnaaaaaabkaaaabachaaaabacjaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
dpaaaaaaaaaaaaaaaaaaaaaaaaaaaaaapaffeaafaaaabcaamcaaaaaaaaaafaaj
aaaabcaameaaaaaaaaaagaaogabebcaabcaaaaaaaaaagabkgacabcaabcaaaaaa
aaaaeacgaaaaccaaaaaaaaaaafpicaaaaaaaagiiaaaaaaaaafpigaaaaaaaagii
aaaaaaaaafpibaaaaaaaaeehaaaaaaaaafpiaaaaaaaaapmiaaaaaaaamiapaaad
aabliiaakbacanaamiapaaadaamgnapiklacamadmiapaaadaalbdepiklacalad
miapaaahaagmnajeklacakadmiapiadoaananaaaocahahaamiahaaadaamamgma
albdaabemiahaaafaamdgfaaobabagaamiahaaaiaalelbleclbcaaadmialaaad
aalkblaakbabbfaamiahaaaeaalbleaakbadbaaamiahaaaiaamagmleclbbaaai
miahaaafablklomaolabagafceihaeafaamablgmobafagiamiahaaacabmablma
klaibfacmiahaaadaagmlemakladapaemiahaaaeaabllemakladaoadaibhabad
aamagmggkbahppaemiamiaacaanlnlaaocahahaamiabiaabaaloloaapaacagaa
miaciaabaaloloaapaafacaamiaeiaabaalomdaapaacabaamiadiaaaaalalabk
ilaabgbgmiamiaaaaakmkmagilaabhbhaicbabacaadoanmbgpadaeaeaiecabac
aadoanlbgpaeaeaeaiieabacaadoanlmgpafaeaemiabaaaaaakhkhaakpabagaa
miacaaaaaakhkhaakpabahaaaibeabaaaakhkhgmkpabaiaeaiciabadaalbgmmg
kbadabaemiadiaacaamgbkbikladacadgeihaaaaaalologboaacaaabmiahiaad
aablmagfklaaajaaaaaaaaaaaaaaaaaaaaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
Matrix 256 [glstate_matrix_mvp]
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Vector 467 [_WorldSpaceCameraPos]
Vector 466 [_ProjectionParams]
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
Vector 456 [_BumpMap_ST]
"sce_vp_rsx // 40 instructions using 5 registers
[Configuration]
8
0000002841050500
[Defaults]
1
455 1
3f000000
[Microcode]
640
00011c6c00400e0c0106c0836041dffc00019c6c005d300c0186c0836041dffc
00001c6c009ca20c013fc0c36041dffc401f9c6c011c8800810040d560607f9c
401f9c6c011c9808010400d740619f9c00009c6c01d0300d8106c0c360403ffc
00009c6c01d0200d8106c0c360405ffc00009c6c01d0100d8106c0c360409ffc
00009c6c01d0000d8106c0c360411ffc00021c6c0150400c008600c360411ffc
00021c6c0150600c008600c360405ffc00001c6c0150500c008600c360403ffc
00001c6c0190a00c0686c0c360405ffc00001c6c0190900c0686c0c360409ffc
00001c6c0190800c0686c0c360411ffc00019c6c00800243011842436041dffc
00011c6c010002308121826301a1dffc401f9c6c0040000d8286c0836041ff80
401f9c6c004000558286c08360407fa400001c6c011ca00c00bfc0e30041dffc
00009c6c009c700e028000c36041dffc00009c6c009d202a828000c360409ffc
00009c6c0080007f80bfc04360403ffc00021c6c0040007f8086c08360409ffc
401f9c6c00c000080286c09540a19fa400011c6c00800e0c04bfc0836041dffc
401f9c6c0140020c0106004360405fa0401f9c6c01400e0c0086008360411fa0
00009c6c019cf00c0886c0c360405ffc00009c6c019d000c0886c0c360409ffc
00009c6c019d100c0886c0c360411ffc00001c6c010000000880047fe0a03ffc
00019c6c0080000d089a04436041fffc401f9c6c0140000c0086024360409fa0
00001c6c01dcc00d8686c0c360405ffc00001c6c01dcd00d8686c0c360409ffc
00001c6c01dce00d8686c0c360411ffc00001c6c00c0000c0286c0830021dffc
00009c6c009cb07f808600c36041dffc401f9c6c00c0000c0286c0830021dfa9
"
}

SubProgram "d3d11 " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 128 // 112 used size, 10 vars
Vector 80 [_MainTex_ST] 4
Vector 96 [_BumpMap_ST] 4
ConstBuffer "UnityPerCamera" 128 // 96 used size, 8 vars
Vector 64 [_WorldSpaceCameraPos] 3
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
Matrix 256 [_World2Object] 4
Vector 320 [unity_Scale] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityLighting" 2
BindCB "UnityPerDraw" 3
// 39 instructions, 4 temp regs, 0 temp arrays:
// ALU 22 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0
eefiecedjglbapihppffmfjgdankmlkbcgeampgpabaaaaaagaahaaaaadaaaaaa
cmaaaaaapeaaaaaajeabaaaaejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaa
abaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaljaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfc
enebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheojiaaaaaaafaaaaaa
aiaaaaaaiaaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaabaaaaaaapaaaaaaimaaaaaaabaaaaaaaaaaaaaa
adaaaaaaacaaaaaaahaiaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaa
apaaaaaaimaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaahaiaaaafdfgfpfa
epfdejfeejepeoaafeeffiedepepfceeaaklklklfdeieefcmeafaaaaeaaaabaa
hbabaaaafjaaaaaeegiocaaaaaaaaaaaahaaaaaafjaaaaaeegiocaaaabaaaaaa
agaaaaaafjaaaaaeegiocaaaacaaaaaabjaaaaaafjaaaaaeegiocaaaadaaaaaa
bfaaaaaafpaaaaadpcbabaaaaaaaaaaafpaaaaadpcbabaaaabaaaaaafpaaaaad
hcbabaaaacaaaaaafpaaaaaddcbabaaaadaaaaaaghaaaaaepccabaaaaaaaaaaa
abaaaaaagfaaaaadpccabaaaabaaaaaagfaaaaadhccabaaaacaaaaaagfaaaaad
pccabaaaadaaaaaagfaaaaadhccabaaaaeaaaaaagiaaaaacaeaaaaaadiaaaaai
pcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaadaaaaaaabaaaaaadcaaaaak
pcaabaaaaaaaaaaaegiocaaaadaaaaaaaaaaaaaaagbabaaaaaaaaaaaegaobaaa
aaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaacaaaaaakgbkbaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaa
adaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadgaaaaafpccabaaaaaaaaaaa
egaobaaaaaaaaaaadcaaaaaldccabaaaabaaaaaaegbabaaaadaaaaaaegiacaaa
aaaaaaaaafaaaaaaogikcaaaaaaaaaaaafaaaaaadcaaaaalmccabaaaabaaaaaa
agbebaaaadaaaaaaagiecaaaaaaaaaaaagaaaaaakgiocaaaaaaaaaaaagaaaaaa
diaaaaahhcaabaaaabaaaaaajgbebaaaabaaaaaacgbjbaaaacaaaaaadcaaaaak
hcaabaaaabaaaaaajgbebaaaacaaaaaacgbjbaaaabaaaaaaegacbaiaebaaaaaa
abaaaaaadiaaaaahhcaabaaaabaaaaaaegacbaaaabaaaaaapgbpbaaaabaaaaaa
diaaaaajhcaabaaaacaaaaaafgifcaaaabaaaaaaaeaaaaaaegiccaaaadaaaaaa
bbaaaaaadcaaaaalhcaabaaaacaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaa
abaaaaaaaeaaaaaaegacbaaaacaaaaaadcaaaaalhcaabaaaacaaaaaaegiccaaa
adaaaaaabcaaaaaakgikcaaaabaaaaaaaeaaaaaaegacbaaaacaaaaaaaaaaaaai
hcaabaaaacaaaaaaegacbaaaacaaaaaaegiccaaaadaaaaaabdaaaaaadcaaaaal
hcaabaaaacaaaaaaegacbaaaacaaaaaapgipcaaaadaaaaaabeaaaaaaegbcbaia
ebaaaaaaaaaaaaaabaaaaaahcccabaaaacaaaaaaegacbaaaabaaaaaaegacbaaa
acaaaaaabaaaaaahbccabaaaacaaaaaaegbcbaaaabaaaaaaegacbaaaacaaaaaa
baaaaaaheccabaaaacaaaaaaegbcbaaaacaaaaaaegacbaaaacaaaaaadiaaaaai
ccaabaaaaaaaaaaabkaabaaaaaaaaaaaakiacaaaabaaaaaaafaaaaaadiaaaaak
ncaabaaaabaaaaaaagahbaaaaaaaaaaaaceaaaaaaaaaaadpaaaaaaaaaaaaaadp
aaaaaadpdgaaaaafmccabaaaadaaaaaakgaobaaaaaaaaaaaaaaaaaahdccabaaa
adaaaaaakgakbaaaabaaaaaamgaabaaaabaaaaaadiaaaaaihcaabaaaaaaaaaaa
egbcbaaaacaaaaaapgipcaaaadaaaaaabeaaaaaadiaaaaaihcaabaaaabaaaaaa
fgafbaaaaaaaaaaaegiccaaaadaaaaaaanaaaaaadcaaaaaklcaabaaaaaaaaaaa
egiicaaaadaaaaaaamaaaaaaagaabaaaaaaaaaaaegaibaaaabaaaaaadcaaaaak
hcaabaaaaaaaaaaaegiccaaaadaaaaaaaoaaaaaakgakbaaaaaaaaaaaegadbaaa
aaaaaaaadgaaaaaficaabaaaaaaaaaaaabeaaaaaaaaaiadpbbaaaaaibcaabaaa
abaaaaaaegiocaaaacaaaaaabcaaaaaaegaobaaaaaaaaaaabbaaaaaiccaabaaa
abaaaaaaegiocaaaacaaaaaabdaaaaaaegaobaaaaaaaaaaabbaaaaaiecaabaaa
abaaaaaaegiocaaaacaaaaaabeaaaaaaegaobaaaaaaaaaaadiaaaaahpcaabaaa
acaaaaaajgacbaaaaaaaaaaaegakbaaaaaaaaaaabbaaaaaibcaabaaaadaaaaaa
egiocaaaacaaaaaabfaaaaaaegaobaaaacaaaaaabbaaaaaiccaabaaaadaaaaaa
egiocaaaacaaaaaabgaaaaaaegaobaaaacaaaaaabbaaaaaiecaabaaaadaaaaaa
egiocaaaacaaaaaabhaaaaaaegaobaaaacaaaaaaaaaaaaahhcaabaaaabaaaaaa
egacbaaaabaaaaaaegacbaaaadaaaaaadiaaaaahccaabaaaaaaaaaaabkaabaaa
aaaaaaaabkaabaaaaaaaaaaadcaaaaakbcaabaaaaaaaaaaaakaabaaaaaaaaaaa
akaabaaaaaaaaaaabkaabaiaebaaaaaaaaaaaaaadcaaaaakhccabaaaaeaaaaaa
egiccaaaacaaaaaabiaaaaaaagaabaaaaaaaaaaaegacbaaaabaaaaaadoaaaaab
"
}

SubProgram "gles " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
"!!GLES
#define SHADER_API_GLES 1
#define tex2D texture2D


#ifdef VERTEX
#define gl_ModelViewProjectionMatrix glstate_matrix_mvp
uniform mat4 glstate_matrix_mvp;

varying highp vec3 xlv_TEXCOORD3;
varying highp vec4 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 _BumpMap_ST;
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
  highp vec4 tmpvar_3;
  highp vec3 tmpvar_4;
  highp vec4 tmpvar_5;
  tmpvar_5 = (gl_ModelViewProjectionMatrix * _glesVertex);
  tmpvar_3.xy = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  tmpvar_3.zw = ((_glesMultiTexCoord0.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
  highp vec4 o_6;
  highp vec4 tmpvar_7;
  tmpvar_7 = (tmpvar_5 * 0.5);
  highp vec2 tmpvar_8;
  tmpvar_8.x = tmpvar_7.x;
  tmpvar_8.y = (tmpvar_7.y * _ProjectionParams.x);
  o_6.xy = (tmpvar_8 + tmpvar_7.w);
  o_6.zw = tmpvar_5.zw;
  mat3 tmpvar_9;
  tmpvar_9[0] = _Object2World[0].xyz;
  tmpvar_9[1] = _Object2World[1].xyz;
  tmpvar_9[2] = _Object2World[2].xyz;
  highp vec4 tmpvar_10;
  tmpvar_10.w = 1.0;
  tmpvar_10.xyz = (tmpvar_9 * (tmpvar_2 * unity_Scale.w));
  mediump vec3 tmpvar_11;
  mediump vec4 normal_12;
  normal_12 = tmpvar_10;
  highp float vC_13;
  mediump vec3 x3_14;
  mediump vec3 x2_15;
  mediump vec3 x1_16;
  highp float tmpvar_17;
  tmpvar_17 = dot (unity_SHAr, normal_12);
  x1_16.x = tmpvar_17;
  highp float tmpvar_18;
  tmpvar_18 = dot (unity_SHAg, normal_12);
  x1_16.y = tmpvar_18;
  highp float tmpvar_19;
  tmpvar_19 = dot (unity_SHAb, normal_12);
  x1_16.z = tmpvar_19;
  mediump vec4 tmpvar_20;
  tmpvar_20 = (normal_12.xyzz * normal_12.yzzx);
  highp float tmpvar_21;
  tmpvar_21 = dot (unity_SHBr, tmpvar_20);
  x2_15.x = tmpvar_21;
  highp float tmpvar_22;
  tmpvar_22 = dot (unity_SHBg, tmpvar_20);
  x2_15.y = tmpvar_22;
  highp float tmpvar_23;
  tmpvar_23 = dot (unity_SHBb, tmpvar_20);
  x2_15.z = tmpvar_23;
  mediump float tmpvar_24;
  tmpvar_24 = ((normal_12.x * normal_12.x) - (normal_12.y * normal_12.y));
  vC_13 = tmpvar_24;
  highp vec3 tmpvar_25;
  tmpvar_25 = (unity_SHC.xyz * vC_13);
  x3_14 = tmpvar_25;
  tmpvar_11 = ((x1_16 + x2_15) + x3_14);
  tmpvar_4 = tmpvar_11;
  highp vec3 tmpvar_26;
  highp vec3 tmpvar_27;
  tmpvar_26 = tmpvar_1.xyz;
  tmpvar_27 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_28;
  tmpvar_28[0].x = tmpvar_26.x;
  tmpvar_28[0].y = tmpvar_27.x;
  tmpvar_28[0].z = tmpvar_2.x;
  tmpvar_28[1].x = tmpvar_26.y;
  tmpvar_28[1].y = tmpvar_27.y;
  tmpvar_28[1].z = tmpvar_2.y;
  tmpvar_28[2].x = tmpvar_26.z;
  tmpvar_28[2].y = tmpvar_27.z;
  tmpvar_28[2].z = tmpvar_2.z;
  highp vec4 tmpvar_29;
  tmpvar_29.w = 1.0;
  tmpvar_29.xyz = _WorldSpaceCameraPos;
  gl_Position = tmpvar_5;
  xlv_TEXCOORD0 = tmpvar_3;
  xlv_TEXCOORD1 = (tmpvar_28 * (((_World2Object * tmpvar_29).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = o_6;
  xlv_TEXCOORD3 = tmpvar_4;
}



#endif
#ifdef FRAGMENT

varying highp vec3 xlv_TEXCOORD3;
varying highp vec4 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform sampler2D _LightBuffer;
uniform highp float _Parallax;
uniform mediump float _Gloss;
uniform lowp vec4 _Color;
uniform sampler2D _ParallaxMap;
uniform sampler2D _SpecMap;
uniform sampler2D _MainTex;
uniform lowp vec4 _SpecColor;
void main ()
{
  lowp vec4 tmpvar_1;
  mediump vec4 light_2;
  mediump vec3 tmpvar_3;
  mediump vec4 spec_4;
  mediump float h_5;
  lowp float tmpvar_6;
  tmpvar_6 = texture2D (_ParallaxMap, xlv_TEXCOORD0.zw).w;
  h_5 = tmpvar_6;
  mediump float height_7;
  height_7 = _Parallax;
  mediump vec3 viewDir_8;
  viewDir_8 = xlv_TEXCOORD1;
  highp vec3 v_9;
  mediump float tmpvar_10;
  tmpvar_10 = ((h_5 * height_7) - (height_7 / 2.0));
  mediump vec3 tmpvar_11;
  tmpvar_11 = normalize(viewDir_8);
  v_9 = tmpvar_11;
  v_9.z = (v_9.z + 0.42);
  highp vec2 tmpvar_12;
  tmpvar_12 = (xlv_TEXCOORD0.xy + (tmpvar_10 * (v_9.xy / v_9.z)));
  lowp vec4 tmpvar_13;
  tmpvar_13 = texture2D (_MainTex, tmpvar_12);
  lowp vec3 tmpvar_14;
  tmpvar_14 = (tmpvar_13.xyz * _Color.xyz);
  tmpvar_3 = tmpvar_14;
  lowp vec4 tmpvar_15;
  tmpvar_15 = texture2D (_SpecMap, tmpvar_12);
  spec_4 = tmpvar_15;
  lowp vec4 tmpvar_16;
  tmpvar_16 = texture2DProj (_LightBuffer, xlv_TEXCOORD2);
  light_2 = tmpvar_16;
  mediump vec4 tmpvar_17;
  tmpvar_17 = max (light_2, vec4(0.001, 0.001, 0.001, 0.001));
  light_2.w = tmpvar_17.w;
  highp vec3 tmpvar_18;
  tmpvar_18 = (tmpvar_17.xyz + xlv_TEXCOORD3);
  light_2.xyz = tmpvar_18;
  mediump vec4 c_19;
  mediump vec3 tmpvar_20;
  tmpvar_20 = (tmpvar_17.w * ((tmpvar_13.w * spec_4.xyz) * _Gloss));
  c_19.xyz = ((tmpvar_3 * light_2.xyz) + (light_2.xyz * tmpvar_20));
  c_19.w = (tmpvar_20 * _SpecColor.w).x;
  tmpvar_1 = c_19;
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

varying highp vec3 xlv_TEXCOORD3;
varying highp vec4 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 _BumpMap_ST;
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
  highp vec4 tmpvar_3;
  highp vec3 tmpvar_4;
  highp vec4 tmpvar_5;
  tmpvar_5 = (gl_ModelViewProjectionMatrix * _glesVertex);
  tmpvar_3.xy = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  tmpvar_3.zw = ((_glesMultiTexCoord0.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
  highp vec4 o_6;
  highp vec4 tmpvar_7;
  tmpvar_7 = (tmpvar_5 * 0.5);
  highp vec2 tmpvar_8;
  tmpvar_8.x = tmpvar_7.x;
  tmpvar_8.y = (tmpvar_7.y * _ProjectionParams.x);
  o_6.xy = (tmpvar_8 + tmpvar_7.w);
  o_6.zw = tmpvar_5.zw;
  mat3 tmpvar_9;
  tmpvar_9[0] = _Object2World[0].xyz;
  tmpvar_9[1] = _Object2World[1].xyz;
  tmpvar_9[2] = _Object2World[2].xyz;
  highp vec4 tmpvar_10;
  tmpvar_10.w = 1.0;
  tmpvar_10.xyz = (tmpvar_9 * (tmpvar_2 * unity_Scale.w));
  mediump vec3 tmpvar_11;
  mediump vec4 normal_12;
  normal_12 = tmpvar_10;
  highp float vC_13;
  mediump vec3 x3_14;
  mediump vec3 x2_15;
  mediump vec3 x1_16;
  highp float tmpvar_17;
  tmpvar_17 = dot (unity_SHAr, normal_12);
  x1_16.x = tmpvar_17;
  highp float tmpvar_18;
  tmpvar_18 = dot (unity_SHAg, normal_12);
  x1_16.y = tmpvar_18;
  highp float tmpvar_19;
  tmpvar_19 = dot (unity_SHAb, normal_12);
  x1_16.z = tmpvar_19;
  mediump vec4 tmpvar_20;
  tmpvar_20 = (normal_12.xyzz * normal_12.yzzx);
  highp float tmpvar_21;
  tmpvar_21 = dot (unity_SHBr, tmpvar_20);
  x2_15.x = tmpvar_21;
  highp float tmpvar_22;
  tmpvar_22 = dot (unity_SHBg, tmpvar_20);
  x2_15.y = tmpvar_22;
  highp float tmpvar_23;
  tmpvar_23 = dot (unity_SHBb, tmpvar_20);
  x2_15.z = tmpvar_23;
  mediump float tmpvar_24;
  tmpvar_24 = ((normal_12.x * normal_12.x) - (normal_12.y * normal_12.y));
  vC_13 = tmpvar_24;
  highp vec3 tmpvar_25;
  tmpvar_25 = (unity_SHC.xyz * vC_13);
  x3_14 = tmpvar_25;
  tmpvar_11 = ((x1_16 + x2_15) + x3_14);
  tmpvar_4 = tmpvar_11;
  highp vec3 tmpvar_26;
  highp vec3 tmpvar_27;
  tmpvar_26 = tmpvar_1.xyz;
  tmpvar_27 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_28;
  tmpvar_28[0].x = tmpvar_26.x;
  tmpvar_28[0].y = tmpvar_27.x;
  tmpvar_28[0].z = tmpvar_2.x;
  tmpvar_28[1].x = tmpvar_26.y;
  tmpvar_28[1].y = tmpvar_27.y;
  tmpvar_28[1].z = tmpvar_2.y;
  tmpvar_28[2].x = tmpvar_26.z;
  tmpvar_28[2].y = tmpvar_27.z;
  tmpvar_28[2].z = tmpvar_2.z;
  highp vec4 tmpvar_29;
  tmpvar_29.w = 1.0;
  tmpvar_29.xyz = _WorldSpaceCameraPos;
  gl_Position = tmpvar_5;
  xlv_TEXCOORD0 = tmpvar_3;
  xlv_TEXCOORD1 = (tmpvar_28 * (((_World2Object * tmpvar_29).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = o_6;
  xlv_TEXCOORD3 = tmpvar_4;
}



#endif
#ifdef FRAGMENT

varying highp vec3 xlv_TEXCOORD3;
varying highp vec4 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform sampler2D _LightBuffer;
uniform highp float _Parallax;
uniform mediump float _Gloss;
uniform lowp vec4 _Color;
uniform sampler2D _ParallaxMap;
uniform sampler2D _SpecMap;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform lowp vec4 _SpecColor;
void main ()
{
  lowp vec4 tmpvar_1;
  mediump vec4 light_2;
  mediump vec3 tmpvar_3;
  mediump vec4 spec_4;
  mediump float h_5;
  lowp float tmpvar_6;
  tmpvar_6 = texture2D (_ParallaxMap, xlv_TEXCOORD0.zw).w;
  h_5 = tmpvar_6;
  highp vec2 tmpvar_7;
  mediump float height_8;
  height_8 = _Parallax;
  mediump vec3 viewDir_9;
  viewDir_9 = xlv_TEXCOORD1;
  highp vec3 v_10;
  mediump float tmpvar_11;
  tmpvar_11 = ((h_5 * height_8) - (height_8 / 2.0));
  mediump vec3 tmpvar_12;
  tmpvar_12 = normalize(viewDir_9);
  v_10 = tmpvar_12;
  v_10.z = (v_10.z + 0.42);
  tmpvar_7 = (tmpvar_11 * (v_10.xy / v_10.z));
  highp vec2 tmpvar_13;
  tmpvar_13 = (xlv_TEXCOORD0.xy + tmpvar_7);
  highp vec2 tmpvar_14;
  tmpvar_14 = (xlv_TEXCOORD0.zw + tmpvar_7);
  lowp vec4 tmpvar_15;
  tmpvar_15 = texture2D (_MainTex, tmpvar_13);
  lowp vec3 tmpvar_16;
  tmpvar_16 = (tmpvar_15.xyz * _Color.xyz);
  tmpvar_3 = tmpvar_16;
  lowp vec4 tmpvar_17;
  tmpvar_17 = texture2D (_SpecMap, tmpvar_13);
  spec_4 = tmpvar_17;
  lowp vec3 normal_18;
  normal_18.xy = ((texture2D (_BumpMap, tmpvar_14).wy * 2.0) - 1.0);
  normal_18.z = sqrt(((1.0 - (normal_18.x * normal_18.x)) - (normal_18.y * normal_18.y)));
  lowp vec4 tmpvar_19;
  tmpvar_19 = texture2DProj (_LightBuffer, xlv_TEXCOORD2);
  light_2 = tmpvar_19;
  mediump vec4 tmpvar_20;
  tmpvar_20 = max (light_2, vec4(0.001, 0.001, 0.001, 0.001));
  light_2.w = tmpvar_20.w;
  highp vec3 tmpvar_21;
  tmpvar_21 = (tmpvar_20.xyz + xlv_TEXCOORD3);
  light_2.xyz = tmpvar_21;
  mediump vec4 c_22;
  mediump vec3 tmpvar_23;
  tmpvar_23 = (tmpvar_20.w * ((tmpvar_15.w * spec_4.xyz) * _Gloss));
  c_22.xyz = ((tmpvar_3 * light_2.xyz) + (light_2.xyz * tmpvar_23));
  c_22.w = (tmpvar_23 * _SpecColor.w).x;
  tmpvar_1 = c_22;
  gl_FragData[0] = tmpvar_1;
}



#endif"
}

SubProgram "d3d11_9x " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 128 // 112 used size, 10 vars
Vector 80 [_MainTex_ST] 4
Vector 96 [_BumpMap_ST] 4
ConstBuffer "UnityPerCamera" 128 // 96 used size, 8 vars
Vector 64 [_WorldSpaceCameraPos] 3
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
Matrix 256 [_World2Object] 4
Vector 320 [unity_Scale] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityLighting" 2
BindCB "UnityPerDraw" 3
// 39 instructions, 4 temp regs, 0 temp arrays:
// ALU 22 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0_level_9_3
eefiecedejebmalhedjmpniplopmdohjpieonibkabaaaaaapaakaaaaaeaaaaaa
daaaaaaalmadaaaaiiajaaaafaakaaaaebgpgodjieadaaaaieadaaaaaaacpopp
beadaaaahaaaaaaaagaaceaaaaaagmaaaaaagmaaaaaaceaaabaagmaaaaaaafaa
acaaabaaaaaaaaaaabaaaeaaacaaadaaaaaaaaaaacaabcaaahaaafaaaaaaaaaa
adaaaaaaaeaaamaaaaaaaaaaadaaamaaadaabaaaaaaaaaaaadaabaaaafaabdaa
aaaaaaaaaaaaaaaaabacpoppfbaaaaafbiaaapkaaaaaaadpaaaaiadpaaaaaaaa
aaaaaaaabpaaaaacafaaaaiaaaaaapjabpaaaaacafaaabiaabaaapjabpaaaaac
afaaaciaacaaapjabpaaaaacafaaadiaadaaapjaaeaaaaaeaaaaadoaadaaoeja
abaaoekaabaaookaaeaaaaaeaaaaamoaadaaeejaacaaeekaacaaoekaafaaaaad
aaaaapiaaaaaffjaanaaoekaaeaaaaaeaaaaapiaamaaoekaaaaaaajaaaaaoeia
aeaaaaaeaaaaapiaaoaaoekaaaaakkjaaaaaoeiaaeaaaaaeaaaaapiaapaaoeka
aaaappjaaaaaoeiaafaaaaadabaaabiaaaaaffiaaeaaaakaafaaaaadabaaaiia
abaaaaiabiaaaakaafaaaaadabaaafiaaaaapeiabiaaaakaacaaaaadacaaadoa
abaakkiaabaaomiaafaaaaadabaaahiaacaaoejabhaappkaafaaaaadacaaahia
abaaffiabbaaoekaaeaaaaaeabaaaliabaaakekaabaaaaiaacaakeiaaeaaaaae
abaaahiabcaaoekaabaakkiaabaapeiaabaaaaacabaaaiiabiaaffkaajaaaaad
acaaabiaafaaoekaabaaoeiaajaaaaadacaaaciaagaaoekaabaaoeiaajaaaaad
acaaaeiaahaaoekaabaaoeiaafaaaaadadaaapiaabaacjiaabaakeiaajaaaaad
aeaaabiaaiaaoekaadaaoeiaajaaaaadaeaaaciaajaaoekaadaaoeiaajaaaaad
aeaaaeiaakaaoekaadaaoeiaacaaaaadacaaahiaacaaoeiaaeaaoeiaafaaaaad
abaaaciaabaaffiaabaaffiaaeaaaaaeabaaabiaabaaaaiaabaaaaiaabaaffib
aeaaaaaeadaaahoaalaaoekaabaaaaiaacaaoeiaabaaaaacabaaahiaadaaoeka
afaaaaadacaaahiaabaaffiabeaaoekaaeaaaaaeabaaaliabdaakekaabaaaaia
acaakeiaaeaaaaaeabaaahiabfaaoekaabaakkiaabaapeiaacaaaaadabaaahia
abaaoeiabgaaoekaaeaaaaaeabaaahiaabaaoeiabhaappkaaaaaoejbaiaaaaad
abaaaboaabaaoejaabaaoeiaabaaaaacacaaahiaacaaoejaafaaaaadadaaahia
acaanciaabaamjjaaeaaaaaeacaaahiaacaamjiaabaancjaadaaoeibafaaaaad
acaaahiaacaaoeiaabaappjaaiaaaaadabaaacoaacaaoeiaabaaoeiaaiaaaaad
abaaaeoaacaaoejaabaaoeiaaeaaaaaeaaaaadmaaaaappiaaaaaoekaaaaaoeia
abaaaaacaaaaammaaaaaoeiaabaaaaacacaaamoaaaaaoeiappppaaaafdeieefc
meafaaaaeaaaabaahbabaaaafjaaaaaeegiocaaaaaaaaaaaahaaaaaafjaaaaae
egiocaaaabaaaaaaagaaaaaafjaaaaaeegiocaaaacaaaaaabjaaaaaafjaaaaae
egiocaaaadaaaaaabfaaaaaafpaaaaadpcbabaaaaaaaaaaafpaaaaadpcbabaaa
abaaaaaafpaaaaadhcbabaaaacaaaaaafpaaaaaddcbabaaaadaaaaaaghaaaaae
pccabaaaaaaaaaaaabaaaaaagfaaaaadpccabaaaabaaaaaagfaaaaadhccabaaa
acaaaaaagfaaaaadpccabaaaadaaaaaagfaaaaadhccabaaaaeaaaaaagiaaaaac
aeaaaaaadiaaaaaipcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaadaaaaaa
abaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaaaaaaaaaagbabaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaa
acaaaaaakgbkbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaa
egiocaaaadaaaaaaadaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadgaaaaaf
pccabaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaaldccabaaaabaaaaaaegbabaaa
adaaaaaaegiacaaaaaaaaaaaafaaaaaaogikcaaaaaaaaaaaafaaaaaadcaaaaal
mccabaaaabaaaaaaagbebaaaadaaaaaaagiecaaaaaaaaaaaagaaaaaakgiocaaa
aaaaaaaaagaaaaaadiaaaaahhcaabaaaabaaaaaajgbebaaaabaaaaaacgbjbaaa
acaaaaaadcaaaaakhcaabaaaabaaaaaajgbebaaaacaaaaaacgbjbaaaabaaaaaa
egacbaiaebaaaaaaabaaaaaadiaaaaahhcaabaaaabaaaaaaegacbaaaabaaaaaa
pgbpbaaaabaaaaaadiaaaaajhcaabaaaacaaaaaafgifcaaaabaaaaaaaeaaaaaa
egiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaaacaaaaaaegiccaaaadaaaaaa
baaaaaaaagiacaaaabaaaaaaaeaaaaaaegacbaaaacaaaaaadcaaaaalhcaabaaa
acaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaaabaaaaaaaeaaaaaaegacbaaa
acaaaaaaaaaaaaaihcaabaaaacaaaaaaegacbaaaacaaaaaaegiccaaaadaaaaaa
bdaaaaaadcaaaaalhcaabaaaacaaaaaaegacbaaaacaaaaaapgipcaaaadaaaaaa
beaaaaaaegbcbaiaebaaaaaaaaaaaaaabaaaaaahcccabaaaacaaaaaaegacbaaa
abaaaaaaegacbaaaacaaaaaabaaaaaahbccabaaaacaaaaaaegbcbaaaabaaaaaa
egacbaaaacaaaaaabaaaaaaheccabaaaacaaaaaaegbcbaaaacaaaaaaegacbaaa
acaaaaaadiaaaaaiccaabaaaaaaaaaaabkaabaaaaaaaaaaaakiacaaaabaaaaaa
afaaaaaadiaaaaakncaabaaaabaaaaaaagahbaaaaaaaaaaaaceaaaaaaaaaaadp
aaaaaaaaaaaaaadpaaaaaadpdgaaaaafmccabaaaadaaaaaakgaobaaaaaaaaaaa
aaaaaaahdccabaaaadaaaaaakgakbaaaabaaaaaamgaabaaaabaaaaaadiaaaaai
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
hccabaaaaeaaaaaaegiccaaaacaaaaaabiaaaaaaagaabaaaaaaaaaaaegacbaaa
abaaaaaadoaaaaabejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaaadaaaaaa
abaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaaahahaaaa
laaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaaabaaaaaa
aaaaaaaaadaaaaaaaeaaaaaaapaaaaaaljaaaaaaaaaaaaaaaaaaaaaaadaaaaaa
afaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfcenebemaa
feeffiedepepfceeaaedepemepfcaaklepfdeheojiaaaaaaafaaaaaaaiaaaaaa
iaaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaabaaaaaaapaaaaaaimaaaaaaabaaaaaaaaaaaaaaadaaaaaa
acaaaaaaahaiaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaapaaaaaa
imaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaahaiaaaafdfgfpfaepfdejfe
ejepeoaafeeffiedepepfceeaaklklkl"
}

SubProgram "opengl " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
Bind "vertex" Vertex
Bind "tangent" ATTR14
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Vector 17 [_WorldSpaceCameraPos]
Vector 18 [_ProjectionParams]
Vector 19 [unity_ShadowFadeCenterAndType]
Matrix 9 [_Object2World]
Matrix 13 [_World2Object]
Vector 20 [unity_Scale]
Vector 21 [unity_LightmapST]
Vector 22 [_MainTex_ST]
Vector 23 [_BumpMap_ST]
"!!ARBvp1.0
# 34 ALU
PARAM c[24] = { { 1, 0.5 },
		state.matrix.modelview[0],
		state.matrix.mvp,
		program.local[9..23] };
TEMP R0;
TEMP R1;
TEMP R2;
MOV R0.xyz, vertex.attrib[14];
MUL R1.xyz, vertex.normal.zxyw, R0.yzxw;
MAD R0.xyz, vertex.normal.yzxw, R0.zxyw, -R1;
MUL R0.xyz, R0, vertex.attrib[14].w;
MOV R1.xyz, c[17];
MOV R1.w, c[0].x;
DP4 R0.w, vertex.position, c[8];
DP4 R2.z, R1, c[15];
DP4 R2.x, R1, c[13];
DP4 R2.y, R1, c[14];
MAD R2.xyz, R2, c[20].w, -vertex.position;
DP3 result.texcoord[1].y, R2, R0;
DP4 R0.z, vertex.position, c[7];
DP4 R0.x, vertex.position, c[5];
DP4 R0.y, vertex.position, c[6];
MUL R1.xyz, R0.xyww, c[0].y;
MUL R1.y, R1, c[18].x;
ADD result.texcoord[2].xy, R1, R1.z;
MOV result.position, R0;
MOV R0.x, c[0];
ADD R0.y, R0.x, -c[19].w;
DP4 R0.x, vertex.position, c[3];
DP4 R1.z, vertex.position, c[11];
DP4 R1.x, vertex.position, c[9];
DP4 R1.y, vertex.position, c[10];
ADD R1.xyz, R1, -c[19];
DP3 result.texcoord[1].z, vertex.normal, R2;
DP3 result.texcoord[1].x, R2, vertex.attrib[14];
MOV result.texcoord[2].zw, R0;
MUL result.texcoord[4].xyz, R1, c[19].w;
MAD result.texcoord[0].zw, vertex.texcoord[0].xyxy, c[23].xyxy, c[23];
MAD result.texcoord[0].xy, vertex.texcoord[0], c[22], c[22].zwzw;
MAD result.texcoord[3].xy, vertex.texcoord[1], c[21], c[21].zwzw;
MUL result.texcoord[4].w, -R0.x, R0.y;
END
# 34 instructions, 3 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Matrix 0 [glstate_matrix_modelview0]
Matrix 4 [glstate_matrix_mvp]
Vector 16 [_WorldSpaceCameraPos]
Vector 17 [_ProjectionParams]
Vector 18 [_ScreenParams]
Vector 19 [unity_ShadowFadeCenterAndType]
Matrix 8 [_Object2World]
Matrix 12 [_World2Object]
Vector 20 [unity_Scale]
Vector 21 [unity_LightmapST]
Vector 22 [_MainTex_ST]
Vector 23 [_BumpMap_ST]
"vs_2_0
; 35 ALU
def c24, 1.00000000, 0.50000000, 0, 0
dcl_position0 v0
dcl_tangent0 v1
dcl_normal0 v2
dcl_texcoord0 v3
dcl_texcoord1 v4
mov r0.xyz, v1
mul r1.xyz, v2.zxyw, r0.yzxw
mov r0.xyz, v1
mad r0.xyz, v2.yzxw, r0.zxyw, -r1
mul r0.xyz, r0, v1.w
mov r1.xyz, c16
mov r1.w, c24.x
dp4 r0.w, v0, c7
dp4 r2.z, r1, c14
dp4 r2.x, r1, c12
dp4 r2.y, r1, c13
mad r2.xyz, r2, c20.w, -v0
dp3 oT1.y, r2, r0
dp4 r0.z, v0, c6
dp4 r0.x, v0, c4
dp4 r0.y, v0, c5
mul r1.xyz, r0.xyww, c24.y
mul r1.y, r1, c17.x
mad oT2.xy, r1.z, c18.zwzw, r1
mov oPos, r0
mov r0.x, c19.w
add r0.y, c24.x, -r0.x
dp4 r0.x, v0, c2
dp4 r1.z, v0, c10
dp4 r1.x, v0, c8
dp4 r1.y, v0, c9
add r1.xyz, r1, -c19
dp3 oT1.z, v2, r2
dp3 oT1.x, r2, v1
mov oT2.zw, r0
mul oT4.xyz, r1, c19.w
mad oT0.zw, v3.xyxy, c23.xyxy, c23
mad oT0.xy, v3, c22, c22.zwzw
mad oT3.xy, v4, c21, c21.zwzw
mul oT4.w, -r0.x, r0.y
"
}

SubProgram "xbox360 " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Vector 23 [_BumpMap_ST]
Vector 22 [_MainTex_ST]
Matrix 12 [_Object2World] 4
Vector 1 [_ProjectionParams]
Vector 2 [_ScreenParams]
Matrix 16 [_World2Object] 4
Vector 0 [_WorldSpaceCameraPos]
Matrix 8 [glstate_matrix_modelview0] 4
Matrix 4 [glstate_matrix_mvp] 4
Vector 21 [unity_LightmapST]
Vector 20 [unity_Scale]
Vector 3 [unity_ShadowFadeCenterAndType]
// Shader Timing Estimate, in Cycles/64 vertex vector:
// ALU: 44.00 (33 instructions), vertex: 64, texture: 0,
//   sequencer: 18,  11 GPRs, 15 threads,
// Performance (if enough threads): ~64 cycles per vector
// * Vertex cycle estimates are assuming 3 vfetch_minis for every vfetch_full,
//     with <= 32 bytes per vfetch_full group.

"vs_360
backbbabaaaaacomaaaaacfaaaaaaaaaaaaaaaceaaaaacemaaaaacheaaaaaaaa
aaaaaaaaaaaaacceaaaaaabmaaaaacbhpppoadaaaaaaaaamaaaaaabmaaaaaaaa
aaaaacbaaaaaabamaaacaabhaaabaaaaaaaaabbiaaaaaaaaaaaaabciaaacaabg
aaabaaaaaaaaabbiaaaaaaaaaaaaabdeaaacaaamaaaeaaaaaaaaabeeaaaaaaaa
aaaaabfeaaacaaabaaabaaaaaaaaabbiaaaaaaaaaaaaabggaaacaaacaaabaaaa
aaaaabbiaaaaaaaaaaaaabheaaacaabaaaaeaaaaaaaaabeeaaaaaaaaaaaaabic
aaacaaaaaaabaaaaaaaaabjiaaaaaaaaaaaaabkiaaacaaaiaaaeaaaaaaaaabee
aaaaaaaaaaaaabmcaaacaaaeaaaeaaaaaaaaabeeaaaaaaaaaaaaabnfaaacaabf
aaabaaaaaaaaabbiaaaaaaaaaaaaabogaaacaabeaaabaaaaaaaaabbiaaaaaaaa
aaaaabpcaaacaaadaaabaaaaaaaaabbiaaaaaaaafpechfgnhaengbhafpfdfeaa
aaabaaadaaabaaaeaaabaaaaaaaaaaaafpengbgjgofegfhifpfdfeaafpepgcgk
gfgdhedcfhgphcgmgeaaklklaaadaaadaaaeaaaeaaabaaaaaaaaaaaafpfahcgp
gkgfgdhegjgpgofagbhcgbgnhdaafpfdgdhcgfgfgofagbhcgbgnhdaafpfhgphc
gmgedcepgcgkgfgdheaafpfhgphcgmgefdhagbgdgfedgbgngfhcgbfagphdaakl
aaabaaadaaabaaadaaabaaaaaaaaaaaaghgmhdhegbhegffpgngbhehcgjhifpgn
gpgegfgmhggjgfhhdaaaghgmhdhegbhegffpgngbhehcgjhifpgnhghaaahfgogj
hehjfpemgjghgihegngbhafdfeaahfgogjhehjfpfdgdgbgmgfaahfgogjhehjfp
fdgigbgegphheggbgegfedgfgohegfhcebgogefehjhagfaahghdfpddfpdaaadc
codacodcdadddfddcodaaaklaaaaaaaaaaaaaaabaaaaaaaaaaaaaaaaaaaaaabe
aapmaabaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaeaaaaaacbaaaebaaak
aaaaaaaaaaaaaaaaaaaaeekfaaaaaaabaaaaaaafaaaaaaakaaaaacjaaabaaaaf
aaaagaagaaaadaahaaaafaaiaacbfaajaaaapafaaaachbfbaaafpcfcaaahddfd
aaaipefeaaaaaaboaaaababpaaaaaaciaaaaaacjaaaabackaaaaaabmaaaabacd
aaaababnaaaaaaccaaaabacfaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaadpiaaaaa
dpaaaaaaaaaaaaaaaaaaaaaapbfffaafaaaabcabmcaaaaaaaaaafaakaaaabcaa
meaaaaaaaaaagaapgabfbcaabcaaaaaaaaaagablgacbbcaabcaaaaaaaaaaeach
aaaaccaaaaaaaaaaafpibaaaaaaaaeedaaaaaaaaafpieaaaaaaaagiiaaaaaaaa
afpicaaaaaaaaoiiaaaaaaaaafpiiaaaaaaaapmiaaaaaaaaafpiiaaaaaaaacdp
aaaaaaaakmipafaaaagmiimaibabahalmiapaaaaaablnapiklabagaamiapaaaa
aamgdepiklabafaamiapaaajaalbnajeklabaeaamiapiadoaananaaaocajajaa
miaiaaacaeblgmaacaadppaamiahaaaaaalelbaacbbbaaaamiahaaagaamagmle
clbaaaaabebhaaafaalemglbcbbcaaabkiiiakahaamgmgmaibabajaibecnaaaa
aabliegmkbabaoabkibhahadaalogfebmbacaeapmiahaaaaaamglebeklabanaa
miahaaadabgflomaolacaeadkmchahadaamablmambadaeapmiahaaakaalblema
klabamaakmehahaaaamalbiaibajppapmiapaaahaadedeaaoaakahaamiamiaac
aanlnlaaocajajaamiadiaadaabklabkilaibfbfmiadiaaaaalalabkilaibgbg
miamiaaaaakmkmagilaibhbhmiaiaaagaablmgblklabakahkiihaaahacmamaeb
iaahadabmiahiaaeaamablaakbahadaamiadiaacaamgbkbiklaaacaamiapaaaa
aadeaaaaoaagafaamiaiiaaeaeblblaaobaaacaamiahaaaaaalemaaakaaabdaa
miahaaaaabmablbfklaabeabmiabiaabaaloloaapaaaaeaamiaciaabaaloloaa
paadaaaamiaeiaabaaloloaapaaaacaaaaaaaaaaaaaaaaaaaaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
Matrix 256 [glstate_matrix_modelview0]
Matrix 260 [glstate_matrix_mvp]
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Vector 467 [_WorldSpaceCameraPos]
Vector 466 [_ProjectionParams]
Vector 465 [unity_ShadowFadeCenterAndType]
Matrix 264 [_Object2World]
Matrix 268 [_World2Object]
Vector 464 [unity_Scale]
Vector 463 [unity_LightmapST]
Vector 462 [_MainTex_ST]
Vector 461 [_BumpMap_ST]
"sce_vp_rsx // 33 instructions using 5 registers
[Configuration]
8
0000002143050500
[Defaults]
1
460 2
3f0000003f800000
[Microcode]
528
00011c6c00400e0c0106c0836041dffc00021c6c005d300c0186c0836041dffc
401f9c6c011cd800810040d560607f9c401f9c6c011ce808010400d740619f9c
401f9c6c011cf908010400d740619fa800001c6c01d0200d8106c0c360403ffc
00009c6c01d0700d8106c0c360403ffc00009c6c01d0600d8106c0c360405ffc
00009c6c01d0500d8106c0c360409ffc00009c6c01d0400d8106c0c360411ffc
00011c6c005d107f8186c08360403ffc00019c6c01d0a00d8106c0c360405ffc
00019c6c01d0900d8106c0c360409ffc00019c6c01d0800d8106c0c360411ffc
00001c6c0190e00c0886c0c360405ffc00001c6c0190d00c0886c0c360409ffc
00001c6c0190c00c0886c0c360411ffc00021c6c00dd108c0186c08301a1dffc
00019c6c00800243011842436041dffc00011c6c00dcc02a8186c0bfe1203ffc
00011c6c010002308121826301a1dffc401f9c6c0040000d8286c0836041ff80
401f9c6c004000558286c08360407fa400001c6c011d000c00bfc0e30041dffc
00009c6c009cc00e028000c36041dffc401f9c6c009d100c08bfc0c36041dfac
401f9c6c008000ff80bfc24360403fac00009c6c009d202a828000c360409ffc
401f9c6c00c000080286c09540a19fa400009c6c00800e0c04bfc0836041dffc
401f9c6c0140020c0106004360405fa0401f9c6c01400e0c0086008360411fa0
401f9c6c0140000c0086014360409fa1
"
}

SubProgram "d3d11 " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Bind "color" Color
ConstBuffer "$Globals" 160 // 128 used size, 12 vars
Vector 80 [unity_LightmapST] 4
Vector 96 [_MainTex_ST] 4
Vector 112 [_BumpMap_ST] 4
ConstBuffer "UnityPerCamera" 128 // 96 used size, 8 vars
Vector 64 [_WorldSpaceCameraPos] 3
Vector 80 [_ProjectionParams] 4
ConstBuffer "UnityShadows" 416 // 416 used size, 8 vars
Vector 400 [unity_ShadowFadeCenterAndType] 4
ConstBuffer "UnityPerDraw" 336 // 336 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
Matrix 64 [glstate_matrix_modelview0] 4
Matrix 192 [_Object2World] 4
Matrix 256 [_World2Object] 4
Vector 320 [unity_Scale] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityShadows" 2
BindCB "UnityPerDraw" 3
// 36 instructions, 3 temp regs, 0 temp arrays:
// ALU 17 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0
eefiecedhpgnfackdfhjkcdnfikgkcikiajkponcabaaaaaagmahaaaaadaaaaaa
cmaaaaaapeaaaaaakmabaaaaejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaa
abaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapadaaaaljaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfc
enebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheolaaaaaaaagaaaaaa
aiaaaaaajiaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaakeaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaabaaaaaaapaaaaaakeaaaaaaabaaaaaaaaaaaaaa
adaaaaaaacaaaaaaahaiaaaakeaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaa
apaaaaaakeaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaadamaaaakeaaaaaa
aeaaaaaaaaaaaaaaadaaaaaaafaaaaaaapaaaaaafdfgfpfaepfdejfeejepeoaa
feeffiedepepfceeaaklklklfdeieefcliafaaaaeaaaabaagoabaaaafjaaaaae
egiocaaaaaaaaaaaaiaaaaaafjaaaaaeegiocaaaabaaaaaaagaaaaaafjaaaaae
egiocaaaacaaaaaabkaaaaaafjaaaaaeegiocaaaadaaaaaabfaaaaaafpaaaaad
pcbabaaaaaaaaaaafpaaaaadpcbabaaaabaaaaaafpaaaaadhcbabaaaacaaaaaa
fpaaaaaddcbabaaaadaaaaaafpaaaaaddcbabaaaaeaaaaaaghaaaaaepccabaaa
aaaaaaaaabaaaaaagfaaaaadpccabaaaabaaaaaagfaaaaadhccabaaaacaaaaaa
gfaaaaadpccabaaaadaaaaaagfaaaaaddccabaaaaeaaaaaagfaaaaadpccabaaa
afaaaaaagiaaaaacadaaaaaadiaaaaaipcaabaaaaaaaaaaafgbfbaaaaaaaaaaa
egiocaaaadaaaaaaabaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaa
aaaaaaaaagbabaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaa
egiocaaaadaaaaaaacaaaaaakgbkbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaak
pcaabaaaaaaaaaaaegiocaaaadaaaaaaadaaaaaapgbpbaaaaaaaaaaaegaobaaa
aaaaaaaadgaaaaafpccabaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaaldccabaaa
abaaaaaaegbabaaaadaaaaaaegiacaaaaaaaaaaaagaaaaaaogikcaaaaaaaaaaa
agaaaaaadcaaaaalmccabaaaabaaaaaaagbebaaaadaaaaaaagiecaaaaaaaaaaa
ahaaaaaakgiocaaaaaaaaaaaahaaaaaadiaaaaahhcaabaaaabaaaaaajgbebaaa
abaaaaaacgbjbaaaacaaaaaadcaaaaakhcaabaaaabaaaaaajgbebaaaacaaaaaa
cgbjbaaaabaaaaaaegacbaiaebaaaaaaabaaaaaadiaaaaahhcaabaaaabaaaaaa
egacbaaaabaaaaaapgbpbaaaabaaaaaadiaaaaajhcaabaaaacaaaaaafgifcaaa
abaaaaaaaeaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaaacaaaaaa
egiccaaaadaaaaaabaaaaaaaagiacaaaabaaaaaaaeaaaaaaegacbaaaacaaaaaa
dcaaaaalhcaabaaaacaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaaabaaaaaa
aeaaaaaaegacbaaaacaaaaaaaaaaaaaihcaabaaaacaaaaaaegacbaaaacaaaaaa
egiccaaaadaaaaaabdaaaaaadcaaaaalhcaabaaaacaaaaaaegacbaaaacaaaaaa
pgipcaaaadaaaaaabeaaaaaaegbcbaiaebaaaaaaaaaaaaaabaaaaaahcccabaaa
acaaaaaaegacbaaaabaaaaaaegacbaaaacaaaaaabaaaaaahbccabaaaacaaaaaa
egbcbaaaabaaaaaaegacbaaaacaaaaaabaaaaaaheccabaaaacaaaaaaegbcbaaa
acaaaaaaegacbaaaacaaaaaadiaaaaaiccaabaaaaaaaaaaabkaabaaaaaaaaaaa
akiacaaaabaaaaaaafaaaaaadiaaaaakncaabaaaabaaaaaaagahbaaaaaaaaaaa
aceaaaaaaaaaaadpaaaaaaaaaaaaaadpaaaaaadpdgaaaaafmccabaaaadaaaaaa
kgaobaaaaaaaaaaaaaaaaaahdccabaaaadaaaaaakgakbaaaabaaaaaamgaabaaa
abaaaaaadcaaaaaldccabaaaaeaaaaaaegbabaaaaeaaaaaaegiacaaaaaaaaaaa
afaaaaaaogikcaaaaaaaaaaaafaaaaaadiaaaaaihcaabaaaaaaaaaaafgbfbaaa
aaaaaaaaegiccaaaadaaaaaaanaaaaaadcaaaaakhcaabaaaaaaaaaaaegiccaaa
adaaaaaaamaaaaaaagbabaaaaaaaaaaaegacbaaaaaaaaaaadcaaaaakhcaabaaa
aaaaaaaaegiccaaaadaaaaaaaoaaaaaakgbkbaaaaaaaaaaaegacbaaaaaaaaaaa
dcaaaaakhcaabaaaaaaaaaaaegiccaaaadaaaaaaapaaaaaapgbpbaaaaaaaaaaa
egacbaaaaaaaaaaaaaaaaaajhcaabaaaaaaaaaaaegacbaaaaaaaaaaaegiccaia
ebaaaaaaacaaaaaabjaaaaaadiaaaaaihccabaaaafaaaaaaegacbaaaaaaaaaaa
pgipcaaaacaaaaaabjaaaaaadiaaaaaibcaabaaaaaaaaaaabkbabaaaaaaaaaaa
ckiacaaaadaaaaaaafaaaaaadcaaaaakbcaabaaaaaaaaaaackiacaaaadaaaaaa
aeaaaaaaakbabaaaaaaaaaaaakaabaaaaaaaaaaadcaaaaakbcaabaaaaaaaaaaa
ckiacaaaadaaaaaaagaaaaaackbabaaaaaaaaaaaakaabaaaaaaaaaaadcaaaaak
bcaabaaaaaaaaaaackiacaaaadaaaaaaahaaaaaadkbabaaaaaaaaaaaakaabaaa
aaaaaaaaaaaaaaajccaabaaaaaaaaaaadkiacaiaebaaaaaaacaaaaaabjaaaaaa
abeaaaaaaaaaiadpdiaaaaaiiccabaaaafaaaaaabkaabaaaaaaaaaaaakaabaia
ebaaaaaaaaaaaaaadoaaaaab"
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

varying highp vec4 xlv_TEXCOORD4;
varying highp vec2 xlv_TEXCOORD3;
varying highp vec4 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 _BumpMap_ST;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 unity_LightmapST;
uniform highp vec4 unity_Scale;
uniform highp mat4 _World2Object;
uniform highp mat4 _Object2World;


uniform highp vec4 unity_ShadowFadeCenterAndType;
uniform highp vec4 _ProjectionParams;
uniform highp vec3 _WorldSpaceCameraPos;
attribute vec4 _glesTANGENT;
attribute vec4 _glesMultiTexCoord1;
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
  highp vec4 tmpvar_3;
  highp vec4 tmpvar_4;
  highp vec4 tmpvar_5;
  tmpvar_5 = (gl_ModelViewProjectionMatrix * _glesVertex);
  tmpvar_3.xy = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  tmpvar_3.zw = ((_glesMultiTexCoord0.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
  highp vec4 o_6;
  highp vec4 tmpvar_7;
  tmpvar_7 = (tmpvar_5 * 0.5);
  highp vec2 tmpvar_8;
  tmpvar_8.x = tmpvar_7.x;
  tmpvar_8.y = (tmpvar_7.y * _ProjectionParams.x);
  o_6.xy = (tmpvar_8 + tmpvar_7.w);
  o_6.zw = tmpvar_5.zw;
  tmpvar_4.xyz = (((_Object2World * _glesVertex).xyz - unity_ShadowFadeCenterAndType.xyz) * unity_ShadowFadeCenterAndType.w);
  tmpvar_4.w = (-((gl_ModelViewMatrix * _glesVertex).z) * (1.0 - unity_ShadowFadeCenterAndType.w));
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
  highp vec4 tmpvar_12;
  tmpvar_12.w = 1.0;
  tmpvar_12.xyz = _WorldSpaceCameraPos;
  gl_Position = tmpvar_5;
  xlv_TEXCOORD0 = tmpvar_3;
  xlv_TEXCOORD1 = (tmpvar_11 * (((_World2Object * tmpvar_12).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = o_6;
  xlv_TEXCOORD3 = ((_glesMultiTexCoord1.xy * unity_LightmapST.xy) + unity_LightmapST.zw);
  xlv_TEXCOORD4 = tmpvar_4;
}



#endif
#ifdef FRAGMENT

varying highp vec4 xlv_TEXCOORD4;
varying highp vec2 xlv_TEXCOORD3;
varying highp vec4 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 unity_LightmapFade;
uniform sampler2D unity_LightmapInd;
uniform sampler2D unity_Lightmap;
uniform sampler2D _LightBuffer;
uniform highp float _Parallax;
uniform mediump float _Gloss;
uniform lowp vec4 _Color;
uniform sampler2D _ParallaxMap;
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
  mediump vec4 spec_6;
  mediump float h_7;
  lowp float tmpvar_8;
  tmpvar_8 = texture2D (_ParallaxMap, xlv_TEXCOORD0.zw).w;
  h_7 = tmpvar_8;
  mediump float height_9;
  height_9 = _Parallax;
  mediump vec3 viewDir_10;
  viewDir_10 = xlv_TEXCOORD1;
  highp vec3 v_11;
  mediump float tmpvar_12;
  tmpvar_12 = ((h_7 * height_9) - (height_9 / 2.0));
  mediump vec3 tmpvar_13;
  tmpvar_13 = normalize(viewDir_10);
  v_11 = tmpvar_13;
  v_11.z = (v_11.z + 0.42);
  highp vec2 tmpvar_14;
  tmpvar_14 = (xlv_TEXCOORD0.xy + (tmpvar_12 * (v_11.xy / v_11.z)));
  lowp vec4 tmpvar_15;
  tmpvar_15 = texture2D (_MainTex, tmpvar_14);
  lowp vec3 tmpvar_16;
  tmpvar_16 = (tmpvar_15.xyz * _Color.xyz);
  tmpvar_5 = tmpvar_16;
  lowp vec4 tmpvar_17;
  tmpvar_17 = texture2D (_SpecMap, tmpvar_14);
  spec_6 = tmpvar_17;
  lowp vec4 tmpvar_18;
  tmpvar_18 = texture2DProj (_LightBuffer, xlv_TEXCOORD2);
  light_4 = tmpvar_18;
  mediump vec4 tmpvar_19;
  tmpvar_19 = max (light_4, vec4(0.001, 0.001, 0.001, 0.001));
  light_4.w = tmpvar_19.w;
  lowp vec3 tmpvar_20;
  tmpvar_20 = (2.0 * texture2D (unity_Lightmap, xlv_TEXCOORD3).xyz);
  lmFull_3 = tmpvar_20;
  lowp vec3 tmpvar_21;
  tmpvar_21 = (2.0 * texture2D (unity_LightmapInd, xlv_TEXCOORD3).xyz);
  lmIndirect_2 = tmpvar_21;
  highp float tmpvar_22;
  tmpvar_22 = clamp (((sqrt(dot (xlv_TEXCOORD4, xlv_TEXCOORD4)) * unity_LightmapFade.z) + unity_LightmapFade.w), 0.0, 1.0);
  light_4.xyz = (tmpvar_19.xyz + mix (lmIndirect_2, lmFull_3, vec3(tmpvar_22)));
  mediump vec4 c_23;
  mediump vec3 tmpvar_24;
  tmpvar_24 = (tmpvar_19.w * ((tmpvar_15.w * spec_6.xyz) * _Gloss));
  c_23.xyz = ((tmpvar_5 * light_4.xyz) + (light_4.xyz * tmpvar_24));
  c_23.w = (tmpvar_24 * _SpecColor.w).x;
  tmpvar_1 = c_23;
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

varying highp vec4 xlv_TEXCOORD4;
varying highp vec2 xlv_TEXCOORD3;
varying highp vec4 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 _BumpMap_ST;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 unity_LightmapST;
uniform highp vec4 unity_Scale;
uniform highp mat4 _World2Object;
uniform highp mat4 _Object2World;


uniform highp vec4 unity_ShadowFadeCenterAndType;
uniform highp vec4 _ProjectionParams;
uniform highp vec3 _WorldSpaceCameraPos;
attribute vec4 _glesTANGENT;
attribute vec4 _glesMultiTexCoord1;
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
  highp vec4 tmpvar_3;
  highp vec4 tmpvar_4;
  highp vec4 tmpvar_5;
  tmpvar_5 = (gl_ModelViewProjectionMatrix * _glesVertex);
  tmpvar_3.xy = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  tmpvar_3.zw = ((_glesMultiTexCoord0.xy * _BumpMap_ST.xy) + _BumpMap_ST.zw);
  highp vec4 o_6;
  highp vec4 tmpvar_7;
  tmpvar_7 = (tmpvar_5 * 0.5);
  highp vec2 tmpvar_8;
  tmpvar_8.x = tmpvar_7.x;
  tmpvar_8.y = (tmpvar_7.y * _ProjectionParams.x);
  o_6.xy = (tmpvar_8 + tmpvar_7.w);
  o_6.zw = tmpvar_5.zw;
  tmpvar_4.xyz = (((_Object2World * _glesVertex).xyz - unity_ShadowFadeCenterAndType.xyz) * unity_ShadowFadeCenterAndType.w);
  tmpvar_4.w = (-((gl_ModelViewMatrix * _glesVertex).z) * (1.0 - unity_ShadowFadeCenterAndType.w));
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
  highp vec4 tmpvar_12;
  tmpvar_12.w = 1.0;
  tmpvar_12.xyz = _WorldSpaceCameraPos;
  gl_Position = tmpvar_5;
  xlv_TEXCOORD0 = tmpvar_3;
  xlv_TEXCOORD1 = (tmpvar_11 * (((_World2Object * tmpvar_12).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = o_6;
  xlv_TEXCOORD3 = ((_glesMultiTexCoord1.xy * unity_LightmapST.xy) + unity_LightmapST.zw);
  xlv_TEXCOORD4 = tmpvar_4;
}



#endif
#ifdef FRAGMENT

varying highp vec4 xlv_TEXCOORD4;
varying highp vec2 xlv_TEXCOORD3;
varying highp vec4 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec4 xlv_TEXCOORD0;
uniform highp vec4 unity_LightmapFade;
uniform sampler2D unity_LightmapInd;
uniform sampler2D unity_Lightmap;
uniform sampler2D _LightBuffer;
uniform highp float _Parallax;
uniform mediump float _Gloss;
uniform lowp vec4 _Color;
uniform sampler2D _ParallaxMap;
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
  mediump vec4 spec_6;
  mediump float h_7;
  lowp float tmpvar_8;
  tmpvar_8 = texture2D (_ParallaxMap, xlv_TEXCOORD0.zw).w;
  h_7 = tmpvar_8;
  highp vec2 tmpvar_9;
  mediump float height_10;
  height_10 = _Parallax;
  mediump vec3 viewDir_11;
  viewDir_11 = xlv_TEXCOORD1;
  highp vec3 v_12;
  mediump float tmpvar_13;
  tmpvar_13 = ((h_7 * height_10) - (height_10 / 2.0));
  mediump vec3 tmpvar_14;
  tmpvar_14 = normalize(viewDir_11);
  v_12 = tmpvar_14;
  v_12.z = (v_12.z + 0.42);
  tmpvar_9 = (tmpvar_13 * (v_12.xy / v_12.z));
  highp vec2 tmpvar_15;
  tmpvar_15 = (xlv_TEXCOORD0.xy + tmpvar_9);
  highp vec2 tmpvar_16;
  tmpvar_16 = (xlv_TEXCOORD0.zw + tmpvar_9);
  lowp vec4 tmpvar_17;
  tmpvar_17 = texture2D (_MainTex, tmpvar_15);
  lowp vec3 tmpvar_18;
  tmpvar_18 = (tmpvar_17.xyz * _Color.xyz);
  tmpvar_5 = tmpvar_18;
  lowp vec4 tmpvar_19;
  tmpvar_19 = texture2D (_SpecMap, tmpvar_15);
  spec_6 = tmpvar_19;
  lowp vec3 normal_20;
  normal_20.xy = ((texture2D (_BumpMap, tmpvar_16).wy * 2.0) - 1.0);
  normal_20.z = sqrt(((1.0 - (normal_20.x * normal_20.x)) - (normal_20.y * normal_20.y)));
  lowp vec4 tmpvar_21;
  tmpvar_21 = texture2DProj (_LightBuffer, xlv_TEXCOORD2);
  light_4 = tmpvar_21;
  mediump vec4 tmpvar_22;
  tmpvar_22 = max (light_4, vec4(0.001, 0.001, 0.001, 0.001));
  light_4.w = tmpvar_22.w;
  lowp vec4 tmpvar_23;
  tmpvar_23 = texture2D (unity_Lightmap, xlv_TEXCOORD3);
  lowp vec3 tmpvar_24;
  tmpvar_24 = ((8.0 * tmpvar_23.w) * tmpvar_23.xyz);
  lmFull_3 = tmpvar_24;
  lowp vec4 tmpvar_25;
  tmpvar_25 = texture2D (unity_LightmapInd, xlv_TEXCOORD3);
  lowp vec3 tmpvar_26;
  tmpvar_26 = ((8.0 * tmpvar_25.w) * tmpvar_25.xyz);
  lmIndirect_2 = tmpvar_26;
  highp float tmpvar_27;
  tmpvar_27 = clamp (((sqrt(dot (xlv_TEXCOORD4, xlv_TEXCOORD4)) * unity_LightmapFade.z) + unity_LightmapFade.w), 0.0, 1.0);
  light_4.xyz = (tmpvar_22.xyz + mix (lmIndirect_2, lmFull_3, vec3(tmpvar_27)));
  mediump vec4 c_28;
  mediump vec3 tmpvar_29;
  tmpvar_29 = (tmpvar_22.w * ((tmpvar_17.w * spec_6.xyz) * _Gloss));
  c_28.xyz = ((tmpvar_5 * light_4.xyz) + (light_4.xyz * tmpvar_29));
  c_28.w = (tmpvar_29 * _SpecColor.w).x;
  tmpvar_1 = c_28;
  gl_FragData[0] = tmpvar_1;
}



#endif"
}

SubProgram "d3d11_9x " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Bind "color" Color
ConstBuffer "$Globals" 160 // 128 used size, 12 vars
Vector 80 [unity_LightmapST] 4
Vector 96 [_MainTex_ST] 4
Vector 112 [_BumpMap_ST] 4
ConstBuffer "UnityPerCamera" 128 // 96 used size, 8 vars
Vector 64 [_WorldSpaceCameraPos] 3
Vector 80 [_ProjectionParams] 4
ConstBuffer "UnityShadows" 416 // 416 used size, 8 vars
Vector 400 [unity_ShadowFadeCenterAndType] 4
ConstBuffer "UnityPerDraw" 336 // 336 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
Matrix 64 [glstate_matrix_modelview0] 4
Matrix 192 [_Object2World] 4
Matrix 256 [_World2Object] 4
Vector 320 [unity_Scale] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityShadows" 2
BindCB "UnityPerDraw" 3
// 36 instructions, 3 temp regs, 0 temp arrays:
// ALU 17 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0_level_9_3
eefieceddnapboddpcmpibnejgblokeedbocphaiabaaaaaaoiakaaaaaeaaaaaa
daaaaaaakiadaaaagiajaaaadaakaaaaebgpgodjhaadaaaahaadaaaaaaacpopp
amadaaaageaaaaaaafaaceaaaaaagaaaaaaagaaaaaaaceaaabaagaaaaaaaafaa
adaaabaaaaaaaaaaabaaaeaaacaaaeaaaaaaaaaaacaabjaaabaaagaaaaaaaaaa
adaaaaaaaiaaahaaaaaaaaaaadaaamaaajaaapaaaaaaaaaaaaaaaaaaabacpopp
fbaaaaafbiaaapkaaaaaaadpaaaaiadpaaaaaaaaaaaaaaaabpaaaaacafaaaaia
aaaaapjabpaaaaacafaaabiaabaaapjabpaaaaacafaaaciaacaaapjabpaaaaac
afaaadiaadaaapjabpaaaaacafaaaeiaaeaaapjaaeaaaaaeaaaaadoaadaaoeja
acaaoekaacaaookaaeaaaaaeaaaaamoaadaaeejaadaaeekaadaaoekaafaaaaad
aaaaapiaaaaaffjaaiaaoekaaeaaaaaeaaaaapiaahaaoekaaaaaaajaaaaaoeia
aeaaaaaeaaaaapiaajaaoekaaaaakkjaaaaaoeiaaeaaaaaeaaaaapiaakaaoeka
aaaappjaaaaaoeiaafaaaaadabaaabiaaaaaffiaafaaaakaafaaaaadabaaaiia
abaaaaiabiaaaakaafaaaaadabaaafiaaaaapeiabiaaaakaacaaaaadacaaadoa
abaakkiaabaaomiaaeaaaaaeadaaadoaaeaaoejaabaaoekaabaaookaafaaaaad
abaaahiaaaaaffjabaaaoekaaeaaaaaeabaaahiaapaaoekaaaaaaajaabaaoeia
aeaaaaaeabaaahiabbaaoekaaaaakkjaabaaoeiaaeaaaaaeabaaahiabcaaoeka
aaaappjaabaaoeiaacaaaaadabaaahiaabaaoeiaagaaoekbafaaaaadaeaaahoa
abaaoeiaagaappkaafaaaaadabaaabiaaaaaffjaamaakkkaaeaaaaaeabaaabia
alaakkkaaaaaaajaabaaaaiaaeaaaaaeabaaabiaanaakkkaaaaakkjaabaaaaia
aeaaaaaeabaaabiaaoaakkkaaaaappjaabaaaaiaabaaaaacabaaaiiaagaappka
acaaaaadabaaaciaabaappibbiaaffkaafaaaaadaeaaaioaabaaffiaabaaaaib
abaaaaacabaaahiaaeaaoekaafaaaaadacaaahiaabaaffiabeaaoekaaeaaaaae
abaaaliabdaakekaabaaaaiaacaakeiaaeaaaaaeabaaahiabfaaoekaabaakkia
abaapeiaacaaaaadabaaahiaabaaoeiabgaaoekaaeaaaaaeabaaahiaabaaoeia
bhaappkaaaaaoejbaiaaaaadabaaaboaabaaoejaabaaoeiaabaaaaacacaaahia
abaaoejaafaaaaadadaaahiaacaamjiaacaancjaaeaaaaaeacaaahiaacaamjja
acaanciaadaaoeibafaaaaadacaaahiaacaaoeiaabaappjaaiaaaaadabaaacoa
acaaoeiaabaaoeiaaiaaaaadabaaaeoaacaaoejaabaaoeiaaeaaaaaeaaaaadma
aaaappiaaaaaoekaaaaaoeiaabaaaaacaaaaammaaaaaoeiaabaaaaacacaaamoa
aaaaoeiappppaaaafdeieefcliafaaaaeaaaabaagoabaaaafjaaaaaeegiocaaa
aaaaaaaaaiaaaaaafjaaaaaeegiocaaaabaaaaaaagaaaaaafjaaaaaeegiocaaa
acaaaaaabkaaaaaafjaaaaaeegiocaaaadaaaaaabfaaaaaafpaaaaadpcbabaaa
aaaaaaaafpaaaaadpcbabaaaabaaaaaafpaaaaadhcbabaaaacaaaaaafpaaaaad
dcbabaaaadaaaaaafpaaaaaddcbabaaaaeaaaaaaghaaaaaepccabaaaaaaaaaaa
abaaaaaagfaaaaadpccabaaaabaaaaaagfaaaaadhccabaaaacaaaaaagfaaaaad
pccabaaaadaaaaaagfaaaaaddccabaaaaeaaaaaagfaaaaadpccabaaaafaaaaaa
giaaaaacadaaaaaadiaaaaaipcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaa
adaaaaaaabaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaaaaaaaaa
agbabaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaa
adaaaaaaacaaaaaakgbkbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaa
aaaaaaaaegiocaaaadaaaaaaadaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaa
dgaaaaafpccabaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaaldccabaaaabaaaaaa
egbabaaaadaaaaaaegiacaaaaaaaaaaaagaaaaaaogikcaaaaaaaaaaaagaaaaaa
dcaaaaalmccabaaaabaaaaaaagbebaaaadaaaaaaagiecaaaaaaaaaaaahaaaaaa
kgiocaaaaaaaaaaaahaaaaaadiaaaaahhcaabaaaabaaaaaajgbebaaaabaaaaaa
cgbjbaaaacaaaaaadcaaaaakhcaabaaaabaaaaaajgbebaaaacaaaaaacgbjbaaa
abaaaaaaegacbaiaebaaaaaaabaaaaaadiaaaaahhcaabaaaabaaaaaaegacbaaa
abaaaaaapgbpbaaaabaaaaaadiaaaaajhcaabaaaacaaaaaafgifcaaaabaaaaaa
aeaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaaacaaaaaaegiccaaa
adaaaaaabaaaaaaaagiacaaaabaaaaaaaeaaaaaaegacbaaaacaaaaaadcaaaaal
hcaabaaaacaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaaabaaaaaaaeaaaaaa
egacbaaaacaaaaaaaaaaaaaihcaabaaaacaaaaaaegacbaaaacaaaaaaegiccaaa
adaaaaaabdaaaaaadcaaaaalhcaabaaaacaaaaaaegacbaaaacaaaaaapgipcaaa
adaaaaaabeaaaaaaegbcbaiaebaaaaaaaaaaaaaabaaaaaahcccabaaaacaaaaaa
egacbaaaabaaaaaaegacbaaaacaaaaaabaaaaaahbccabaaaacaaaaaaegbcbaaa
abaaaaaaegacbaaaacaaaaaabaaaaaaheccabaaaacaaaaaaegbcbaaaacaaaaaa
egacbaaaacaaaaaadiaaaaaiccaabaaaaaaaaaaabkaabaaaaaaaaaaaakiacaaa
abaaaaaaafaaaaaadiaaaaakncaabaaaabaaaaaaagahbaaaaaaaaaaaaceaaaaa
aaaaaadpaaaaaaaaaaaaaadpaaaaaadpdgaaaaafmccabaaaadaaaaaakgaobaaa
aaaaaaaaaaaaaaahdccabaaaadaaaaaakgakbaaaabaaaaaamgaabaaaabaaaaaa
dcaaaaaldccabaaaaeaaaaaaegbabaaaaeaaaaaaegiacaaaaaaaaaaaafaaaaaa
ogikcaaaaaaaaaaaafaaaaaadiaaaaaihcaabaaaaaaaaaaafgbfbaaaaaaaaaaa
egiccaaaadaaaaaaanaaaaaadcaaaaakhcaabaaaaaaaaaaaegiccaaaadaaaaaa
amaaaaaaagbabaaaaaaaaaaaegacbaaaaaaaaaaadcaaaaakhcaabaaaaaaaaaaa
egiccaaaadaaaaaaaoaaaaaakgbkbaaaaaaaaaaaegacbaaaaaaaaaaadcaaaaak
hcaabaaaaaaaaaaaegiccaaaadaaaaaaapaaaaaapgbpbaaaaaaaaaaaegacbaaa
aaaaaaaaaaaaaaajhcaabaaaaaaaaaaaegacbaaaaaaaaaaaegiccaiaebaaaaaa
acaaaaaabjaaaaaadiaaaaaihccabaaaafaaaaaaegacbaaaaaaaaaaapgipcaaa
acaaaaaabjaaaaaadiaaaaaibcaabaaaaaaaaaaabkbabaaaaaaaaaaackiacaaa
adaaaaaaafaaaaaadcaaaaakbcaabaaaaaaaaaaackiacaaaadaaaaaaaeaaaaaa
akbabaaaaaaaaaaaakaabaaaaaaaaaaadcaaaaakbcaabaaaaaaaaaaackiacaaa
adaaaaaaagaaaaaackbabaaaaaaaaaaaakaabaaaaaaaaaaadcaaaaakbcaabaaa
aaaaaaaackiacaaaadaaaaaaahaaaaaadkbabaaaaaaaaaaaakaabaaaaaaaaaaa
aaaaaaajccaabaaaaaaaaaaadkiacaiaebaaaaaaacaaaaaabjaaaaaaabeaaaaa
aaaaiadpdiaaaaaiiccabaaaafaaaaaabkaabaaaaaaaaaaaakaabaiaebaaaaaa
aaaaaaaadoaaaaabejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaaadaaaaaa
abaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaaahahaaaa
laaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaaabaaaaaa
aaaaaaaaadaaaaaaaeaaaaaaapadaaaaljaaaaaaaaaaaaaaaaaaaaaaadaaaaaa
afaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfcenebemaa
feeffiedepepfceeaaedepemepfcaaklepfdeheolaaaaaaaagaaaaaaaiaaaaaa
jiaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaakeaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaabaaaaaaapaaaaaakeaaaaaaabaaaaaaaaaaaaaaadaaaaaa
acaaaaaaahaiaaaakeaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaapaaaaaa
keaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaadamaaaakeaaaaaaaeaaaaaa
aaaaaaaaadaaaaaaafaaaaaaapaaaaaafdfgfpfaepfdejfeejepeoaafeeffied
epepfceeaaklklkl"
}

}
Program "fp" {
// Fragment combos: 4
//   opengl - ALU: 22 to 37, TEX: 4 to 6
//   d3d9 - ALU: 21 to 34, TEX: 4 to 6
//   d3d11 - ALU: 13 to 19, TEX: 4 to 6, FLOW: 1 to 1
//   d3d11_9x - ALU: 13 to 19, TEX: 4 to 6, FLOW: 1 to 1
SubProgram "opengl " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
Vector 0 [_SpecColor]
Vector 1 [_Color]
Float 2 [_Gloss]
Float 3 [_Parallax]
SetTexture 0 [_ParallaxMap] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 2 [_SpecMap] 2D
SetTexture 4 [_LightBuffer] 2D
"!!ARBfp1.0
OPTION ARB_precision_hint_fastest;
# 26 ALU, 4 TEX
PARAM c[5] = { program.local[0..3],
		{ 0.5, 0.41999999 } };
TEMP R0;
TEMP R1;
TEMP R2;
TEX R0.w, fragment.texcoord[0].zwzw, texture[0], 2D;
DP3 R0.x, fragment.texcoord[1], fragment.texcoord[1];
RSQ R0.x, R0.x;
MUL R1.xyz, R0.x, fragment.texcoord[1];
ADD R0.y, R1.z, c[4];
RCP R0.y, R0.y;
MOV R0.x, c[3];
MUL R0.x, R0, c[4];
MUL R1.xy, R1, R0.y;
MAD R0.x, R0.w, c[3], -R0;
MAD R1.xy, R0.x, R1, fragment.texcoord[0];
TXP R0, fragment.texcoord[2], texture[4], 2D;
TEX R2.xyz, R1, texture[2], 2D;
TEX R1, R1, texture[1], 2D;
MUL R2.xyz, R1.w, R2;
LG2 R0.x, R0.x;
LG2 R0.z, R0.z;
LG2 R0.y, R0.y;
ADD R0.xyz, -R0, fragment.texcoord[3];
MUL R2.xyz, R2, c[2].x;
LG2 R0.w, R0.w;
MUL R2.xyz, -R0.w, R2;
MUL R2.yzw, R2.xxyz, R0.xxyz;
MUL R1.xyz, R1, c[1];
MAD result.color.xyz, R1, R0, R2.yzww;
MUL result.color.w, R2.x, c[0];
END
# 26 instructions, 3 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
Vector 0 [_SpecColor]
Vector 1 [_Color]
Float 2 [_Gloss]
Float 3 [_Parallax]
SetTexture 0 [_ParallaxMap] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 2 [_SpecMap] 2D
SetTexture 4 [_LightBuffer] 2D
"ps_2_0
; 25 ALU, 4 TEX
dcl_2d s0
dcl_2d s1
dcl_2d s2
dcl_2d s4
def c4, 0.50000000, 0.41999999, 0, 0
dcl t0
dcl t1.xyz
dcl t2
dcl t3.xyz
mov r0.y, t0.w
mov r0.x, t0.z
texld r0, r0, s0
dp3_pp r0.x, t1, t1
rsq_pp r0.x, r0.x
mul_pp r2.xyz, r0.x, t1
add r0.x, r2.z, c4.y
rcp r1.x, r0.x
mov_pp r0.x, c4
mul_pp r0.x, c3, r0
mul r1.xy, r2, r1.x
mad_pp r0.x, r0.w, c3, -r0
mad r0.xy, r0.x, r1, t0
texld r2, r0, s2
texld r1, r0, s1
texldp r0, t2, s4
mul_pp r2.xyz, r1.w, r2
log_pp r0.x, r0.x
log_pp r0.z, r0.z
log_pp r0.y, r0.y
add_pp r3.xyz, -r0, t3
mul_pp r2.xyz, r2, c2.x
log_pp r0.x, r0.w
mul_pp r0.xyz, -r0.x, r2
mul_pp r2.xyz, r0, r3
mul_pp r1.xyz, r1, c1
mad_pp r1.xyz, r1, r3, r2
mul_pp r1.w, r0.x, c0
mov_pp oC0, r1
"
}

SubProgram "xbox360 " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
Vector 1 [_Color]
Float 2 [_Gloss]
Float 3 [_Parallax]
Vector 0 [_SpecColor]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_ParallaxMap] 2D
SetTexture 3 [_LightBuffer] 2D
// Shader Timing Estimate, in Cycles/64 pixel vector:
// ALU: 21.33 (16 instructions), vertex: 0, texture: 16,
//   sequencer: 10, interpolator: 16;    5 GPRs, 36 threads,
// Performance (if enough threads): ~21 cycles per vector
// * Texture cycle estimates are assuming an 8bit/component texture with no
//     aniso or trilinear filtering.

"ps_360
backbbaaaaaaabneaaaaabgaaaaaaaaaaaaaaaceaaaaabhmaaaaabkeaaaaaaaa
aaaaaaaaaaaaabfeaaaaaabmaaaaabehppppadaaaaaaaaaiaaaaaabmaaaaaaaa
aaaaabeaaaaaaalmaaacaaabaaabaaaaaaaaaameaaaaaaaaaaaaaaneaaacaaac
aaabaaaaaaaaaanmaaaaaaaaaaaaaaomaaadaaadaaabaaaaaaaaaapmaaaaaaaa
aaaaabamaaadaaaaaaabaaaaaaaaaapmaaaaaaaaaaaaabbfaaacaaadaaabaaaa
aaaaaanmaaaaaaaaaaaaabbpaaadaaacaaabaaaaaaaaaapmaaaaaaaaaaaaabcm
aaacaaaaaaabaaaaaaaaaameaaaaaaaaaaaaabdhaaadaaabaaabaaaaaaaaaapm
aaaaaaaafpedgpgmgphcaaklaaabaaadaaabaaaeaaabaaaaaaaaaaaafpehgmgp
hdhdaaklaaaaaaadaaabaaabaaabaaaaaaaaaaaafpemgjghgiheechfgggggfhc
aaklklklaaaeaaamaaabaaabaaabaaaaaaaaaaaafpengbgjgofegfhiaafpfagb
hcgbgmgmgbhiaafpfagbhcgbgmgmgbhiengbhaaafpfdhagfgdedgpgmgphcaafp
fdhagfgdengbhaaahahdfpddfpdaaadccodacodcdadddfddcodaaaklaaaaaaaa
aaaaaaabaaaaaaaaaaaaaaaaaaaaaabeabpmaabaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaeaaaaaabcabaaaaeaaaaaaaaaeaaaaaaaaaaaadiieaaapaaap
aaaaaaabaaaapafaaaaahbfbaaaapcfcaaaahdfdaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaadonhakdndpaaaaaaaaaaaaaaaaaaaaaaaaajgaadgaajbcaabcaaafea
aaaaaaaagaapmeaabcaaaaaaaaaacabfaaaaccaaaaaaaaaadicaeaabbpbppppl
aaaaeaaamiaiaaadaaloloaapaababaafiiiadabaagmlbblcbadppidmiaiaaab
abgmgmblklaeadabmiahaaaeaablmaaaobadabaaembeababaamggmblkaaeppac
emedababaagmlamgobabacabmiadaaacaalamgaaobaeabaamiadaaacaalablla
olacabaababiaaebbpbppoiiaaaaeaaabadibacbbpbppgiiaaaaeaaabaaicaeb
bpbppgiiaaaaeaaaeabhaeacaamamagmkbacabibeachaeaaaablmalbobacaaib
eaeoaeaaaapmgmmgkbaaacibeabhaaabaemamabloaaeadibmiahaaaaacbfgmaa
obaaaaaakiiaiaaaaaaaaaaamcaaaaaamiahaaaaaamamaaaobaaabaamiahiaaa
aamamamaolacabaaaaaaaaaaaaaaaaaaaaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
Vector 0 [_SpecColor]
Vector 1 [_Color]
Float 2 [_Gloss]
Float 3 [_Parallax]
SetTexture 0 [_ParallaxMap] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 2 [_SpecMap] 2D
SetTexture 4 [_LightBuffer] 2D
"sce_fp_rsx // 31 instructions using 3 registers
[Configuration]
24
ffffffff0003c020000ffff0000000000000840003000000
[Offsets]
4
_SpecColor 1 0
000001d0
_Color 1 0
00000130
_Gloss 1 0
00000160
_Parallax 2 0
0000005000000030
[Microcode]
496
9804010080011c9cc8000001c8003fe1900017005c011c9dc8000001c8003fe1
02820240fe001c9d00020000c800000100000000000000000000000000000000
1082014000021c9cc8000001c800000100000000000000000000000000000000
de021808c8011c9dc8000001c8003fe102841d40c8041c9dc8000001c8000001
04840440ff041c9d00020000010400000000bf00000000000000000000000000
ae803940c8011c9dc8000029c800bfe10400030055001c9d00020000c8000001
0a3d3ed700000000000000000000000008861d40fe041c9dc8000001c8000001
06043a00c9001c9daa000000c800000106040400ab081c9cc80800015c080001
04841d40aa041c9cc8000001c80000011e001702c8081c9dc8000001c8000001
0e800240c8001c9dc8020001c800000100000000000000000000000000000000
08841d4054041c9dc8000001c800000110840240c8001c9d00020000c8000001
00000000000000000000000000000000ee820300c8011c9dc9080003c8003fe1
10800240550c1c9fc9080001c80000010e041704c8081c9dc8000001c8000001
0e880240ff001c9dc8080001c80000010e840240c9101c9dc9040001c8000001
1080024001101c9cc8020001c800000100000000000000000000000000000000
0e810440c9001c9dc9040001c9080001
"
}

SubProgram "d3d11 " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
ConstBuffer "$Globals" 128 // 76 used size, 10 vars
Vector 32 [_SpecColor] 4
Vector 48 [_Color] 4
Float 68 [_Gloss]
Float 72 [_Parallax]
BindCB "$Globals" 0
SetTexture 0 [_ParallaxMap] 2D 2
SetTexture 1 [_MainTex] 2D 0
SetTexture 2 [_SpecMap] 2D 1
SetTexture 3 [_LightBuffer] 2D 3
// 23 instructions, 3 temp regs, 0 temp arrays:
// ALU 14 float, 0 int, 0 uint
// TEX 4 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0
eefiecedeocelamlbomljjgbpkkjhogblheldcmiabaaaaaajaaeaaaaadaaaaaa
cmaaaaaammaaaaaaaaabaaaaejfdeheojiaaaaaaafaaaaaaaiaaaaaaiaaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaaimaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaapalaaaaimaaaaaa
adaaaaaaaaaaaaaaadaaaaaaaeaaaaaaahahaaaafdfgfpfaepfdejfeejepeoaa
feeffiedepepfceeaaklklklepfdeheocmaaaaaaabaaaaaaaiaaaaaacaaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaafdfgfpfegbhcghgfheaaklkl
fdeieefciiadaaaaeaaaaaaaocaaaaaafjaaaaaeegiocaaaaaaaaaaaafaaaaaa
fkaaaaadaagabaaaaaaaaaaafkaaaaadaagabaaaabaaaaaafkaaaaadaagabaaa
acaaaaaafkaaaaadaagabaaaadaaaaaafibiaaaeaahabaaaaaaaaaaaffffaaaa
fibiaaaeaahabaaaabaaaaaaffffaaaafibiaaaeaahabaaaacaaaaaaffffaaaa
fibiaaaeaahabaaaadaaaaaaffffaaaagcbaaaadpcbabaaaabaaaaaagcbaaaad
hcbabaaaacaaaaaagcbaaaadlcbabaaaadaaaaaagcbaaaadhcbabaaaaeaaaaaa
gfaaaaadpccabaaaaaaaaaaagiaaaaacadaaaaaabaaaaaahbcaabaaaaaaaaaaa
egbcbaaaacaaaaaaegbcbaaaacaaaaaaeeaaaaafbcaabaaaaaaaaaaaakaabaaa
aaaaaaaadiaaaaahgcaabaaaaaaaaaaaagaabaaaaaaaaaaaagbbbaaaacaaaaaa
dcaaaaajbcaabaaaaaaaaaaackbabaaaacaaaaaaakaabaaaaaaaaaaaabeaaaaa
dnaknhdoaoaaaaahdcaabaaaaaaaaaaajgafbaaaaaaaaaaaagaabaaaaaaaaaaa
efaaaaajpcaabaaaabaaaaaaogbkbaaaabaaaaaaeghobaaaaaaaaaaaaagabaaa
acaaaaaadiaaaaaiecaabaaaaaaaaaaackiacaaaaaaaaaaaaeaaaaaaabeaaaaa
aaaaaadpdcaaaaalecaabaaaaaaaaaaadkaabaaaabaaaaaackiacaaaaaaaaaaa
aeaaaaaackaabaiaebaaaaaaaaaaaaaadcaaaaajdcaabaaaaaaaaaaakgakbaaa
aaaaaaaaegaabaaaaaaaaaaaegbabaaaabaaaaaaefaaaaajpcaabaaaabaaaaaa
egaabaaaaaaaaaaaeghobaaaacaaaaaaaagabaaaabaaaaaaefaaaaajpcaabaaa
aaaaaaaaegaabaaaaaaaaaaaeghobaaaabaaaaaaaagabaaaaaaaaaaadiaaaaah
hcaabaaaabaaaaaaegacbaaaabaaaaaapgapbaaaaaaaaaaadiaaaaaihcaabaaa
aaaaaaaaegacbaaaaaaaaaaaegiccaaaaaaaaaaaadaaaaaadiaaaaaihcaabaaa
abaaaaaaegacbaaaabaaaaaafgifcaaaaaaaaaaaaeaaaaaaaoaaaaahdcaabaaa
acaaaaaaegbabaaaadaaaaaapgbpbaaaadaaaaaaefaaaaajpcaabaaaacaaaaaa
egaabaaaacaaaaaaeghobaaaadaaaaaaaagabaaaadaaaaaacpaaaaafpcaabaaa
acaaaaaaegaobaaaacaaaaaadiaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaa
pgapbaiaebaaaaaaacaaaaaaaaaaaaaihcaabaaaacaaaaaaegacbaiaebaaaaaa
acaaaaaaegbcbaaaaeaaaaaadiaaaaahocaabaaaabaaaaaaagajbaaaabaaaaaa
agajbaaaacaaaaaadiaaaaaiiccabaaaaaaaaaaaakaabaaaabaaaaaadkiacaaa
aaaaaaaaacaaaaaadcaaaaajhccabaaaaaaaaaaaegacbaaaaaaaaaaaegacbaaa
acaaaaaajgahbaaaabaaaaaadoaaaaab"
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
ConstBuffer "$Globals" 128 // 76 used size, 10 vars
Vector 32 [_SpecColor] 4
Vector 48 [_Color] 4
Float 68 [_Gloss]
Float 72 [_Parallax]
BindCB "$Globals" 0
SetTexture 0 [_ParallaxMap] 2D 2
SetTexture 1 [_MainTex] 2D 0
SetTexture 2 [_SpecMap] 2D 1
SetTexture 3 [_LightBuffer] 2D 3
// 23 instructions, 3 temp regs, 0 temp arrays:
// ALU 14 float, 0 int, 0 uint
// TEX 4 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0_level_9_3
eefiecednllhibkfilkbhdppeanjblalgmebdefkabaaaaaaceahaaaaaeaaaaaa
daaaaaaamaacaaaafaagaaaapaagaaaaebgpgodjiiacaaaaiiacaaaaaaacpppp
eiacaaaaeaaaaaaaabaadeaaaaaaeaaaaaaaeaaaaeaaceaaaaaaeaaaabaaaaaa
acababaaaaacacaaadadadaaaaaaacaaadaaaaaaaaaaaaaaabacppppfbaaaaaf
adaaapkaaaaaaadpdnaknhdoaaaaaaaaaaaaaaaabpaaaaacaaaaaaiaaaaaapla
bpaaaaacaaaaaaiaabaaahlabpaaaaacaaaaaaiaacaaaplabpaaaaacaaaaaaia
adaaahlabpaaaaacaaaaaajaaaaiapkabpaaaaacaaaaaajaabaiapkabpaaaaac
aaaaaajaacaiapkabpaaaaacaaaaaajaadaiapkaaiaaaaadaaaaciiaabaaoela
abaaoelaahaaaaacaaaacbiaaaaappiaaeaaaaaeaaaaaciaabaakklaaaaaaaia
adaaffkaafaaaaadaaaaafiaaaaaaaiaabaanelaagaaaaacaaaaaciaaaaaffia
afaaaaadaaaaadiaaaaaffiaaaaaoiiaabaaaaacabaaadiaaaaaoolaecaaaaad
abaacpiaabaaoeiaacaioekaabaaaaacaaaaaeiaacaakkkaafaaaaadaaaaceia
aaaakkiaadaaaakaaeaaaaaeaaaaceiaabaappiaacaakkkaaaaakkibaeaaaaae
aaaaadiaaaaakkiaaaaaoeiaaaaaoelaecaaaaadabaacpiaaaaaoeiaaaaioeka
ecaaaaadaaaacpiaaaaaoeiaabaioekaafaaaaadaaaachiaaaaaoeiaabaappia
afaaaaadabaachiaabaaoeiaabaaoekaafaaaaadaaaachiaaaaaoeiaacaaffka
agaaaaacaaaaaiiaacaapplaafaaaaadacaaadiaaaaappiaacaaoelaecaaaaad
acaacpiaacaaoeiaadaioekaapaaaaacaaaaciiaacaappiaafaaaaadaaaachia
aaaaoeiaaaaappibapaaaaacadaacbiaacaaaaiaapaaaaacadaacciaacaaffia
apaaaaacadaaceiaacaakkiaacaaaaadacaachiaadaaoeibadaaoelaafaaaaad
aaaacoiaaaaajaiaacaajaiaafaaaaadadaaciiaaaaaaaiaaaaappkaaeaaaaae
adaachiaabaaoeiaacaaoeiaaaaapjiaabaaaaacaaaicpiaadaaoeiappppaaaa
fdeieefciiadaaaaeaaaaaaaocaaaaaafjaaaaaeegiocaaaaaaaaaaaafaaaaaa
fkaaaaadaagabaaaaaaaaaaafkaaaaadaagabaaaabaaaaaafkaaaaadaagabaaa
acaaaaaafkaaaaadaagabaaaadaaaaaafibiaaaeaahabaaaaaaaaaaaffffaaaa
fibiaaaeaahabaaaabaaaaaaffffaaaafibiaaaeaahabaaaacaaaaaaffffaaaa
fibiaaaeaahabaaaadaaaaaaffffaaaagcbaaaadpcbabaaaabaaaaaagcbaaaad
hcbabaaaacaaaaaagcbaaaadlcbabaaaadaaaaaagcbaaaadhcbabaaaaeaaaaaa
gfaaaaadpccabaaaaaaaaaaagiaaaaacadaaaaaabaaaaaahbcaabaaaaaaaaaaa
egbcbaaaacaaaaaaegbcbaaaacaaaaaaeeaaaaafbcaabaaaaaaaaaaaakaabaaa
aaaaaaaadiaaaaahgcaabaaaaaaaaaaaagaabaaaaaaaaaaaagbbbaaaacaaaaaa
dcaaaaajbcaabaaaaaaaaaaackbabaaaacaaaaaaakaabaaaaaaaaaaaabeaaaaa
dnaknhdoaoaaaaahdcaabaaaaaaaaaaajgafbaaaaaaaaaaaagaabaaaaaaaaaaa
efaaaaajpcaabaaaabaaaaaaogbkbaaaabaaaaaaeghobaaaaaaaaaaaaagabaaa
acaaaaaadiaaaaaiecaabaaaaaaaaaaackiacaaaaaaaaaaaaeaaaaaaabeaaaaa
aaaaaadpdcaaaaalecaabaaaaaaaaaaadkaabaaaabaaaaaackiacaaaaaaaaaaa
aeaaaaaackaabaiaebaaaaaaaaaaaaaadcaaaaajdcaabaaaaaaaaaaakgakbaaa
aaaaaaaaegaabaaaaaaaaaaaegbabaaaabaaaaaaefaaaaajpcaabaaaabaaaaaa
egaabaaaaaaaaaaaeghobaaaacaaaaaaaagabaaaabaaaaaaefaaaaajpcaabaaa
aaaaaaaaegaabaaaaaaaaaaaeghobaaaabaaaaaaaagabaaaaaaaaaaadiaaaaah
hcaabaaaabaaaaaaegacbaaaabaaaaaapgapbaaaaaaaaaaadiaaaaaihcaabaaa
aaaaaaaaegacbaaaaaaaaaaaegiccaaaaaaaaaaaadaaaaaadiaaaaaihcaabaaa
abaaaaaaegacbaaaabaaaaaafgifcaaaaaaaaaaaaeaaaaaaaoaaaaahdcaabaaa
acaaaaaaegbabaaaadaaaaaapgbpbaaaadaaaaaaefaaaaajpcaabaaaacaaaaaa
egaabaaaacaaaaaaeghobaaaadaaaaaaaagabaaaadaaaaaacpaaaaafpcaabaaa
acaaaaaaegaobaaaacaaaaaadiaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaa
pgapbaiaebaaaaaaacaaaaaaaaaaaaaihcaabaaaacaaaaaaegacbaiaebaaaaaa
acaaaaaaegbcbaaaaeaaaaaadiaaaaahocaabaaaabaaaaaaagajbaaaabaaaaaa
agajbaaaacaaaaaadiaaaaaiiccabaaaaaaaaaaaakaabaaaabaaaaaadkiacaaa
aaaaaaaaacaaaaaadcaaaaajhccabaaaaaaaaaaaegacbaaaaaaaaaaaegacbaaa
acaaaaaajgahbaaaabaaaaaadoaaaaabejfdeheojiaaaaaaafaaaaaaaiaaaaaa
iaaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaaaaaaaaaa
aaaaaaaaadaaaaaaabaaaaaaapapaaaaimaaaaaaabaaaaaaaaaaaaaaadaaaaaa
acaaaaaaahahaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaapalaaaa
imaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaahahaaaafdfgfpfaepfdejfe
ejepeoaafeeffiedepepfceeaaklklklepfdeheocmaaaaaaabaaaaaaaiaaaaaa
caaaaaaaaaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaafdfgfpfegbhcghgf
heaaklkl"
}

SubProgram "opengl " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
Vector 0 [_SpecColor]
Vector 1 [_Color]
Float 2 [_Gloss]
Float 3 [_Parallax]
Vector 4 [unity_LightmapFade]
SetTexture 0 [_ParallaxMap] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 2 [_SpecMap] 2D
SetTexture 4 [_LightBuffer] 2D
SetTexture 5 [unity_Lightmap] 2D
SetTexture 6 [unity_LightmapInd] 2D
"!!ARBfp1.0
OPTION ARB_precision_hint_fastest;
# 37 ALU, 6 TEX
PARAM c[6] = { program.local[0..4],
		{ 0.5, 0.41999999, 8 } };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
TEMP R4;
TEX R3, fragment.texcoord[3], texture[6], 2D;
TEX R0.w, fragment.texcoord[0].zwzw, texture[0], 2D;
DP3 R0.x, fragment.texcoord[1], fragment.texcoord[1];
RSQ R0.x, R0.x;
MUL R1.xyz, R0.x, fragment.texcoord[1];
ADD R0.y, R1.z, c[5];
RCP R0.y, R0.y;
MOV R0.x, c[3];
MUL R0.x, R0, c[5];
MUL R3.xyz, R3.w, R3;
MUL R1.xy, R1, R0.y;
MAD R0.x, R0.w, c[3], -R0;
MAD R2.xy, R0.x, R1, fragment.texcoord[0];
MUL R3.xyz, R3, c[5].z;
TXP R0, fragment.texcoord[2], texture[4], 2D;
TEX R1, R2, texture[1], 2D;
TEX R4.xyz, R2, texture[2], 2D;
TEX R2, fragment.texcoord[3], texture[5], 2D;
MUL R2.xyz, R2.w, R2;
DP4 R2.w, fragment.texcoord[4], fragment.texcoord[4];
RSQ R2.w, R2.w;
RCP R2.w, R2.w;
MAD R2.xyz, R2, c[5].z, -R3;
MAD_SAT R2.w, R2, c[4].z, c[4];
MAD R2.xyz, R2.w, R2, R3;
MUL R3.xyz, R1.w, R4;
LG2 R0.x, R0.x;
LG2 R0.y, R0.y;
LG2 R0.z, R0.z;
ADD R0.xyz, -R0, R2;
MUL R2.xyz, R3, c[2].x;
LG2 R0.w, R0.w;
MUL R2.xyz, -R0.w, R2;
MUL R3.xyz, R2, R0;
MUL R1.xyz, R1, c[1];
MAD result.color.xyz, R1, R0, R3;
MUL result.color.w, R2.x, c[0];
END
# 37 instructions, 5 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
Vector 0 [_SpecColor]
Vector 1 [_Color]
Float 2 [_Gloss]
Float 3 [_Parallax]
Vector 4 [unity_LightmapFade]
SetTexture 0 [_ParallaxMap] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 2 [_SpecMap] 2D
SetTexture 4 [_LightBuffer] 2D
SetTexture 5 [unity_Lightmap] 2D
SetTexture 6 [unity_LightmapInd] 2D
"ps_2_0
; 34 ALU, 6 TEX
dcl_2d s0
dcl_2d s1
dcl_2d s2
dcl_2d s4
dcl_2d s5
dcl_2d s6
def c5, 0.50000000, 0.41999999, 8.00000000, 0
dcl t0
dcl t1.xyz
dcl t2
dcl t3.xy
dcl t4
texld r3, t3, s6
mul_pp r3.xyz, r3.w, r3
mul_pp r3.xyz, r3, c5.z
mov r0.y, t0.w
mov r0.x, t0.z
texld r0, r0, s0
dp3_pp r0.x, t1, t1
rsq_pp r0.x, r0.x
mul_pp r2.xyz, r0.x, t1
add r0.x, r2.z, c5.y
rcp r1.x, r0.x
mul r1.xy, r2, r1.x
mov_pp r0.x, c5
mul_pp r0.x, c3, r0
mad_pp r0.x, r0.w, c3, -r0
mad r0.xy, r0.x, r1, t0
texld r4, r0, s2
texld r1, r0, s1
texldp r2, t2, s4
texld r0, t3, s5
mul_pp r5.xyz, r0.w, r0
dp4 r0.x, t4, t4
rsq r0.x, r0.x
rcp r0.x, r0.x
mad_pp r5.xyz, r5, c5.z, -r3
mad_sat r0.x, r0, c4.z, c4.w
mad_pp r0.xyz, r0.x, r5, r3
mul_pp r3.xyz, r1.w, r4
mul_pp r3.xyz, r3, c2.x
log_pp r2.x, r2.x
log_pp r2.y, r2.y
log_pp r2.z, r2.z
add_pp r2.xyz, -r2, r0
log_pp r0.x, r2.w
mul_pp r0.xyz, -r0.x, r3
mul_pp r3.xyz, r0, r2
mul_pp r1.xyz, r1, c1
mad_pp r1.xyz, r1, r2, r3
mul_pp r1.w, r0.x, c0
mov_pp oC0, r1
"
}

SubProgram "xbox360 " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
Vector 1 [_Color]
Float 2 [_Gloss]
Float 3 [_Parallax]
Vector 0 [_SpecColor]
Vector 4 [unity_LightmapFade]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_ParallaxMap] 2D
SetTexture 3 [_LightBuffer] 2D
SetTexture 4 [unity_Lightmap] 2D
SetTexture 5 [unity_LightmapInd] 2D
// Shader Timing Estimate, in Cycles/64 pixel vector:
// ALU: 29.33 (22 instructions), vertex: 0, texture: 24,
//   sequencer: 14, interpolator: 20;    8 GPRs, 24 threads,
// Performance (if enough threads): ~29 cycles per vector
// * Texture cycle estimates are assuming an 8bit/component texture with no
//     aniso or trilinear filtering.

"ps_360
backbbaaaaaaaceiaaaaabmmaaaaaaaaaaaaaaceaaaaabomaaaaacbeaaaaaaaa
aaaaaaaaaaaaabmeaaaaaabmaaaaablhppppadaaaaaaaaalaaaaaabmaaaaaaaa
aaaaablaaaaaaapiaaacaaabaaabaaaaaaaaabaaaaaaaaaaaaaaabbaaaacaaac
aaabaaaaaaaaabbiaaaaaaaaaaaaabciaaadaaadaaabaaaaaaaaabdiaaaaaaaa
aaaaabeiaaadaaaaaaabaaaaaaaaabdiaaaaaaaaaaaaabfbaaacaaadaaabaaaa
aaaaabbiaaaaaaaaaaaaabflaaadaaacaaabaaaaaaaaabdiaaaaaaaaaaaaabgi
aaacaaaaaaabaaaaaaaaabaaaaaaaaaaaaaaabhdaaadaaabaaabaaaaaaaaabdi
aaaaaaaaaaaaabhmaaadaaaeaaabaaaaaaaaabdiaaaaaaaaaaaaabilaaacaaae
aaabaaaaaaaaabaaaaaaaaaaaaaaabjoaaadaaafaaabaaaaaaaaabdiaaaaaaaa
fpedgpgmgphcaaklaaabaaadaaabaaaeaaabaaaaaaaaaaaafpehgmgphdhdaakl
aaaaaaadaaabaaabaaabaaaaaaaaaaaafpemgjghgiheechfgggggfhcaaklklkl
aaaeaaamaaabaaabaaabaaaaaaaaaaaafpengbgjgofegfhiaafpfagbhcgbgmgm
gbhiaafpfagbhcgbgmgmgbhiengbhaaafpfdhagfgdedgpgmgphcaafpfdhagfgd
engbhaaahfgogjhehjfpemgjghgihegngbhaaahfgogjhehjfpemgjghgihegngb
haeggbgegfaahfgogjhehjfpemgjghgihegngbhaejgogeaahahdfpddfpdaaadc
codacodcdadddfddcodaaaklaaaaaaaaaaaaaaabaaaaaaaaaaaaaaaaaaaaaabe
abpmaabaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaeaaaaaabimbaaaahaa
aaaaaaaeaaaaaaaaaaaaeekfaabpaabpaaaaaaabaaaapafaaaaahbfbaaaapcfc
aaaaddfdaaaapefeaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaebaaaaaadpaaaaaa
donhakdnaaaaaaaaaffagaaegaakbcaabcaaaaacaaaagabadabgbcaabcaaaabe
aaaaaaaagabjmeaabcaaaaaaaaaababpaaaaccaaaaaaaaaaemiiabadaakhkhbl
opaeaeacmiadaaacaabllaaaobabacaadicadaabbpbppoppaaaaeaaabafahagb
bpbppgiiaaaaeaaabaeafagbbpbppgiiaaaaeaaabadacaebbpbppgiiaaaaeaaa
miaiaaaeaagmlbaacbadppaaeabiaeabaalologmpaababicfibbadagaablgmbl
kbafppibkaciadabaablgmblkbahppidmjaeaaaeaalbmgbliladaeaeeachaeab
aagmlolbobadabicleilabadaablmamambabahppmiahaaafabgmmabaolagafad
miaoaaafaapmmgdmolafaeademibabafaamggmblkbadadabeaedaeadaamfblmg
obababicmiapaaabacaappaaoaafaeaamiadaaadaalagmlaoladabaababiaagb
bpbppoiiaaaaeaaabaaidagbbpbppgiiaaaaeaaamiahaaacaamamaaakbadabaa
miahaaaaaablmaaaobadaaaaeaboaaaaaapmgmblkbaaacicmiahaaaaacbfgmaa
obaaaaaakiiaiaaaaaaaaaaamcaaaaaamiahaaaaaamabfaaobaaabaamiahiaaa
aamabfmaolacabaaaaaaaaaaaaaaaaaaaaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
Vector 0 [_SpecColor]
Vector 1 [_Color]
Float 2 [_Gloss]
Float 3 [_Parallax]
Vector 4 [unity_LightmapFade]
SetTexture 0 [_ParallaxMap] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 2 [_SpecMap] 2D
SetTexture 4 [_LightBuffer] 2D
SetTexture 5 [unity_Lightmap] 2D
SetTexture 6 [unity_LightmapInd] 2D
"sce_fp_rsx // 46 instructions using 4 registers
[Configuration]
24
ffffffff0007c020001fffe8000000000000840004000000
[Offsets]
5
_SpecColor 1 0
00000260
_Color 1 0
000002a0
_Gloss 1 0
000001f0
_Parallax 2 0
000000e000000020
unity_LightmapFade 2 0
0000016000000130
[Microcode]
736
900017005c011c9dc8000001c8003fe1028a0240fe001c9d00020000c8000001
00000000000000000000000000000000fe02170ac8011c9dc8000001c8003fe1
ae803940c8011c9dc8000029c800bfe11000030055001c9d00020000c8000001
0a3d3ed700000000000000000000000008860140fe041c9dc8003001c8000001
06043a00c9001c9dfe000001c80000011e000101c8011c9dc8000001c8003fe1
10040600c8001c9dc8000001c8000001fe00170cc8011c9dc8000001c8003fe1
0e820240fe001c9dc8003001c80000010280014000021c9cc8000001c8000001
0000000000000000000000000000000010041b00fe081c9dc8000001c8000001
02800440c9001c9d00020000c91400010000bf00000000000000000000000000
10043a0054021c9dfe080001c800000100000000000000000000000000000000
0e020400550c1c9dc8040001c904000310028300c8081c9dc8020001c8000001
000000000000000000000000000000009806010080011c9cc8000001c8003fe1
0600040001001c9cc80800015c0c0001de061808c8011c9dc8000001c8003fe1
028c1d40c80c1c9dc8000001c80000010e820400fe041c9dc8040001c9040001
1e041702c8001c9dc8000001c8000001048c1d40aa0c1c9cc8000001c8000001
088c0240fe081c9d00020000c800000100000000000000000000000000000000
0e021704c8001c9dc8000001c800000108861d40fe0c1c9dc8000001c8000001
08860240c90c1c9fc9180001c8000001088c1d40540c1c9dc8000001c8000001
0e8e0240550c1c9dc8040001c800000110800240011c1c9cc8020001c8000001
0000000000000000000000000000000006060100c80c1c9dc8000001c8000001
0e820340c9181c9fc9040001c80000010e800240c8081c9dc8020001c8000001
000000000000000000000000000000001e7e7d00c8001c9dc8000001c8000001
0e840240c91c1c9dc9040001c80000010e810440c9001c9dc9040001c9080001
"
}

SubProgram "d3d11 " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
ConstBuffer "$Globals" 160 // 144 used size, 12 vars
Vector 32 [_SpecColor] 4
Vector 48 [_Color] 4
Float 68 [_Gloss]
Float 72 [_Parallax]
Vector 128 [unity_LightmapFade] 4
BindCB "$Globals" 0
SetTexture 0 [_ParallaxMap] 2D 2
SetTexture 1 [_MainTex] 2D 0
SetTexture 2 [_SpecMap] 2D 1
SetTexture 3 [_LightBuffer] 2D 3
SetTexture 4 [unity_Lightmap] 2D 4
SetTexture 5 [unity_LightmapInd] 2D 5
// 33 instructions, 4 temp regs, 0 temp arrays:
// ALU 19 float, 0 int, 0 uint
// TEX 6 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0
eefiecednofahahioeafhacnpiojfpjgacbgdnkkabaaaaaadaagaaaaadaaaaaa
cmaaaaaaoeaaaaaabiabaaaaejfdeheolaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaakeaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaakeaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaakeaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaapalaaaakeaaaaaa
adaaaaaaaaaaaaaaadaaaaaaaeaaaaaaadadaaaakeaaaaaaaeaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapapaaaafdfgfpfaepfdejfeejepeoaafeeffiedepepfcee
aaklklklepfdeheocmaaaaaaabaaaaaaaiaaaaaacaaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaaaaaaaaaapaaaaaafdfgfpfegbhcghgfheaaklklfdeieefcbaafaaaa
eaaaaaaaeeabaaaafjaaaaaeegiocaaaaaaaaaaaajaaaaaafkaaaaadaagabaaa
aaaaaaaafkaaaaadaagabaaaabaaaaaafkaaaaadaagabaaaacaaaaaafkaaaaad
aagabaaaadaaaaaafkaaaaadaagabaaaaeaaaaaafkaaaaadaagabaaaafaaaaaa
fibiaaaeaahabaaaaaaaaaaaffffaaaafibiaaaeaahabaaaabaaaaaaffffaaaa
fibiaaaeaahabaaaacaaaaaaffffaaaafibiaaaeaahabaaaadaaaaaaffffaaaa
fibiaaaeaahabaaaaeaaaaaaffffaaaafibiaaaeaahabaaaafaaaaaaffffaaaa
gcbaaaadpcbabaaaabaaaaaagcbaaaadhcbabaaaacaaaaaagcbaaaadlcbabaaa
adaaaaaagcbaaaaddcbabaaaaeaaaaaagcbaaaadpcbabaaaafaaaaaagfaaaaad
pccabaaaaaaaaaaagiaaaaacaeaaaaaabbaaaaahbcaabaaaaaaaaaaaegbobaaa
afaaaaaaegbobaaaafaaaaaaelaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaa
dccaaaalbcaabaaaaaaaaaaaakaabaaaaaaaaaaackiacaaaaaaaaaaaaiaaaaaa
dkiacaaaaaaaaaaaaiaaaaaaefaaaaajpcaabaaaabaaaaaaegbabaaaaeaaaaaa
eghobaaaafaaaaaaaagabaaaafaaaaaadiaaaaahccaabaaaaaaaaaaadkaabaaa
abaaaaaaabeaaaaaaaaaaaebdiaaaaahocaabaaaaaaaaaaaagajbaaaabaaaaaa
fgafbaaaaaaaaaaaefaaaaajpcaabaaaabaaaaaaegbabaaaaeaaaaaaeghobaaa
aeaaaaaaaagabaaaaeaaaaaadiaaaaahicaabaaaabaaaaaadkaabaaaabaaaaaa
abeaaaaaaaaaaaebdcaaaaakhcaabaaaabaaaaaapgapbaaaabaaaaaaegacbaaa
abaaaaaajgahbaiaebaaaaaaaaaaaaaadcaaaaajhcaabaaaaaaaaaaaagaabaaa
aaaaaaaaegacbaaaabaaaaaajgahbaaaaaaaaaaaaoaaaaahdcaabaaaabaaaaaa
egbabaaaadaaaaaapgbpbaaaadaaaaaaefaaaaajpcaabaaaabaaaaaaegaabaaa
abaaaaaaeghobaaaadaaaaaaaagabaaaadaaaaaacpaaaaafpcaabaaaabaaaaaa
egaobaaaabaaaaaaaaaaaaaihcaabaaaaaaaaaaaegacbaaaaaaaaaaaegacbaia
ebaaaaaaabaaaaaabaaaaaahicaabaaaaaaaaaaaegbcbaaaacaaaaaaegbcbaaa
acaaaaaaeeaaaaaficaabaaaaaaaaaaadkaabaaaaaaaaaaadiaaaaahdcaabaaa
abaaaaaapgapbaaaaaaaaaaaegbabaaaacaaaaaadcaaaaajicaabaaaaaaaaaaa
ckbabaaaacaaaaaadkaabaaaaaaaaaaaabeaaaaadnaknhdoaoaaaaahdcaabaaa
abaaaaaaegaabaaaabaaaaaapgapbaaaaaaaaaaaefaaaaajpcaabaaaacaaaaaa
ogbkbaaaabaaaaaaeghobaaaaaaaaaaaaagabaaaacaaaaaadiaaaaaiicaabaaa
aaaaaaaackiacaaaaaaaaaaaaeaaaaaaabeaaaaaaaaaaadpdcaaaaalicaabaaa
aaaaaaaadkaabaaaacaaaaaackiacaaaaaaaaaaaaeaaaaaadkaabaiaebaaaaaa
aaaaaaaadcaaaaajdcaabaaaabaaaaaapgapbaaaaaaaaaaaegaabaaaabaaaaaa
egbabaaaabaaaaaaefaaaaajpcaabaaaacaaaaaaegaabaaaabaaaaaaeghobaaa
acaaaaaaaagabaaaabaaaaaaefaaaaajpcaabaaaadaaaaaaegaabaaaabaaaaaa
eghobaaaabaaaaaaaagabaaaaaaaaaaadiaaaaahhcaabaaaabaaaaaaegacbaaa
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
ConstBuffer "$Globals" 160 // 144 used size, 12 vars
Vector 32 [_SpecColor] 4
Vector 48 [_Color] 4
Float 68 [_Gloss]
Float 72 [_Parallax]
Vector 128 [unity_LightmapFade] 4
BindCB "$Globals" 0
SetTexture 0 [_ParallaxMap] 2D 2
SetTexture 1 [_MainTex] 2D 0
SetTexture 2 [_SpecMap] 2D 1
SetTexture 3 [_LightBuffer] 2D 3
SetTexture 4 [unity_Lightmap] 2D 4
SetTexture 5 [unity_LightmapInd] 2D 5
// 33 instructions, 4 temp regs, 0 temp arrays:
// ALU 19 float, 0 int, 0 uint
// TEX 6 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0_level_9_3
eefiecedakpkofcdgfadfinhneigapgkkhooaienabaaaaaalaajaaaaaeaaaaaa
daaaaaaakmadaaaameaiaaaahmajaaaaebgpgodjheadaaaaheadaaaaaaacpppp
caadaaaafeaaaaaaacaadmaaaaaafeaaaaaafeaaagaaceaaaaaafeaaabaaaaaa
acababaaaaacacaaadadadaaaeaeaeaaafafafaaaaaaacaaadaaaaaaaaaaaaaa
aaaaaiaaabaaadaaaaaaaaaaabacppppfbaaaaafaeaaapkaaaaaaadpdnaknhdo
aaaaaaebaaaaaaaabpaaaaacaaaaaaiaaaaaaplabpaaaaacaaaaaaiaabaaahla
bpaaaaacaaaaaaiaacaaaplabpaaaaacaaaaaaiaadaaadlabpaaaaacaaaaaaia
aeaaaplabpaaaaacaaaaaajaaaaiapkabpaaaaacaaaaaajaabaiapkabpaaaaac
aaaaaajaacaiapkabpaaaaacaaaaaajaadaiapkabpaaaaacaaaaaajaaeaiapka
bpaaaaacaaaaaajaafaiapkaajaaaaadaaaaaiiaaeaaoelaaeaaoelaahaaaaac
aaaaabiaaaaappiaagaaaaacaaaaabiaaaaaaaiaaeaaaaaeaaaabbiaaaaaaaia
adaakkkaadaappkaecaaaaadabaacpiaadaaoelaaeaioekaecaaaaadacaacpia
adaaoelaafaioekaafaaaaadacaaciiaacaappiaaeaakkkaafaaaaadaaaacoia
acaajaiaacaappiaafaaaaadabaaciiaabaappiaaeaakkkaaeaaaaaeabaaahia
abaappiaabaaoeiaaaaapjibaeaaaaaeaaaachiaaaaaaaiaabaaoeiaaaaapjia
agaaaaacaaaaaiiaacaapplaafaaaaadabaaadiaaaaappiaacaaoelaecaaaaad
abaacpiaabaaoeiaadaioekaapaaaaacacaacbiaabaaaaiaapaaaaacacaaccia
abaaffiaapaaaaacacaaceiaabaakkiaapaaaaacaaaaciiaabaappiaacaaaaad
aaaachiaaaaaoeiaacaaoeibaiaaaaadabaacbiaabaaoelaabaaoelaahaaaaac
abaacbiaabaaaaiaaeaaaaaeabaaaciaabaakklaabaaaaiaaeaaffkaafaaaaad
abaaafiaabaaaaiaabaanelaagaaaaacabaaaciaabaaffiaafaaaaadabaaadia
abaaffiaabaaoiiaabaaaaacacaaadiaaaaaoolaecaaaaadacaacpiaacaaoeia
acaioekaabaaaaacabaaaeiaacaakkkaafaaaaadabaaceiaabaakkiaaeaaaaka
aeaaaaaeabaaceiaacaappiaacaakkkaabaakkibaeaaaaaeabaaadiaabaakkia
abaaoeiaaaaaoelaecaaaaadacaacpiaabaaoeiaaaaioekaecaaaaadabaacpia
abaaoeiaabaioekaafaaaaadabaachiaabaaoeiaacaappiaafaaaaadacaachia
acaaoeiaabaaoekaafaaaaadabaachiaabaaoeiaacaaffkaafaaaaadabaachia
aaaappibabaaoeiaafaaaaadabaacoiaaaaajaiaabaajaiaaeaaaaaeaaaachia
acaaoeiaaaaaoeiaabaapjiaafaaaaadaaaaciiaabaaaaiaaaaappkaabaaaaac
aaaicpiaaaaaoeiappppaaaafdeieefcbaafaaaaeaaaaaaaeeabaaaafjaaaaae
egiocaaaaaaaaaaaajaaaaaafkaaaaadaagabaaaaaaaaaaafkaaaaadaagabaaa
abaaaaaafkaaaaadaagabaaaacaaaaaafkaaaaadaagabaaaadaaaaaafkaaaaad
aagabaaaaeaaaaaafkaaaaadaagabaaaafaaaaaafibiaaaeaahabaaaaaaaaaaa
ffffaaaafibiaaaeaahabaaaabaaaaaaffffaaaafibiaaaeaahabaaaacaaaaaa
ffffaaaafibiaaaeaahabaaaadaaaaaaffffaaaafibiaaaeaahabaaaaeaaaaaa
ffffaaaafibiaaaeaahabaaaafaaaaaaffffaaaagcbaaaadpcbabaaaabaaaaaa
gcbaaaadhcbabaaaacaaaaaagcbaaaadlcbabaaaadaaaaaagcbaaaaddcbabaaa
aeaaaaaagcbaaaadpcbabaaaafaaaaaagfaaaaadpccabaaaaaaaaaaagiaaaaac
aeaaaaaabbaaaaahbcaabaaaaaaaaaaaegbobaaaafaaaaaaegbobaaaafaaaaaa
elaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaadccaaaalbcaabaaaaaaaaaaa
akaabaaaaaaaaaaackiacaaaaaaaaaaaaiaaaaaadkiacaaaaaaaaaaaaiaaaaaa
efaaaaajpcaabaaaabaaaaaaegbabaaaaeaaaaaaeghobaaaafaaaaaaaagabaaa
afaaaaaadiaaaaahccaabaaaaaaaaaaadkaabaaaabaaaaaaabeaaaaaaaaaaaeb
diaaaaahocaabaaaaaaaaaaaagajbaaaabaaaaaafgafbaaaaaaaaaaaefaaaaaj
pcaabaaaabaaaaaaegbabaaaaeaaaaaaeghobaaaaeaaaaaaaagabaaaaeaaaaaa
diaaaaahicaabaaaabaaaaaadkaabaaaabaaaaaaabeaaaaaaaaaaaebdcaaaaak
hcaabaaaabaaaaaapgapbaaaabaaaaaaegacbaaaabaaaaaajgahbaiaebaaaaaa
aaaaaaaadcaaaaajhcaabaaaaaaaaaaaagaabaaaaaaaaaaaegacbaaaabaaaaaa
jgahbaaaaaaaaaaaaoaaaaahdcaabaaaabaaaaaaegbabaaaadaaaaaapgbpbaaa
adaaaaaaefaaaaajpcaabaaaabaaaaaaegaabaaaabaaaaaaeghobaaaadaaaaaa
aagabaaaadaaaaaacpaaaaafpcaabaaaabaaaaaaegaobaaaabaaaaaaaaaaaaai
hcaabaaaaaaaaaaaegacbaaaaaaaaaaaegacbaiaebaaaaaaabaaaaaabaaaaaah
icaabaaaaaaaaaaaegbcbaaaacaaaaaaegbcbaaaacaaaaaaeeaaaaaficaabaaa
aaaaaaaadkaabaaaaaaaaaaadiaaaaahdcaabaaaabaaaaaapgapbaaaaaaaaaaa
egbabaaaacaaaaaadcaaaaajicaabaaaaaaaaaaackbabaaaacaaaaaadkaabaaa
aaaaaaaaabeaaaaadnaknhdoaoaaaaahdcaabaaaabaaaaaaegaabaaaabaaaaaa
pgapbaaaaaaaaaaaefaaaaajpcaabaaaacaaaaaaogbkbaaaabaaaaaaeghobaaa
aaaaaaaaaagabaaaacaaaaaadiaaaaaiicaabaaaaaaaaaaackiacaaaaaaaaaaa
aeaaaaaaabeaaaaaaaaaaadpdcaaaaalicaabaaaaaaaaaaadkaabaaaacaaaaaa
ckiacaaaaaaaaaaaaeaaaaaadkaabaiaebaaaaaaaaaaaaaadcaaaaajdcaabaaa
abaaaaaapgapbaaaaaaaaaaaegaabaaaabaaaaaaegbabaaaabaaaaaaefaaaaaj
pcaabaaaacaaaaaaegaabaaaabaaaaaaeghobaaaacaaaaaaaagabaaaabaaaaaa
efaaaaajpcaabaaaadaaaaaaegaabaaaabaaaaaaeghobaaaabaaaaaaaagabaaa
aaaaaaaadiaaaaahhcaabaaaabaaaaaaegacbaaaacaaaaaapgapbaaaadaaaaaa
diaaaaaihcaabaaaacaaaaaaegacbaaaadaaaaaaegiccaaaaaaaaaaaadaaaaaa
diaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaafgifcaaaaaaaaaaaaeaaaaaa
diaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaapgapbaiaebaaaaaaabaaaaaa
diaaaaahocaabaaaabaaaaaaagajbaaaaaaaaaaaagajbaaaabaaaaaadcaaaaaj
hccabaaaaaaaaaaaegacbaaaacaaaaaaegacbaaaaaaaaaaajgahbaaaabaaaaaa
diaaaaaiiccabaaaaaaaaaaaakaabaaaabaaaaaadkiacaaaaaaaaaaaacaaaaaa
doaaaaabejfdeheolaaaaaaaagaaaaaaaiaaaaaajiaaaaaaaaaaaaaaabaaaaaa
adaaaaaaaaaaaaaaapaaaaaakeaaaaaaaaaaaaaaaaaaaaaaadaaaaaaabaaaaaa
apapaaaakeaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaaahahaaaakeaaaaaa
acaaaaaaaaaaaaaaadaaaaaaadaaaaaaapalaaaakeaaaaaaadaaaaaaaaaaaaaa
adaaaaaaaeaaaaaaadadaaaakeaaaaaaaeaaaaaaaaaaaaaaadaaaaaaafaaaaaa
apapaaaafdfgfpfaepfdejfeejepeoaafeeffiedepepfceeaaklklklepfdeheo
cmaaaaaaabaaaaaaaiaaaaaacaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaaaaaaaaa
apaaaaaafdfgfpfegbhcghgfheaaklkl"
}

SubProgram "opengl " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
Vector 0 [_SpecColor]
Vector 1 [_Color]
Float 2 [_Gloss]
Float 3 [_Parallax]
SetTexture 0 [_ParallaxMap] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 2 [_SpecMap] 2D
SetTexture 4 [_LightBuffer] 2D
"!!ARBfp1.0
OPTION ARB_precision_hint_fastest;
# 22 ALU, 4 TEX
PARAM c[5] = { program.local[0..3],
		{ 0.5, 0.41999999 } };
TEMP R0;
TEMP R1;
TEMP R2;
TEX R0.w, fragment.texcoord[0].zwzw, texture[0], 2D;
DP3 R0.x, fragment.texcoord[1], fragment.texcoord[1];
RSQ R0.x, R0.x;
MUL R1.xyz, R0.x, fragment.texcoord[1];
ADD R0.y, R1.z, c[4];
RCP R0.y, R0.y;
MOV R0.x, c[3];
MUL R0.x, R0, c[4];
MUL R1.xy, R1, R0.y;
MAD R0.x, R0.w, c[3], -R0;
MAD R1.xy, R0.x, R1, fragment.texcoord[0];
TXP R0, fragment.texcoord[2], texture[4], 2D;
TEX R2.xyz, R1, texture[2], 2D;
TEX R1, R1, texture[1], 2D;
MUL R2.xyz, R1.w, R2;
MUL R2.xyz, R2, c[2].x;
MUL R2.xyz, R0.w, R2;
ADD R0.xyz, R0, fragment.texcoord[3];
MUL R2.yzw, R2.xxyz, R0.xxyz;
MUL R1.xyz, R1, c[1];
MAD result.color.xyz, R1, R0, R2.yzww;
MUL result.color.w, R2.x, c[0];
END
# 22 instructions, 3 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
Vector 0 [_SpecColor]
Vector 1 [_Color]
Float 2 [_Gloss]
Float 3 [_Parallax]
SetTexture 0 [_ParallaxMap] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 2 [_SpecMap] 2D
SetTexture 4 [_LightBuffer] 2D
"ps_2_0
; 21 ALU, 4 TEX
dcl_2d s0
dcl_2d s1
dcl_2d s2
dcl_2d s4
def c4, 0.50000000, 0.41999999, 0, 0
dcl t0
dcl t1.xyz
dcl t2
dcl t3.xyz
mov r0.y, t0.w
mov r0.x, t0.z
texld r0, r0, s0
dp3_pp r0.x, t1, t1
rsq_pp r0.x, r0.x
mul_pp r2.xyz, r0.x, t1
add r0.x, r2.z, c4.y
rcp r1.x, r0.x
mul r1.xy, r2, r1.x
mov_pp r0.x, c4
mul_pp r0.x, c3, r0
mad_pp r0.x, r0.w, c3, -r0
mad r0.xy, r0.x, r1, t0
texld r1, r0, s1
texld r0, r0, s2
texldp r2, t2, s4
mul_pp r0.xyz, r1.w, r0
mul_pp r0.xyz, r0, c2.x
mul_pp r0.xyz, r2.w, r0
add_pp r2.xyz, r2, t3
mul_pp r3.xyz, r0, r2
mul_pp r1.xyz, r1, c1
mad_pp r1.xyz, r1, r2, r3
mul_pp r1.w, r0.x, c0
mov_pp oC0, r1
"
}

SubProgram "xbox360 " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
Vector 1 [_Color]
Float 2 [_Gloss]
Float 3 [_Parallax]
Vector 0 [_SpecColor]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_ParallaxMap] 2D
SetTexture 3 [_LightBuffer] 2D
SetTexture 4 [_LightSpecBuffer] 2D
// Shader Timing Estimate, in Cycles/64 pixel vector:
// ALU: 20.00 (15 instructions), vertex: 0, texture: 20,
//   sequencer: 12, interpolator: 16;    5 GPRs, 36 threads,
// Performance (if enough threads): ~20 cycles per vector
// * Texture cycle estimates are assuming an 8bit/component texture with no
//     aniso or trilinear filtering.

"ps_360
backbbaaaaaaabpiaaaaabgaaaaaaaaaaaaaaaceaaaaabkaaaaaabmiaaaaaaaa
aaaaaaaaaaaaabhiaaaaaabmaaaaabgmppppadaaaaaaaaajaaaaaabmaaaaaaaa
aaaaabgfaaaaaanaaaacaaabaaabaaaaaaaaaaniaaaaaaaaaaaaaaoiaaacaaac
aaabaaaaaaaaaapaaaaaaaaaaaaaabaaaaadaaadaaabaaaaaaaaabbaaaaaaaaa
aaaaabcaaaadaaaeaaabaaaaaaaaabbaaaaaaaaaaaaaabdbaaadaaaaaaabaaaa
aaaaabbaaaaaaaaaaaaaabdkaaacaaadaaabaaaaaaaaaapaaaaaaaaaaaaaabee
aaadaaacaaabaaaaaaaaabbaaaaaaaaaaaaaabfbaaacaaaaaaabaaaaaaaaaani
aaaaaaaaaaaaabfmaaadaaabaaabaaaaaaaaabbaaaaaaaaafpedgpgmgphcaakl
aaabaaadaaabaaaeaaabaaaaaaaaaaaafpehgmgphdhdaaklaaaaaaadaaabaaab
aaabaaaaaaaaaaaafpemgjghgiheechfgggggfhcaaklklklaaaeaaamaaabaaab
aaabaaaaaaaaaaaafpemgjghgihefdhagfgdechfgggggfhcaafpengbgjgofegf
hiaafpfagbhcgbgmgmgbhiaafpfagbhcgbgmgmgbhiengbhaaafpfdhagfgdedgp
gmgphcaafpfdhagfgdengbhaaahahdfpddfpdaaadccodacodcdadddfddcodaaa
aaaaaaaaaaaaaaabaaaaaaaaaaaaaaaaaaaaaabeabpmaabaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaeaaaaaabcabaaaaeaaaaaaaaaeaaaaaaaaaaaadiie
aaapaaapaaaaaaabaaaapafaaaaahbfbaaaapcfcaaaahdfdaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaadonhakdndpaaaaaaaaaaaaaaaaaaaaaaaaajgaadgaajbcaa
bcaaafeaaaabbaapaaaabcaameaaaaaaaaaagabababgbcaaccaaaaaadicaeaab
bpbpppplaaaaeaaamiaiaaadaaloloaapaababaafiiiadabaagmlbblcbadppid
miaiaaababgmgmblklaeadabmiahaaabaablloaaobadabaaembiaeadaagmgmbl
kaabppacembdabacaagmlablobaeacadmiadaaabaamfgmaaobababaamiadaaaa
aalabllaolababaabaeibaebbpbppppiaaaaeaaababibaabbpbppeehaaaaeaaa
badicaebbpbppoiiaaaaeaaabaaiaaabbpbppgecaaaaeaaamiahaaacaamamaaa
oaacadaabecoaaabaablablbobaaabaakiboadabaaabgmebibabacabkichadab
aabfgmicmbabababkmiaiaaaaaaaaaaamcaaaaaakiehadabaamamamambabacab
miahiaaaaamamamaoladacabaaaaaaaaaaaaaaaaaaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
Vector 0 [_SpecColor]
Vector 1 [_Color]
Float 2 [_Gloss]
Float 3 [_Parallax]
SetTexture 0 [_ParallaxMap] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 2 [_SpecMap] 2D
SetTexture 4 [_LightBuffer] 2D
"sce_fp_rsx // 28 instructions using 3 registers
[Configuration]
24
ffffffff0003c020000ffff0000000000000840003000000
[Offsets]
4
_SpecColor 1 0
000001a0
_Color 1 0
00000100
_Gloss 1 0
00000130
_Parallax 2 0
0000005000000030
[Microcode]
448
9800010080011c9cc8000001c8003fe1900417005c011c9dc8000001c8003fe1
02880240fe081c9d00020000c800000100000000000000000000000000000000
1088014000021c9cc8000001c800000100000000000000000000000000000000
ae803940c8011c9dc8000029c800bfe10400030055001c9d00020000c8000001
0a3d3ed7000000000000000000000000de021808c8011c9dc8000001c8003fe1
02880440ff101c9d00020000c91000010000bf00000000000000000000000000
06003a00c9001c9daa000000c80000010604040001101c9cc80000015c000001
1e001702c8081c9dc8000001c80000010e800240c8001c9dc8020001c8000001
000000000000000000000000000000001e7e7d00c8001c9dc8000001c8000001
10800240c8001c9d00020000c800000100000000000000000000000000000000
ee820300c8011c9dc8040001c8003fe110800240c8041c9dc9000001c8000001
0e041704c8081c9dc8000001c80000010e880240ff001c9dc8080001c8000001
0e840240c9101c9dc9040001c80000011080024001101c9cc8020001c8000001
000000000000000000000000000000000e810440c9001c9dc9040001c9080001
"
}

SubProgram "d3d11 " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
ConstBuffer "$Globals" 128 // 76 used size, 10 vars
Vector 32 [_SpecColor] 4
Vector 48 [_Color] 4
Float 68 [_Gloss]
Float 72 [_Parallax]
BindCB "$Globals" 0
SetTexture 0 [_ParallaxMap] 2D 2
SetTexture 1 [_MainTex] 2D 0
SetTexture 2 [_SpecMap] 2D 1
SetTexture 3 [_LightBuffer] 2D 3
// 22 instructions, 3 temp regs, 0 temp arrays:
// ALU 13 float, 0 int, 0 uint
// TEX 4 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0
eefiecedeckcijebokfloipimhkkigaidmkdafnpabaaaaaaheaeaaaaadaaaaaa
cmaaaaaammaaaaaaaaabaaaaejfdeheojiaaaaaaafaaaaaaaiaaaaaaiaaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaaimaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaapalaaaaimaaaaaa
adaaaaaaaaaaaaaaadaaaaaaaeaaaaaaahahaaaafdfgfpfaepfdejfeejepeoaa
feeffiedepepfceeaaklklklepfdeheocmaaaaaaabaaaaaaaiaaaaaacaaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaafdfgfpfegbhcghgfheaaklkl
fdeieefcgmadaaaaeaaaaaaanlaaaaaafjaaaaaeegiocaaaaaaaaaaaafaaaaaa
fkaaaaadaagabaaaaaaaaaaafkaaaaadaagabaaaabaaaaaafkaaaaadaagabaaa
acaaaaaafkaaaaadaagabaaaadaaaaaafibiaaaeaahabaaaaaaaaaaaffffaaaa
fibiaaaeaahabaaaabaaaaaaffffaaaafibiaaaeaahabaaaacaaaaaaffffaaaa
fibiaaaeaahabaaaadaaaaaaffffaaaagcbaaaadpcbabaaaabaaaaaagcbaaaad
hcbabaaaacaaaaaagcbaaaadlcbabaaaadaaaaaagcbaaaadhcbabaaaaeaaaaaa
gfaaaaadpccabaaaaaaaaaaagiaaaaacadaaaaaabaaaaaahbcaabaaaaaaaaaaa
egbcbaaaacaaaaaaegbcbaaaacaaaaaaeeaaaaafbcaabaaaaaaaaaaaakaabaaa
aaaaaaaadiaaaaahgcaabaaaaaaaaaaaagaabaaaaaaaaaaaagbbbaaaacaaaaaa
dcaaaaajbcaabaaaaaaaaaaackbabaaaacaaaaaaakaabaaaaaaaaaaaabeaaaaa
dnaknhdoaoaaaaahdcaabaaaaaaaaaaajgafbaaaaaaaaaaaagaabaaaaaaaaaaa
efaaaaajpcaabaaaabaaaaaaogbkbaaaabaaaaaaeghobaaaaaaaaaaaaagabaaa
acaaaaaadiaaaaaiecaabaaaaaaaaaaackiacaaaaaaaaaaaaeaaaaaaabeaaaaa
aaaaaadpdcaaaaalecaabaaaaaaaaaaadkaabaaaabaaaaaackiacaaaaaaaaaaa
aeaaaaaackaabaiaebaaaaaaaaaaaaaadcaaaaajdcaabaaaaaaaaaaakgakbaaa
aaaaaaaaegaabaaaaaaaaaaaegbabaaaabaaaaaaefaaaaajpcaabaaaabaaaaaa
egaabaaaaaaaaaaaeghobaaaacaaaaaaaagabaaaabaaaaaaefaaaaajpcaabaaa
aaaaaaaaegaabaaaaaaaaaaaeghobaaaabaaaaaaaagabaaaaaaaaaaadiaaaaah
hcaabaaaabaaaaaaegacbaaaabaaaaaapgapbaaaaaaaaaaadiaaaaaihcaabaaa
aaaaaaaaegacbaaaaaaaaaaaegiccaaaaaaaaaaaadaaaaaadiaaaaaihcaabaaa
abaaaaaaegacbaaaabaaaaaafgifcaaaaaaaaaaaaeaaaaaaaoaaaaahdcaabaaa
acaaaaaaegbabaaaadaaaaaapgbpbaaaadaaaaaaefaaaaajpcaabaaaacaaaaaa
egaabaaaacaaaaaaeghobaaaadaaaaaaaagabaaaadaaaaaadiaaaaahhcaabaaa
abaaaaaaegacbaaaabaaaaaapgapbaaaacaaaaaaaaaaaaahhcaabaaaacaaaaaa
egacbaaaacaaaaaaegbcbaaaaeaaaaaadiaaaaahocaabaaaabaaaaaaagajbaaa
abaaaaaaagajbaaaacaaaaaadiaaaaaiiccabaaaaaaaaaaaakaabaaaabaaaaaa
dkiacaaaaaaaaaaaacaaaaaadcaaaaajhccabaaaaaaaaaaaegacbaaaaaaaaaaa
egacbaaaacaaaaaajgahbaaaabaaaaaadoaaaaab"
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
ConstBuffer "$Globals" 128 // 76 used size, 10 vars
Vector 32 [_SpecColor] 4
Vector 48 [_Color] 4
Float 68 [_Gloss]
Float 72 [_Parallax]
BindCB "$Globals" 0
SetTexture 0 [_ParallaxMap] 2D 2
SetTexture 1 [_MainTex] 2D 0
SetTexture 2 [_SpecMap] 2D 1
SetTexture 3 [_LightBuffer] 2D 3
// 22 instructions, 3 temp regs, 0 temp arrays:
// ALU 13 float, 0 int, 0 uint
// TEX 4 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0_level_9_3
eefiecedbpmkenhdemcpgkdahhknkpgmlipnfcncabaaaaaaniagaaaaaeaaaaaa
daaaaaaajaacaaaaaeagaaaakeagaaaaebgpgodjfiacaaaafiacaaaaaaacpppp
biacaaaaeaaaaaaaabaadeaaaaaaeaaaaaaaeaaaaeaaceaaaaaaeaaaabaaaaaa
acababaaaaacacaaadadadaaaaaaacaaadaaaaaaaaaaaaaaabacppppfbaaaaaf
adaaapkaaaaaaadpdnaknhdoaaaaaaaaaaaaaaaabpaaaaacaaaaaaiaaaaaapla
bpaaaaacaaaaaaiaabaaahlabpaaaaacaaaaaaiaacaaaplabpaaaaacaaaaaaia
adaaahlabpaaaaacaaaaaajaaaaiapkabpaaaaacaaaaaajaabaiapkabpaaaaac
aaaaaajaacaiapkabpaaaaacaaaaaajaadaiapkaaiaaaaadaaaaciiaabaaoela
abaaoelaahaaaaacaaaacbiaaaaappiaaeaaaaaeaaaaaciaabaakklaaaaaaaia
adaaffkaafaaaaadaaaaafiaaaaaaaiaabaanelaagaaaaacaaaaaciaaaaaffia
afaaaaadaaaaadiaaaaaffiaaaaaoiiaabaaaaacabaaadiaaaaaoolaecaaaaad
abaacpiaabaaoeiaacaioekaabaaaaacaaaaaeiaacaakkkaafaaaaadaaaaceia
aaaakkiaadaaaakaaeaaaaaeaaaaceiaabaappiaacaakkkaaaaakkibaeaaaaae
aaaaadiaaaaakkiaaaaaoeiaaaaaoelaecaaaaadabaacpiaaaaaoeiaaaaioeka
ecaaaaadaaaacpiaaaaaoeiaabaioekaafaaaaadaaaachiaaaaaoeiaabaappia
afaaaaadabaachiaabaaoeiaabaaoekaafaaaaadaaaachiaaaaaoeiaacaaffka
agaaaaacaaaaaiiaacaapplaafaaaaadacaaadiaaaaappiaacaaoelaecaaaaad
acaacpiaacaaoeiaadaioekaafaaaaadaaaachiaaaaaoeiaacaappiaacaaaaad
acaachiaacaaoeiaadaaoelaafaaaaadaaaacoiaaaaajaiaacaajaiaafaaaaad
adaaciiaaaaaaaiaaaaappkaaeaaaaaeadaachiaabaaoeiaacaaoeiaaaaapjia
abaaaaacaaaicpiaadaaoeiappppaaaafdeieefcgmadaaaaeaaaaaaanlaaaaaa
fjaaaaaeegiocaaaaaaaaaaaafaaaaaafkaaaaadaagabaaaaaaaaaaafkaaaaad
aagabaaaabaaaaaafkaaaaadaagabaaaacaaaaaafkaaaaadaagabaaaadaaaaaa
fibiaaaeaahabaaaaaaaaaaaffffaaaafibiaaaeaahabaaaabaaaaaaffffaaaa
fibiaaaeaahabaaaacaaaaaaffffaaaafibiaaaeaahabaaaadaaaaaaffffaaaa
gcbaaaadpcbabaaaabaaaaaagcbaaaadhcbabaaaacaaaaaagcbaaaadlcbabaaa
adaaaaaagcbaaaadhcbabaaaaeaaaaaagfaaaaadpccabaaaaaaaaaaagiaaaaac
adaaaaaabaaaaaahbcaabaaaaaaaaaaaegbcbaaaacaaaaaaegbcbaaaacaaaaaa
eeaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaadiaaaaahgcaabaaaaaaaaaaa
agaabaaaaaaaaaaaagbbbaaaacaaaaaadcaaaaajbcaabaaaaaaaaaaackbabaaa
acaaaaaaakaabaaaaaaaaaaaabeaaaaadnaknhdoaoaaaaahdcaabaaaaaaaaaaa
jgafbaaaaaaaaaaaagaabaaaaaaaaaaaefaaaaajpcaabaaaabaaaaaaogbkbaaa
abaaaaaaeghobaaaaaaaaaaaaagabaaaacaaaaaadiaaaaaiecaabaaaaaaaaaaa
ckiacaaaaaaaaaaaaeaaaaaaabeaaaaaaaaaaadpdcaaaaalecaabaaaaaaaaaaa
dkaabaaaabaaaaaackiacaaaaaaaaaaaaeaaaaaackaabaiaebaaaaaaaaaaaaaa
dcaaaaajdcaabaaaaaaaaaaakgakbaaaaaaaaaaaegaabaaaaaaaaaaaegbabaaa
abaaaaaaefaaaaajpcaabaaaabaaaaaaegaabaaaaaaaaaaaeghobaaaacaaaaaa
aagabaaaabaaaaaaefaaaaajpcaabaaaaaaaaaaaegaabaaaaaaaaaaaeghobaaa
abaaaaaaaagabaaaaaaaaaaadiaaaaahhcaabaaaabaaaaaaegacbaaaabaaaaaa
pgapbaaaaaaaaaaadiaaaaaihcaabaaaaaaaaaaaegacbaaaaaaaaaaaegiccaaa
aaaaaaaaadaaaaaadiaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaafgifcaaa
aaaaaaaaaeaaaaaaaoaaaaahdcaabaaaacaaaaaaegbabaaaadaaaaaapgbpbaaa
adaaaaaaefaaaaajpcaabaaaacaaaaaaegaabaaaacaaaaaaeghobaaaadaaaaaa
aagabaaaadaaaaaadiaaaaahhcaabaaaabaaaaaaegacbaaaabaaaaaapgapbaaa
acaaaaaaaaaaaaahhcaabaaaacaaaaaaegacbaaaacaaaaaaegbcbaaaaeaaaaaa
diaaaaahocaabaaaabaaaaaaagajbaaaabaaaaaaagajbaaaacaaaaaadiaaaaai
iccabaaaaaaaaaaaakaabaaaabaaaaaadkiacaaaaaaaaaaaacaaaaaadcaaaaaj
hccabaaaaaaaaaaaegacbaaaaaaaaaaaegacbaaaacaaaaaajgahbaaaabaaaaaa
doaaaaabejfdeheojiaaaaaaafaaaaaaaiaaaaaaiaaaaaaaaaaaaaaaabaaaaaa
adaaaaaaaaaaaaaaapaaaaaaimaaaaaaaaaaaaaaaaaaaaaaadaaaaaaabaaaaaa
apapaaaaimaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaaahahaaaaimaaaaaa
acaaaaaaaaaaaaaaadaaaaaaadaaaaaaapalaaaaimaaaaaaadaaaaaaaaaaaaaa
adaaaaaaaeaaaaaaahahaaaafdfgfpfaepfdejfeejepeoaafeeffiedepepfcee
aaklklklepfdeheocmaaaaaaabaaaaaaaiaaaaaacaaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaaaaaaaaaapaaaaaafdfgfpfegbhcghgfheaaklkl"
}

SubProgram "opengl " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
Vector 0 [_SpecColor]
Vector 1 [_Color]
Float 2 [_Gloss]
Float 3 [_Parallax]
Vector 4 [unity_LightmapFade]
SetTexture 0 [_ParallaxMap] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 2 [_SpecMap] 2D
SetTexture 4 [_LightBuffer] 2D
SetTexture 5 [unity_Lightmap] 2D
SetTexture 6 [unity_LightmapInd] 2D
"!!ARBfp1.0
OPTION ARB_precision_hint_fastest;
# 33 ALU, 6 TEX
PARAM c[6] = { program.local[0..4],
		{ 0.5, 0.41999999, 8 } };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
TEMP R4;
TEX R3, fragment.texcoord[3], texture[6], 2D;
TEX R0.w, fragment.texcoord[0].zwzw, texture[0], 2D;
DP3 R0.x, fragment.texcoord[1], fragment.texcoord[1];
RSQ R0.x, R0.x;
MUL R1.xyz, R0.x, fragment.texcoord[1];
ADD R0.y, R1.z, c[5];
RCP R0.y, R0.y;
MOV R0.x, c[3];
MUL R0.x, R0, c[5];
MUL R3.xyz, R3.w, R3;
MUL R1.xy, R1, R0.y;
MAD R0.x, R0.w, c[3], -R0;
MAD R2.xy, R0.x, R1, fragment.texcoord[0];
MUL R3.xyz, R3, c[5].z;
TEX R1, R2, texture[1], 2D;
TEX R4.xyz, R2, texture[2], 2D;
TEX R2, fragment.texcoord[3], texture[5], 2D;
TXP R0, fragment.texcoord[2], texture[4], 2D;
MUL R2.xyz, R2.w, R2;
DP4 R2.w, fragment.texcoord[4], fragment.texcoord[4];
RSQ R2.w, R2.w;
MAD R2.xyz, R2, c[5].z, -R3;
MUL R4.xyz, R1.w, R4;
RCP R2.w, R2.w;
MAD_SAT R1.w, R2, c[4].z, c[4];
MAD R2.xyz, R1.w, R2, R3;
ADD R0.xyz, R0, R2;
MUL R3.xyz, R4, c[2].x;
MUL R2.xyz, R0.w, R3;
MUL R3.xyz, R2, R0;
MUL R1.xyz, R1, c[1];
MAD result.color.xyz, R1, R0, R3;
MUL result.color.w, R2.x, c[0];
END
# 33 instructions, 5 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
Vector 0 [_SpecColor]
Vector 1 [_Color]
Float 2 [_Gloss]
Float 3 [_Parallax]
Vector 4 [unity_LightmapFade]
SetTexture 0 [_ParallaxMap] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 2 [_SpecMap] 2D
SetTexture 4 [_LightBuffer] 2D
SetTexture 5 [unity_Lightmap] 2D
SetTexture 6 [unity_LightmapInd] 2D
"ps_2_0
; 30 ALU, 6 TEX
dcl_2d s0
dcl_2d s1
dcl_2d s2
dcl_2d s4
dcl_2d s5
dcl_2d s6
def c5, 0.50000000, 0.41999999, 8.00000000, 0
dcl t0
dcl t1.xyz
dcl t2
dcl t3.xy
dcl t4
texld r3, t3, s5
mov r0.y, t0.w
mov r0.x, t0.z
texld r0, r0, s0
dp3_pp r0.x, t1, t1
rsq_pp r0.x, r0.x
mul_pp r2.xyz, r0.x, t1
add r0.x, r2.z, c5.y
rcp r1.x, r0.x
mul r1.xy, r2, r1.x
mov_pp r0.x, c5
mul_pp r0.x, c3, r0
mad_pp r0.x, r0.w, c3, -r0
mad r0.xy, r0.x, r1, t0
texld r4, r0, s2
texld r1, r0, s1
texldp r2, t2, s4
texld r0, t3, s6
mul_pp r0.xyz, r0.w, r0
mul_pp r5.xyz, r0, c5.z
mul_pp r0.xyz, r3.w, r3
mad_pp r3.xyz, r0, c5.z, -r5
dp4 r0.x, t4, t4
rsq r0.x, r0.x
rcp r0.x, r0.x
mad_sat r0.x, r0, c4.z, c4.w
mad_pp r0.xyz, r0.x, r3, r5
mul_pp r4.xyz, r1.w, r4
add_pp r0.xyz, r2, r0
mul_pp r3.xyz, r4, c2.x
mul_pp r2.xyz, r2.w, r3
mul_pp r3.xyz, r2, r0
mul_pp r1.xyz, r1, c1
mad_pp r0.xyz, r1, r0, r3
mul_pp r0.w, r2.x, c0
mov_pp oC0, r0
"
}

SubProgram "xbox360 " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
Vector 1 [_Color]
Float 2 [_Gloss]
Float 3 [_Parallax]
Vector 0 [_SpecColor]
Vector 4 [unity_LightmapFade]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_ParallaxMap] 2D
SetTexture 3 [_LightBuffer] 2D
SetTexture 4 [_LightSpecBuffer] 2D
SetTexture 5 [unity_Lightmap] 2D
SetTexture 6 [unity_LightmapInd] 2D
// Shader Timing Estimate, in Cycles/64 pixel vector:
// ALU: 26.67 (20 instructions), vertex: 0, texture: 28,
//   sequencer: 12, interpolator: 20;    8 GPRs, 24 threads,
// Performance (if enough threads): ~28 cycles per vector
// * Texture cycle estimates are assuming an 8bit/component texture with no
//     aniso or trilinear filtering.

"ps_360
backbbaaaaaaacgmaaaaableaaaaaaaaaaaaaaceaaaaacbaaaaaacdiaaaaaaaa
aaaaaaaaaaaaaboiaaaaaabmaaaaabnmppppadaaaaaaaaamaaaaaabmaaaaaaaa
aaaaabnfaaaaabamaaacaaabaaabaaaaaaaaabbeaaaaaaaaaaaaabceaaacaaac
aaabaaaaaaaaabcmaaaaaaaaaaaaabdmaaadaaadaaabaaaaaaaaabemaaaaaaaa
aaaaabfmaaadaaaeaaabaaaaaaaaabemaaaaaaaaaaaaabgnaaadaaaaaaabaaaa
aaaaabemaaaaaaaaaaaaabhgaaacaaadaaabaaaaaaaaabcmaaaaaaaaaaaaabia
aaadaaacaaabaaaaaaaaabemaaaaaaaaaaaaabinaaacaaaaaaabaaaaaaaaabbe
aaaaaaaaaaaaabjiaaadaaabaaabaaaaaaaaabemaaaaaaaaaaaaabkbaaadaaaf
aaabaaaaaaaaabemaaaaaaaaaaaaablaaaacaaaeaaabaaaaaaaaabbeaaaaaaaa
aaaaabmdaaadaaagaaabaaaaaaaaabemaaaaaaaafpedgpgmgphcaaklaaabaaad
aaabaaaeaaabaaaaaaaaaaaafpehgmgphdhdaaklaaaaaaadaaabaaabaaabaaaa
aaaaaaaafpemgjghgiheechfgggggfhcaaklklklaaaeaaamaaabaaabaaabaaaa
aaaaaaaafpemgjghgihefdhagfgdechfgggggfhcaafpengbgjgofegfhiaafpfa
gbhcgbgmgmgbhiaafpfagbhcgbgmgmgbhiengbhaaafpfdhagfgdedgpgmgphcaa
fpfdhagfgdengbhaaahfgogjhehjfpemgjghgihegngbhaaahfgogjhehjfpemgj
ghgihegngbhaeggbgegfaahfgogjhehjfpemgjghgihegngbhaejgogeaahahdfp
ddfpdaaadccodacodcdadddfddcodaaaaaaaaaaaaaaaaaabaaaaaaaaaaaaaaaa
aaaaaabeabpmaabaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaeaaaaaabhe
baaaahaaaaaaaaaeaaaaaaaaaaaaeekfaabpaabpaaaaaaabaaaapafaaaaahbfb
aaaapcfcaaaaddfdaaaapefeaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaebaaaaaa
dpaaaaaadonhakdnaaaaaaaaaajfgaadgaajbcaabcaaaaaaabfefaapaaaabcaa
meaaaaaaaaaagabeeabkbcaaccaaaaaabafahagbbpbppgiiaaaaeaaabagadagb
bpbppgiiaaaaeaaadicaaaabbpbpphppaaaaeaaamiabaaafaagmlbaacbadppaa
emeiafabaablgmblkbadppacbeecaaafaaloloblpaababahkibdacagaamglaec
mbafacppfiboacacaagmpmlbobacahifkibhacabaagmloedmbacabadleioabaf
aablpmmambabadppemipabadacaaaabloaacafabmiadaaabaamfblaaobababaa
miadaaaaaalagmlaolabadaabadicambbpbppoiiaaaaeaaabaeibambbpbppppi
aaaaeaaababigaabbpbppoiiaaaaeaaabaaiaaabbpbppgecaaaaeaaamiacaaab
aakhkhaaopaeaeaakachabaeaablmalbobaaagibmjaiaaabaalbmgblilabaeae
bechaaaeaamagmlbkbaeacaakibhadabaamagmebmbaeababkmiaiaaaaaaaaaaa
mcaaaaaamiaoaaadaaabblaboladabafkichadacaabfmaicmaadacabkiehadab
aamamamambabacabmiahiaaaaamamamaoladacabaaaaaaaaaaaaaaaaaaaaaaaa
"
}

SubProgram "ps3 " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
Vector 0 [_SpecColor]
Vector 1 [_Color]
Float 2 [_Gloss]
Float 3 [_Parallax]
Vector 4 [unity_LightmapFade]
SetTexture 0 [_ParallaxMap] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 2 [_SpecMap] 2D
SetTexture 4 [_LightBuffer] 2D
SetTexture 5 [unity_Lightmap] 2D
SetTexture 6 [unity_LightmapInd] 2D
"sce_fp_rsx // 40 instructions using 4 registers
[Configuration]
24
ffffffff0007c020001fffe8000000000000840004000000
[Offsets]
5
_SpecColor 1 0
00000260
_Color 1 0
00000150
_Gloss 1 0
00000180
_Parallax 2 0
000000c0000000a0
unity_LightmapFade 2 0
000001d0000001b0
[Microcode]
640
1e020101c8011c9dc8000001c8003fe108060600c8041c9dc8040001c8000001
fe04170cc8011c9dc8000001c8003fe1ae863940c8011c9dc8000029c800bfe1
10020300550c1c9d00020000c80000010a3d3ed7000000000000000000000000
0e840240fe081c9dc8083001c80000019804010080011c9cc8000001c8003fe1
900617005c011c9dc8000001c8003fe102880240fe0c1c9d00020000c8000001
000000000000000000000000000000001088014000021c9cc8000001c8000001
00000000000000000000000000000000028c0440ff101c9d00020000c9100001
0000bf0000000000000000000000000018023a00810c1c9cfe040001c8000001
0606040001181c9c5c0400015c080001fe00170ac8011c9dc8000001c8003fe1
10840140c8001c9dc8003001c80000011e041702c80c1c9dc8000001c8000001
0e860240c8081c9dc8020001c800000100000000000000000000000000000000
10001b00540c1c9dc8000001c8000001028e0240fe081c9d00020000c8000001
000000000000000000000000000000000e000400ff081c9dc8000001c9080003
10003a0054021c9dfe000001c800000100000000000000000000000000000000
10008300c8001c9dc8020001c800000100000000000000000000000000000000
0e800400fe001c9dc8000001c9080001de041808c8011c9dc8000001c8003fe1
0e800340c8081c9dc9000001c800000110800240c8081c9d011c0000c8000001
0e041704c80c1c9dc8000001c80000010e820240ff001c9dc8080001c8000001
0e840240c9041c9dc9000001c80000011080024001041c9cc8020001c8000001
000000000000000000000000000000000e810440c90c1c9dc9000001c9080001
"
}

SubProgram "d3d11 " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
ConstBuffer "$Globals" 160 // 144 used size, 12 vars
Vector 32 [_SpecColor] 4
Vector 48 [_Color] 4
Float 68 [_Gloss]
Float 72 [_Parallax]
Vector 128 [unity_LightmapFade] 4
BindCB "$Globals" 0
SetTexture 0 [_ParallaxMap] 2D 2
SetTexture 1 [_MainTex] 2D 0
SetTexture 2 [_SpecMap] 2D 1
SetTexture 3 [_LightBuffer] 2D 3
SetTexture 4 [unity_Lightmap] 2D 4
SetTexture 5 [unity_LightmapInd] 2D 5
// 32 instructions, 4 temp regs, 0 temp arrays:
// ALU 18 float, 0 int, 0 uint
// TEX 6 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0
eefiecedbonbidimppbefgmammgebcmdiohncahcabaaaaaabeagaaaaadaaaaaa
cmaaaaaaoeaaaaaabiabaaaaejfdeheolaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaakeaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaakeaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaakeaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaapalaaaakeaaaaaa
adaaaaaaaaaaaaaaadaaaaaaaeaaaaaaadadaaaakeaaaaaaaeaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapapaaaafdfgfpfaepfdejfeejepeoaafeeffiedepepfcee
aaklklklepfdeheocmaaaaaaabaaaaaaaiaaaaaacaaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaaaaaaaaaapaaaaaafdfgfpfegbhcghgfheaaklklfdeieefcpeaeaaaa
eaaaaaaadnabaaaafjaaaaaeegiocaaaaaaaaaaaajaaaaaafkaaaaadaagabaaa
aaaaaaaafkaaaaadaagabaaaabaaaaaafkaaaaadaagabaaaacaaaaaafkaaaaad
aagabaaaadaaaaaafkaaaaadaagabaaaaeaaaaaafkaaaaadaagabaaaafaaaaaa
fibiaaaeaahabaaaaaaaaaaaffffaaaafibiaaaeaahabaaaabaaaaaaffffaaaa
fibiaaaeaahabaaaacaaaaaaffffaaaafibiaaaeaahabaaaadaaaaaaffffaaaa
fibiaaaeaahabaaaaeaaaaaaffffaaaafibiaaaeaahabaaaafaaaaaaffffaaaa
gcbaaaadpcbabaaaabaaaaaagcbaaaadhcbabaaaacaaaaaagcbaaaadlcbabaaa
adaaaaaagcbaaaaddcbabaaaaeaaaaaagcbaaaadpcbabaaaafaaaaaagfaaaaad
pccabaaaaaaaaaaagiaaaaacaeaaaaaabbaaaaahbcaabaaaaaaaaaaaegbobaaa
afaaaaaaegbobaaaafaaaaaaelaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaa
dccaaaalbcaabaaaaaaaaaaaakaabaaaaaaaaaaackiacaaaaaaaaaaaaiaaaaaa
dkiacaaaaaaaaaaaaiaaaaaaefaaaaajpcaabaaaabaaaaaaegbabaaaaeaaaaaa
eghobaaaafaaaaaaaagabaaaafaaaaaadiaaaaahccaabaaaaaaaaaaadkaabaaa
abaaaaaaabeaaaaaaaaaaaebdiaaaaahocaabaaaaaaaaaaaagajbaaaabaaaaaa
fgafbaaaaaaaaaaaefaaaaajpcaabaaaabaaaaaaegbabaaaaeaaaaaaeghobaaa
aeaaaaaaaagabaaaaeaaaaaadiaaaaahicaabaaaabaaaaaadkaabaaaabaaaaaa
abeaaaaaaaaaaaebdcaaaaakhcaabaaaabaaaaaapgapbaaaabaaaaaaegacbaaa
abaaaaaajgahbaiaebaaaaaaaaaaaaaadcaaaaajhcaabaaaaaaaaaaaagaabaaa
aaaaaaaaegacbaaaabaaaaaajgahbaaaaaaaaaaaaoaaaaahdcaabaaaabaaaaaa
egbabaaaadaaaaaapgbpbaaaadaaaaaaefaaaaajpcaabaaaabaaaaaaegaabaaa
abaaaaaaeghobaaaadaaaaaaaagabaaaadaaaaaaaaaaaaahhcaabaaaaaaaaaaa
egacbaaaaaaaaaaaegacbaaaabaaaaaabaaaaaahicaabaaaaaaaaaaaegbcbaaa
acaaaaaaegbcbaaaacaaaaaaeeaaaaaficaabaaaaaaaaaaadkaabaaaaaaaaaaa
diaaaaahdcaabaaaabaaaaaapgapbaaaaaaaaaaaegbabaaaacaaaaaadcaaaaaj
icaabaaaaaaaaaaackbabaaaacaaaaaadkaabaaaaaaaaaaaabeaaaaadnaknhdo
aoaaaaahdcaabaaaabaaaaaaegaabaaaabaaaaaapgapbaaaaaaaaaaaefaaaaaj
pcaabaaaacaaaaaaogbkbaaaabaaaaaaeghobaaaaaaaaaaaaagabaaaacaaaaaa
diaaaaaiicaabaaaaaaaaaaackiacaaaaaaaaaaaaeaaaaaaabeaaaaaaaaaaadp
dcaaaaalicaabaaaaaaaaaaadkaabaaaacaaaaaackiacaaaaaaaaaaaaeaaaaaa
dkaabaiaebaaaaaaaaaaaaaadcaaaaajdcaabaaaabaaaaaapgapbaaaaaaaaaaa
egaabaaaabaaaaaaegbabaaaabaaaaaaefaaaaajpcaabaaaacaaaaaaegaabaaa
abaaaaaaeghobaaaacaaaaaaaagabaaaabaaaaaaefaaaaajpcaabaaaadaaaaaa
egaabaaaabaaaaaaeghobaaaabaaaaaaaagabaaaaaaaaaaadiaaaaahhcaabaaa
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
ConstBuffer "$Globals" 160 // 144 used size, 12 vars
Vector 32 [_SpecColor] 4
Vector 48 [_Color] 4
Float 68 [_Gloss]
Float 72 [_Parallax]
Vector 128 [unity_LightmapFade] 4
BindCB "$Globals" 0
SetTexture 0 [_ParallaxMap] 2D 2
SetTexture 1 [_MainTex] 2D 0
SetTexture 2 [_SpecMap] 2D 1
SetTexture 3 [_LightBuffer] 2D 3
SetTexture 4 [unity_Lightmap] 2D 4
SetTexture 5 [unity_LightmapInd] 2D 5
// 32 instructions, 4 temp regs, 0 temp arrays:
// ALU 18 float, 0 int, 0 uint
// TEX 6 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0_level_9_3
eefiecedooekjgffldddhgkdilcbhagbfipijpmhabaaaaaageajaaaaaeaaaaaa
daaaaaaahmadaaaahiaiaaaadaajaaaaebgpgodjeeadaaaaeeadaaaaaaacpppp
paacaaaafeaaaaaaacaadmaaaaaafeaaaaaafeaaagaaceaaaaaafeaaabaaaaaa
acababaaaaacacaaadadadaaaeaeaeaaafafafaaaaaaacaaadaaaaaaaaaaaaaa
aaaaaiaaabaaadaaaaaaaaaaabacppppfbaaaaafaeaaapkaaaaaaadpdnaknhdo
aaaaaaebaaaaaaaabpaaaaacaaaaaaiaaaaaaplabpaaaaacaaaaaaiaabaaahla
bpaaaaacaaaaaaiaacaaaplabpaaaaacaaaaaaiaadaaadlabpaaaaacaaaaaaia
aeaaaplabpaaaaacaaaaaajaaaaiapkabpaaaaacaaaaaajaabaiapkabpaaaaac
aaaaaajaacaiapkabpaaaaacaaaaaajaadaiapkabpaaaaacaaaaaajaaeaiapka
bpaaaaacaaaaaajaafaiapkaajaaaaadaaaaaiiaaeaaoelaaeaaoelaahaaaaac
aaaaabiaaaaappiaagaaaaacaaaaabiaaaaaaaiaaeaaaaaeaaaabbiaaaaaaaia
adaakkkaadaappkaecaaaaadabaacpiaadaaoelaaeaioekaecaaaaadacaacpia
adaaoelaafaioekaafaaaaadacaaciiaacaappiaaeaakkkaafaaaaadaaaacoia
acaajaiaacaappiaafaaaaadabaaciiaabaappiaaeaakkkaaeaaaaaeabaaahia
abaappiaabaaoeiaaaaapjibaeaaaaaeaaaachiaaaaaaaiaabaaoeiaaaaapjia
agaaaaacaaaaaiiaacaapplaafaaaaadabaaadiaaaaappiaacaaoelaecaaaaad
abaacpiaabaaoeiaadaioekaacaaaaadaaaachiaaaaaoeiaabaaoeiaaiaaaaad
aaaaciiaabaaoelaabaaoelaahaaaaacaaaaciiaaaaappiaaeaaaaaeabaaabia
abaakklaaaaappiaaeaaffkaafaaaaadabaaagiaaaaappiaabaanalaagaaaaac
aaaaaiiaabaaaaiaafaaaaadabaaadiaaaaappiaabaaojiaabaaaaacacaaadia
aaaaoolaecaaaaadacaacpiaacaaoeiaacaioekaabaaaaacaaaaaiiaacaakkka
afaaaaadaaaaciiaaaaappiaaeaaaakaaeaaaaaeaaaaciiaacaappiaacaakkka
aaaappibaeaaaaaeabaaadiaaaaappiaabaaoeiaaaaaoelaecaaaaadacaacpia
abaaoeiaaaaioekaecaaaaadadaacpiaabaaoeiaabaioekaafaaaaadabaachia
acaappiaadaaoeiaafaaaaadacaachiaacaaoeiaabaaoekaafaaaaadabaachia
abaaoeiaacaaffkaafaaaaadabaachiaabaaoeiaabaappiaafaaaaadabaacoia
aaaajaiaabaajaiaaeaaaaaeaaaachiaacaaoeiaaaaaoeiaabaapjiaafaaaaad
aaaaciiaabaaaaiaaaaappkaabaaaaacaaaicpiaaaaaoeiappppaaaafdeieefc
peaeaaaaeaaaaaaadnabaaaafjaaaaaeegiocaaaaaaaaaaaajaaaaaafkaaaaad
aagabaaaaaaaaaaafkaaaaadaagabaaaabaaaaaafkaaaaadaagabaaaacaaaaaa
fkaaaaadaagabaaaadaaaaaafkaaaaadaagabaaaaeaaaaaafkaaaaadaagabaaa
afaaaaaafibiaaaeaahabaaaaaaaaaaaffffaaaafibiaaaeaahabaaaabaaaaaa
ffffaaaafibiaaaeaahabaaaacaaaaaaffffaaaafibiaaaeaahabaaaadaaaaaa
ffffaaaafibiaaaeaahabaaaaeaaaaaaffffaaaafibiaaaeaahabaaaafaaaaaa
ffffaaaagcbaaaadpcbabaaaabaaaaaagcbaaaadhcbabaaaacaaaaaagcbaaaad
lcbabaaaadaaaaaagcbaaaaddcbabaaaaeaaaaaagcbaaaadpcbabaaaafaaaaaa
gfaaaaadpccabaaaaaaaaaaagiaaaaacaeaaaaaabbaaaaahbcaabaaaaaaaaaaa
egbobaaaafaaaaaaegbobaaaafaaaaaaelaaaaafbcaabaaaaaaaaaaaakaabaaa
aaaaaaaadccaaaalbcaabaaaaaaaaaaaakaabaaaaaaaaaaackiacaaaaaaaaaaa
aiaaaaaadkiacaaaaaaaaaaaaiaaaaaaefaaaaajpcaabaaaabaaaaaaegbabaaa
aeaaaaaaeghobaaaafaaaaaaaagabaaaafaaaaaadiaaaaahccaabaaaaaaaaaaa
dkaabaaaabaaaaaaabeaaaaaaaaaaaebdiaaaaahocaabaaaaaaaaaaaagajbaaa
abaaaaaafgafbaaaaaaaaaaaefaaaaajpcaabaaaabaaaaaaegbabaaaaeaaaaaa
eghobaaaaeaaaaaaaagabaaaaeaaaaaadiaaaaahicaabaaaabaaaaaadkaabaaa
abaaaaaaabeaaaaaaaaaaaebdcaaaaakhcaabaaaabaaaaaapgapbaaaabaaaaaa
egacbaaaabaaaaaajgahbaiaebaaaaaaaaaaaaaadcaaaaajhcaabaaaaaaaaaaa
agaabaaaaaaaaaaaegacbaaaabaaaaaajgahbaaaaaaaaaaaaoaaaaahdcaabaaa
abaaaaaaegbabaaaadaaaaaapgbpbaaaadaaaaaaefaaaaajpcaabaaaabaaaaaa
egaabaaaabaaaaaaeghobaaaadaaaaaaaagabaaaadaaaaaaaaaaaaahhcaabaaa
aaaaaaaaegacbaaaaaaaaaaaegacbaaaabaaaaaabaaaaaahicaabaaaaaaaaaaa
egbcbaaaacaaaaaaegbcbaaaacaaaaaaeeaaaaaficaabaaaaaaaaaaadkaabaaa
aaaaaaaadiaaaaahdcaabaaaabaaaaaapgapbaaaaaaaaaaaegbabaaaacaaaaaa
dcaaaaajicaabaaaaaaaaaaackbabaaaacaaaaaadkaabaaaaaaaaaaaabeaaaaa
dnaknhdoaoaaaaahdcaabaaaabaaaaaaegaabaaaabaaaaaapgapbaaaaaaaaaaa
efaaaaajpcaabaaaacaaaaaaogbkbaaaabaaaaaaeghobaaaaaaaaaaaaagabaaa
acaaaaaadiaaaaaiicaabaaaaaaaaaaackiacaaaaaaaaaaaaeaaaaaaabeaaaaa
aaaaaadpdcaaaaalicaabaaaaaaaaaaadkaabaaaacaaaaaackiacaaaaaaaaaaa
aeaaaaaadkaabaiaebaaaaaaaaaaaaaadcaaaaajdcaabaaaabaaaaaapgapbaaa
aaaaaaaaegaabaaaabaaaaaaegbabaaaabaaaaaaefaaaaajpcaabaaaacaaaaaa
egaabaaaabaaaaaaeghobaaaacaaaaaaaagabaaaabaaaaaaefaaaaajpcaabaaa
adaaaaaaegaabaaaabaaaaaaeghobaaaabaaaaaaaagabaaaaaaaaaaadiaaaaah
hcaabaaaabaaaaaaegacbaaaacaaaaaapgapbaaaadaaaaaadiaaaaaihcaabaaa
acaaaaaaegacbaaaadaaaaaaegiccaaaaaaaaaaaadaaaaaadiaaaaaihcaabaaa
abaaaaaaegacbaaaabaaaaaafgifcaaaaaaaaaaaaeaaaaaadiaaaaahhcaabaaa
abaaaaaaegacbaaaabaaaaaapgapbaaaabaaaaaadiaaaaahocaabaaaabaaaaaa
agajbaaaaaaaaaaaagajbaaaabaaaaaadcaaaaajhccabaaaaaaaaaaaegacbaaa
acaaaaaaegacbaaaaaaaaaaajgahbaaaabaaaaaadiaaaaaiiccabaaaaaaaaaaa
akaabaaaabaaaaaadkiacaaaaaaaaaaaacaaaaaadoaaaaabejfdeheolaaaaaaa
agaaaaaaaiaaaaaajiaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaa
keaaaaaaaaaaaaaaaaaaaaaaadaaaaaaabaaaaaaapapaaaakeaaaaaaabaaaaaa
aaaaaaaaadaaaaaaacaaaaaaahahaaaakeaaaaaaacaaaaaaaaaaaaaaadaaaaaa
adaaaaaaapalaaaakeaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaadadaaaa
keaaaaaaaeaaaaaaaaaaaaaaadaaaaaaafaaaaaaapapaaaafdfgfpfaepfdejfe
ejepeoaafeeffiedepepfceeaaklklklepfdeheocmaaaaaaabaaaaaaaiaaaaaa
caaaaaaaaaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaafdfgfpfegbhcghgf
heaaklkl"
}

}
	}

#LINE 84

	}
	Fallback "Specular"
}
