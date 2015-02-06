Shader "ColorGloss/Bumped Specular Glass Reflective" {
	Properties {
		_Color ("Main Color", Color) = (1,1,1,1)
		_Shininess ("Shininess", Range (0.03, 1)) = 0.078125
		_Gloss ("Gloss Power", Float) = 1
		_ReflectColor ("Reflection Color", Color) = (1,1,1,0.5)
		_ReflectPower ("Reflection Power", Float) = 1
		_MainTex ("Base (RGB) Gloss (A)", 2D) = "white" {}
		_BumpMap ("Normalmap", 2D) = "bump" {}
		_SpecMap ("SpecMap", 2D) = "white" {}
		_Cube ("Reflection Cubemap", Cube) = "" { TexGen CubeReflect }
		_FresnelPower ("Fresnel Power", Range(0.05,5.0)) = 0.75
	}
	SubShader {
		Tags { "RenderType" = "Transparent" }
		LOD 400

			Alphatest Greater 0 ZWrite Off ColorMask RGB
	
	Pass {
		Name "FORWARD"
		Tags { "LightMode" = "ForwardBase" }
		Blend SrcAlpha OneMinusSrcAlpha
Program "vp" {
// Vertex combos: 3
//   opengl - ALU: 34 to 89
//   d3d9 - ALU: 35 to 92
//   d3d11 - ALU: 21 to 49, TEX: 0 to 0, FLOW: 1 to 1
SubProgram "opengl " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" }
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
"3.0-!!ARBvp1.0
# 58 ALU
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
MAD R0.xyz, vertex.normal.yzxw, R1.zxyw, -R0;
ADD result.texcoord[6].xyz, R2, R3;
MUL R3.xyz, R0, vertex.attrib[14].w;
MOV R0, c[14];
MOV R1.xyz, c[13];
MOV R1.w, c[0].x;
DP4 R2.z, R1, c[11];
DP4 R2.x, R1, c[9];
DP4 R2.y, R1, c[10];
MAD R2.xyz, R2, c[22].w, -vertex.position;
DP4 R1.z, R0, c[11];
DP4 R1.x, R0, c[9];
DP4 R1.y, R0, c[10];
DP3 R0.y, R3, c[5];
DP3 R0.w, -R2, c[5];
DP3 R0.x, vertex.attrib[14], c[5];
DP3 R0.z, vertex.normal, c[5];
MUL result.texcoord[2], R0, c[22].w;
DP3 R0.y, R3, c[6];
DP3 R0.w, -R2, c[6];
DP3 R0.x, vertex.attrib[14], c[6];
DP3 R0.z, vertex.normal, c[6];
MUL result.texcoord[3], R0, c[22].w;
DP3 R0.y, R3, c[7];
DP3 R0.w, -R2, c[7];
DP3 R0.x, vertex.attrib[14], c[7];
DP3 R0.z, vertex.normal, c[7];
DP3 result.texcoord[1].y, R2, R3;
DP3 result.texcoord[5].y, R3, R1;
MUL result.texcoord[4], R0, c[22].w;
DP3 result.texcoord[1].z, vertex.normal, R2;
DP3 result.texcoord[1].x, R2, vertex.attrib[14];
DP3 result.texcoord[5].z, vertex.normal, R1;
DP3 result.texcoord[5].x, vertex.attrib[14], R1;
MAD result.texcoord[0].xy, vertex.texcoord[0], c[23], c[23].zwzw;
DP4 result.position.w, vertex.position, c[4];
DP4 result.position.z, vertex.position, c[3];
DP4 result.position.y, vertex.position, c[2];
DP4 result.position.x, vertex.position, c[1];
END
# 58 instructions, 4 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" }
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
"vs_3_0
; 61 ALU
dcl_position o0
dcl_texcoord0 o1
dcl_texcoord1 o2
dcl_texcoord2 o3
dcl_texcoord3 o4
dcl_texcoord4 o5
dcl_texcoord5 o6
dcl_texcoord6 o7
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
mul r1, r0.xyzz, r0.yzzx
mov r0.w, c23.x
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
add o7.xyz, r1, r2
mov r0.xyz, v1
mul r1.xyz, v2.zxyw, r0.yzxw
mov r0.xyz, v1
mad r0.xyz, v2.yzxw, r0.zxyw, -r1
mul r3.xyz, r0, v1.w
mov r0, c10
dp4 r4.z, c13, r0
mov r0, c9
dp4 r4.y, c13, r0
mov r1.w, c23.x
mov r1.xyz, c12
dp4 r2.z, r1, c10
dp4 r2.x, r1, c8
dp4 r2.y, r1, c9
mad r2.xyz, r2, c21.w, -v0
mov r1, c8
dp4 r4.x, c13, r1
dp3 r0.y, r3, c4
dp3 r0.w, -r2, c4
dp3 r0.x, v1, c4
dp3 r0.z, v2, c4
mul o3, r0, c21.w
dp3 r0.y, r3, c5
dp3 r0.w, -r2, c5
dp3 r0.x, v1, c5
dp3 r0.z, v2, c5
mul o4, r0, c21.w
dp3 r0.y, r3, c6
dp3 r0.w, -r2, c6
dp3 r0.x, v1, c6
dp3 r0.z, v2, c6
dp3 o2.y, r2, r3
dp3 o6.y, r3, r4
mul o5, r0, c21.w
dp3 o2.z, v2, r2
dp3 o2.x, r2, v1
dp3 o6.z, v2, r4
dp3 o6.x, v1, r4
mad o1.xy, v3, c22, c22.zwzw
dp4 o0.w, v0, c3
dp4 o0.z, v0, c2
dp4 o0.y, v0, c1
dp4 o0.x, v0, c0
"
}

SubProgram "xbox360 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" }
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
// ALU: 73.33 (55 instructions), vertex: 32, texture: 0,
//   sequencer: 26,  13 GPRs, 12 threads,
// Performance (if enough threads): ~73 cycles per vector
// * Vertex cycle estimates are assuming 3 vfetch_minis for every vfetch_full,
//     with <= 32 bytes per vfetch_full group.

"vs_360
backbbabaaaaacoaaaaaadceaaaaaaaaaaaaaaceaaaaaaaaaaaaacgaaaaaaaaa
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
aaaaaaaaaaaaadceaagbaaamaaaaaaaaaaaaaaaaaaaafmohaaaaaaabaaaaaaae
aaaaaaalaaaaacjaaabaaaahaaaagaaiaaaadaajaacafaakaaaadafaaaabhbfb
aaaepcfcaaafpdfdaaagpefeaaahhfffaaakhgfgaaaabadaaaaaaackaaaaaacl
aaaabacmaaaabadnaaaabadoaaaabadpaaaaaacnaaaaaacoaaaabacpaaaabaeb
paffeaahaaaabcaamcaaaaaaaaaaeaalaaaabcaameaaaaaaaaaagaapgabfbcaa
bcaaaaaaaaaagablgacbbcaabcaaaaaaaaaagachgacnbcaabcaaaaaaaaaagadd
gadjbcaabcaaaaaaaaaadadpaaaaccaaaaaaaaaaafpigaaaaaaaagiiaaaaaaaa
afpibaaaaaaaagiiaaaaaaaaafpiiaaaaaaaaoiiaaaaaaaaafpieaaaaaaaadmh
aaaaaaaakmepalaaaabliiedibagamapmiapaaaaaamgiiaaklagalaamiapaaaa
aalbdejeklagakaamiapiadoaagmaadeklagajaamiahaaahaamamgmaalbcaabd
ceibajalaablgmgmkbabaniabechaaajaaleblblcbbdababkiclalacaamggceb
ibaiapaobebhaaadaalblomgkbaiaoabkiihadafaalbgcmaibabaoapmialaaaa
aamamgleclbcabajmiahaaajaalelbleclbbaaahkmehaaahaalogficmbaiabap
miahaaahabgflomaolaiabahmiahaaajaamagmleclbaaaajmiahaaakaalelbli
clbbabaamiahaaamaagmloleklabanafmiahaaadaagmmngcklaianadkmeeacad
aamggmecmaadacapbealaaaaaagfblgmkbaibeamaebeadaeaagmlbbloaadacad
beaoaaafaalbimmgkbaaapamaebbaeafaalbmgmgoaamacaamiahaaakaamagmle
clbaabakmiahaaacabmablmaklajbeagmiahaaajaagmlebfklaaaoafbeahaaaa
aamnbllbobahabadaeedafagaamglcblkbaaapacmiahaaajaabllemaklaaanaj
kiccadafaalolomanaalahaokiihadahaelbgciaibacaoaomiabiaabaaloloaa
paacabaamiaciaabaagcloaapaaaacaamiaeiaabaaloloaapaacaiaamiabiaaf
aaloloaapaakabaamiaciaafaagcloaapaaaakaamiaeiaafaaloloaapaakaiaa
miadiaaaaabjlabkilaebfbfmiamaaabaalbigdbklaaanadmiahaaahaegmlole
klacanahaiboaiaaaemghgggkbacapajaicbaiacaadoanmbgpacajajaiecaiac
aadoanlbgpadajajaiieaiacaadoanlmgpaeajajaibbabaaaakhkhgmkpaiafaj
aiciabadaagmlbmgoaahaaajbeacaaaaaakhkhmgkpaiagabaeciadaeaamgmggm
oaahaaagbeaeaaaaaakhkhblkpaiahabaeciaeafaalbbllboaahaaagmiapiaac
aaaablaakbafbeaamiapiaadaaaablaakbaebeaamiapiaaeaaaablaakbadbeaa
geihaaaaaalologboaacaaabmiahiaagaablmagfklaaaiaaaaaaaaaaaaaaaaaa
aaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" }
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
"sce_vp_rsx // 56 instructions using 8 registers
[Configuration]
8
0000003841050800
[Microcode]
896
00019c6c005d200d8186c0836041fffc00029c6c00400e0c0106c0836041dffc
00021c6c005d300c0186c0836041dffc00039c6c009ca20c013fc0c36041dffc
401f9c6c011c9808010400d740619f9c401f9c6c01d0300d8106c0c360403f80
401f9c6c01d0200d8106c0c360405f80401f9c6c01d0100d8106c0c360409f80
401f9c6c01d0000d8106c0c360411f8000001c6c01506e0c010600c360411ffc
00001c6c0150620c010600c360405ffc00009c6c01505e0c010600c360411ffc
00009c6c0150520c010600c360405ffc00011c6c01504e0c010600c360411ffc
00011c6c0150420c010600c360405ffc00031c6c01d0a00d8686c0c360405ffc
00031c6c01d0900d8686c0c360409ffc00031c6c01d0800d8686c0c360411ffc
00019c6c0150400c0e8600c360411ffc00019c6c0150600c0e8600c360405ffc
00001c6c0150500c0e8600c360409ffc00039c6c0190a00c0886c0c360405ffc
00039c6c0190900c0886c0c360409ffc00039c6c0190800c0886c0c360411ffc
00021c6c00800243011845436041dffc00021c6c01000230812185630221dffc
00039c6c011ca00c0ebfc0e30041dffc401f9c6c0140020c0106064360405fb0
401f9c6c01400e0c0106064360411fb000009c6c0080002a8095404360409ffc
00019c6c0040002a8086c08360409ffc00029c6c00800e0c08bfc0836041dffc
401f9c6c0140020c0106074360405fa0401f9c6c01400e0c0e86008360411fa0
00001c6c0150608c0e8600c360403ffc00009c6c0150508c0e8600c360403ffc
00011c6c0150408c0e8600c360403ffc00021c6c019cf00c0686c0c360405ffc
00021c6c019d000c0686c0c360409ffc00021c6c019d100c0686c0c360411ffc
00021c6c010000000680036aa0a03ffc00019c6c0080000d069a03436041fffc
401f9c6c0140000c0a86064360409fb0401f9c6c0140000c0e86054360409fa0
00001c6c0150600c0a8600c360409ffc00009c6c0150500c0a8600c360409ffc
00011c6c0150400c0a8600c360409ffc00029c6c01dcc00d8686c0c360405ffc
00029c6c01dcd00d8686c0c360409ffc00029c6c01dce00d8686c0c360411ffc
00019c6c00c0000c0886c08302a1dffc00021c6c009cb07f888600c36041dffc
401f9c6c00c0000c0886c08301a1dfb4401f9c6c009ca00d84bfc0c36041ffa4
401f9c6c009ca00d82bfc0c36041ffa8401f9c6c009ca00d80bfc0c36041ffad
"
}

SubProgram "d3d11 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 112 // 112 used size, 10 vars
Vector 96 [_MainTex_ST] 4
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
// 66 instructions, 4 temp regs, 0 temp arrays:
// ALU 36 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0
eefiecedhpnioafilanpoobphefpdjjnlnofigcnabaaaaaaoiakaaaaadaaaaaa
cmaaaaaapeaaaaaanmabaaaaejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaa
abaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaljaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfc
enebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheooaaaaaaaaiaaaaaa
aiaaaaaamiaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaneaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaabaaaaaaadamaaaaneaaaaaaabaaaaaaaaaaaaaa
adaaaaaaacaaaaaaahaiaaaaneaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaa
apaaaaaaneaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaneaaaaaa
aeaaaaaaaaaaaaaaadaaaaaaafaaaaaaapaaaaaaneaaaaaaafaaaaaaaaaaaaaa
adaaaaaaagaaaaaaahaiaaaaneaaaaaaagaaaaaaaaaaaaaaadaaaaaaahaaaaaa
ahaiaaaafdfgfpfaepfdejfeejepeoaafeeffiedepepfceeaaklklklfdeieefc
aeajaaaaeaaaabaaebacaaaafjaaaaaeegiocaaaaaaaaaaaahaaaaaafjaaaaae
egiocaaaabaaaaaaafaaaaaafjaaaaaeegiocaaaacaaaaaabjaaaaaafjaaaaae
egiocaaaadaaaaaabfaaaaaafpaaaaadpcbabaaaaaaaaaaafpaaaaadpcbabaaa
abaaaaaafpaaaaadhcbabaaaacaaaaaafpaaaaaddcbabaaaadaaaaaaghaaaaae
pccabaaaaaaaaaaaabaaaaaagfaaaaaddccabaaaabaaaaaagfaaaaadhccabaaa
acaaaaaagfaaaaadpccabaaaadaaaaaagfaaaaadpccabaaaaeaaaaaagfaaaaad
pccabaaaafaaaaaagfaaaaadhccabaaaagaaaaaagfaaaaadhccabaaaahaaaaaa
giaaaaacaeaaaaaadiaaaaaipcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaa
adaaaaaaabaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaaaaaaaaa
agbabaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaa
adaaaaaaacaaaaaakgbkbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpccabaaa
aaaaaaaaegiocaaaadaaaaaaadaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaa
dcaaaaaldccabaaaabaaaaaaegbabaaaadaaaaaaegiacaaaaaaaaaaaagaaaaaa
ogikcaaaaaaaaaaaagaaaaaadiaaaaajhcaabaaaaaaaaaaafgifcaaaabaaaaaa
aeaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaaaaaaaaaaegiccaaa
adaaaaaabaaaaaaaagiacaaaabaaaaaaaeaaaaaaegacbaaaaaaaaaaadcaaaaal
hcaabaaaaaaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaaabaaaaaaaeaaaaaa
egacbaaaaaaaaaaaaaaaaaaihcaabaaaaaaaaaaaegacbaaaaaaaaaaaegiccaaa
adaaaaaabdaaaaaadcaaaaalhcaabaaaaaaaaaaaegacbaaaaaaaaaaapgipcaaa
adaaaaaabeaaaaaaegbcbaiaebaaaaaaaaaaaaaabaaaaaahbccabaaaacaaaaaa
egbcbaaaabaaaaaaegacbaaaaaaaaaaabaaaaaaheccabaaaacaaaaaaegbcbaaa
acaaaaaaegacbaaaaaaaaaaadiaaaaahhcaabaaaabaaaaaajgbebaaaabaaaaaa
cgbjbaaaacaaaaaadcaaaaakhcaabaaaabaaaaaajgbebaaaacaaaaaacgbjbaaa
abaaaaaaegacbaiaebaaaaaaabaaaaaadiaaaaahhcaabaaaabaaaaaaegacbaaa
abaaaaaapgbpbaaaabaaaaaabaaaaaahcccabaaaacaaaaaaegacbaaaabaaaaaa
egacbaaaaaaaaaaadiaaaaajhcaabaaaacaaaaaafgafbaiaebaaaaaaaaaaaaaa
egiccaaaadaaaaaaanaaaaaadcaaaaallcaabaaaaaaaaaaaegiicaaaadaaaaaa
amaaaaaaagaabaiaebaaaaaaaaaaaaaaegaibaaaacaaaaaadcaaaaallcaabaaa
aaaaaaaaegiicaaaadaaaaaaaoaaaaaakgakbaiaebaaaaaaaaaaaaaaegambaaa
aaaaaaaadgaaaaaficaabaaaacaaaaaaakaabaaaaaaaaaaadgaaaaagbcaabaaa
adaaaaaaakiacaaaadaaaaaaamaaaaaadgaaaaagccaabaaaadaaaaaaakiacaaa
adaaaaaaanaaaaaadgaaaaagecaabaaaadaaaaaaakiacaaaadaaaaaaaoaaaaaa
baaaaaahccaabaaaacaaaaaaegacbaaaabaaaaaaegacbaaaadaaaaaabaaaaaah
bcaabaaaacaaaaaaegbcbaaaabaaaaaaegacbaaaadaaaaaabaaaaaahecaabaaa
acaaaaaaegbcbaaaacaaaaaaegacbaaaadaaaaaadiaaaaaipccabaaaadaaaaaa
egaobaaaacaaaaaapgipcaaaadaaaaaabeaaaaaadgaaaaaficaabaaaacaaaaaa
bkaabaaaaaaaaaaadgaaaaagbcaabaaaadaaaaaabkiacaaaadaaaaaaamaaaaaa
dgaaaaagccaabaaaadaaaaaabkiacaaaadaaaaaaanaaaaaadgaaaaagecaabaaa
adaaaaaabkiacaaaadaaaaaaaoaaaaaabaaaaaahccaabaaaacaaaaaaegacbaaa
abaaaaaaegacbaaaadaaaaaabaaaaaahbcaabaaaacaaaaaaegbcbaaaabaaaaaa
egacbaaaadaaaaaabaaaaaahecaabaaaacaaaaaaegbcbaaaacaaaaaaegacbaaa
adaaaaaadiaaaaaipccabaaaaeaaaaaaegaobaaaacaaaaaapgipcaaaadaaaaaa
beaaaaaadgaaaaagbcaabaaaacaaaaaackiacaaaadaaaaaaamaaaaaadgaaaaag
ccaabaaaacaaaaaackiacaaaadaaaaaaanaaaaaadgaaaaagecaabaaaacaaaaaa
ckiacaaaadaaaaaaaoaaaaaabaaaaaahccaabaaaaaaaaaaaegacbaaaabaaaaaa
egacbaaaacaaaaaabaaaaaahbcaabaaaaaaaaaaaegbcbaaaabaaaaaaegacbaaa
acaaaaaabaaaaaahecaabaaaaaaaaaaaegbcbaaaacaaaaaaegacbaaaacaaaaaa
diaaaaaipccabaaaafaaaaaaegaobaaaaaaaaaaapgipcaaaadaaaaaabeaaaaaa
diaaaaajhcaabaaaaaaaaaaafgifcaaaacaaaaaaaaaaaaaaegiccaaaadaaaaaa
bbaaaaaadcaaaaalhcaabaaaaaaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaa
acaaaaaaaaaaaaaaegacbaaaaaaaaaaadcaaaaalhcaabaaaaaaaaaaaegiccaaa
adaaaaaabcaaaaaakgikcaaaacaaaaaaaaaaaaaaegacbaaaaaaaaaaadcaaaaal
hcaabaaaaaaaaaaaegiccaaaadaaaaaabdaaaaaapgipcaaaacaaaaaaaaaaaaaa
egacbaaaaaaaaaaabaaaaaahcccabaaaagaaaaaaegacbaaaabaaaaaaegacbaaa
aaaaaaaabaaaaaahbccabaaaagaaaaaaegbcbaaaabaaaaaaegacbaaaaaaaaaaa
baaaaaaheccabaaaagaaaaaaegbcbaaaacaaaaaaegacbaaaaaaaaaaadiaaaaai
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
hccabaaaahaaaaaaegiccaaaacaaaaaabiaaaaaaagaabaaaaaaaaaaaegacbaaa
abaaaaaadoaaaaab"
}

SubProgram "gles " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" }
"!!GLES
#define SHADER_API_GLES 1
#define tex2D texture2D


#ifdef VERTEX
#define gl_ModelViewProjectionMatrix glstate_matrix_mvp
uniform mat4 glstate_matrix_mvp;

varying lowp vec3 xlv_TEXCOORD6;
varying lowp vec3 xlv_TEXCOORD5;
varying lowp vec4 xlv_TEXCOORD4;
varying lowp vec4 xlv_TEXCOORD3;
varying lowp vec4 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
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
  lowp vec4 tmpvar_4;
  lowp vec4 tmpvar_5;
  lowp vec4 tmpvar_6;
  lowp vec3 tmpvar_7;
  lowp vec3 tmpvar_8;
  highp vec4 tmpvar_9;
  tmpvar_9.w = 1.0;
  tmpvar_9.xyz = _WorldSpaceCameraPos;
  mat3 tmpvar_10;
  tmpvar_10[0] = _Object2World[0].xyz;
  tmpvar_10[1] = _Object2World[1].xyz;
  tmpvar_10[2] = _Object2World[2].xyz;
  highp vec3 tmpvar_11;
  tmpvar_11 = (tmpvar_10 * (_glesVertex.xyz - ((_World2Object * tmpvar_9).xyz * unity_Scale.w)));
  highp vec3 tmpvar_12;
  highp vec3 tmpvar_13;
  tmpvar_12 = tmpvar_1.xyz;
  tmpvar_13 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_14;
  tmpvar_14[0].x = tmpvar_12.x;
  tmpvar_14[0].y = tmpvar_13.x;
  tmpvar_14[0].z = tmpvar_2.x;
  tmpvar_14[1].x = tmpvar_12.y;
  tmpvar_14[1].y = tmpvar_13.y;
  tmpvar_14[1].z = tmpvar_2.y;
  tmpvar_14[2].x = tmpvar_12.z;
  tmpvar_14[2].y = tmpvar_13.z;
  tmpvar_14[2].z = tmpvar_2.z;
  vec4 v_15;
  v_15.x = _Object2World[0].x;
  v_15.y = _Object2World[1].x;
  v_15.z = _Object2World[2].x;
  v_15.w = _Object2World[3].x;
  highp vec4 tmpvar_16;
  tmpvar_16.xyz = (tmpvar_14 * v_15.xyz);
  tmpvar_16.w = tmpvar_11.x;
  highp vec4 tmpvar_17;
  tmpvar_17 = (tmpvar_16 * unity_Scale.w);
  tmpvar_4 = tmpvar_17;
  vec4 v_18;
  v_18.x = _Object2World[0].y;
  v_18.y = _Object2World[1].y;
  v_18.z = _Object2World[2].y;
  v_18.w = _Object2World[3].y;
  highp vec4 tmpvar_19;
  tmpvar_19.xyz = (tmpvar_14 * v_18.xyz);
  tmpvar_19.w = tmpvar_11.y;
  highp vec4 tmpvar_20;
  tmpvar_20 = (tmpvar_19 * unity_Scale.w);
  tmpvar_5 = tmpvar_20;
  vec4 v_21;
  v_21.x = _Object2World[0].z;
  v_21.y = _Object2World[1].z;
  v_21.z = _Object2World[2].z;
  v_21.w = _Object2World[3].z;
  highp vec4 tmpvar_22;
  tmpvar_22.xyz = (tmpvar_14 * v_21.xyz);
  tmpvar_22.w = tmpvar_11.z;
  highp vec4 tmpvar_23;
  tmpvar_23 = (tmpvar_22 * unity_Scale.w);
  tmpvar_6 = tmpvar_23;
  mat3 tmpvar_24;
  tmpvar_24[0] = _Object2World[0].xyz;
  tmpvar_24[1] = _Object2World[1].xyz;
  tmpvar_24[2] = _Object2World[2].xyz;
  highp vec3 tmpvar_25;
  tmpvar_25 = (tmpvar_14 * (_World2Object * _WorldSpaceLightPos0).xyz);
  tmpvar_7 = tmpvar_25;
  highp vec4 tmpvar_26;
  tmpvar_26.w = 1.0;
  tmpvar_26.xyz = _WorldSpaceCameraPos;
  highp vec4 tmpvar_27;
  tmpvar_27.w = 1.0;
  tmpvar_27.xyz = (tmpvar_24 * (tmpvar_2 * unity_Scale.w));
  mediump vec3 tmpvar_28;
  mediump vec4 normal_29;
  normal_29 = tmpvar_27;
  highp float vC_30;
  mediump vec3 x3_31;
  mediump vec3 x2_32;
  mediump vec3 x1_33;
  highp float tmpvar_34;
  tmpvar_34 = dot (unity_SHAr, normal_29);
  x1_33.x = tmpvar_34;
  highp float tmpvar_35;
  tmpvar_35 = dot (unity_SHAg, normal_29);
  x1_33.y = tmpvar_35;
  highp float tmpvar_36;
  tmpvar_36 = dot (unity_SHAb, normal_29);
  x1_33.z = tmpvar_36;
  mediump vec4 tmpvar_37;
  tmpvar_37 = (normal_29.xyzz * normal_29.yzzx);
  highp float tmpvar_38;
  tmpvar_38 = dot (unity_SHBr, tmpvar_37);
  x2_32.x = tmpvar_38;
  highp float tmpvar_39;
  tmpvar_39 = dot (unity_SHBg, tmpvar_37);
  x2_32.y = tmpvar_39;
  highp float tmpvar_40;
  tmpvar_40 = dot (unity_SHBb, tmpvar_37);
  x2_32.z = tmpvar_40;
  mediump float tmpvar_41;
  tmpvar_41 = ((normal_29.x * normal_29.x) - (normal_29.y * normal_29.y));
  vC_30 = tmpvar_41;
  highp vec3 tmpvar_42;
  tmpvar_42 = (unity_SHC.xyz * vC_30);
  x3_31 = tmpvar_42;
  tmpvar_28 = ((x1_33 + x2_32) + x3_31);
  shlight_3 = tmpvar_28;
  tmpvar_8 = shlight_3;
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = (tmpvar_14 * (((_World2Object * tmpvar_26).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = tmpvar_4;
  xlv_TEXCOORD3 = tmpvar_5;
  xlv_TEXCOORD4 = tmpvar_6;
  xlv_TEXCOORD5 = tmpvar_7;
  xlv_TEXCOORD6 = tmpvar_8;
}



#endif
#ifdef FRAGMENT

varying lowp vec3 xlv_TEXCOORD6;
varying lowp vec3 xlv_TEXCOORD5;
varying lowp vec4 xlv_TEXCOORD4;
varying lowp vec4 xlv_TEXCOORD3;
varying lowp vec4 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp float _FresnelPower;
uniform mediump float _Gloss;
uniform mediump float _Shininess;
uniform mediump float _ReflectPower;
uniform lowp vec4 _ReflectColor;
uniform lowp vec4 _Color;
uniform samplerCube _Cube;
uniform sampler2D _SpecMap;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform lowp vec4 _LightColor0;
void main ()
{
  lowp vec4 c_1;
  highp vec3 tmpvar_2;
  mediump vec3 tmpvar_3;
  mediump vec3 tmpvar_4;
  mediump vec3 tmpvar_5;
  lowp vec3 tmpvar_6;
  tmpvar_6.x = xlv_TEXCOORD2.w;
  tmpvar_6.y = xlv_TEXCOORD3.w;
  tmpvar_6.z = xlv_TEXCOORD4.w;
  tmpvar_2 = tmpvar_6;
  lowp vec3 tmpvar_7;
  tmpvar_7 = xlv_TEXCOORD2.xyz;
  tmpvar_3 = tmpvar_7;
  lowp vec3 tmpvar_8;
  tmpvar_8 = xlv_TEXCOORD3.xyz;
  tmpvar_4 = tmpvar_8;
  lowp vec3 tmpvar_9;
  tmpvar_9 = xlv_TEXCOORD4.xyz;
  tmpvar_5 = tmpvar_9;
  mediump vec3 tmpvar_10;
  mediump vec3 tmpvar_11;
  mediump float tmpvar_12;
  lowp vec4 tmpvar_13;
  tmpvar_13 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_14;
  tmpvar_14 = (tmpvar_13.xyz * _Color.xyz);
  tmpvar_10 = tmpvar_14;
  lowp vec4 tmpvar_15;
  tmpvar_15 = texture2D (_SpecMap, xlv_TEXCOORD0);
  mediump vec3 tmpvar_16;
  tmpvar_16 = ((tmpvar_13.w * tmpvar_15.xyz) * _Gloss);
  lowp vec3 tmpvar_17;
  tmpvar_17 = ((texture2D (_BumpMap, xlv_TEXCOORD0).xyz * 2.0) - 1.0);
  tmpvar_11 = tmpvar_17;
  mediump vec3 tmpvar_18;
  tmpvar_18.x = dot (tmpvar_3, tmpvar_11);
  tmpvar_18.y = dot (tmpvar_4, tmpvar_11);
  tmpvar_18.z = dot (tmpvar_5, tmpvar_11);
  highp vec3 tmpvar_19;
  tmpvar_19 = (tmpvar_2 - (2.0 * (dot (tmpvar_18, tmpvar_2) * tmpvar_18)));
  mediump vec3 tmpvar_20;
  tmpvar_20 = normalize(tmpvar_11);
  highp float tmpvar_21;
  tmpvar_21 = max ((0.20373 + (0.79627 * pow (clamp ((1.0 - max (dot (normalize(xlv_TEXCOORD1), tmpvar_20), 0.0)), 0.0, 1.0), _FresnelPower))), 0.0);
  lowp vec4 tmpvar_22;
  tmpvar_22 = (textureCube (_Cube, tmpvar_19) * tmpvar_13.w);
  tmpvar_12 = tmpvar_21;
  highp vec3 tmpvar_23;
  tmpvar_23 = normalize(xlv_TEXCOORD1);
  mediump vec3 lightDir_24;
  lightDir_24 = xlv_TEXCOORD5;
  mediump vec3 viewDir_25;
  viewDir_25 = tmpvar_23;
  mediump vec4 c_26;
  mediump vec3 specCol_27;
  highp float nh_28;
  mediump float tmpvar_29;
  tmpvar_29 = max (0.0, dot (tmpvar_11, normalize((lightDir_24 + viewDir_25))));
  nh_28 = tmpvar_29;
  mediump float arg1_30;
  arg1_30 = (32.0 * _Shininess);
  highp vec3 tmpvar_31;
  tmpvar_31 = (pow (nh_28, arg1_30) * tmpvar_16);
  specCol_27 = tmpvar_31;
  c_26.xyz = ((((tmpvar_10 * _LightColor0.xyz) * max (0.0, dot (tmpvar_11, lightDir_24))) + (_LightColor0.xyz * specCol_27)) * 2.0);
  c_26.w = tmpvar_12;
  c_1 = c_26;
  mediump vec3 tmpvar_32;
  tmpvar_32 = (c_1.xyz + (tmpvar_10 * xlv_TEXCOORD6));
  c_1.xyz = tmpvar_32;
  mediump vec3 tmpvar_33;
  tmpvar_33 = (c_1.xyz + ((tmpvar_22.xyz * _ReflectColor.xyz) * _ReflectPower));
  c_1.xyz = tmpvar_33;
  c_1.w = tmpvar_12;
  gl_FragData[0] = c_1;
}



#endif"
}

SubProgram "glesdesktop " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" }
"!!GLES
#define SHADER_API_GLES 1
#define tex2D texture2D


#ifdef VERTEX
#define gl_ModelViewProjectionMatrix glstate_matrix_mvp
uniform mat4 glstate_matrix_mvp;

varying lowp vec3 xlv_TEXCOORD6;
varying lowp vec3 xlv_TEXCOORD5;
varying lowp vec4 xlv_TEXCOORD4;
varying lowp vec4 xlv_TEXCOORD3;
varying lowp vec4 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
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
  lowp vec4 tmpvar_4;
  lowp vec4 tmpvar_5;
  lowp vec4 tmpvar_6;
  lowp vec3 tmpvar_7;
  lowp vec3 tmpvar_8;
  highp vec4 tmpvar_9;
  tmpvar_9.w = 1.0;
  tmpvar_9.xyz = _WorldSpaceCameraPos;
  mat3 tmpvar_10;
  tmpvar_10[0] = _Object2World[0].xyz;
  tmpvar_10[1] = _Object2World[1].xyz;
  tmpvar_10[2] = _Object2World[2].xyz;
  highp vec3 tmpvar_11;
  tmpvar_11 = (tmpvar_10 * (_glesVertex.xyz - ((_World2Object * tmpvar_9).xyz * unity_Scale.w)));
  highp vec3 tmpvar_12;
  highp vec3 tmpvar_13;
  tmpvar_12 = tmpvar_1.xyz;
  tmpvar_13 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_14;
  tmpvar_14[0].x = tmpvar_12.x;
  tmpvar_14[0].y = tmpvar_13.x;
  tmpvar_14[0].z = tmpvar_2.x;
  tmpvar_14[1].x = tmpvar_12.y;
  tmpvar_14[1].y = tmpvar_13.y;
  tmpvar_14[1].z = tmpvar_2.y;
  tmpvar_14[2].x = tmpvar_12.z;
  tmpvar_14[2].y = tmpvar_13.z;
  tmpvar_14[2].z = tmpvar_2.z;
  vec4 v_15;
  v_15.x = _Object2World[0].x;
  v_15.y = _Object2World[1].x;
  v_15.z = _Object2World[2].x;
  v_15.w = _Object2World[3].x;
  highp vec4 tmpvar_16;
  tmpvar_16.xyz = (tmpvar_14 * v_15.xyz);
  tmpvar_16.w = tmpvar_11.x;
  highp vec4 tmpvar_17;
  tmpvar_17 = (tmpvar_16 * unity_Scale.w);
  tmpvar_4 = tmpvar_17;
  vec4 v_18;
  v_18.x = _Object2World[0].y;
  v_18.y = _Object2World[1].y;
  v_18.z = _Object2World[2].y;
  v_18.w = _Object2World[3].y;
  highp vec4 tmpvar_19;
  tmpvar_19.xyz = (tmpvar_14 * v_18.xyz);
  tmpvar_19.w = tmpvar_11.y;
  highp vec4 tmpvar_20;
  tmpvar_20 = (tmpvar_19 * unity_Scale.w);
  tmpvar_5 = tmpvar_20;
  vec4 v_21;
  v_21.x = _Object2World[0].z;
  v_21.y = _Object2World[1].z;
  v_21.z = _Object2World[2].z;
  v_21.w = _Object2World[3].z;
  highp vec4 tmpvar_22;
  tmpvar_22.xyz = (tmpvar_14 * v_21.xyz);
  tmpvar_22.w = tmpvar_11.z;
  highp vec4 tmpvar_23;
  tmpvar_23 = (tmpvar_22 * unity_Scale.w);
  tmpvar_6 = tmpvar_23;
  mat3 tmpvar_24;
  tmpvar_24[0] = _Object2World[0].xyz;
  tmpvar_24[1] = _Object2World[1].xyz;
  tmpvar_24[2] = _Object2World[2].xyz;
  highp vec3 tmpvar_25;
  tmpvar_25 = (tmpvar_14 * (_World2Object * _WorldSpaceLightPos0).xyz);
  tmpvar_7 = tmpvar_25;
  highp vec4 tmpvar_26;
  tmpvar_26.w = 1.0;
  tmpvar_26.xyz = _WorldSpaceCameraPos;
  highp vec4 tmpvar_27;
  tmpvar_27.w = 1.0;
  tmpvar_27.xyz = (tmpvar_24 * (tmpvar_2 * unity_Scale.w));
  mediump vec3 tmpvar_28;
  mediump vec4 normal_29;
  normal_29 = tmpvar_27;
  highp float vC_30;
  mediump vec3 x3_31;
  mediump vec3 x2_32;
  mediump vec3 x1_33;
  highp float tmpvar_34;
  tmpvar_34 = dot (unity_SHAr, normal_29);
  x1_33.x = tmpvar_34;
  highp float tmpvar_35;
  tmpvar_35 = dot (unity_SHAg, normal_29);
  x1_33.y = tmpvar_35;
  highp float tmpvar_36;
  tmpvar_36 = dot (unity_SHAb, normal_29);
  x1_33.z = tmpvar_36;
  mediump vec4 tmpvar_37;
  tmpvar_37 = (normal_29.xyzz * normal_29.yzzx);
  highp float tmpvar_38;
  tmpvar_38 = dot (unity_SHBr, tmpvar_37);
  x2_32.x = tmpvar_38;
  highp float tmpvar_39;
  tmpvar_39 = dot (unity_SHBg, tmpvar_37);
  x2_32.y = tmpvar_39;
  highp float tmpvar_40;
  tmpvar_40 = dot (unity_SHBb, tmpvar_37);
  x2_32.z = tmpvar_40;
  mediump float tmpvar_41;
  tmpvar_41 = ((normal_29.x * normal_29.x) - (normal_29.y * normal_29.y));
  vC_30 = tmpvar_41;
  highp vec3 tmpvar_42;
  tmpvar_42 = (unity_SHC.xyz * vC_30);
  x3_31 = tmpvar_42;
  tmpvar_28 = ((x1_33 + x2_32) + x3_31);
  shlight_3 = tmpvar_28;
  tmpvar_8 = shlight_3;
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = (tmpvar_14 * (((_World2Object * tmpvar_26).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = tmpvar_4;
  xlv_TEXCOORD3 = tmpvar_5;
  xlv_TEXCOORD4 = tmpvar_6;
  xlv_TEXCOORD5 = tmpvar_7;
  xlv_TEXCOORD6 = tmpvar_8;
}



#endif
#ifdef FRAGMENT

varying lowp vec3 xlv_TEXCOORD6;
varying lowp vec3 xlv_TEXCOORD5;
varying lowp vec4 xlv_TEXCOORD4;
varying lowp vec4 xlv_TEXCOORD3;
varying lowp vec4 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp float _FresnelPower;
uniform mediump float _Gloss;
uniform mediump float _Shininess;
uniform mediump float _ReflectPower;
uniform lowp vec4 _ReflectColor;
uniform lowp vec4 _Color;
uniform samplerCube _Cube;
uniform sampler2D _SpecMap;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform lowp vec4 _LightColor0;
void main ()
{
  lowp vec4 c_1;
  highp vec3 tmpvar_2;
  mediump vec3 tmpvar_3;
  mediump vec3 tmpvar_4;
  mediump vec3 tmpvar_5;
  lowp vec3 tmpvar_6;
  tmpvar_6.x = xlv_TEXCOORD2.w;
  tmpvar_6.y = xlv_TEXCOORD3.w;
  tmpvar_6.z = xlv_TEXCOORD4.w;
  tmpvar_2 = tmpvar_6;
  lowp vec3 tmpvar_7;
  tmpvar_7 = xlv_TEXCOORD2.xyz;
  tmpvar_3 = tmpvar_7;
  lowp vec3 tmpvar_8;
  tmpvar_8 = xlv_TEXCOORD3.xyz;
  tmpvar_4 = tmpvar_8;
  lowp vec3 tmpvar_9;
  tmpvar_9 = xlv_TEXCOORD4.xyz;
  tmpvar_5 = tmpvar_9;
  mediump vec3 tmpvar_10;
  mediump vec3 tmpvar_11;
  mediump float tmpvar_12;
  lowp vec4 tmpvar_13;
  tmpvar_13 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_14;
  tmpvar_14 = (tmpvar_13.xyz * _Color.xyz);
  tmpvar_10 = tmpvar_14;
  lowp vec4 tmpvar_15;
  tmpvar_15 = texture2D (_SpecMap, xlv_TEXCOORD0);
  mediump vec3 tmpvar_16;
  tmpvar_16 = ((tmpvar_13.w * tmpvar_15.xyz) * _Gloss);
  lowp vec3 normal_17;
  normal_17.xy = ((texture2D (_BumpMap, xlv_TEXCOORD0).wy * 2.0) - 1.0);
  normal_17.z = sqrt(((1.0 - (normal_17.x * normal_17.x)) - (normal_17.y * normal_17.y)));
  tmpvar_11 = normal_17;
  mediump vec3 tmpvar_18;
  tmpvar_18.x = dot (tmpvar_3, tmpvar_11);
  tmpvar_18.y = dot (tmpvar_4, tmpvar_11);
  tmpvar_18.z = dot (tmpvar_5, tmpvar_11);
  highp vec3 tmpvar_19;
  tmpvar_19 = (tmpvar_2 - (2.0 * (dot (tmpvar_18, tmpvar_2) * tmpvar_18)));
  mediump vec3 tmpvar_20;
  tmpvar_20 = normalize(tmpvar_11);
  highp float tmpvar_21;
  tmpvar_21 = max ((0.20373 + (0.79627 * pow (clamp ((1.0 - max (dot (normalize(xlv_TEXCOORD1), tmpvar_20), 0.0)), 0.0, 1.0), _FresnelPower))), 0.0);
  lowp vec4 tmpvar_22;
  tmpvar_22 = (textureCube (_Cube, tmpvar_19) * tmpvar_13.w);
  tmpvar_12 = tmpvar_21;
  highp vec3 tmpvar_23;
  tmpvar_23 = normalize(xlv_TEXCOORD1);
  mediump vec3 lightDir_24;
  lightDir_24 = xlv_TEXCOORD5;
  mediump vec3 viewDir_25;
  viewDir_25 = tmpvar_23;
  mediump vec4 c_26;
  mediump vec3 specCol_27;
  highp float nh_28;
  mediump float tmpvar_29;
  tmpvar_29 = max (0.0, dot (tmpvar_11, normalize((lightDir_24 + viewDir_25))));
  nh_28 = tmpvar_29;
  mediump float arg1_30;
  arg1_30 = (32.0 * _Shininess);
  highp vec3 tmpvar_31;
  tmpvar_31 = (pow (nh_28, arg1_30) * tmpvar_16);
  specCol_27 = tmpvar_31;
  c_26.xyz = ((((tmpvar_10 * _LightColor0.xyz) * max (0.0, dot (tmpvar_11, lightDir_24))) + (_LightColor0.xyz * specCol_27)) * 2.0);
  c_26.w = tmpvar_12;
  c_1 = c_26;
  mediump vec3 tmpvar_32;
  tmpvar_32 = (c_1.xyz + (tmpvar_10 * xlv_TEXCOORD6));
  c_1.xyz = tmpvar_32;
  mediump vec3 tmpvar_33;
  tmpvar_33 = (c_1.xyz + ((tmpvar_22.xyz * _ReflectColor.xyz) * _ReflectPower));
  c_1.xyz = tmpvar_33;
  c_1.w = tmpvar_12;
  gl_FragData[0] = c_1;
}



#endif"
}

SubProgram "opengl " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" }
Bind "vertex" Vertex
Bind "tangent" ATTR14
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Vector 13 [_WorldSpaceCameraPos]
Matrix 5 [_Object2World]
Matrix 9 [_World2Object]
Vector 15 [unity_Scale]
Vector 16 [unity_LightmapST]
Vector 17 [_MainTex_ST]
"3.0-!!ARBvp1.0
# 34 ALU
PARAM c[18] = { { 1 },
		state.matrix.mvp,
		program.local[5..17] };
TEMP R0;
TEMP R1;
TEMP R2;
MOV R0.xyz, vertex.attrib[14];
MUL R1.xyz, vertex.normal.zxyw, R0.yzxw;
MAD R0.xyz, vertex.normal.yzxw, R0.zxyw, -R1;
MUL R2.xyz, R0, vertex.attrib[14].w;
MOV R0.xyz, c[13];
MOV R0.w, c[0].x;
DP4 R1.z, R0, c[11];
DP4 R1.x, R0, c[9];
DP4 R1.y, R0, c[10];
MAD R1.xyz, R1, c[15].w, -vertex.position;
DP3 R0.y, R2, c[5];
DP3 R0.w, -R1, c[5];
DP3 R0.x, vertex.attrib[14], c[5];
DP3 R0.z, vertex.normal, c[5];
MUL result.texcoord[2], R0, c[15].w;
DP3 R0.y, R2, c[6];
DP3 R0.w, -R1, c[6];
DP3 R0.x, vertex.attrib[14], c[6];
DP3 R0.z, vertex.normal, c[6];
MUL result.texcoord[3], R0, c[15].w;
DP3 R0.y, R2, c[7];
DP3 R0.w, -R1, c[7];
DP3 R0.x, vertex.attrib[14], c[7];
DP3 R0.z, vertex.normal, c[7];
DP3 result.texcoord[1].y, R1, R2;
MUL result.texcoord[4], R0, c[15].w;
DP3 result.texcoord[1].z, vertex.normal, R1;
DP3 result.texcoord[1].x, R1, vertex.attrib[14];
MAD result.texcoord[0].xy, vertex.texcoord[0], c[17], c[17].zwzw;
MAD result.texcoord[5].xy, vertex.texcoord[1], c[16], c[16].zwzw;
DP4 result.position.w, vertex.position, c[4];
DP4 result.position.z, vertex.position, c[3];
DP4 result.position.y, vertex.position, c[2];
DP4 result.position.x, vertex.position, c[1];
END
# 34 instructions, 3 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Matrix 0 [glstate_matrix_mvp]
Vector 12 [_WorldSpaceCameraPos]
Matrix 4 [_Object2World]
Matrix 8 [_World2Object]
Vector 13 [unity_Scale]
Vector 14 [unity_LightmapST]
Vector 15 [_MainTex_ST]
"vs_3_0
; 35 ALU
dcl_position o0
dcl_texcoord0 o1
dcl_texcoord1 o2
dcl_texcoord2 o3
dcl_texcoord3 o4
dcl_texcoord4 o5
dcl_texcoord5 o6
def c16, 1.00000000, 0, 0, 0
dcl_position0 v0
dcl_tangent0 v1
dcl_normal0 v2
dcl_texcoord0 v3
dcl_texcoord1 v4
mov r0.xyz, v1
mul r1.xyz, v2.zxyw, r0.yzxw
mov r0.xyz, v1
mad r0.xyz, v2.yzxw, r0.zxyw, -r1
mul r2.xyz, r0, v1.w
mov r0.xyz, c12
mov r0.w, c16.x
dp4 r1.z, r0, c10
dp4 r1.x, r0, c8
dp4 r1.y, r0, c9
mad r1.xyz, r1, c13.w, -v0
dp3 r0.y, r2, c4
dp3 r0.w, -r1, c4
dp3 r0.x, v1, c4
dp3 r0.z, v2, c4
mul o3, r0, c13.w
dp3 r0.y, r2, c5
dp3 r0.w, -r1, c5
dp3 r0.x, v1, c5
dp3 r0.z, v2, c5
mul o4, r0, c13.w
dp3 r0.y, r2, c6
dp3 r0.w, -r1, c6
dp3 r0.x, v1, c6
dp3 r0.z, v2, c6
dp3 o2.y, r1, r2
mul o5, r0, c13.w
dp3 o2.z, v2, r1
dp3 o2.x, r1, v1
mad o1.xy, v3, c15, c15.zwzw
mad o6.xy, v4, c14, c14.zwzw
dp4 o0.w, v0, c3
dp4 o0.z, v0, c2
dp4 o0.y, v0, c1
dp4 o0.x, v0, c0
"
}

SubProgram "xbox360 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Vector 14 [_MainTex_ST]
Matrix 5 [_Object2World] 3
Matrix 8 [_World2Object] 4
Vector 0 [_WorldSpaceCameraPos]
Matrix 1 [glstate_matrix_mvp] 4
Vector 13 [unity_LightmapST]
Vector 12 [unity_Scale]
// Shader Timing Estimate, in Cycles/64 vertex vector:
// ALU: 50.67 (38 instructions), vertex: 64, texture: 0,
//   sequencer: 20,  12 GPRs, 15 threads,
// Performance (if enough threads): ~64 cycles per vector
// * Vertex cycle estimates are assuming 3 vfetch_minis for every vfetch_full,
//     with <= 32 bytes per vfetch_full group.

"vs_360
backbbabaaaaabpiaaaaacemaaaaaaaaaaaaaaceaaaaaaaaaaaaabieaaaaaaaa
aaaaaaaaaaaaabfmaaaaaabmaaaaabeppppoadaaaaaaaaahaaaaaabmaaaaaaaa
aaaaabeiaaaaaakiaaacaaaoaaabaaaaaaaaaaleaaaaaaaaaaaaaameaaacaaaf
aaadaaaaaaaaaaneaaaaaaaaaaaaaaoeaaacaaaiaaaeaaaaaaaaaaneaaaaaaaa
aaaaaapcaaacaaaaaaabaaaaaaaaabaiaaaaaaaaaaaaabbiaaacaaabaaaeaaaa
aaaaaaneaaaaaaaaaaaaabclaaacaaanaaabaaaaaaaaaaleaaaaaaaaaaaaabdm
aaacaaamaaabaaaaaaaaaaleaaaaaaaafpengbgjgofegfhifpfdfeaaaaabaaad
aaabaaaeaaabaaaaaaaaaaaafpepgcgkgfgdhedcfhgphcgmgeaaklklaaadaaad
aaaeaaaeaaabaaaaaaaaaaaafpfhgphcgmgedcepgcgkgfgdheaafpfhgphcgmge
fdhagbgdgfedgbgngfhcgbfagphdaaklaaabaaadaaabaaadaaabaaaaaaaaaaaa
ghgmhdhegbhegffpgngbhehcgjhifpgnhghaaahfgogjhehjfpemgjghgihegngb
hafdfeaahfgogjhehjfpfdgdgbgmgfaahghdfpddfpdaaadccodacodcdadddfdd
codaaaklaaaaaaaaaaaaacemaafbaaalaaaaaaaaaaaaaaaaaaaaemmgaaaaaaab
aaaaaaafaaaaaaaiaaaaacjaaabaaaafaaaagaagaaaadaahaaaafaaiaacbfaaj
aaaadafaaaabhbfbaaaepcfcaaafpdfdaaagpefeaaahdfffaaaabacfaaaaaacc
aaaaaacdaaaabaceaaaabacnaaaabacoaaaabacpaaaabacgpbfffaafaaaabcab
mcaaaaaaaaaaeaakaaaabcaameaaaaaaaaaagaaogabebcaabcaaaaaaaaaagabk
gacabcaabcaaaaaaaaaagacgeacmbcaaccaaaaaaafpifaaaaaaaagiiaaaaaaaa
afpilaaaaaaaagiiaaaaaaaaafpiaaaaaaaaacihaaaaaaaaafpieaaaaaaaadmh
aaaaaaaaafpidaaaaaaaadmhaaaaaaaamiapaaabaabliiaakbafaeaamiapaaab
aamgiiaaklafadabmiapaaabaalbdejeklafacabmiapiadoaagmaadeklafabab
miahaaagaamamgmaalakaaalbeccabaiaablgmblkbalagalkmbeaiaiaablgmeb
ibalahafkibhacajaamggcmdibalahagkichacabaalbgcedibalagagmiahaaag
aalelbleclajaaagkiehacahaabcgfidmbaaalagmiahaaahablhlomaolaaalah
miahaaagaamagmleclaiaaagmialaaacaalbgcleklaaafacmiahaaakaagmlole
klalafabbealaaabaamggcgmkbaaahakaebeacacaagmgmgmoaacabajbeaeaaad
aalblbmgoaacabakaebbadaeaalbmglboaakajajmiahaaakabmablmaklagamaf
beahaaabaamnblblobahalacaeedaeafaamglcblkbabahabkmchacagaemggcma
ibakahagkmihacajaelbgciaibakagagmiabiaabaaloloaapaakalaamiaciaab
aagcloaapaabakaamiaeiaabaalobcaapaakaaaamiadiaaaaabjlabkilaeaoao
miadiaafaabjlabkiladananmiahaaaaaegmloleklakafajmiadaaabaalblcbj
klabafacbeaiaaacaagmgmgmoaaaagabaeciacadaamglbgmoaaaagafbeacaaae
aalololbpaaiahabaeciadaeaalbmglboaaaagafmiapiaacaaaablaakbaeamaa
miapiaadaaaablaakbadamaamiapiaaeaaaablaakbacamaaaaaaaaaaaaaaaaaa
aaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" }
Matrix 256 [glstate_matrix_mvp]
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Vector 467 [_WorldSpaceCameraPos]
Matrix 260 [_Object2World]
Matrix 264 [_World2Object]
Vector 466 [unity_Scale]
Vector 465 [unity_LightmapST]
Vector 464 [_MainTex_ST]
"sce_vp_rsx // 33 instructions using 6 registers
[Configuration]
8
0000002143050600
[Microcode]
528
00021c6c00400e0c0106c0836041dffc00029c6c005d300c0186c0836041dffc
401f9c6c011d0808010400d740619f9c401f9c6c011d1908010400d740619fb0
401f9c6c01d0300d8106c0c360403f80401f9c6c01d0200d8106c0c360405f80
401f9c6c01d0100d8106c0c360409f80401f9c6c01d0000d8106c0c360411f80
00001c6c01506e0c010600c360411ffc00001c6c0150620c010600c360405ffc
00009c6c01505e0c010600c360411ffc00009c6c0150520c010600c360405ffc
00011c6c01504e0c010600c360411ffc00011c6c0150420c010600c360405ffc
00019c6c0190a00c0a86c0c360405ffc00019c6c0190900c0a86c0c360409ffc
00019c6c0190800c0a86c0c360411ffc00029c6c00800243011844436041dffc
00029c6c010002308121846302a1dffc00021c6c011d200c06bfc0e30041dffc
00019c6c00800e0c0abfc0836041dffc401f9c6c0140020c0106044360405fa0
401f9c6c01400e0c0886008360411fa000001c6c0150608c088600c360403ffc
00009c6c0150508c088600c360403ffc00011c6c0150408c088600c360403ffc
401f9c6c0140000c0886034360409fa000001c6c0150600c068600c360409ffc
00011c6c0150400c068600c360409ffc00009c6c0150500c068600c360409ffc
401f9c6c009d200d84bfc0c36041ffa4401f9c6c009d200d82bfc0c36041ffa8
401f9c6c009d200d80bfc0c36041ffad
"
}

SubProgram "d3d11 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Bind "color" Color
ConstBuffer "$Globals" 128 // 128 used size, 11 vars
Vector 96 [unity_LightmapST] 4
Vector 112 [_MainTex_ST] 4
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
// 44 instructions, 4 temp regs, 0 temp arrays:
// ALU 21 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0
eefiecednbnldjglkhlibljoikddnohmfkgmleinabaaaaaaoiahaaaaadaaaaaa
cmaaaaaapeaaaaaameabaaaaejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaa
abaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapadaaaaljaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfc
enebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheomiaaaaaaahaaaaaa
aiaaaaaalaaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaalmaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaabaaaaaaadamaaaalmaaaaaaafaaaaaaaaaaaaaa
adaaaaaaabaaaaaaamadaaaalmaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahaiaaaalmaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaapaaaaaalmaaaaaa
adaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaalmaaaaaaaeaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaafdfgfpfaepfdejfeejepeoaafeeffiedepepfcee
aaklklklfdeieefcbmagaaaaeaaaabaaihabaaaafjaaaaaeegiocaaaaaaaaaaa
aiaaaaaafjaaaaaeegiocaaaabaaaaaaafaaaaaafjaaaaaeegiocaaaacaaaaaa
bfaaaaaafpaaaaadpcbabaaaaaaaaaaafpaaaaadpcbabaaaabaaaaaafpaaaaad
hcbabaaaacaaaaaafpaaaaaddcbabaaaadaaaaaafpaaaaaddcbabaaaaeaaaaaa
ghaaaaaepccabaaaaaaaaaaaabaaaaaagfaaaaaddccabaaaabaaaaaagfaaaaad
mccabaaaabaaaaaagfaaaaadhccabaaaacaaaaaagfaaaaadpccabaaaadaaaaaa
gfaaaaadpccabaaaaeaaaaaagfaaaaadpccabaaaafaaaaaagiaaaaacaeaaaaaa
diaaaaaipcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaacaaaaaaabaaaaaa
dcaaaaakpcaabaaaaaaaaaaaegiocaaaacaaaaaaaaaaaaaaagbabaaaaaaaaaaa
egaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaacaaaaaaacaaaaaa
kgbkbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpccabaaaaaaaaaaaegiocaaa
acaaaaaaadaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaaldccabaaa
abaaaaaaegbabaaaadaaaaaaegiacaaaaaaaaaaaahaaaaaaogikcaaaaaaaaaaa
ahaaaaaadcaaaaalmccabaaaabaaaaaaagbebaaaaeaaaaaaagiecaaaaaaaaaaa
agaaaaaakgiocaaaaaaaaaaaagaaaaaadiaaaaajhcaabaaaaaaaaaaafgifcaaa
abaaaaaaaeaaaaaaegiccaaaacaaaaaabbaaaaaadcaaaaalhcaabaaaaaaaaaaa
egiccaaaacaaaaaabaaaaaaaagiacaaaabaaaaaaaeaaaaaaegacbaaaaaaaaaaa
dcaaaaalhcaabaaaaaaaaaaaegiccaaaacaaaaaabcaaaaaakgikcaaaabaaaaaa
aeaaaaaaegacbaaaaaaaaaaaaaaaaaaihcaabaaaaaaaaaaaegacbaaaaaaaaaaa
egiccaaaacaaaaaabdaaaaaadcaaaaalhcaabaaaaaaaaaaaegacbaaaaaaaaaaa
pgipcaaaacaaaaaabeaaaaaaegbcbaiaebaaaaaaaaaaaaaabaaaaaahbccabaaa
acaaaaaaegbcbaaaabaaaaaaegacbaaaaaaaaaaabaaaaaaheccabaaaacaaaaaa
egbcbaaaacaaaaaaegacbaaaaaaaaaaadiaaaaahhcaabaaaabaaaaaajgbebaaa
abaaaaaacgbjbaaaacaaaaaadcaaaaakhcaabaaaabaaaaaajgbebaaaacaaaaaa
cgbjbaaaabaaaaaaegacbaiaebaaaaaaabaaaaaadiaaaaahhcaabaaaabaaaaaa
egacbaaaabaaaaaapgbpbaaaabaaaaaabaaaaaahcccabaaaacaaaaaaegacbaaa
abaaaaaaegacbaaaaaaaaaaadiaaaaajhcaabaaaacaaaaaafgafbaiaebaaaaaa
aaaaaaaaegiccaaaacaaaaaaanaaaaaadcaaaaallcaabaaaaaaaaaaaegiicaaa
acaaaaaaamaaaaaaagaabaiaebaaaaaaaaaaaaaaegaibaaaacaaaaaadcaaaaal
lcaabaaaaaaaaaaaegiicaaaacaaaaaaaoaaaaaakgakbaiaebaaaaaaaaaaaaaa
egambaaaaaaaaaaadgaaaaaficaabaaaacaaaaaaakaabaaaaaaaaaaadgaaaaag
bcaabaaaadaaaaaaakiacaaaacaaaaaaamaaaaaadgaaaaagccaabaaaadaaaaaa
akiacaaaacaaaaaaanaaaaaadgaaaaagecaabaaaadaaaaaaakiacaaaacaaaaaa
aoaaaaaabaaaaaahccaabaaaacaaaaaaegacbaaaabaaaaaaegacbaaaadaaaaaa
baaaaaahbcaabaaaacaaaaaaegbcbaaaabaaaaaaegacbaaaadaaaaaabaaaaaah
ecaabaaaacaaaaaaegbcbaaaacaaaaaaegacbaaaadaaaaaadiaaaaaipccabaaa
adaaaaaaegaobaaaacaaaaaapgipcaaaacaaaaaabeaaaaaadgaaaaaficaabaaa
acaaaaaabkaabaaaaaaaaaaadgaaaaagbcaabaaaadaaaaaabkiacaaaacaaaaaa
amaaaaaadgaaaaagccaabaaaadaaaaaabkiacaaaacaaaaaaanaaaaaadgaaaaag
ecaabaaaadaaaaaabkiacaaaacaaaaaaaoaaaaaabaaaaaahccaabaaaacaaaaaa
egacbaaaabaaaaaaegacbaaaadaaaaaabaaaaaahbcaabaaaacaaaaaaegbcbaaa
abaaaaaaegacbaaaadaaaaaabaaaaaahecaabaaaacaaaaaaegbcbaaaacaaaaaa
egacbaaaadaaaaaadiaaaaaipccabaaaaeaaaaaaegaobaaaacaaaaaapgipcaaa
acaaaaaabeaaaaaadgaaaaagbcaabaaaacaaaaaackiacaaaacaaaaaaamaaaaaa
dgaaaaagccaabaaaacaaaaaackiacaaaacaaaaaaanaaaaaadgaaaaagecaabaaa
acaaaaaackiacaaaacaaaaaaaoaaaaaabaaaaaahccaabaaaaaaaaaaaegacbaaa
abaaaaaaegacbaaaacaaaaaabaaaaaahbcaabaaaaaaaaaaaegbcbaaaabaaaaaa
egacbaaaacaaaaaabaaaaaahecaabaaaaaaaaaaaegbcbaaaacaaaaaaegacbaaa
acaaaaaadiaaaaaipccabaaaafaaaaaaegaobaaaaaaaaaaapgipcaaaacaaaaaa
beaaaaaadoaaaaab"
}

SubProgram "gles " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" }
"!!GLES
#define SHADER_API_GLES 1
#define tex2D texture2D


#ifdef VERTEX
#define gl_ModelViewProjectionMatrix glstate_matrix_mvp
uniform mat4 glstate_matrix_mvp;

varying highp vec2 xlv_TEXCOORD5;
varying lowp vec4 xlv_TEXCOORD4;
varying lowp vec4 xlv_TEXCOORD3;
varying lowp vec4 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 unity_LightmapST;
uniform highp vec4 unity_Scale;
uniform highp mat4 _World2Object;
uniform highp mat4 _Object2World;

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
  lowp vec4 tmpvar_3;
  lowp vec4 tmpvar_4;
  lowp vec4 tmpvar_5;
  highp vec4 tmpvar_6;
  tmpvar_6.w = 1.0;
  tmpvar_6.xyz = _WorldSpaceCameraPos;
  mat3 tmpvar_7;
  tmpvar_7[0] = _Object2World[0].xyz;
  tmpvar_7[1] = _Object2World[1].xyz;
  tmpvar_7[2] = _Object2World[2].xyz;
  highp vec3 tmpvar_8;
  tmpvar_8 = (tmpvar_7 * (_glesVertex.xyz - ((_World2Object * tmpvar_6).xyz * unity_Scale.w)));
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
  vec4 v_12;
  v_12.x = _Object2World[0].x;
  v_12.y = _Object2World[1].x;
  v_12.z = _Object2World[2].x;
  v_12.w = _Object2World[3].x;
  highp vec4 tmpvar_13;
  tmpvar_13.xyz = (tmpvar_11 * v_12.xyz);
  tmpvar_13.w = tmpvar_8.x;
  highp vec4 tmpvar_14;
  tmpvar_14 = (tmpvar_13 * unity_Scale.w);
  tmpvar_3 = tmpvar_14;
  vec4 v_15;
  v_15.x = _Object2World[0].y;
  v_15.y = _Object2World[1].y;
  v_15.z = _Object2World[2].y;
  v_15.w = _Object2World[3].y;
  highp vec4 tmpvar_16;
  tmpvar_16.xyz = (tmpvar_11 * v_15.xyz);
  tmpvar_16.w = tmpvar_8.y;
  highp vec4 tmpvar_17;
  tmpvar_17 = (tmpvar_16 * unity_Scale.w);
  tmpvar_4 = tmpvar_17;
  vec4 v_18;
  v_18.x = _Object2World[0].z;
  v_18.y = _Object2World[1].z;
  v_18.z = _Object2World[2].z;
  v_18.w = _Object2World[3].z;
  highp vec4 tmpvar_19;
  tmpvar_19.xyz = (tmpvar_11 * v_18.xyz);
  tmpvar_19.w = tmpvar_8.z;
  highp vec4 tmpvar_20;
  tmpvar_20 = (tmpvar_19 * unity_Scale.w);
  tmpvar_5 = tmpvar_20;
  highp vec4 tmpvar_21;
  tmpvar_21.w = 1.0;
  tmpvar_21.xyz = _WorldSpaceCameraPos;
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = (tmpvar_11 * (((_World2Object * tmpvar_21).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = tmpvar_3;
  xlv_TEXCOORD3 = tmpvar_4;
  xlv_TEXCOORD4 = tmpvar_5;
  xlv_TEXCOORD5 = ((_glesMultiTexCoord1.xy * unity_LightmapST.xy) + unity_LightmapST.zw);
}



#endif
#ifdef FRAGMENT

varying highp vec2 xlv_TEXCOORD5;
varying lowp vec4 xlv_TEXCOORD4;
varying lowp vec4 xlv_TEXCOORD3;
varying lowp vec4 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform sampler2D unity_Lightmap;
uniform highp float _FresnelPower;
uniform mediump float _ReflectPower;
uniform lowp vec4 _ReflectColor;
uniform lowp vec4 _Color;
uniform samplerCube _Cube;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
void main ()
{
  lowp vec4 c_1;
  highp vec3 tmpvar_2;
  mediump vec3 tmpvar_3;
  mediump vec3 tmpvar_4;
  mediump vec3 tmpvar_5;
  lowp vec3 tmpvar_6;
  tmpvar_6.x = xlv_TEXCOORD2.w;
  tmpvar_6.y = xlv_TEXCOORD3.w;
  tmpvar_6.z = xlv_TEXCOORD4.w;
  tmpvar_2 = tmpvar_6;
  lowp vec3 tmpvar_7;
  tmpvar_7 = xlv_TEXCOORD2.xyz;
  tmpvar_3 = tmpvar_7;
  lowp vec3 tmpvar_8;
  tmpvar_8 = xlv_TEXCOORD3.xyz;
  tmpvar_4 = tmpvar_8;
  lowp vec3 tmpvar_9;
  tmpvar_9 = xlv_TEXCOORD4.xyz;
  tmpvar_5 = tmpvar_9;
  mediump vec3 tmpvar_10;
  mediump vec3 tmpvar_11;
  mediump float tmpvar_12;
  lowp vec4 tmpvar_13;
  tmpvar_13 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_14;
  tmpvar_14 = (tmpvar_13.xyz * _Color.xyz);
  tmpvar_10 = tmpvar_14;
  lowp vec3 tmpvar_15;
  tmpvar_15 = ((texture2D (_BumpMap, xlv_TEXCOORD0).xyz * 2.0) - 1.0);
  tmpvar_11 = tmpvar_15;
  mediump vec3 tmpvar_16;
  tmpvar_16.x = dot (tmpvar_3, tmpvar_11);
  tmpvar_16.y = dot (tmpvar_4, tmpvar_11);
  tmpvar_16.z = dot (tmpvar_5, tmpvar_11);
  highp vec3 tmpvar_17;
  tmpvar_17 = (tmpvar_2 - (2.0 * (dot (tmpvar_16, tmpvar_2) * tmpvar_16)));
  mediump vec3 tmpvar_18;
  tmpvar_18 = normalize(tmpvar_11);
  highp float tmpvar_19;
  tmpvar_19 = max ((0.20373 + (0.79627 * pow (clamp ((1.0 - max (dot (normalize(xlv_TEXCOORD1), tmpvar_18), 0.0)), 0.0, 1.0), _FresnelPower))), 0.0);
  lowp vec4 tmpvar_20;
  tmpvar_20 = (textureCube (_Cube, tmpvar_17) * tmpvar_13.w);
  tmpvar_12 = tmpvar_19;
  lowp vec3 tmpvar_21;
  tmpvar_21 = (2.0 * texture2D (unity_Lightmap, xlv_TEXCOORD5).xyz);
  mediump vec3 tmpvar_22;
  tmpvar_22 = (tmpvar_10 * tmpvar_21);
  c_1.xyz = tmpvar_22;
  c_1.w = tmpvar_12;
  mediump vec3 tmpvar_23;
  tmpvar_23 = (c_1.xyz + ((tmpvar_20.xyz * _ReflectColor.xyz) * _ReflectPower));
  c_1.xyz = tmpvar_23;
  c_1.w = tmpvar_12;
  gl_FragData[0] = c_1;
}



#endif"
}

SubProgram "glesdesktop " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" }
"!!GLES
#define SHADER_API_GLES 1
#define tex2D texture2D


#ifdef VERTEX
#define gl_ModelViewProjectionMatrix glstate_matrix_mvp
uniform mat4 glstate_matrix_mvp;

varying highp vec2 xlv_TEXCOORD5;
varying lowp vec4 xlv_TEXCOORD4;
varying lowp vec4 xlv_TEXCOORD3;
varying lowp vec4 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 unity_LightmapST;
uniform highp vec4 unity_Scale;
uniform highp mat4 _World2Object;
uniform highp mat4 _Object2World;

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
  lowp vec4 tmpvar_3;
  lowp vec4 tmpvar_4;
  lowp vec4 tmpvar_5;
  highp vec4 tmpvar_6;
  tmpvar_6.w = 1.0;
  tmpvar_6.xyz = _WorldSpaceCameraPos;
  mat3 tmpvar_7;
  tmpvar_7[0] = _Object2World[0].xyz;
  tmpvar_7[1] = _Object2World[1].xyz;
  tmpvar_7[2] = _Object2World[2].xyz;
  highp vec3 tmpvar_8;
  tmpvar_8 = (tmpvar_7 * (_glesVertex.xyz - ((_World2Object * tmpvar_6).xyz * unity_Scale.w)));
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
  vec4 v_12;
  v_12.x = _Object2World[0].x;
  v_12.y = _Object2World[1].x;
  v_12.z = _Object2World[2].x;
  v_12.w = _Object2World[3].x;
  highp vec4 tmpvar_13;
  tmpvar_13.xyz = (tmpvar_11 * v_12.xyz);
  tmpvar_13.w = tmpvar_8.x;
  highp vec4 tmpvar_14;
  tmpvar_14 = (tmpvar_13 * unity_Scale.w);
  tmpvar_3 = tmpvar_14;
  vec4 v_15;
  v_15.x = _Object2World[0].y;
  v_15.y = _Object2World[1].y;
  v_15.z = _Object2World[2].y;
  v_15.w = _Object2World[3].y;
  highp vec4 tmpvar_16;
  tmpvar_16.xyz = (tmpvar_11 * v_15.xyz);
  tmpvar_16.w = tmpvar_8.y;
  highp vec4 tmpvar_17;
  tmpvar_17 = (tmpvar_16 * unity_Scale.w);
  tmpvar_4 = tmpvar_17;
  vec4 v_18;
  v_18.x = _Object2World[0].z;
  v_18.y = _Object2World[1].z;
  v_18.z = _Object2World[2].z;
  v_18.w = _Object2World[3].z;
  highp vec4 tmpvar_19;
  tmpvar_19.xyz = (tmpvar_11 * v_18.xyz);
  tmpvar_19.w = tmpvar_8.z;
  highp vec4 tmpvar_20;
  tmpvar_20 = (tmpvar_19 * unity_Scale.w);
  tmpvar_5 = tmpvar_20;
  highp vec4 tmpvar_21;
  tmpvar_21.w = 1.0;
  tmpvar_21.xyz = _WorldSpaceCameraPos;
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = (tmpvar_11 * (((_World2Object * tmpvar_21).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = tmpvar_3;
  xlv_TEXCOORD3 = tmpvar_4;
  xlv_TEXCOORD4 = tmpvar_5;
  xlv_TEXCOORD5 = ((_glesMultiTexCoord1.xy * unity_LightmapST.xy) + unity_LightmapST.zw);
}



#endif
#ifdef FRAGMENT

varying highp vec2 xlv_TEXCOORD5;
varying lowp vec4 xlv_TEXCOORD4;
varying lowp vec4 xlv_TEXCOORD3;
varying lowp vec4 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform sampler2D unity_Lightmap;
uniform highp float _FresnelPower;
uniform mediump float _ReflectPower;
uniform lowp vec4 _ReflectColor;
uniform lowp vec4 _Color;
uniform samplerCube _Cube;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
void main ()
{
  lowp vec4 c_1;
  highp vec3 tmpvar_2;
  mediump vec3 tmpvar_3;
  mediump vec3 tmpvar_4;
  mediump vec3 tmpvar_5;
  lowp vec3 tmpvar_6;
  tmpvar_6.x = xlv_TEXCOORD2.w;
  tmpvar_6.y = xlv_TEXCOORD3.w;
  tmpvar_6.z = xlv_TEXCOORD4.w;
  tmpvar_2 = tmpvar_6;
  lowp vec3 tmpvar_7;
  tmpvar_7 = xlv_TEXCOORD2.xyz;
  tmpvar_3 = tmpvar_7;
  lowp vec3 tmpvar_8;
  tmpvar_8 = xlv_TEXCOORD3.xyz;
  tmpvar_4 = tmpvar_8;
  lowp vec3 tmpvar_9;
  tmpvar_9 = xlv_TEXCOORD4.xyz;
  tmpvar_5 = tmpvar_9;
  mediump vec3 tmpvar_10;
  mediump vec3 tmpvar_11;
  mediump float tmpvar_12;
  lowp vec4 tmpvar_13;
  tmpvar_13 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_14;
  tmpvar_14 = (tmpvar_13.xyz * _Color.xyz);
  tmpvar_10 = tmpvar_14;
  lowp vec3 normal_15;
  normal_15.xy = ((texture2D (_BumpMap, xlv_TEXCOORD0).wy * 2.0) - 1.0);
  normal_15.z = sqrt(((1.0 - (normal_15.x * normal_15.x)) - (normal_15.y * normal_15.y)));
  tmpvar_11 = normal_15;
  mediump vec3 tmpvar_16;
  tmpvar_16.x = dot (tmpvar_3, tmpvar_11);
  tmpvar_16.y = dot (tmpvar_4, tmpvar_11);
  tmpvar_16.z = dot (tmpvar_5, tmpvar_11);
  highp vec3 tmpvar_17;
  tmpvar_17 = (tmpvar_2 - (2.0 * (dot (tmpvar_16, tmpvar_2) * tmpvar_16)));
  mediump vec3 tmpvar_18;
  tmpvar_18 = normalize(tmpvar_11);
  highp float tmpvar_19;
  tmpvar_19 = max ((0.20373 + (0.79627 * pow (clamp ((1.0 - max (dot (normalize(xlv_TEXCOORD1), tmpvar_18), 0.0)), 0.0, 1.0), _FresnelPower))), 0.0);
  lowp vec4 tmpvar_20;
  tmpvar_20 = (textureCube (_Cube, tmpvar_17) * tmpvar_13.w);
  tmpvar_12 = tmpvar_19;
  lowp vec4 tmpvar_21;
  tmpvar_21 = texture2D (unity_Lightmap, xlv_TEXCOORD5);
  lowp vec3 tmpvar_22;
  tmpvar_22 = ((8.0 * tmpvar_21.w) * tmpvar_21.xyz);
  mediump vec3 tmpvar_23;
  tmpvar_23 = (tmpvar_10 * tmpvar_22);
  c_1.xyz = tmpvar_23;
  c_1.w = tmpvar_12;
  mediump vec3 tmpvar_24;
  tmpvar_24 = (c_1.xyz + ((tmpvar_20.xyz * _ReflectColor.xyz) * _ReflectPower));
  c_1.xyz = tmpvar_24;
  c_1.w = tmpvar_12;
  gl_FragData[0] = c_1;
}



#endif"
}

SubProgram "opengl " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "VERTEXLIGHT_ON" }
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
"3.0-!!ARBvp1.0
# 89 ALU
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
DP4 R3.z, R0, c[28];
DP4 R3.y, R0, c[27];
DP4 R3.x, R0, c[26];
MUL R1.w, R3, R3;
MAD R0.x, R4, R4, -R1.w;
MOV R1.w, c[0].x;
DP4 R2.z, R4, c[25];
DP4 R2.y, R4, c[24];
DP4 R2.x, R4, c[23];
ADD R2.xyz, R2, R3;
MUL R3.xyz, R0.x, c[29];
ADD R3.xyz, R2, R3;
MOV R0.xyz, vertex.attrib[14];
MUL R2.xyz, vertex.normal.zxyw, R0.yzxw;
ADD result.texcoord[6].xyz, R3, R1;
MAD R0.xyz, vertex.normal.yzxw, R0.zxyw, -R2;
MOV R1.xyz, c[13];
MUL R3.xyz, R0, vertex.attrib[14].w;
MOV R0, c[14];
DP4 R2.z, R1, c[11];
DP4 R2.x, R1, c[9];
DP4 R2.y, R1, c[10];
MAD R2.xyz, R2, c[30].w, -vertex.position;
DP4 R1.z, R0, c[11];
DP4 R1.x, R0, c[9];
DP4 R1.y, R0, c[10];
DP3 R0.y, R3, c[5];
DP3 R0.w, -R2, c[5];
DP3 R0.x, vertex.attrib[14], c[5];
DP3 R0.z, vertex.normal, c[5];
MUL result.texcoord[2], R0, c[30].w;
DP3 R0.y, R3, c[6];
DP3 R0.w, -R2, c[6];
DP3 R0.x, vertex.attrib[14], c[6];
DP3 R0.z, vertex.normal, c[6];
MUL result.texcoord[3], R0, c[30].w;
DP3 R0.y, R3, c[7];
DP3 R0.w, -R2, c[7];
DP3 R0.x, vertex.attrib[14], c[7];
DP3 R0.z, vertex.normal, c[7];
DP3 result.texcoord[1].y, R2, R3;
DP3 result.texcoord[5].y, R3, R1;
MUL result.texcoord[4], R0, c[30].w;
DP3 result.texcoord[1].z, vertex.normal, R2;
DP3 result.texcoord[1].x, R2, vertex.attrib[14];
DP3 result.texcoord[5].z, vertex.normal, R1;
DP3 result.texcoord[5].x, vertex.attrib[14], R1;
MAD result.texcoord[0].xy, vertex.texcoord[0], c[31], c[31].zwzw;
DP4 result.position.w, vertex.position, c[4];
DP4 result.position.z, vertex.position, c[3];
DP4 result.position.y, vertex.position, c[2];
DP4 result.position.x, vertex.position, c[1];
END
# 89 instructions, 5 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "VERTEXLIGHT_ON" }
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
"vs_3_0
; 92 ALU
dcl_position o0
dcl_texcoord0 o1
dcl_texcoord1 o2
dcl_texcoord2 o3
dcl_texcoord3 o4
dcl_texcoord4 o5
dcl_texcoord5 o6
dcl_texcoord6 o7
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
add r2.xyz, r2, r0
add o7.xyz, r2, r1
mov r0.xyz, v1
mul r1.xyz, v2.zxyw, r0.yzxw
mov r0.xyz, v1
mad r0.xyz, v2.yzxw, r0.zxyw, -r1
mul r3.xyz, r0, v1.w
mov r0, c10
dp4 r4.z, c13, r0
mov r0, c9
dp4 r4.y, c13, r0
mov r1.w, c31.x
mov r1.xyz, c12
dp4 r2.z, r1, c10
dp4 r2.x, r1, c8
dp4 r2.y, r1, c9
mad r2.xyz, r2, c29.w, -v0
mov r1, c8
dp4 r4.x, c13, r1
dp3 r0.y, r3, c4
dp3 r0.w, -r2, c4
dp3 r0.x, v1, c4
dp3 r0.z, v2, c4
mul o3, r0, c29.w
dp3 r0.y, r3, c5
dp3 r0.w, -r2, c5
dp3 r0.x, v1, c5
dp3 r0.z, v2, c5
mul o4, r0, c29.w
dp3 r0.y, r3, c6
dp3 r0.w, -r2, c6
dp3 r0.x, v1, c6
dp3 r0.z, v2, c6
dp3 o2.y, r2, r3
dp3 o6.y, r3, r4
mul o5, r0, c29.w
dp3 o2.z, v2, r2
dp3 o2.x, r2, v1
dp3 o6.z, v2, r4
dp3 o6.x, v1, r4
mad o1.xy, v3, c30, c30.zwzw
dp4 o0.w, v0, c3
dp4 o0.z, v0, c2
dp4 o0.y, v0, c1
dp4 o0.x, v0, c0
"
}

SubProgram "xbox360 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "VERTEXLIGHT_ON" }
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
// ALU: 114.67 (86 instructions), vertex: 32, texture: 0,
//   sequencer: 36,  14 GPRs, 12 threads,
// Performance (if enough threads): ~114 cycles per vector
// * Vertex cycle estimates are assuming 3 vfetch_minis for every vfetch_full,
//     with <= 32 bytes per vfetch_full group.

"vs_360
backbbabaaaaadniaaaaaepaaaaaaaaaaaaaaaceaaaaaddaaaaaadfiaaaaaaaa
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
aaaaaabeaapmaabaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaeaaaaaaela
aagbaaanaaaaaaaaaaaaaaaaaaaafmohaaaaaaabaaaaaaaeaaaaaaalaaaaacja
aabaaaajaaaagaakaaaadaalaacafaamaaaadafaaaabhbfbaaaepcfcaaafpdfd
aaagpefeaaahhfffaaakhgfgaaaabadgaaaaaadaaaaaaadbaaaabadcaaaabaei
aaaabaejaaaabaekaaaaaaddaaaaaadeaaaabadfaaaabagcaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaadpiaaaaaaaaaaaaaaaaaaaaaaaaaaaaapaffeaajaaaabcaa
mcaaaaaaaaaaeaanaaaabcaameaaaaaaaaaagabbgabhbcaabcaaaaaaaaaagabn
gacdbcaabcaaaaaaaaaagacjgacpbcaabcaaaaaaaaaagadfgadlbcaabcaaaaaa
aaaagaebgaehbcaabcaaaaaaaaaagaengafdbcaabcaaaaaaaaaagafjeafpbcaa
ccaaaaaaafpieaaaaaaaaanbaaaaaaaaafpibaaaaaaaagiiaaaaaaaaafpifaaa
aaaaaoiiaaaaaaaaafpidaaaaaaaadmhaaaaaaaakmepanaaaamgiiedibaebebh
miapaaaaaalbiiaaklaebdaamiapaaaaaagmdejeklaebcaamiapiadoaablaade
klaebbaamiahaaakaamamgmaalblaabmceibaganaablgmgmkbabbfiabechaaac
aaleblblcbbmababkichanagaamgloebibafbhbgbebhaaaiaalblomgkbafbgab
kibhadahaalbgcmaibabbgbhmiahaaajaamamgleclblabackmihacacaamgleic
ibaebibhmialaaaaaalelbleclbkaaakkmehaaakaalogfecmbafabbhmiahaaal
abgflomaolafabakmialaaaaaamagmliclbjaaaamiahaaacaalbmaleklaebhac
miahaaakaalelbleclbkabajmiahaaaiaagmmagfklafbfaimialaaahaagmlole
klabbfahbeaeaaahaamggmgmoaaiagahaebeahaiaalbmggmoaaiagadbealaaaj
aagfblblkbafbnahaebeaiajaagmlbbloaaiagacbeahaaagaalblelbkbajbhah
miahaaamaamagmleclbjabakmiahaaacaagmleleklaebgacmiahaaagaagmlema
klajbgagmiahaaakabbabllpklaabnaeaeblajaaaamnblmgobalabaakiecadaj
aalolomdnaanalbhkiehaaalaemggcidibakbhbhmiahaaagaabllemaklajbfag
kibhadaeaelbgcmaibakbgbgmiahaaacaablmaleklaebfacmiabiaabaaloloaa
paakabaamiaciaabaagdloaapaaaakaamiaeiaabaaloloaapaakafaamiabiaaf
aaloloaapaamabaamiaciaafaagdloaapaaaamaamiaeiaafaaloloaapaamafaa
miadiaaaaabjlabkiladbobokicpadafaegmaaiaiaacacbgmiahaaamaegmlole
klakbfaemiadaaaaaalblclaklaabfadaibpadakaelbaagmkaacadagaibpanac
aemgaaggkaacaeagaicbanaeaadoanmbgpakagagaiecanaeaadoanlbgpalagag
aiieanaeaadoanlmgpamagagaicbadabaakhkhmgkpananagbeacaaabaakhkhgm
kpanaoaaaeciahahaagmgmmgoaamaladbeaeaaabaakhkhlbkpanapaaaeciaiai
aamglbmgoaamalaabeapaaaaaapipilbobacacamaeipajacaapilbmgobacagal
miapaaaaaajejepiolakakaamiapaaacaajemgpiolakagacmiapiaacaaaablaa
kbajbnaamiapiaadaaaablaakbaibnaamiapiaaeaaaablaakbahbnaamiapaaac
aajegmaaolafagacmiapaaaaaaaaaapiolafafaageihababaalologboaaeabad
miahaaabaabllemnklabbaabmiapaaaeaapipigmilaaafppfibaaaaaaaaaaagm
ocaaaaiaficaaaaaaaaaaalbocaaaaiafieaaaaaaaaaaamgocaaaaiafiiaaaaa
aaaaaablocaaaaiamiapaaaaaapiaaaaobacaaaaemipaaadaapilbmgkcaappae
emecacaaaamgblgmobadaaaeemciacacaagmmgblobadacaeembbaaacaabllblb
obadacaemiaeaaaaaalbgmaaobadaaaakibhacaeaalmmaecibacaiajkiciacae
aamgblicmbaeadajkieoacafaabgpmmaibacagajbeahaaaaaabbmalbkbaaahaf
ambiafaaaamgmggmobaaadadbeahaaaaaabebamgoaafaaacamihacaaaamabalb
oaaaaeadmiahaaaaaamabaaaoaaaacaamiahiaagaalemaaaoaabaaaaaaaaaaaa
aaaaaaaaaaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "VERTEXLIGHT_ON" }
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
"sce_vp_rsx // 79 instructions using 11 registers
[Configuration]
8
0000004f41050b00
[Defaults]
1
448 2
000000003f800000
[Microcode]
1264
00019c6c005d200d8186c0836041fffc00041c6c00400e0c0106c0836041dffc
00029c6c005d300c0186c0836041dffc00031c6c009c220c013fc0c36041dffc
401f9c6c011c1808010400d740619f9c401f9c6c01d0300d8106c0c360403f80
401f9c6c01d0200d8106c0c360405f80401f9c6c01d0100d8106c0c360409f80
401f9c6c01d0000d8106c0c360411f8000001c6c01506e0c010600c360411ffc
00001c6c0150620c010600c360405ffc00009c6c01505e0c010600c360411ffc
00009c6c0150520c010600c360405ffc00011c6c01504e0c010600c360411ffc
00011c6c0150420c010600c360405ffc00009c6c01d0500d8106c0c360409ffc
00009c6c01d0400d8106c0c360403ffc00011c6c01d0600d8106c0c360409ffc
00051c6c01d0a00d8686c0c360405ffc00051c6c01d0900d8686c0c360409ffc
00051c6c01d0800d8686c0c360411ffc00021c6c0150400c0c8600c360411ffc
00001c6c0150600c0c8600c360409ffc00001c6c0150500c0c8600c360403ffc
00019c6c0190a00c0a86c0c360405ffc00019c6c0190900c0a86c0c360409ffc
00019c6c0190800c0a86c0c360411ffc00029c6c00dcf00d8186c0aaa121fffc
00031c6c00dd100d8186c0bfe0a1fffc00039c6c00dd000d8186c0aaa0a1fffc
00049c6c00800243011848436041dffc00041c6c010002308121886304a1dffc
00049c6c011c200c06bfc0e30041dffc401f9c6c0140020c01060a4360405fb0
401f9c6c01400e0c01060a4360411fb000019c6c0080007f8086c7436041fffc
00039c6c0080000d8e86c7436041fffc00009c6c0080007f80bfc04360409ffc
00021c6c0040007f8086c08360409ffc00021c6c0040002a8086c08360405ffc
00019c6c010000000886c64361a1fffc00031c6c0100000d8c86c64363a1fffc
00039c6c00800e0c10bfc0836041dffc401f9c6c0140020c0106094360405fa0
401f9c6c01400e0c1286008360411fa000001c6c0150608c128600c360403ffc
00009c6c0150508c128600c360403ffc00011c6c0150408c128600c360403ffc
00041c6c019c700c0886c0c360405ffc00041c6c019c800c0886c0c360409ffc
00041c6c019c900c0886c0c360411ffc00009c6c010000000880046aa0a09ffc
00021c6c0080000d089a04436041fffc401f9c6c0140000c0e860a4360409fb0
00019c6c0100002a8086c54361a1fffc00029c6c0100000d8a86c5436321fffc
401f9c6c0140000c1286074360409fa000031c6c01dc400d8886c0c360405ffc
00031c6c01dc500d8886c0c360409ffc00031c6c01dc600d8886c0c360411ffc
00021c6c00c0000c1086c0830321dffc00031c6c009c302a828600c36041dffc
00041c6c00c0000c0c86c0830221dffc00001c6c2150600c0e8600c002b0827c
00011c6c2150400c0e8600caa2a8827c00031c6c209ce00d8a86c0d542a5e27c
00031c6c00dc002a8186c0836321fffc00009c6c2150500c0e8600dfe2a2827c
401f9c6c109c200d84bfc0c00331e2a4401f9c6c109c200d82bfc0caa329e2a8
401f9c6c109c200d80bfc0d54325e2ac00001c6c1080000d8686c45fe323e2fc
00001c6c029c000d808000c36041fffc00001c6c0080000d8086c5436041fffc
00009c6c009cc02a808600c36041dffc00009c6c011cd000008600c300a1dffc
00001c6c011cb055008600c300a1dffc00001c6c011ca07f808600c30021dffc
401f9c6c00c0000c1086c0830021dfb5
"
}

SubProgram "d3d11 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "VERTEXLIGHT_ON" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 112 // 112 used size, 10 vars
Vector 96 [_MainTex_ST] 4
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
// 90 instructions, 6 temp regs, 0 temp arrays:
// ALU 49 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0
eefiecedbjofjcdbfcbfkhkambdpeinladbkibgmabaaaaaadiaoaaaaadaaaaaa
cmaaaaaapeaaaaaanmabaaaaejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaa
abaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaljaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfc
enebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheooaaaaaaaaiaaaaaa
aiaaaaaamiaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaneaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaabaaaaaaadamaaaaneaaaaaaabaaaaaaaaaaaaaa
adaaaaaaacaaaaaaahaiaaaaneaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaa
apaaaaaaneaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaneaaaaaa
aeaaaaaaaaaaaaaaadaaaaaaafaaaaaaapaaaaaaneaaaaaaafaaaaaaaaaaaaaa
adaaaaaaagaaaaaaahaiaaaaneaaaaaaagaaaaaaaaaaaaaaadaaaaaaahaaaaaa
ahaiaaaafdfgfpfaepfdejfeejepeoaafeeffiedepepfceeaaklklklfdeieefc
feamaaaaeaaaabaabfadaaaafjaaaaaeegiocaaaaaaaaaaaahaaaaaafjaaaaae
egiocaaaabaaaaaaafaaaaaafjaaaaaeegiocaaaacaaaaaabjaaaaaafjaaaaae
egiocaaaadaaaaaabfaaaaaafpaaaaadpcbabaaaaaaaaaaafpaaaaadpcbabaaa
abaaaaaafpaaaaadhcbabaaaacaaaaaafpaaaaaddcbabaaaadaaaaaaghaaaaae
pccabaaaaaaaaaaaabaaaaaagfaaaaaddccabaaaabaaaaaagfaaaaadhccabaaa
acaaaaaagfaaaaadpccabaaaadaaaaaagfaaaaadpccabaaaaeaaaaaagfaaaaad
pccabaaaafaaaaaagfaaaaadhccabaaaagaaaaaagfaaaaadhccabaaaahaaaaaa
giaaaaacagaaaaaadiaaaaaipcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaa
adaaaaaaabaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaaaaaaaaa
agbabaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaa
adaaaaaaacaaaaaakgbkbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpccabaaa
aaaaaaaaegiocaaaadaaaaaaadaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaa
dcaaaaaldccabaaaabaaaaaaegbabaaaadaaaaaaegiacaaaaaaaaaaaagaaaaaa
ogikcaaaaaaaaaaaagaaaaaadiaaaaajhcaabaaaaaaaaaaafgifcaaaabaaaaaa
aeaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaaaaaaaaaaegiccaaa
adaaaaaabaaaaaaaagiacaaaabaaaaaaaeaaaaaaegacbaaaaaaaaaaadcaaaaal
hcaabaaaaaaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaaabaaaaaaaeaaaaaa
egacbaaaaaaaaaaaaaaaaaaihcaabaaaaaaaaaaaegacbaaaaaaaaaaaegiccaaa
adaaaaaabdaaaaaadcaaaaalhcaabaaaaaaaaaaaegacbaaaaaaaaaaapgipcaaa
adaaaaaabeaaaaaaegbcbaiaebaaaaaaaaaaaaaabaaaaaahbccabaaaacaaaaaa
egbcbaaaabaaaaaaegacbaaaaaaaaaaabaaaaaaheccabaaaacaaaaaaegbcbaaa
acaaaaaaegacbaaaaaaaaaaadiaaaaahhcaabaaaabaaaaaajgbebaaaabaaaaaa
cgbjbaaaacaaaaaadcaaaaakhcaabaaaabaaaaaajgbebaaaacaaaaaacgbjbaaa
abaaaaaaegacbaiaebaaaaaaabaaaaaadiaaaaahhcaabaaaabaaaaaaegacbaaa
abaaaaaapgbpbaaaabaaaaaabaaaaaahcccabaaaacaaaaaaegacbaaaabaaaaaa
egacbaaaaaaaaaaadiaaaaajhcaabaaaacaaaaaafgafbaiaebaaaaaaaaaaaaaa
egiccaaaadaaaaaaanaaaaaadcaaaaallcaabaaaaaaaaaaaegiicaaaadaaaaaa
amaaaaaaagaabaiaebaaaaaaaaaaaaaaegaibaaaacaaaaaadcaaaaallcaabaaa
aaaaaaaaegiicaaaadaaaaaaaoaaaaaakgakbaiaebaaaaaaaaaaaaaaegambaaa
aaaaaaaadgaaaaaficaabaaaacaaaaaaakaabaaaaaaaaaaadgaaaaagbcaabaaa
adaaaaaaakiacaaaadaaaaaaamaaaaaadgaaaaagccaabaaaadaaaaaaakiacaaa
adaaaaaaanaaaaaadgaaaaagecaabaaaadaaaaaaakiacaaaadaaaaaaaoaaaaaa
baaaaaahccaabaaaacaaaaaaegacbaaaabaaaaaaegacbaaaadaaaaaabaaaaaah
bcaabaaaacaaaaaaegbcbaaaabaaaaaaegacbaaaadaaaaaabaaaaaahecaabaaa
acaaaaaaegbcbaaaacaaaaaaegacbaaaadaaaaaadiaaaaaipccabaaaadaaaaaa
egaobaaaacaaaaaapgipcaaaadaaaaaabeaaaaaadgaaaaaficaabaaaacaaaaaa
bkaabaaaaaaaaaaadgaaaaagbcaabaaaadaaaaaabkiacaaaadaaaaaaamaaaaaa
dgaaaaagccaabaaaadaaaaaabkiacaaaadaaaaaaanaaaaaadgaaaaagecaabaaa
adaaaaaabkiacaaaadaaaaaaaoaaaaaabaaaaaahccaabaaaacaaaaaaegacbaaa
abaaaaaaegacbaaaadaaaaaabaaaaaahbcaabaaaacaaaaaaegbcbaaaabaaaaaa
egacbaaaadaaaaaabaaaaaahecaabaaaacaaaaaaegbcbaaaacaaaaaaegacbaaa
adaaaaaadiaaaaaipccabaaaaeaaaaaaegaobaaaacaaaaaapgipcaaaadaaaaaa
beaaaaaadgaaaaagbcaabaaaacaaaaaackiacaaaadaaaaaaamaaaaaadgaaaaag
ccaabaaaacaaaaaackiacaaaadaaaaaaanaaaaaadgaaaaagecaabaaaacaaaaaa
ckiacaaaadaaaaaaaoaaaaaabaaaaaahccaabaaaaaaaaaaaegacbaaaabaaaaaa
egacbaaaacaaaaaabaaaaaahbcaabaaaaaaaaaaaegbcbaaaabaaaaaaegacbaaa
acaaaaaabaaaaaahecaabaaaaaaaaaaaegbcbaaaacaaaaaaegacbaaaacaaaaaa
diaaaaaipccabaaaafaaaaaaegaobaaaaaaaaaaapgipcaaaadaaaaaabeaaaaaa
diaaaaajhcaabaaaaaaaaaaafgifcaaaacaaaaaaaaaaaaaaegiccaaaadaaaaaa
bbaaaaaadcaaaaalhcaabaaaaaaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaa
acaaaaaaaaaaaaaaegacbaaaaaaaaaaadcaaaaalhcaabaaaaaaaaaaaegiccaaa
adaaaaaabcaaaaaakgikcaaaacaaaaaaaaaaaaaaegacbaaaaaaaaaaadcaaaaal
hcaabaaaaaaaaaaaegiccaaaadaaaaaabdaaaaaapgipcaaaacaaaaaaaaaaaaaa
egacbaaaaaaaaaaabaaaaaahcccabaaaagaaaaaaegacbaaaabaaaaaaegacbaaa
aaaaaaaabaaaaaahbccabaaaagaaaaaaegbcbaaaabaaaaaaegacbaaaaaaaaaaa
baaaaaaheccabaaaagaaaaaaegbcbaaaacaaaaaaegacbaaaaaaaaaaadgaaaaaf
icaabaaaaaaaaaaaabeaaaaaaaaaiadpdiaaaaaihcaabaaaabaaaaaaegbcbaaa
acaaaaaapgipcaaaadaaaaaabeaaaaaadiaaaaaihcaabaaaacaaaaaafgafbaaa
abaaaaaaegiccaaaadaaaaaaanaaaaaadcaaaaaklcaabaaaabaaaaaaegiicaaa
adaaaaaaamaaaaaaagaabaaaabaaaaaaegaibaaaacaaaaaadcaaaaakhcaabaaa
aaaaaaaaegiccaaaadaaaaaaaoaaaaaakgakbaaaabaaaaaaegadbaaaabaaaaaa
bbaaaaaibcaabaaaabaaaaaaegiocaaaacaaaaaabcaaaaaaegaobaaaaaaaaaaa
bbaaaaaiccaabaaaabaaaaaaegiocaaaacaaaaaabdaaaaaaegaobaaaaaaaaaaa
bbaaaaaiecaabaaaabaaaaaaegiocaaaacaaaaaabeaaaaaaegaobaaaaaaaaaaa
diaaaaahpcaabaaaacaaaaaajgacbaaaaaaaaaaaegakbaaaaaaaaaaabbaaaaai
bcaabaaaadaaaaaaegiocaaaacaaaaaabfaaaaaaegaobaaaacaaaaaabbaaaaai
ccaabaaaadaaaaaaegiocaaaacaaaaaabgaaaaaaegaobaaaacaaaaaabbaaaaai
ecaabaaaadaaaaaaegiocaaaacaaaaaabhaaaaaaegaobaaaacaaaaaaaaaaaaah
hcaabaaaabaaaaaaegacbaaaabaaaaaaegacbaaaadaaaaaadiaaaaahicaabaaa
aaaaaaaabkaabaaaaaaaaaaabkaabaaaaaaaaaaadcaaaaakicaabaaaaaaaaaaa
akaabaaaaaaaaaaaakaabaaaaaaaaaaadkaabaiaebaaaaaaaaaaaaaadcaaaaak
hcaabaaaabaaaaaaegiccaaaacaaaaaabiaaaaaapgapbaaaaaaaaaaaegacbaaa
abaaaaaadiaaaaaihcaabaaaacaaaaaafgbfbaaaaaaaaaaaegiccaaaadaaaaaa
anaaaaaadcaaaaakhcaabaaaacaaaaaaegiccaaaadaaaaaaamaaaaaaagbabaaa
aaaaaaaaegacbaaaacaaaaaadcaaaaakhcaabaaaacaaaaaaegiccaaaadaaaaaa
aoaaaaaakgbkbaaaaaaaaaaaegacbaaaacaaaaaadcaaaaakhcaabaaaacaaaaaa
egiccaaaadaaaaaaapaaaaaapgbpbaaaaaaaaaaaegacbaaaacaaaaaaaaaaaaaj
pcaabaaaadaaaaaafgafbaiaebaaaaaaacaaaaaaegiocaaaacaaaaaaadaaaaaa
diaaaaahpcaabaaaaeaaaaaafgafbaaaaaaaaaaaegaobaaaadaaaaaadiaaaaah
pcaabaaaadaaaaaaegaobaaaadaaaaaaegaobaaaadaaaaaaaaaaaaajpcaabaaa
afaaaaaaagaabaiaebaaaaaaacaaaaaaegiocaaaacaaaaaaacaaaaaaaaaaaaaj
pcaabaaaacaaaaaakgakbaiaebaaaaaaacaaaaaaegiocaaaacaaaaaaaeaaaaaa
dcaaaaajpcaabaaaaeaaaaaaegaobaaaafaaaaaaagaabaaaaaaaaaaaegaobaaa
aeaaaaaadcaaaaajpcaabaaaaaaaaaaaegaobaaaacaaaaaakgakbaaaaaaaaaaa
egaobaaaaeaaaaaadcaaaaajpcaabaaaadaaaaaaegaobaaaafaaaaaaegaobaaa
afaaaaaaegaobaaaadaaaaaadcaaaaajpcaabaaaacaaaaaaegaobaaaacaaaaaa
egaobaaaacaaaaaaegaobaaaadaaaaaaeeaaaaafpcaabaaaadaaaaaaegaobaaa
acaaaaaadcaaaaanpcaabaaaacaaaaaaegaobaaaacaaaaaaegiocaaaacaaaaaa
afaaaaaaaceaaaaaaaaaiadpaaaaiadpaaaaiadpaaaaiadpaoaaaaakpcaabaaa
acaaaaaaaceaaaaaaaaaiadpaaaaiadpaaaaiadpaaaaiadpegaobaaaacaaaaaa
diaaaaahpcaabaaaaaaaaaaaegaobaaaaaaaaaaaegaobaaaadaaaaaadeaaaaak
pcaabaaaaaaaaaaaegaobaaaaaaaaaaaaceaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaadiaaaaahpcaabaaaaaaaaaaaegaobaaaacaaaaaaegaobaaaaaaaaaaa
diaaaaaihcaabaaaacaaaaaafgafbaaaaaaaaaaaegiccaaaacaaaaaaahaaaaaa
dcaaaaakhcaabaaaacaaaaaaegiccaaaacaaaaaaagaaaaaaagaabaaaaaaaaaaa
egacbaaaacaaaaaadcaaaaakhcaabaaaaaaaaaaaegiccaaaacaaaaaaaiaaaaaa
kgakbaaaaaaaaaaaegacbaaaacaaaaaadcaaaaakhcaabaaaaaaaaaaaegiccaaa
acaaaaaaajaaaaaapgapbaaaaaaaaaaaegacbaaaaaaaaaaaaaaaaaahhccabaaa
ahaaaaaaegacbaaaaaaaaaaaegacbaaaabaaaaaadoaaaaab"
}

SubProgram "gles " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "VERTEXLIGHT_ON" }
"!!GLES
#define SHADER_API_GLES 1
#define tex2D texture2D


#ifdef VERTEX
#define gl_ModelViewProjectionMatrix glstate_matrix_mvp
uniform mat4 glstate_matrix_mvp;

varying lowp vec3 xlv_TEXCOORD6;
varying lowp vec3 xlv_TEXCOORD5;
varying lowp vec4 xlv_TEXCOORD4;
varying lowp vec4 xlv_TEXCOORD3;
varying lowp vec4 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
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
  lowp vec4 tmpvar_4;
  lowp vec4 tmpvar_5;
  lowp vec4 tmpvar_6;
  lowp vec3 tmpvar_7;
  lowp vec3 tmpvar_8;
  highp vec4 tmpvar_9;
  tmpvar_9.w = 1.0;
  tmpvar_9.xyz = _WorldSpaceCameraPos;
  mat3 tmpvar_10;
  tmpvar_10[0] = _Object2World[0].xyz;
  tmpvar_10[1] = _Object2World[1].xyz;
  tmpvar_10[2] = _Object2World[2].xyz;
  highp vec3 tmpvar_11;
  tmpvar_11 = (tmpvar_10 * (_glesVertex.xyz - ((_World2Object * tmpvar_9).xyz * unity_Scale.w)));
  highp vec3 tmpvar_12;
  highp vec3 tmpvar_13;
  tmpvar_12 = tmpvar_1.xyz;
  tmpvar_13 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_14;
  tmpvar_14[0].x = tmpvar_12.x;
  tmpvar_14[0].y = tmpvar_13.x;
  tmpvar_14[0].z = tmpvar_2.x;
  tmpvar_14[1].x = tmpvar_12.y;
  tmpvar_14[1].y = tmpvar_13.y;
  tmpvar_14[1].z = tmpvar_2.y;
  tmpvar_14[2].x = tmpvar_12.z;
  tmpvar_14[2].y = tmpvar_13.z;
  tmpvar_14[2].z = tmpvar_2.z;
  vec4 v_15;
  v_15.x = _Object2World[0].x;
  v_15.y = _Object2World[1].x;
  v_15.z = _Object2World[2].x;
  v_15.w = _Object2World[3].x;
  highp vec4 tmpvar_16;
  tmpvar_16.xyz = (tmpvar_14 * v_15.xyz);
  tmpvar_16.w = tmpvar_11.x;
  highp vec4 tmpvar_17;
  tmpvar_17 = (tmpvar_16 * unity_Scale.w);
  tmpvar_4 = tmpvar_17;
  vec4 v_18;
  v_18.x = _Object2World[0].y;
  v_18.y = _Object2World[1].y;
  v_18.z = _Object2World[2].y;
  v_18.w = _Object2World[3].y;
  highp vec4 tmpvar_19;
  tmpvar_19.xyz = (tmpvar_14 * v_18.xyz);
  tmpvar_19.w = tmpvar_11.y;
  highp vec4 tmpvar_20;
  tmpvar_20 = (tmpvar_19 * unity_Scale.w);
  tmpvar_5 = tmpvar_20;
  vec4 v_21;
  v_21.x = _Object2World[0].z;
  v_21.y = _Object2World[1].z;
  v_21.z = _Object2World[2].z;
  v_21.w = _Object2World[3].z;
  highp vec4 tmpvar_22;
  tmpvar_22.xyz = (tmpvar_14 * v_21.xyz);
  tmpvar_22.w = tmpvar_11.z;
  highp vec4 tmpvar_23;
  tmpvar_23 = (tmpvar_22 * unity_Scale.w);
  tmpvar_6 = tmpvar_23;
  mat3 tmpvar_24;
  tmpvar_24[0] = _Object2World[0].xyz;
  tmpvar_24[1] = _Object2World[1].xyz;
  tmpvar_24[2] = _Object2World[2].xyz;
  highp vec3 tmpvar_25;
  tmpvar_25 = (tmpvar_24 * (tmpvar_2 * unity_Scale.w));
  highp vec3 tmpvar_26;
  tmpvar_26 = (tmpvar_14 * (_World2Object * _WorldSpaceLightPos0).xyz);
  tmpvar_7 = tmpvar_26;
  highp vec4 tmpvar_27;
  tmpvar_27.w = 1.0;
  tmpvar_27.xyz = _WorldSpaceCameraPos;
  highp vec4 tmpvar_28;
  tmpvar_28.w = 1.0;
  tmpvar_28.xyz = tmpvar_25;
  mediump vec3 tmpvar_29;
  mediump vec4 normal_30;
  normal_30 = tmpvar_28;
  highp float vC_31;
  mediump vec3 x3_32;
  mediump vec3 x2_33;
  mediump vec3 x1_34;
  highp float tmpvar_35;
  tmpvar_35 = dot (unity_SHAr, normal_30);
  x1_34.x = tmpvar_35;
  highp float tmpvar_36;
  tmpvar_36 = dot (unity_SHAg, normal_30);
  x1_34.y = tmpvar_36;
  highp float tmpvar_37;
  tmpvar_37 = dot (unity_SHAb, normal_30);
  x1_34.z = tmpvar_37;
  mediump vec4 tmpvar_38;
  tmpvar_38 = (normal_30.xyzz * normal_30.yzzx);
  highp float tmpvar_39;
  tmpvar_39 = dot (unity_SHBr, tmpvar_38);
  x2_33.x = tmpvar_39;
  highp float tmpvar_40;
  tmpvar_40 = dot (unity_SHBg, tmpvar_38);
  x2_33.y = tmpvar_40;
  highp float tmpvar_41;
  tmpvar_41 = dot (unity_SHBb, tmpvar_38);
  x2_33.z = tmpvar_41;
  mediump float tmpvar_42;
  tmpvar_42 = ((normal_30.x * normal_30.x) - (normal_30.y * normal_30.y));
  vC_31 = tmpvar_42;
  highp vec3 tmpvar_43;
  tmpvar_43 = (unity_SHC.xyz * vC_31);
  x3_32 = tmpvar_43;
  tmpvar_29 = ((x1_34 + x2_33) + x3_32);
  shlight_3 = tmpvar_29;
  tmpvar_8 = shlight_3;
  highp vec3 tmpvar_44;
  tmpvar_44 = (_Object2World * _glesVertex).xyz;
  highp vec4 tmpvar_45;
  tmpvar_45 = (unity_4LightPosX0 - tmpvar_44.x);
  highp vec4 tmpvar_46;
  tmpvar_46 = (unity_4LightPosY0 - tmpvar_44.y);
  highp vec4 tmpvar_47;
  tmpvar_47 = (unity_4LightPosZ0 - tmpvar_44.z);
  highp vec4 tmpvar_48;
  tmpvar_48 = (((tmpvar_45 * tmpvar_45) + (tmpvar_46 * tmpvar_46)) + (tmpvar_47 * tmpvar_47));
  highp vec4 tmpvar_49;
  tmpvar_49 = (max (vec4(0.0, 0.0, 0.0, 0.0), ((((tmpvar_45 * tmpvar_25.x) + (tmpvar_46 * tmpvar_25.y)) + (tmpvar_47 * tmpvar_25.z)) * inversesqrt(tmpvar_48))) * (1.0/((1.0 + (tmpvar_48 * unity_4LightAtten0)))));
  highp vec3 tmpvar_50;
  tmpvar_50 = (tmpvar_8 + ((((unity_LightColor[0].xyz * tmpvar_49.x) + (unity_LightColor[1].xyz * tmpvar_49.y)) + (unity_LightColor[2].xyz * tmpvar_49.z)) + (unity_LightColor[3].xyz * tmpvar_49.w)));
  tmpvar_8 = tmpvar_50;
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = (tmpvar_14 * (((_World2Object * tmpvar_27).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = tmpvar_4;
  xlv_TEXCOORD3 = tmpvar_5;
  xlv_TEXCOORD4 = tmpvar_6;
  xlv_TEXCOORD5 = tmpvar_7;
  xlv_TEXCOORD6 = tmpvar_8;
}



#endif
#ifdef FRAGMENT

varying lowp vec3 xlv_TEXCOORD6;
varying lowp vec3 xlv_TEXCOORD5;
varying lowp vec4 xlv_TEXCOORD4;
varying lowp vec4 xlv_TEXCOORD3;
varying lowp vec4 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp float _FresnelPower;
uniform mediump float _Gloss;
uniform mediump float _Shininess;
uniform mediump float _ReflectPower;
uniform lowp vec4 _ReflectColor;
uniform lowp vec4 _Color;
uniform samplerCube _Cube;
uniform sampler2D _SpecMap;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform lowp vec4 _LightColor0;
void main ()
{
  lowp vec4 c_1;
  highp vec3 tmpvar_2;
  mediump vec3 tmpvar_3;
  mediump vec3 tmpvar_4;
  mediump vec3 tmpvar_5;
  lowp vec3 tmpvar_6;
  tmpvar_6.x = xlv_TEXCOORD2.w;
  tmpvar_6.y = xlv_TEXCOORD3.w;
  tmpvar_6.z = xlv_TEXCOORD4.w;
  tmpvar_2 = tmpvar_6;
  lowp vec3 tmpvar_7;
  tmpvar_7 = xlv_TEXCOORD2.xyz;
  tmpvar_3 = tmpvar_7;
  lowp vec3 tmpvar_8;
  tmpvar_8 = xlv_TEXCOORD3.xyz;
  tmpvar_4 = tmpvar_8;
  lowp vec3 tmpvar_9;
  tmpvar_9 = xlv_TEXCOORD4.xyz;
  tmpvar_5 = tmpvar_9;
  mediump vec3 tmpvar_10;
  mediump vec3 tmpvar_11;
  mediump float tmpvar_12;
  lowp vec4 tmpvar_13;
  tmpvar_13 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_14;
  tmpvar_14 = (tmpvar_13.xyz * _Color.xyz);
  tmpvar_10 = tmpvar_14;
  lowp vec4 tmpvar_15;
  tmpvar_15 = texture2D (_SpecMap, xlv_TEXCOORD0);
  mediump vec3 tmpvar_16;
  tmpvar_16 = ((tmpvar_13.w * tmpvar_15.xyz) * _Gloss);
  lowp vec3 tmpvar_17;
  tmpvar_17 = ((texture2D (_BumpMap, xlv_TEXCOORD0).xyz * 2.0) - 1.0);
  tmpvar_11 = tmpvar_17;
  mediump vec3 tmpvar_18;
  tmpvar_18.x = dot (tmpvar_3, tmpvar_11);
  tmpvar_18.y = dot (tmpvar_4, tmpvar_11);
  tmpvar_18.z = dot (tmpvar_5, tmpvar_11);
  highp vec3 tmpvar_19;
  tmpvar_19 = (tmpvar_2 - (2.0 * (dot (tmpvar_18, tmpvar_2) * tmpvar_18)));
  mediump vec3 tmpvar_20;
  tmpvar_20 = normalize(tmpvar_11);
  highp float tmpvar_21;
  tmpvar_21 = max ((0.20373 + (0.79627 * pow (clamp ((1.0 - max (dot (normalize(xlv_TEXCOORD1), tmpvar_20), 0.0)), 0.0, 1.0), _FresnelPower))), 0.0);
  lowp vec4 tmpvar_22;
  tmpvar_22 = (textureCube (_Cube, tmpvar_19) * tmpvar_13.w);
  tmpvar_12 = tmpvar_21;
  highp vec3 tmpvar_23;
  tmpvar_23 = normalize(xlv_TEXCOORD1);
  mediump vec3 lightDir_24;
  lightDir_24 = xlv_TEXCOORD5;
  mediump vec3 viewDir_25;
  viewDir_25 = tmpvar_23;
  mediump vec4 c_26;
  mediump vec3 specCol_27;
  highp float nh_28;
  mediump float tmpvar_29;
  tmpvar_29 = max (0.0, dot (tmpvar_11, normalize((lightDir_24 + viewDir_25))));
  nh_28 = tmpvar_29;
  mediump float arg1_30;
  arg1_30 = (32.0 * _Shininess);
  highp vec3 tmpvar_31;
  tmpvar_31 = (pow (nh_28, arg1_30) * tmpvar_16);
  specCol_27 = tmpvar_31;
  c_26.xyz = ((((tmpvar_10 * _LightColor0.xyz) * max (0.0, dot (tmpvar_11, lightDir_24))) + (_LightColor0.xyz * specCol_27)) * 2.0);
  c_26.w = tmpvar_12;
  c_1 = c_26;
  mediump vec3 tmpvar_32;
  tmpvar_32 = (c_1.xyz + (tmpvar_10 * xlv_TEXCOORD6));
  c_1.xyz = tmpvar_32;
  mediump vec3 tmpvar_33;
  tmpvar_33 = (c_1.xyz + ((tmpvar_22.xyz * _ReflectColor.xyz) * _ReflectPower));
  c_1.xyz = tmpvar_33;
  c_1.w = tmpvar_12;
  gl_FragData[0] = c_1;
}



#endif"
}

SubProgram "glesdesktop " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "VERTEXLIGHT_ON" }
"!!GLES
#define SHADER_API_GLES 1
#define tex2D texture2D


#ifdef VERTEX
#define gl_ModelViewProjectionMatrix glstate_matrix_mvp
uniform mat4 glstate_matrix_mvp;

varying lowp vec3 xlv_TEXCOORD6;
varying lowp vec3 xlv_TEXCOORD5;
varying lowp vec4 xlv_TEXCOORD4;
varying lowp vec4 xlv_TEXCOORD3;
varying lowp vec4 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
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
  lowp vec4 tmpvar_4;
  lowp vec4 tmpvar_5;
  lowp vec4 tmpvar_6;
  lowp vec3 tmpvar_7;
  lowp vec3 tmpvar_8;
  highp vec4 tmpvar_9;
  tmpvar_9.w = 1.0;
  tmpvar_9.xyz = _WorldSpaceCameraPos;
  mat3 tmpvar_10;
  tmpvar_10[0] = _Object2World[0].xyz;
  tmpvar_10[1] = _Object2World[1].xyz;
  tmpvar_10[2] = _Object2World[2].xyz;
  highp vec3 tmpvar_11;
  tmpvar_11 = (tmpvar_10 * (_glesVertex.xyz - ((_World2Object * tmpvar_9).xyz * unity_Scale.w)));
  highp vec3 tmpvar_12;
  highp vec3 tmpvar_13;
  tmpvar_12 = tmpvar_1.xyz;
  tmpvar_13 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_14;
  tmpvar_14[0].x = tmpvar_12.x;
  tmpvar_14[0].y = tmpvar_13.x;
  tmpvar_14[0].z = tmpvar_2.x;
  tmpvar_14[1].x = tmpvar_12.y;
  tmpvar_14[1].y = tmpvar_13.y;
  tmpvar_14[1].z = tmpvar_2.y;
  tmpvar_14[2].x = tmpvar_12.z;
  tmpvar_14[2].y = tmpvar_13.z;
  tmpvar_14[2].z = tmpvar_2.z;
  vec4 v_15;
  v_15.x = _Object2World[0].x;
  v_15.y = _Object2World[1].x;
  v_15.z = _Object2World[2].x;
  v_15.w = _Object2World[3].x;
  highp vec4 tmpvar_16;
  tmpvar_16.xyz = (tmpvar_14 * v_15.xyz);
  tmpvar_16.w = tmpvar_11.x;
  highp vec4 tmpvar_17;
  tmpvar_17 = (tmpvar_16 * unity_Scale.w);
  tmpvar_4 = tmpvar_17;
  vec4 v_18;
  v_18.x = _Object2World[0].y;
  v_18.y = _Object2World[1].y;
  v_18.z = _Object2World[2].y;
  v_18.w = _Object2World[3].y;
  highp vec4 tmpvar_19;
  tmpvar_19.xyz = (tmpvar_14 * v_18.xyz);
  tmpvar_19.w = tmpvar_11.y;
  highp vec4 tmpvar_20;
  tmpvar_20 = (tmpvar_19 * unity_Scale.w);
  tmpvar_5 = tmpvar_20;
  vec4 v_21;
  v_21.x = _Object2World[0].z;
  v_21.y = _Object2World[1].z;
  v_21.z = _Object2World[2].z;
  v_21.w = _Object2World[3].z;
  highp vec4 tmpvar_22;
  tmpvar_22.xyz = (tmpvar_14 * v_21.xyz);
  tmpvar_22.w = tmpvar_11.z;
  highp vec4 tmpvar_23;
  tmpvar_23 = (tmpvar_22 * unity_Scale.w);
  tmpvar_6 = tmpvar_23;
  mat3 tmpvar_24;
  tmpvar_24[0] = _Object2World[0].xyz;
  tmpvar_24[1] = _Object2World[1].xyz;
  tmpvar_24[2] = _Object2World[2].xyz;
  highp vec3 tmpvar_25;
  tmpvar_25 = (tmpvar_24 * (tmpvar_2 * unity_Scale.w));
  highp vec3 tmpvar_26;
  tmpvar_26 = (tmpvar_14 * (_World2Object * _WorldSpaceLightPos0).xyz);
  tmpvar_7 = tmpvar_26;
  highp vec4 tmpvar_27;
  tmpvar_27.w = 1.0;
  tmpvar_27.xyz = _WorldSpaceCameraPos;
  highp vec4 tmpvar_28;
  tmpvar_28.w = 1.0;
  tmpvar_28.xyz = tmpvar_25;
  mediump vec3 tmpvar_29;
  mediump vec4 normal_30;
  normal_30 = tmpvar_28;
  highp float vC_31;
  mediump vec3 x3_32;
  mediump vec3 x2_33;
  mediump vec3 x1_34;
  highp float tmpvar_35;
  tmpvar_35 = dot (unity_SHAr, normal_30);
  x1_34.x = tmpvar_35;
  highp float tmpvar_36;
  tmpvar_36 = dot (unity_SHAg, normal_30);
  x1_34.y = tmpvar_36;
  highp float tmpvar_37;
  tmpvar_37 = dot (unity_SHAb, normal_30);
  x1_34.z = tmpvar_37;
  mediump vec4 tmpvar_38;
  tmpvar_38 = (normal_30.xyzz * normal_30.yzzx);
  highp float tmpvar_39;
  tmpvar_39 = dot (unity_SHBr, tmpvar_38);
  x2_33.x = tmpvar_39;
  highp float tmpvar_40;
  tmpvar_40 = dot (unity_SHBg, tmpvar_38);
  x2_33.y = tmpvar_40;
  highp float tmpvar_41;
  tmpvar_41 = dot (unity_SHBb, tmpvar_38);
  x2_33.z = tmpvar_41;
  mediump float tmpvar_42;
  tmpvar_42 = ((normal_30.x * normal_30.x) - (normal_30.y * normal_30.y));
  vC_31 = tmpvar_42;
  highp vec3 tmpvar_43;
  tmpvar_43 = (unity_SHC.xyz * vC_31);
  x3_32 = tmpvar_43;
  tmpvar_29 = ((x1_34 + x2_33) + x3_32);
  shlight_3 = tmpvar_29;
  tmpvar_8 = shlight_3;
  highp vec3 tmpvar_44;
  tmpvar_44 = (_Object2World * _glesVertex).xyz;
  highp vec4 tmpvar_45;
  tmpvar_45 = (unity_4LightPosX0 - tmpvar_44.x);
  highp vec4 tmpvar_46;
  tmpvar_46 = (unity_4LightPosY0 - tmpvar_44.y);
  highp vec4 tmpvar_47;
  tmpvar_47 = (unity_4LightPosZ0 - tmpvar_44.z);
  highp vec4 tmpvar_48;
  tmpvar_48 = (((tmpvar_45 * tmpvar_45) + (tmpvar_46 * tmpvar_46)) + (tmpvar_47 * tmpvar_47));
  highp vec4 tmpvar_49;
  tmpvar_49 = (max (vec4(0.0, 0.0, 0.0, 0.0), ((((tmpvar_45 * tmpvar_25.x) + (tmpvar_46 * tmpvar_25.y)) + (tmpvar_47 * tmpvar_25.z)) * inversesqrt(tmpvar_48))) * (1.0/((1.0 + (tmpvar_48 * unity_4LightAtten0)))));
  highp vec3 tmpvar_50;
  tmpvar_50 = (tmpvar_8 + ((((unity_LightColor[0].xyz * tmpvar_49.x) + (unity_LightColor[1].xyz * tmpvar_49.y)) + (unity_LightColor[2].xyz * tmpvar_49.z)) + (unity_LightColor[3].xyz * tmpvar_49.w)));
  tmpvar_8 = tmpvar_50;
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = (tmpvar_14 * (((_World2Object * tmpvar_27).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = tmpvar_4;
  xlv_TEXCOORD3 = tmpvar_5;
  xlv_TEXCOORD4 = tmpvar_6;
  xlv_TEXCOORD5 = tmpvar_7;
  xlv_TEXCOORD6 = tmpvar_8;
}



#endif
#ifdef FRAGMENT

varying lowp vec3 xlv_TEXCOORD6;
varying lowp vec3 xlv_TEXCOORD5;
varying lowp vec4 xlv_TEXCOORD4;
varying lowp vec4 xlv_TEXCOORD3;
varying lowp vec4 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp float _FresnelPower;
uniform mediump float _Gloss;
uniform mediump float _Shininess;
uniform mediump float _ReflectPower;
uniform lowp vec4 _ReflectColor;
uniform lowp vec4 _Color;
uniform samplerCube _Cube;
uniform sampler2D _SpecMap;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform lowp vec4 _LightColor0;
void main ()
{
  lowp vec4 c_1;
  highp vec3 tmpvar_2;
  mediump vec3 tmpvar_3;
  mediump vec3 tmpvar_4;
  mediump vec3 tmpvar_5;
  lowp vec3 tmpvar_6;
  tmpvar_6.x = xlv_TEXCOORD2.w;
  tmpvar_6.y = xlv_TEXCOORD3.w;
  tmpvar_6.z = xlv_TEXCOORD4.w;
  tmpvar_2 = tmpvar_6;
  lowp vec3 tmpvar_7;
  tmpvar_7 = xlv_TEXCOORD2.xyz;
  tmpvar_3 = tmpvar_7;
  lowp vec3 tmpvar_8;
  tmpvar_8 = xlv_TEXCOORD3.xyz;
  tmpvar_4 = tmpvar_8;
  lowp vec3 tmpvar_9;
  tmpvar_9 = xlv_TEXCOORD4.xyz;
  tmpvar_5 = tmpvar_9;
  mediump vec3 tmpvar_10;
  mediump vec3 tmpvar_11;
  mediump float tmpvar_12;
  lowp vec4 tmpvar_13;
  tmpvar_13 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_14;
  tmpvar_14 = (tmpvar_13.xyz * _Color.xyz);
  tmpvar_10 = tmpvar_14;
  lowp vec4 tmpvar_15;
  tmpvar_15 = texture2D (_SpecMap, xlv_TEXCOORD0);
  mediump vec3 tmpvar_16;
  tmpvar_16 = ((tmpvar_13.w * tmpvar_15.xyz) * _Gloss);
  lowp vec3 normal_17;
  normal_17.xy = ((texture2D (_BumpMap, xlv_TEXCOORD0).wy * 2.0) - 1.0);
  normal_17.z = sqrt(((1.0 - (normal_17.x * normal_17.x)) - (normal_17.y * normal_17.y)));
  tmpvar_11 = normal_17;
  mediump vec3 tmpvar_18;
  tmpvar_18.x = dot (tmpvar_3, tmpvar_11);
  tmpvar_18.y = dot (tmpvar_4, tmpvar_11);
  tmpvar_18.z = dot (tmpvar_5, tmpvar_11);
  highp vec3 tmpvar_19;
  tmpvar_19 = (tmpvar_2 - (2.0 * (dot (tmpvar_18, tmpvar_2) * tmpvar_18)));
  mediump vec3 tmpvar_20;
  tmpvar_20 = normalize(tmpvar_11);
  highp float tmpvar_21;
  tmpvar_21 = max ((0.20373 + (0.79627 * pow (clamp ((1.0 - max (dot (normalize(xlv_TEXCOORD1), tmpvar_20), 0.0)), 0.0, 1.0), _FresnelPower))), 0.0);
  lowp vec4 tmpvar_22;
  tmpvar_22 = (textureCube (_Cube, tmpvar_19) * tmpvar_13.w);
  tmpvar_12 = tmpvar_21;
  highp vec3 tmpvar_23;
  tmpvar_23 = normalize(xlv_TEXCOORD1);
  mediump vec3 lightDir_24;
  lightDir_24 = xlv_TEXCOORD5;
  mediump vec3 viewDir_25;
  viewDir_25 = tmpvar_23;
  mediump vec4 c_26;
  mediump vec3 specCol_27;
  highp float nh_28;
  mediump float tmpvar_29;
  tmpvar_29 = max (0.0, dot (tmpvar_11, normalize((lightDir_24 + viewDir_25))));
  nh_28 = tmpvar_29;
  mediump float arg1_30;
  arg1_30 = (32.0 * _Shininess);
  highp vec3 tmpvar_31;
  tmpvar_31 = (pow (nh_28, arg1_30) * tmpvar_16);
  specCol_27 = tmpvar_31;
  c_26.xyz = ((((tmpvar_10 * _LightColor0.xyz) * max (0.0, dot (tmpvar_11, lightDir_24))) + (_LightColor0.xyz * specCol_27)) * 2.0);
  c_26.w = tmpvar_12;
  c_1 = c_26;
  mediump vec3 tmpvar_32;
  tmpvar_32 = (c_1.xyz + (tmpvar_10 * xlv_TEXCOORD6));
  c_1.xyz = tmpvar_32;
  mediump vec3 tmpvar_33;
  tmpvar_33 = (c_1.xyz + ((tmpvar_22.xyz * _ReflectColor.xyz) * _ReflectPower));
  c_1.xyz = tmpvar_33;
  c_1.w = tmpvar_12;
  gl_FragData[0] = c_1;
}



#endif"
}

}
Program "fp" {
// Fragment combos: 2
//   opengl - ALU: 39 to 58, TEX: 4 to 4
//   d3d9 - ALU: 36 to 58, TEX: 4 to 4
//   d3d11 - ALU: 25 to 39, TEX: 4 to 4, FLOW: 1 to 1
SubProgram "opengl " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Vector 2 [_ReflectColor]
Float 3 [_ReflectPower]
Float 4 [_Shininess]
Float 5 [_Gloss]
Float 6 [_FresnelPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_Cube] CUBE
"3.0-!!ARBfp1.0
OPTION ARB_precision_hint_fastest;
# 58 ALU, 4 TEX
PARAM c[9] = { program.local[0..6],
		{ 2, 1, 0, 0.79627001 },
		{ 0.20373, 32 } };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
TEMP R4;
TEX R0.yw, fragment.texcoord[0], texture[2], 2D;
MAD R0.xy, R0.wyzw, c[7].x, -c[7].y;
MUL R0.z, R0.y, R0.y;
MAD R0.z, -R0.x, R0.x, -R0;
DP3 R0.w, fragment.texcoord[1], fragment.texcoord[1];
ADD R0.z, R0, c[7].y;
RSQ R0.z, R0.z;
RCP R0.z, R0.z;
RSQ R0.w, R0.w;
MOV R1.xyz, fragment.texcoord[5];
MAD R1.xyz, fragment.texcoord[1], R0.w, R1;
DP3 R0.w, R1, R1;
RSQ R0.w, R0.w;
MUL R1.xyz, R0.w, R1;
DP3 R0.w, R0, R1;
MAX R2.w, R0, c[7].z;
TEX R1, fragment.texcoord[0], texture[0], 2D;
MUL R1.xyz, R1, c[1];
MUL R3.xyz, R1, fragment.texcoord[6];
TEX R2.xyz, fragment.texcoord[0], texture[1], 2D;
MOV R0.w, c[8].y;
MUL R2.xyz, R1.w, R2;
MUL R0.w, R0, c[4].x;
POW R0.w, R2.w, R0.w;
MUL R2.xyz, R2, c[5].x;
MUL R2.xyz, R0.w, R2;
DP3 R0.w, R0, fragment.texcoord[5];
DP3 R2.w, fragment.texcoord[1], fragment.texcoord[1];
MUL R2.xyz, R2, c[0];
MAX R0.w, R0, c[7].z;
MUL R1.xyz, R1, c[0];
MAD R1.xyz, R1, R0.w, R2;
MAD R4.xyz, R1, c[7].x, R3;
DP3 R3.x, R0, R0;
RSQ R3.x, R3.x;
DP3 R2.x, R0, fragment.texcoord[2];
DP3 R2.y, R0, fragment.texcoord[3];
DP3 R2.z, R0, fragment.texcoord[4];
MOV R1.x, fragment.texcoord[2].w;
MOV R1.z, fragment.texcoord[4].w;
MOV R1.y, fragment.texcoord[3].w;
DP3 R0.w, R2, R1;
RSQ R2.w, R2.w;
MUL R3.xyz, R3.x, R0;
MUL R0.xyz, R2.w, fragment.texcoord[1];
DP3 R2.w, R0, R3;
MUL R0.xyz, R2, R0.w;
MAD R0.xyz, -R0, c[7].x, R1;
TEX R0.xyz, R0, texture[3], CUBE;
MUL R1.xyz, R0, R1.w;
MAX R0.w, R2, c[7].z;
ADD_SAT R0.w, -R0, c[7].y;
POW R0.w, R0.w, c[6].x;
MUL R0.x, R0.w, c[7].w;
MUL R1.xyz, R1, c[2];
ADD R0.x, R0, c[8];
MAD result.color.xyz, R1, c[3].x, R4;
MAX result.color.w, R0.x, c[7].z;
END
# 58 instructions, 5 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Vector 2 [_ReflectColor]
Float 3 [_ReflectPower]
Float 4 [_Shininess]
Float 5 [_Gloss]
Float 6 [_FresnelPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_Cube] CUBE
"ps_3_0
; 58 ALU, 4 TEX
dcl_2d s0
dcl_2d s1
dcl_2d s2
dcl_cube s3
def c7, 2.00000000, -1.00000000, 1.00000000, 0.00000000
def c8, 0.79627001, 0.20373000, 32.00000000, 0
dcl_texcoord0 v0.xy
dcl_texcoord1 v1.xyz
dcl_texcoord2 v2
dcl_texcoord3 v3
dcl_texcoord4 v4
dcl_texcoord5 v5.xyz
dcl_texcoord6 v6.xyz
texld r0.yw, v0, s2
mad_pp r0.xy, r0.wyzw, c7.x, c7.y
mul_pp r0.z, r0.y, r0.y
mad_pp r0.z, -r0.x, r0.x, -r0
dp3_pp r0.w, v1, v1
add_pp r0.z, r0, c7
rsq_pp r0.z, r0.z
rcp_pp r0.z, r0.z
rsq_pp r0.w, r0.w
mov_pp r1.xyz, v5
mad_pp r1.xyz, v1, r0.w, r1
dp3_pp r0.w, r1, r1
rsq_pp r0.w, r0.w
mul_pp r1.xyz, r0.w, r1
dp3_pp r0.w, r0, r1
mov_pp r1.w, c4.x
mul_pp r1.x, c8.z, r1.w
max_pp r0.w, r0, c7
pow r3, r0.w, r1.x
texld r1, v0, s0
texld r2.xyz, v0, s1
mul_pp r2.xyz, r1.w, r2
mov r0.w, r3.x
mul_pp r1.xyz, r1, c1
mul_pp r3.xyz, r1, v6
mul_pp r2.xyz, r2, c5.x
mul r2.xyz, r0.w, r2
dp3_pp r0.w, r0, v5
mul_pp r2.xyz, r2, c0
max_pp r0.w, r0, c7
mul_pp r1.xyz, r1, c0
mad_pp r1.xyz, r1, r0.w, r2
mad_pp r3.xyz, r1, c7.x, r3
dp3_pp r1.x, r0, r0
rsq_pp r1.x, r1.x
dp3 r0.w, v1, v1
mul_pp r2.xyz, r1.x, r0
rsq r0.w, r0.w
mul r1.xyz, r0.w, v1
dp3 r0.w, r1, r2
dp3_pp r1.x, r0, v2
dp3_pp r1.y, r0, v3
dp3_pp r1.z, r0, v4
mov r0.x, v2.w
mov r0.z, v4.w
mov r0.y, v3.w
dp3 r2.x, r1, r0
mul r1.xyz, r1, r2.x
mad r1.xyz, -r1, c7.x, r0
max r0.w, r0, c7
add_sat r2.x, -r0.w, c7.z
pow r0, r2.x, c6.x
texld r1.xyz, r1, s3
mul_pp r1.xyz, r1, r1.w
mul_pp r1.xyz, r1, c2
mad r0.x, r0, c8, c8.y
mad_pp oC0.xyz, r1, c3.x, r3
max oC0.w, r0.x, c7
"
}

SubProgram "xbox360 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" }
Vector 1 [_Color]
Float 6 [_FresnelPower]
Float 5 [_Gloss]
Vector 0 [_LightColor0]
Vector 2 [_ReflectColor]
Float 3 [_ReflectPower]
Float 4 [_Shininess]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_BumpMap] 2D
SetTexture 2 [_SpecMap] 2D
SetTexture 3 [_Cube] CUBE
// Shader Timing Estimate, in Cycles/64 pixel vector:
// ALU: 58.67 (44 instructions), vertex: 0, texture: 16,
//   sequencer: 20, interpolator: 28;    12 GPRs, 15 threads,
// Performance (if enough threads): ~58 cycles per vector
// * Texture cycle estimates are assuming an 8bit/component texture with no
//     aniso or trilinear filtering.

"ps_360
backbbaaaaaaacfaaaaaacmiaaaaaaaaaaaaaaceaaaaabomaaaaacbeaaaaaaaa
aaaaaaaaaaaaabmeaaaaaabmaaaaablippppadaaaaaaaaalaaaaaabmaaaaaaaa
aaaaablbaaaaaapiaaadaaabaaabaaaaaaaaabaeaaaaaaaaaaaaabbeaaacaaab
aaabaaaaaaaaabbmaaaaaaaaaaaaabcmaaadaaadaaabaaaaaaaaabdeaaaaaaaa
aaaaabeeaaacaaagaaabaaaaaaaaabfeaaaaaaaaaaaaabgeaaacaaafaaabaaaa
aaaaabfeaaaaaaaaaaaaabglaaacaaaaaaabaaaaaaaaabbmaaaaaaaaaaaaabhi
aaadaaaaaaabaaaaaaaaabaeaaaaaaaaaaaaabibaaacaaacaaabaaaaaaaaabbm
aaaaaaaaaaaaabipaaacaaadaaabaaaaaaaaabfeaaaaaaaaaaaaabjnaaacaaae
aaabaaaaaaaaabfeaaaaaaaaaaaaabkiaaadaaacaaabaaaaaaaaabaeaaaaaaaa
fpechfgnhaengbhaaaklklklaaaeaaamaaabaaabaaabaaaaaaaaaaaafpedgpgm
gphcaaklaaabaaadaaabaaaeaaabaaaaaaaaaaaafpedhfgcgfaaklklaaaeaaao
aaabaaabaaabaaaaaaaaaaaafpeghcgfhdgogfgmfagphhgfhcaaklklaaaaaaad
aaabaaabaaabaaaaaaaaaaaafpehgmgphdhdaafpemgjghgiheedgpgmgphcdaaa
fpengbgjgofegfhiaafpfcgfgggmgfgdheedgpgmgphcaafpfcgfgggmgfgdhefa
gphhgfhcaafpfdgigjgogjgogfhdhdaafpfdhagfgdengbhaaahahdfpddfpdaaa
dccodacodcdadddfddcodaaaaaaaaaaaaaaaaaabaaaaaaaaaaaaaaaaaaaaaabe
abpmaabaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaeaaaaaaciibaaaalaa
aaaaaaaeaaaaaaaaaaaafmohaahpaahpaaaaaaabaaaadafaaaaahbfbaaaapcfc
aaaapdfdaaaapefeaaaahfffaaaahgfgaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaadpiaaaaaecaaaaaaeaaaaaaa
dpmaaaaadofajojjlpiaaaaadpelnifkaajfgaafgaalbcaabcaaaaaaaaaagabb
gabhbcaabcaaaaaaaaaagabngacdbcaabcaaaaaaaaaagacjbacpbcaabcaaaaab
aaaaaaaafadameaaccaaaaaabacajaabbpbppoiiaaaaeaaabaaahaabbpbppgii
aaaaeaaababaaaabbpbpppnjaaaaeaaamiaiaaaaaaloloaapaababaamiadaaal
aagnblmgilaapoppmiabaaaaaegngnlbnbalalpofiigaaaaaalmlmblobalalia
miahaaakaablmaaaobaaabaaaachaaaiaamamalgoaakafaakaicalaaaalbgmgm
oaaaaaiafiebalaaaalololbpaaiaiiamiacaaaaaalploaapaalaeaafibiaaai
aalbblgmobaaaeiamiahaaahaamamaaakbahabaamiahaaabaablmaaaobahajaa
miaoaaabaapmgmaakbabafaamiahaaajaaleleaakbahaaaamianaaaaaapagmaa
obaiaaaamiabaaafaalploaapaalafaamiacaaafaamplpaapaaaalaamiabaaaa
aalploaapaalacaamiaeaaaaaalploaapaaladaamiahaaaiaamabgaaobalalaa
miaeaaafaaloloaapaakaiaamiaiaaaaaamgblaaobaaadaamiaiaaafaagmblbl
olaaacaabeahaaaiaalogmgmicafpoaeamihagafaalelbmgmbajaipoebbiabaa
aegmlbmgkaaipoiieaibaaabaablgmblobagabaadibiabaaaablgmgmkbaaagab
diboababaanbgmblobababaamiahaaaiaamjmaaakbabaaaamiapaaafaadedeaa
oaafaiaamiapaaafaaipipaaoaafafaamiahaaaaaagmleaaobafaaaabeacaaaa
aflbblmgoaaaadaaaeebaaaaaegmblbloaaaacaemiapaaaaaakgmnaapcaaaaaa
emceabacaablblmgocaaaaiakmedaaaaaagnlbaambaaabppkmilaaacaamalmeh
kaaappacjadiaaebbpbppoiiaaaamaaabeadaaabaamfblblobaaahaaambgabab
aalmmbgmkbabacaamiahaaaaaalelemjolahagafmiahiaaaaamagmleklabadaa
miaiiaaaaablgmaakcacpoaaaaaaaaaaaaaaaaaaaaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Vector 2 [_ReflectColor]
Float 3 [_ReflectPower]
Float 4 [_Shininess]
Float 5 [_Gloss]
Float 6 [_FresnelPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_Cube] CUBE
"sce_fp_rsx // 74 instructions using 4 registers
[Configuration]
24
ffffffff001fc020007fff81000000000000840004000000
[Offsets]
7
_LightColor0 2 0
000003e000000310
_Color 1 0
00000160
_ReflectColor 1 0
00000370
_ReflectPower 1 0
00000490
_Shininess 1 0
00000050
_Gloss 1 0
000000e0
_FresnelPower 1 0
00000410
[Microcode]
1184
94041704c8011c9dc8000001c8003fe1ae843940c8011c9dc8000029c800bfe1
06880440ce081c9daa0200005402000100000000000040000000bf8000000000
1088014000021c9cc8000001c800000100000000000000000000000000000000
2e8c0141c8011c9dc8000001c8003fe10e020340c9181c9dc9080001c8000001
9e001700c8011c9dc8000001c8003fe10e8e3940c8041c9dc8000029c8000001
a8040500c8011c9dc8010001c800bfe110880240c9101c9d00020000c8000001
0000420000000000000000000000000008860240fe001c9d00020000c8000001
000000000000000000000000000000008e021702c8011c9dc8000001c8003fe1
0e840240550c1c9dc8040001c800000110840240ab101c9cab100000c8000001
1084044001101c9e01100000c908000310840340c9081c9dc8020001c8000001
00000000000000000000000000003f800e860240c8001c9dc8020001c8000001
0000000000000000000000000000000008883b40ff083c9dff080001c8000001
10840540c9101c9dc91c0001c8000001ae003b00c8011c9d54080001c800bfe1
10860540c9101c9dc9180001c80000011e8c0141c8011c9dc8000001c8003fe1
088e0540c9101c9dc9180001c8000001de8a0140c8011c9dc8000001c8003fe1
028e0540c9101c9dc9140001c80000010e8a3940c9101c9dc8000029c8000001
08040500c8001c9dc9140001c800000110860900c90c1c9d00020000c8000001
00000000000000000000000000000000fe800140c8011c9dc8000001c8003fe1
048e0540c9101c9dc9000001c8000001028c0140ff141c9dc8000001c8000001
10800100c9001c9dc8000001c8000001048c0140ff001c9dc8000001c8000001
088c0140ff181c9dc8000001c800000102000500c91c1c9dc9181001c8000001
10040900c9081c9d00020000c800000100000000000000000000000000000000
0e000400c91c1c9f00000000c91800010e001706c8001c9dc8000001c8000001
0206090054081c9d00020000c800000100000000000000000000000000000000
0e880240c90c1c9dc8020001c800000100000000000000000000000000000000
10020100c8041c9dc8000001c80000010e880240c9101c9dff0c0001c8000001
08041d00fe081c9dc8000001c80000011004020054081c9dc9100001c8000001
0e800240c8001c9dc8020001c800000100000000000000000000000000000000
0e800240c9001c9dfe000001c800000108001c00fe081c9dc8000001c8000001
0e82020054001c9dc9080001c800000110048300000c1c9ec8020001c8000001
00000000000000000000000000003f800e820440c9041c9dc8021001c9100001
0000000000000000000000000000000002041d00fe081c9dc8000001c8000001
1004020000081c9c00020000c800000100000000000000000000000000000000
4e880441c90c1c9dc8010001c9043fe108001c00fe081c9dc8000001c8000001
0202040054001c9d00020000aa020000d85a3f4b9e993e500000000000000000
1080090000041c9caa020000c800000100000000000000000000000000000000
0e810440c9001c9d00020000c910000100000000000000000000000000000000
"
}

SubProgram "d3d11 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" }
ConstBuffer "$Globals" 112 // 96 used size, 10 vars
Vector 16 [_LightColor0] 4
Vector 48 [_Color] 4
Vector 64 [_ReflectColor] 4
Float 80 [_ReflectPower]
Float 84 [_Shininess]
Float 88 [_Gloss]
Float 92 [_FresnelPower]
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 0
SetTexture 1 [_SpecMap] 2D 2
SetTexture 2 [_BumpMap] 2D 1
SetTexture 3 [_Cube] CUBE 3
// 56 instructions, 5 temp regs, 0 temp arrays:
// ALU 39 float, 0 int, 0 uint
// TEX 4 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0
eefiecedahkmllaejjcphabcaomndfedphimhkjeabaaaaaakaaiaaaaadaaaaaa
cmaaaaaabeabaaaaeiabaaaaejfdeheooaaaaaaaaiaaaaaaaiaaaaaamiaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaneaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaadadaaaaneaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaaneaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaapapaaaaneaaaaaa
adaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapapaaaaneaaaaaaaeaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapapaaaaneaaaaaaafaaaaaaaaaaaaaaadaaaaaaagaaaaaa
ahahaaaaneaaaaaaagaaaaaaaaaaaaaaadaaaaaaahaaaaaaahahaaaafdfgfpfa
epfdejfeejepeoaafeeffiedepepfceeaaklklklepfdeheocmaaaaaaabaaaaaa
aiaaaaaacaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaafdfgfpfe
gbhcghgfheaaklklfdeieefcfaahaaaaeaaaaaaaneabaaaafjaaaaaeegiocaaa
aaaaaaaaagaaaaaafkaaaaadaagabaaaaaaaaaaafkaaaaadaagabaaaabaaaaaa
fkaaaaadaagabaaaacaaaaaafkaaaaadaagabaaaadaaaaaafibiaaaeaahabaaa
aaaaaaaaffffaaaafibiaaaeaahabaaaabaaaaaaffffaaaafibiaaaeaahabaaa
acaaaaaaffffaaaafidaaaaeaahabaaaadaaaaaaffffaaaagcbaaaaddcbabaaa
abaaaaaagcbaaaadhcbabaaaacaaaaaagcbaaaadpcbabaaaadaaaaaagcbaaaad
pcbabaaaaeaaaaaagcbaaaadpcbabaaaafaaaaaagcbaaaadhcbabaaaagaaaaaa
gcbaaaadhcbabaaaahaaaaaagfaaaaadpccabaaaaaaaaaaagiaaaaacafaaaaaa
efaaaaajpcaabaaaaaaaaaaaegbabaaaabaaaaaaeghobaaaabaaaaaaaagabaaa
acaaaaaaefaaaaajpcaabaaaabaaaaaaegbabaaaabaaaaaaeghobaaaaaaaaaaa
aagabaaaaaaaaaaadiaaaaahhcaabaaaaaaaaaaaegacbaaaaaaaaaaapgapbaaa
abaaaaaadiaaaaaihcaabaaaaaaaaaaaegacbaaaaaaaaaaakgikcaaaaaaaaaaa
afaaaaaadiaaaaaiicaabaaaaaaaaaaabkiacaaaaaaaaaaaafaaaaaaabeaaaaa
aaaaaaecbaaaaaahbcaabaaaacaaaaaaegbcbaaaacaaaaaaegbcbaaaacaaaaaa
eeaaaaafbcaabaaaacaaaaaaakaabaaaacaaaaaadcaaaaajocaabaaaacaaaaaa
agbjbaaaacaaaaaaagaabaaaacaaaaaaagbjbaaaagaaaaaadiaaaaahhcaabaaa
adaaaaaaagaabaaaacaaaaaaegbcbaaaacaaaaaabaaaaaahbcaabaaaacaaaaaa
jgahbaaaacaaaaaajgahbaaaacaaaaaaeeaaaaafbcaabaaaacaaaaaaakaabaaa
acaaaaaadiaaaaahhcaabaaaacaaaaaaagaabaaaacaaaaaajgahbaaaacaaaaaa
efaaaaajpcaabaaaaeaaaaaaegbabaaaabaaaaaaeghobaaaacaaaaaaaagabaaa
abaaaaaadcaaaaapdcaabaaaaeaaaaaahgapbaaaaeaaaaaaaceaaaaaaaaaaaea
aaaaaaeaaaaaaaaaaaaaaaaaaceaaaaaaaaaialpaaaaialpaaaaaaaaaaaaaaaa
dcaaaaakicaabaaaacaaaaaaakaabaiaebaaaaaaaeaaaaaaakaabaaaaeaaaaaa
abeaaaaaaaaaiadpdcaaaaakicaabaaaacaaaaaabkaabaiaebaaaaaaaeaaaaaa
bkaabaaaaeaaaaaadkaabaaaacaaaaaaelaaaaafecaabaaaaeaaaaaadkaabaaa
acaaaaaabaaaaaahbcaabaaaacaaaaaaegacbaaaaeaaaaaaegacbaaaacaaaaaa
deaaaaahbcaabaaaacaaaaaaakaabaaaacaaaaaaabeaaaaaaaaaaaaacpaaaaaf
bcaabaaaacaaaaaaakaabaaaacaaaaaadiaaaaahicaabaaaaaaaaaaadkaabaaa
aaaaaaaaakaabaaaacaaaaaabjaaaaaficaabaaaaaaaaaaadkaabaaaaaaaaaaa
diaaaaahhcaabaaaaaaaaaaaegacbaaaaaaaaaaapgapbaaaaaaaaaaadiaaaaai
hcaabaaaaaaaaaaaegacbaaaaaaaaaaaegiccaaaaaaaaaaaabaaaaaadiaaaaai
hcaabaaaabaaaaaaegacbaaaabaaaaaaegiccaaaaaaaaaaaadaaaaaadiaaaaai
hcaabaaaacaaaaaaegacbaaaabaaaaaaegiccaaaaaaaaaaaabaaaaaabaaaaaah
icaabaaaaaaaaaaaegacbaaaaeaaaaaaegbcbaaaagaaaaaadeaaaaahicaabaaa
aaaaaaaadkaabaaaaaaaaaaaabeaaaaaaaaaaaaadcaaaaajhcaabaaaaaaaaaaa
egacbaaaacaaaaaapgapbaaaaaaaaaaaegacbaaaaaaaaaaaaaaaaaahhcaabaaa
aaaaaaaaegacbaaaaaaaaaaaegacbaaaaaaaaaaadcaaaaajhcaabaaaaaaaaaaa
egacbaaaabaaaaaaegbcbaaaahaaaaaaegacbaaaaaaaaaaadgaaaaafbcaabaaa
abaaaaaadkbabaaaadaaaaaadgaaaaafccaabaaaabaaaaaadkbabaaaaeaaaaaa
dgaaaaafecaabaaaabaaaaaadkbabaaaafaaaaaabaaaaaahbcaabaaaacaaaaaa
egbcbaaaadaaaaaaegacbaaaaeaaaaaabaaaaaahccaabaaaacaaaaaaegbcbaaa
aeaaaaaaegacbaaaaeaaaaaabaaaaaahecaabaaaacaaaaaaegbcbaaaafaaaaaa
egacbaaaaeaaaaaabaaaaaahicaabaaaaaaaaaaaegacbaaaabaaaaaaegacbaaa
acaaaaaaaaaaaaahicaabaaaaaaaaaaadkaabaaaaaaaaaaadkaabaaaaaaaaaaa
dcaaaaakhcaabaaaabaaaaaaegacbaaaacaaaaaapgapbaiaebaaaaaaaaaaaaaa
egacbaaaabaaaaaaefaaaaajpcaabaaaacaaaaaaegacbaaaabaaaaaaeghobaaa
adaaaaaaaagabaaaadaaaaaadiaaaaahhcaabaaaabaaaaaapgapbaaaabaaaaaa
egacbaaaacaaaaaadiaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaaegiccaaa
aaaaaaaaaeaaaaaadcaaaaakhccabaaaaaaaaaaaegacbaaaabaaaaaaagiacaaa
aaaaaaaaafaaaaaaegacbaaaaaaaaaaabaaaaaahbcaabaaaaaaaaaaaegacbaaa
aeaaaaaaegacbaaaaeaaaaaaeeaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaa
diaaaaahhcaabaaaaaaaaaaaagaabaaaaaaaaaaaegacbaaaaeaaaaaabaaaaaah
bcaabaaaaaaaaaaaegacbaaaadaaaaaaegacbaaaaaaaaaaadeaaaaahbcaabaaa
aaaaaaaaakaabaaaaaaaaaaaabeaaaaaaaaaaaaaaaaaaaaibcaabaaaaaaaaaaa
akaabaiaebaaaaaaaaaaaaaaabeaaaaaaaaaiadpdeaaaaahbcaabaaaaaaaaaaa
akaabaaaaaaaaaaaabeaaaaaaaaaaaaacpaaaaafbcaabaaaaaaaaaaaakaabaaa
aaaaaaaadiaaaaaibcaabaaaaaaaaaaaakaabaaaaaaaaaaadkiacaaaaaaaaaaa
afaaaaaabjaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaadcaaaaajiccabaaa
aaaaaaaaakaabaaaaaaaaaaaabeaaaaafknieldpabeaaaaajjjofadodoaaaaab
"
}

SubProgram "gles " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" }
"!!GLES"
}

SubProgram "glesdesktop " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" }
"!!GLES"
}

SubProgram "opengl " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" }
Vector 0 [_Color]
Vector 1 [_ReflectColor]
Float 2 [_ReflectPower]
Float 3 [_FresnelPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_BumpMap] 2D
SetTexture 2 [_Cube] CUBE
SetTexture 3 [unity_Lightmap] 2D
"3.0-!!ARBfp1.0
OPTION ARB_precision_hint_fastest;
# 39 ALU, 4 TEX
PARAM c[6] = { program.local[0..3],
		{ 2, 1, 0, 0.79627001 },
		{ 0.20373, 8 } };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
TEMP R4;
TEX R0.yw, fragment.texcoord[0], texture[1], 2D;
MAD R4.xy, R0.wyzw, c[4].x, -c[4].y;
MUL R0.x, R4.y, R4.y;
MAD R0.x, -R4, R4, -R0;
TEX R1, fragment.texcoord[0], texture[0], 2D;
ADD R0.x, R0, c[4].y;
RSQ R0.x, R0.x;
RCP R4.z, R0.x;
TEX R0, fragment.texcoord[5], texture[3], 2D;
MUL R0.xyz, R0.w, R0;
MUL R1.xyz, R1, c[0];
DP3 R0.w, fragment.texcoord[1], fragment.texcoord[1];
DP3 R3.x, R4, fragment.texcoord[2];
DP3 R3.y, R4, fragment.texcoord[3];
DP3 R3.z, R4, fragment.texcoord[4];
MOV R2.x, fragment.texcoord[2].w;
MOV R2.z, fragment.texcoord[4].w;
MOV R2.y, fragment.texcoord[3].w;
DP3 R2.w, R3, R2;
MUL R0.xyz, R0, R1;
MUL R1.xyz, R3, R2.w;
DP3 R2.w, R4, R4;
RSQ R0.w, R0.w;
RSQ R2.w, R2.w;
MAD R1.xyz, -R1, c[4].x, R2;
TEX R1.xyz, R1, texture[2], CUBE;
MUL R1.xyz, R1.w, R1;
MUL R1.xyz, R1, c[1];
MUL R1.xyz, R1, c[2].x;
MUL R4.xyz, R2.w, R4;
MUL R3.xyz, R0.w, fragment.texcoord[1];
DP3 R0.w, R3, R4;
MAX R0.w, R0, c[4].z;
ADD_SAT R0.w, -R0, c[4].y;
POW R0.w, R0.w, c[3].x;
MUL R0.w, R0, c[4];
ADD R0.w, R0, c[5].x;
MAD result.color.xyz, R0, c[5].y, R1;
MAX result.color.w, R0, c[4].z;
END
# 39 instructions, 5 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" }
Vector 0 [_Color]
Vector 1 [_ReflectColor]
Float 2 [_ReflectPower]
Float 3 [_FresnelPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_BumpMap] 2D
SetTexture 2 [_Cube] CUBE
SetTexture 3 [unity_Lightmap] 2D
"ps_3_0
; 36 ALU, 4 TEX
dcl_2d s0
dcl_2d s1
dcl_cube s2
dcl_2d s3
def c4, 2.00000000, -1.00000000, 1.00000000, 0.00000000
def c5, 0.79627001, 0.20373000, 8.00000000, 0
dcl_texcoord0 v0.xy
dcl_texcoord1 v1.xyz
dcl_texcoord2 v2
dcl_texcoord3 v3
dcl_texcoord4 v4
dcl_texcoord5 v5.xy
texld r0.yw, v0, s1
mad_pp r0.xy, r0.wyzw, c4.x, c4.y
mul_pp r0.z, r0.y, r0.y
texld r1, v5, s3
mul_pp r2.xyz, r1.w, r1
mad_pp r0.z, -r0.x, r0.x, -r0
texld r1, v0, s0
mul_pp r1.xyz, r1, c0
mul_pp r4.xyz, r2, r1
add_pp r0.z, r0, c4
rsq_pp r0.z, r0.z
rcp_pp r0.z, r0.z
dp3_pp r3.x, r0, r0
dp3 r2.w, v1, v1
rsq_pp r3.x, r3.x
dp3_pp r2.x, r0, v2
dp3_pp r2.y, r0, v3
dp3_pp r2.z, r0, v4
mov r1.x, v2.w
mov r1.z, v4.w
mov r1.y, v3.w
dp3 r0.w, r2, r1
rsq r2.w, r2.w
mul_pp r3.xyz, r3.x, r0
mul r0.xyz, r2.w, v1
dp3 r2.w, r0, r3
mul r0.xyz, r2, r0.w
mad r0.xyz, -r0, c4.x, r1
max r0.w, r2, c4
texld r1.xyz, r0, s2
add_sat r2.x, -r0.w, c4.z
pow r0, r2.x, c3.x
mul_pp r1.xyz, r1.w, r1
mul_pp r1.xyz, r1, c1
mul_pp r1.xyz, r1, c2.x
mad r0.x, r0, c5, c5.y
mad_pp oC0.xyz, r4, c5.z, r1
max oC0.w, r0.x, c4
"
}

SubProgram "xbox360 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" }
Vector 0 [_Color]
Float 3 [_FresnelPower]
Vector 1 [_ReflectColor]
Float 2 [_ReflectPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_BumpMap] 2D
SetTexture 2 [_Cube] CUBE
SetTexture 3 [unity_Lightmap] 2D
// Shader Timing Estimate, in Cycles/64 pixel vector:
// ALU: 42.67 (32 instructions), vertex: 0, texture: 16,
//   sequencer: 16, interpolator: 24;    9 GPRs, 21 threads,
// Performance (if enough threads): ~42 cycles per vector
// * Texture cycle estimates are assuming an 8bit/component texture with no
//     aniso or trilinear filtering.

"ps_360
backbbaaaaaaabpiaaaaaccmaaaaaaaaaaaaaaceaaaaabjiaaaaabmaaaaaaaaa
aaaaaaaaaaaaabhaaaaaaabmaaaaabgdppppadaaaaaaaaaiaaaaaabmaaaaaaaa
aaaaabfmaaaaaalmaaadaaabaaabaaaaaaaaaamiaaaaaaaaaaaaaaniaaacaaaa
aaabaaaaaaaaaaoaaaaaaaaaaaaaaapaaaadaaacaaabaaaaaaaaaapiaaaaaaaa
aaaaabaiaaacaaadaaabaaaaaaaaabbiaaaaaaaaaaaaabciaaadaaaaaaabaaaa
aaaaaamiaaaaaaaaaaaaabdbaaacaaabaaabaaaaaaaaaaoaaaaaaaaaaaaaabdp
aaacaaacaaabaaaaaaaaabbiaaaaaaaaaaaaabenaaadaaadaaabaaaaaaaaaami
aaaaaaaafpechfgnhaengbhaaaklklklaaaeaaamaaabaaabaaabaaaaaaaaaaaa
fpedgpgmgphcaaklaaabaaadaaabaaaeaaabaaaaaaaaaaaafpedhfgcgfaaklkl
aaaeaaaoaaabaaabaaabaaaaaaaaaaaafpeghcgfhdgogfgmfagphhgfhcaaklkl
aaaaaaadaaabaaabaaabaaaaaaaaaaaafpengbgjgofegfhiaafpfcgfgggmgfgd
heedgpgmgphcaafpfcgfgggmgfgdhefagphhgfhcaahfgogjhehjfpemgjghgihe
gngbhaaahahdfpddfpdaaadccodacodcdadddfddcodaaaklaaaaaaaaaaaaaaab
aaaaaaaaaaaaaaaaaaaaaabeabpmaabaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaeaaaaaabombaaaaiaaaaaaaaaeaaaaaaaaaaaaemmgaadpaadpaaaaaaab
aaaadafaaaaahbfbaaaapcfcaaaapdfdaaaapefeaaaadfffaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaadpiaaaaa
ebaaaaaaeaaaaaaadpmaaaaadofajojjlpiaaaaadpelnifkaaajgaaegaakbcaa
bcaaaaaaaaaagabagabgbcaabcaaaaaaaafeeabmaaaabcaameaaaaaaaaaagaca
cacgbcaaccaaaaaababafaabbpbppghpaaaaeaaamiaiaaaaaaloloaapaababaa
miadaaagaamhblmgilafpoppmiaiaaabaegngnlbnbagagpomiamaaafaakmkmaa
obagagaafiieaaaaaamgblbloaafafiakaieagaaaamgblbloaaaabibfiehagai
aablmamgobaaabiamiaeaaabaalploaapaagadaamiahaaahaamabgaaobagagaa
miacaaabaalploaapaagaeaabeaeaaaaaalololbpaaiahabamieaaaaaamggmbl
kcaapoaemiaiaaaaaamgblblolabadaalkebaaabaalploicnaagacpomiaiaaaa
aagmblblolabacaaeaeiaaaaaablblmgoaaaaaaamiahaaabaablleaaobaaabaa
miaeaaabaemgblaaoaabaeaakiecaaabaelbblecmaabadaddiebaaabaegmblmg
oaabacaamiapaaabaakgmnaapcababaaemieaaacaablblmgocababibkiedabab
aagnblacmbabaappmialaaacaamalmaakaabppaajacieaebbpbppoiiaaaamaaa
baaidaabbpbppgiiaaaaeaaabadibakbbpbppgiiaaaaeaaakmiaaaaaaaaaaaed
ocaaaaabbebgaaaaaalgblblobaeadabkibbaaaeaablgmmambaaaepokichaead
aamamaicibadaaabkiehaeabaagmmambmbaaababmiahaaaaaamagmaakbaeacaa
miahiaaaaamamamaoladabaamiaiiaaaaablgmaakcacpoaaaaaaaaaaaaaaaaaa
aaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" }
Vector 0 [_Color]
Vector 1 [_ReflectColor]
Float 2 [_ReflectPower]
Float 3 [_FresnelPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_BumpMap] 2D
SetTexture 2 [_Cube] CUBE
SetTexture 3 [unity_Lightmap] 2D
"sce_fp_rsx // 49 instructions using 4 registers
[Configuration]
24
ffffffff000fc020003fffe1000000000000840004000000
[Offsets]
4
_Color 1 0
00000020
_ReflectColor 1 0
00000290
_ReflectPower 1 0
00000300
_FresnelPower 1 0
00000240
[Microcode]
784
9e041700c8011c9dc8000001c8003fe10e840240c8081c9dc8020001c8000001
0000000000000000000000000000000094001702c8011c9dc8000001c8003fe1
06860440ce001c9d00020000aa020000000040000000bf800000000000000000
3e001707c8011c9dc8000001c8003fe10e840240fe001c9dc9080001c8000001
10820240ab0c1c9cab0c0000c800000110840440010c1c9e010c0000c9040003
b0000500c8011c9dc8010001c800bfe110840340c9081c9d00020000c8000001
00003f800000000000000000000000000e800240c9081c9dc8003001c8000001
08863b40ff083c9dff080001c8000001ae063b00c8011c9dfe000001c800bfe1
0e883940c90c1c9dc8000029c800000108040500c80c1c9dc9100001c8000001
de880140c8011c9dc8000001c8003fe102880540c90c1c9dc9100001c8000001
1000090054081c9d00020000c800000100000000000000000000000000000000
1e8c0141c8011c9dc8000001c8003fe108880540c90c1c9dc9180001c8000001
fe8e0140c8011c9dc8000001c8003fe104880540c90c1c9dc91c0001c8000001
048c0140ff1c1c9dc8000001c8000001088c0140ff181c9dc8000001c8000001
028c0140ff101c9dc8000001c800000110008300c8001c9f00020000c8000001
00003f8000000000000000000000000010000100c8001c9dc8000001c8000001
06040100c8081c9dc8000001c800000108040500c9101c9dc9181001c8000001
10001d00fe001c9dc8000001c800000110020200c8001c9d00020000c8000001
000000000000000000000000000000000e040400c9101c9f54080001c9180001
02021c00fe041c9dc8000001c80000010e041704c8081c9dc8000001c8000001
0e880240c8081c9dc8020001c800000100000000000000000000000000000000
02020400c8041c9d00020000aa020000d85a3f4b9e993e500000000000000000
0e820240c9101c9dfe080001c80000011080090000041c9c00020000c8000001
000000000000000000000000000000000e810440c9041c9d00020000c9000001
00000000000000000000000000000000
"
}

SubProgram "d3d11 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" }
ConstBuffer "$Globals" 128 // 96 used size, 11 vars
Vector 48 [_Color] 4
Vector 64 [_ReflectColor] 4
Float 80 [_ReflectPower]
Float 92 [_FresnelPower]
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 0
SetTexture 1 [_BumpMap] 2D 1
SetTexture 2 [_Cube] CUBE 2
SetTexture 3 [unity_Lightmap] 2D 3
// 39 instructions, 4 temp regs, 0 temp arrays:
// ALU 25 float, 0 int, 0 uint
// TEX 4 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0
eefiecedccenmjhkcffpknomfomolfkpcificgiaabaaaaaajaagaaaaadaaaaaa
cmaaaaaapmaaaaaadaabaaaaejfdeheomiaaaaaaahaaaaaaaiaaaaaalaaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaalmaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaadadaaaalmaaaaaaafaaaaaaaaaaaaaaadaaaaaaabaaaaaa
amamaaaalmaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaaahahaaaalmaaaaaa
acaaaaaaaaaaaaaaadaaaaaaadaaaaaaapapaaaalmaaaaaaadaaaaaaaaaaaaaa
adaaaaaaaeaaaaaaapapaaaalmaaaaaaaeaaaaaaaaaaaaaaadaaaaaaafaaaaaa
apapaaaafdfgfpfaepfdejfeejepeoaafeeffiedepepfceeaaklklklepfdeheo
cmaaaaaaabaaaaaaaiaaaaaacaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaaaaaaaaa
apaaaaaafdfgfpfegbhcghgfheaaklklfdeieefcfiafaaaaeaaaaaaafgabaaaa
fjaaaaaeegiocaaaaaaaaaaaagaaaaaafkaaaaadaagabaaaaaaaaaaafkaaaaad
aagabaaaabaaaaaafkaaaaadaagabaaaacaaaaaafkaaaaadaagabaaaadaaaaaa
fibiaaaeaahabaaaaaaaaaaaffffaaaafibiaaaeaahabaaaabaaaaaaffffaaaa
fidaaaaeaahabaaaacaaaaaaffffaaaafibiaaaeaahabaaaadaaaaaaffffaaaa
gcbaaaaddcbabaaaabaaaaaagcbaaaadmcbabaaaabaaaaaagcbaaaadhcbabaaa
acaaaaaagcbaaaadpcbabaaaadaaaaaagcbaaaadpcbabaaaaeaaaaaagcbaaaad
pcbabaaaafaaaaaagfaaaaadpccabaaaaaaaaaaagiaaaaacaeaaaaaaefaaaaaj
pcaabaaaaaaaaaaaogbkbaaaabaaaaaaeghobaaaadaaaaaaaagabaaaadaaaaaa
diaaaaahicaabaaaaaaaaaaadkaabaaaaaaaaaaaabeaaaaaaaaaaaebdiaaaaah
hcaabaaaaaaaaaaaegacbaaaaaaaaaaapgapbaaaaaaaaaaadgaaaaafbcaabaaa
abaaaaaadkbabaaaadaaaaaadgaaaaafccaabaaaabaaaaaadkbabaaaaeaaaaaa
dgaaaaafecaabaaaabaaaaaadkbabaaaafaaaaaaefaaaaajpcaabaaaacaaaaaa
egbabaaaabaaaaaaeghobaaaabaaaaaaaagabaaaabaaaaaadcaaaaapdcaabaaa
acaaaaaahgapbaaaacaaaaaaaceaaaaaaaaaaaeaaaaaaaeaaaaaaaaaaaaaaaaa
aceaaaaaaaaaialpaaaaialpaaaaaaaaaaaaaaaadcaaaaakicaabaaaaaaaaaaa
akaabaiaebaaaaaaacaaaaaaakaabaaaacaaaaaaabeaaaaaaaaaiadpdcaaaaak
icaabaaaaaaaaaaabkaabaiaebaaaaaaacaaaaaabkaabaaaacaaaaaadkaabaaa
aaaaaaaaelaaaaafecaabaaaacaaaaaadkaabaaaaaaaaaaabaaaaaahbcaabaaa
adaaaaaaegbcbaaaadaaaaaaegacbaaaacaaaaaabaaaaaahccaabaaaadaaaaaa
egbcbaaaaeaaaaaaegacbaaaacaaaaaabaaaaaahecaabaaaadaaaaaaegbcbaaa
afaaaaaaegacbaaaacaaaaaabaaaaaahicaabaaaaaaaaaaaegacbaaaabaaaaaa
egacbaaaadaaaaaaaaaaaaahicaabaaaaaaaaaaadkaabaaaaaaaaaaadkaabaaa
aaaaaaaadcaaaaakhcaabaaaabaaaaaaegacbaaaadaaaaaapgapbaiaebaaaaaa
aaaaaaaaegacbaaaabaaaaaaefaaaaajpcaabaaaabaaaaaaegacbaaaabaaaaaa
eghobaaaacaaaaaaaagabaaaacaaaaaaefaaaaajpcaabaaaadaaaaaaegbabaaa
abaaaaaaeghobaaaaaaaaaaaaagabaaaaaaaaaaadiaaaaahhcaabaaaabaaaaaa
egacbaaaabaaaaaapgapbaaaadaaaaaadiaaaaaihcaabaaaadaaaaaaegacbaaa
adaaaaaaegiccaaaaaaaaaaaadaaaaaadiaaaaaihcaabaaaabaaaaaaegacbaaa
abaaaaaaegiccaaaaaaaaaaaaeaaaaaadiaaaaaihcaabaaaabaaaaaaegacbaaa
abaaaaaaagiacaaaaaaaaaaaafaaaaaadcaaaaajhccabaaaaaaaaaaaegacbaaa
adaaaaaaegacbaaaaaaaaaaaegacbaaaabaaaaaabaaaaaahbcaabaaaaaaaaaaa
egacbaaaacaaaaaaegacbaaaacaaaaaaeeaaaaafbcaabaaaaaaaaaaaakaabaaa
aaaaaaaadiaaaaahhcaabaaaaaaaaaaaagaabaaaaaaaaaaaegacbaaaacaaaaaa
baaaaaahicaabaaaaaaaaaaaegbcbaaaacaaaaaaegbcbaaaacaaaaaaeeaaaaaf
icaabaaaaaaaaaaadkaabaaaaaaaaaaadiaaaaahhcaabaaaabaaaaaapgapbaaa
aaaaaaaaegbcbaaaacaaaaaabaaaaaahbcaabaaaaaaaaaaaegacbaaaabaaaaaa
egacbaaaaaaaaaaadeaaaaahbcaabaaaaaaaaaaaakaabaaaaaaaaaaaabeaaaaa
aaaaaaaaaaaaaaaibcaabaaaaaaaaaaaakaabaiaebaaaaaaaaaaaaaaabeaaaaa
aaaaiadpdeaaaaahbcaabaaaaaaaaaaaakaabaaaaaaaaaaaabeaaaaaaaaaaaaa
cpaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaadiaaaaaibcaabaaaaaaaaaaa
akaabaaaaaaaaaaadkiacaaaaaaaaaaaafaaaaaabjaaaaafbcaabaaaaaaaaaaa
akaabaaaaaaaaaaadcaaaaajiccabaaaaaaaaaaaakaabaaaaaaaaaaaabeaaaaa
fknieldpabeaaaaajjjofadodoaaaaab"
}

SubProgram "gles " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" }
"!!GLES"
}

SubProgram "glesdesktop " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" }
"!!GLES"
}

}
	}
	Pass {
		Name "FORWARD"
		Tags { "LightMode" = "ForwardAdd" }
		ZWrite Off Blend One One Fog { Color (0,0,0,0) }
		Blend SrcAlpha One
Program "vp" {
// Vertex combos: 5
//   opengl - ALU: 25 to 34
//   d3d9 - ALU: 28 to 37
//   d3d11 - ALU: 12 to 14, TEX: 0 to 0, FLOW: 1 to 1
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
"3.0-!!ARBvp1.0
# 33 ALU
PARAM c[21] = { { 1 },
		state.matrix.mvp,
		program.local[5..20] };
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
"vs_3_0
; 36 ALU
dcl_position o0
dcl_texcoord0 o1
dcl_texcoord1 o2
dcl_texcoord2 o3
dcl_texcoord3 o4
def c20, 1.00000000, 0, 0, 0
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
mov r1.w, c20.x
dp3 o3.y, r2, r0
dp3 o3.z, v2, r0
dp3 o3.x, v1, r0
dp4 r0.w, v0, c7
dp4 r0.z, v0, c6
dp4 r0.x, v0, c4
dp4 r0.y, v0, c5
dp4 r3.z, r1, c10
dp4 r3.x, r1, c8
dp4 r3.y, r1, c9
mad r1.xyz, r3, c18.w, -v0
dp3 o2.y, r1, r2
dp3 o2.z, v2, r1
dp3 o2.x, r1, v1
dp4 o4.z, r0, c14
dp4 o4.y, r0, c13
dp4 o4.x, r0, c12
mad o1.xy, v3, c19, c19.zwzw
dp4 o0.w, v0, c3
dp4 o0.z, v0, c2
dp4 o0.y, v0, c1
dp4 o0.x, v0, c0
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
bcaaaaaaaaaadacfaaaaccaaaaaaaaaaafpidaaaaaaaagiiaaaaaaaaafpifaaa
aaaaagiiaaaaaaaaafpicaaaaaaaaoiiaaaaaaaaafpibaaaaaaaapmiaaaaaaaa
miapaaaaaabliiaakbadafaamiapaaaaaamgiiaakladaeaamiapaaaaaalbdeje
kladadaamiapiadoaagmaadekladacaamiahaaaaaaleblaacbanabaamiahaaag
aamamgmaalamaaanmiahaaaeaalogfaaobacafaamiahaaagaalelbleclalaaag
miahaaahaamamgleclamabaamiapaaaaaabliiaakbadajaamiapaaaaaamgiiaa
kladaiaamiahaaahaalelbleclalabahmiahaaagaamagmleclakaaagmiahaaae
abgflomaolacafaemiahaaaeaamablaaobaeafaamiahaaagabmablmaklagaoad
miahaaahaamagmleclakabahmiapaaaaaalbdejekladahaamiapaaaaaagmejhk
kladagaamiahaaadabmablmaklahaoadmiabiaabaaloloaapaagafaamiaciaab
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
00009c6c005d200d8186c0836041fffc00019c6c00400e0c0106c0836041dffc
00011c6c005d300c0186c0836041dffc401f9c6c011d0808010400d740619f9c
401f9c6c01d0300d8106c0c360403f80401f9c6c01d0200d8106c0c360405f80
401f9c6c01d0100d8106c0c360409f80401f9c6c01d0000d8106c0c360411f80
00001c6c01d0700d8106c0c360403ffc00001c6c01d0600d8106c0c360405ffc
00001c6c01d0500d8106c0c360409ffc00001c6c01d0400d8106c0c360411ffc
00021c6c01d0a00d8286c0c360405ffc00021c6c01d0900d8286c0c360409ffc
00021c6c01d0800d8286c0c360411ffc00009c6c0190a00c0486c0c360405ffc
00009c6c0190900c0486c0c360409ffc00009c6c0190800c0486c0c360411ffc
00011c6c00800243011843436041dffc00011c6c01000230812183630121dffc
401f9c6c01d0e00d8086c0c360405fa8401f9c6c01d0d00d8086c0c360409fa8
401f9c6c01d0c00d8086c0c360411fa800001c6c011d100c08bfc0e30041dffc
00009c6c011d100c02bfc0e30041dffc401f9c6c0140020c0106004360405fa4
401f9c6c01400e0c0106004360411fa400011c6c00800e0c04bfc0836041dffc
401f9c6c0140020c0106014360405fa0401f9c6c01400e0c0286008360411fa0
401f9c6c0140000c0486004360409fa4401f9c6c0140000c0286024360409fa1
"
}

SubProgram "d3d11 " {
Keywords { "POINT" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 176 // 176 used size, 11 vars
Matrix 48 [_LightMatrix0] 4
Vector 160 [_MainTex_ST] 4
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
eefiecedkhafdghledbaandiffiakgoaiikhgdafabaaaaaapiagaaaaadaaaaaa
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
fhabaaaafjaaaaaeegiocaaaaaaaaaaaalaaaaaafjaaaaaeegiocaaaabaaaaaa
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
egbabaaaadaaaaaaegiacaaaaaaaaaaaakaaaaaaogikcaaaaaaaaaaaakaaaaaa
diaaaaahhcaabaaaaaaaaaaajgbebaaaabaaaaaacgbjbaaaacaaaaaadcaaaaak
hcaabaaaaaaaaaaajgbebaaaacaaaaaacgbjbaaaabaaaaaaegacbaiaebaaaaaa
aaaaaaaadiaaaaahhcaabaaaaaaaaaaaegacbaaaaaaaaaaapgbpbaaaabaaaaaa
diaaaaajhcaabaaaabaaaaaafgifcaaaabaaaaaaaeaaaaaaegiccaaaadaaaaaa
bbaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaa
abaaaaaaaeaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaa
adaaaaaabcaaaaaakgikcaaaabaaaaaaaeaaaaaaegacbaaaabaaaaaaaaaaaaai
hcaabaaaabaaaaaaegacbaaaabaaaaaaegiccaaaadaaaaaabdaaaaaadcaaaaal
hcaabaaaabaaaaaaegacbaaaabaaaaaapgipcaaaadaaaaaabeaaaaaaegbcbaia
ebaaaaaaaaaaaaaabaaaaaahcccabaaaacaaaaaaegacbaaaaaaaaaaaegacbaaa
abaaaaaabaaaaaahbccabaaaacaaaaaaegbcbaaaabaaaaaaegacbaaaabaaaaaa
baaaaaaheccabaaaacaaaaaaegbcbaaaacaaaaaaegacbaaaabaaaaaadiaaaaaj
hcaabaaaabaaaaaafgifcaaaacaaaaaaaaaaaaaaegiccaaaadaaaaaabbaaaaaa
dcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaaacaaaaaa
aaaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaa
bcaaaaaakgikcaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaa
abaaaaaaegiccaaaadaaaaaabdaaaaaapgipcaaaacaaaaaaaaaaaaaaegacbaaa
abaaaaaadcaaaaalhcaabaaaabaaaaaaegacbaaaabaaaaaapgipcaaaadaaaaaa
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
varying highp vec3 xlv_TEXCOORD1;
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
  highp vec3 tmpvar_7;
  tmpvar_7 = (tmpvar_6 * (((_World2Object * _WorldSpaceLightPos0).xyz * unity_Scale.w) - _glesVertex.xyz));
  tmpvar_3 = tmpvar_7;
  highp vec4 tmpvar_8;
  tmpvar_8.w = 1.0;
  tmpvar_8.xyz = _WorldSpaceCameraPos;
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = (tmpvar_6 * (((_World2Object * tmpvar_8).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = tmpvar_3;
  xlv_TEXCOORD3 = (_LightMatrix0 * (_Object2World * _glesVertex)).xyz;
}



#endif
#ifdef FRAGMENT

varying highp vec3 xlv_TEXCOORD3;
varying mediump vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp float _FresnelPower;
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
  mediump float tmpvar_5;
  lowp vec4 tmpvar_6;
  tmpvar_6 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_7;
  tmpvar_7 = (tmpvar_6.xyz * _Color.xyz);
  tmpvar_3 = tmpvar_7;
  lowp vec4 tmpvar_8;
  tmpvar_8 = texture2D (_SpecMap, xlv_TEXCOORD0);
  mediump vec3 tmpvar_9;
  tmpvar_9 = ((tmpvar_6.w * tmpvar_8.xyz) * _Gloss);
  lowp vec3 tmpvar_10;
  tmpvar_10 = ((texture2D (_BumpMap, xlv_TEXCOORD0).xyz * 2.0) - 1.0);
  tmpvar_4 = tmpvar_10;
  mediump vec3 tmpvar_11;
  tmpvar_11 = normalize(tmpvar_4);
  highp float tmpvar_12;
  tmpvar_12 = max ((0.20373 + (0.79627 * pow (clamp ((1.0 - max (dot (normalize(xlv_TEXCOORD1), tmpvar_11), 0.0)), 0.0, 1.0), _FresnelPower))), 0.0);
  tmpvar_5 = tmpvar_12;
  mediump vec3 tmpvar_13;
  tmpvar_13 = normalize(xlv_TEXCOORD2);
  lightDir_2 = tmpvar_13;
  highp vec3 tmpvar_14;
  tmpvar_14 = normalize(xlv_TEXCOORD1);
  highp float tmpvar_15;
  tmpvar_15 = dot (xlv_TEXCOORD3, xlv_TEXCOORD3);
  lowp vec4 tmpvar_16;
  tmpvar_16 = texture2D (_LightTexture0, vec2(tmpvar_15));
  mediump vec3 lightDir_17;
  lightDir_17 = lightDir_2;
  mediump vec3 viewDir_18;
  viewDir_18 = tmpvar_14;
  mediump float atten_19;
  atten_19 = tmpvar_16.w;
  mediump vec4 c_20;
  mediump vec3 specCol_21;
  highp float nh_22;
  mediump float tmpvar_23;
  tmpvar_23 = max (0.0, dot (tmpvar_4, normalize((lightDir_17 + viewDir_18))));
  nh_22 = tmpvar_23;
  mediump float arg1_24;
  arg1_24 = (32.0 * _Shininess);
  highp vec3 tmpvar_25;
  tmpvar_25 = (pow (nh_22, arg1_24) * tmpvar_9);
  specCol_21 = tmpvar_25;
  c_20.xyz = ((((tmpvar_3 * _LightColor0.xyz) * max (0.0, dot (tmpvar_4, lightDir_17))) + (_LightColor0.xyz * specCol_21)) * (atten_19 * 2.0));
  c_20.w = tmpvar_5;
  c_1.xyz = c_20.xyz;
  c_1.w = tmpvar_5;
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
  highp vec3 tmpvar_7;
  tmpvar_7 = (tmpvar_6 * (((_World2Object * _WorldSpaceLightPos0).xyz * unity_Scale.w) - _glesVertex.xyz));
  tmpvar_3 = tmpvar_7;
  highp vec4 tmpvar_8;
  tmpvar_8.w = 1.0;
  tmpvar_8.xyz = _WorldSpaceCameraPos;
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = (tmpvar_6 * (((_World2Object * tmpvar_8).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = tmpvar_3;
  xlv_TEXCOORD3 = (_LightMatrix0 * (_Object2World * _glesVertex)).xyz;
}



#endif
#ifdef FRAGMENT

varying highp vec3 xlv_TEXCOORD3;
varying mediump vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp float _FresnelPower;
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
  mediump float tmpvar_5;
  lowp vec4 tmpvar_6;
  tmpvar_6 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_7;
  tmpvar_7 = (tmpvar_6.xyz * _Color.xyz);
  tmpvar_3 = tmpvar_7;
  lowp vec4 tmpvar_8;
  tmpvar_8 = texture2D (_SpecMap, xlv_TEXCOORD0);
  mediump vec3 tmpvar_9;
  tmpvar_9 = ((tmpvar_6.w * tmpvar_8.xyz) * _Gloss);
  lowp vec3 normal_10;
  normal_10.xy = ((texture2D (_BumpMap, xlv_TEXCOORD0).wy * 2.0) - 1.0);
  normal_10.z = sqrt(((1.0 - (normal_10.x * normal_10.x)) - (normal_10.y * normal_10.y)));
  tmpvar_4 = normal_10;
  mediump vec3 tmpvar_11;
  tmpvar_11 = normalize(tmpvar_4);
  highp float tmpvar_12;
  tmpvar_12 = max ((0.20373 + (0.79627 * pow (clamp ((1.0 - max (dot (normalize(xlv_TEXCOORD1), tmpvar_11), 0.0)), 0.0, 1.0), _FresnelPower))), 0.0);
  tmpvar_5 = tmpvar_12;
  mediump vec3 tmpvar_13;
  tmpvar_13 = normalize(xlv_TEXCOORD2);
  lightDir_2 = tmpvar_13;
  highp vec3 tmpvar_14;
  tmpvar_14 = normalize(xlv_TEXCOORD1);
  highp float tmpvar_15;
  tmpvar_15 = dot (xlv_TEXCOORD3, xlv_TEXCOORD3);
  lowp vec4 tmpvar_16;
  tmpvar_16 = texture2D (_LightTexture0, vec2(tmpvar_15));
  mediump vec3 lightDir_17;
  lightDir_17 = lightDir_2;
  mediump vec3 viewDir_18;
  viewDir_18 = tmpvar_14;
  mediump float atten_19;
  atten_19 = tmpvar_16.w;
  mediump vec4 c_20;
  mediump vec3 specCol_21;
  highp float nh_22;
  mediump float tmpvar_23;
  tmpvar_23 = max (0.0, dot (tmpvar_4, normalize((lightDir_17 + viewDir_18))));
  nh_22 = tmpvar_23;
  mediump float arg1_24;
  arg1_24 = (32.0 * _Shininess);
  highp vec3 tmpvar_25;
  tmpvar_25 = (pow (nh_22, arg1_24) * tmpvar_9);
  specCol_21 = tmpvar_25;
  c_20.xyz = ((((tmpvar_3 * _LightColor0.xyz) * max (0.0, dot (tmpvar_4, lightDir_17))) + (_LightColor0.xyz * specCol_21)) * (atten_19 * 2.0));
  c_20.w = tmpvar_5;
  c_1.xyz = c_20.xyz;
  c_1.w = tmpvar_5;
  gl_FragData[0] = c_1;
}



#endif"
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
"3.0-!!ARBvp1.0
# 25 ALU
PARAM c[13] = { { 1 },
		state.matrix.mvp,
		program.local[5..12] };
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
"vs_3_0
; 28 ALU
dcl_position o0
dcl_texcoord0 o1
dcl_texcoord1 o2
dcl_texcoord2 o3
def c12, 1.00000000, 0, 0, 0
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
mov r1.w, c12.x
mov r1.xyz, c8
dp4 r4.y, c9, r0
dp4 r2.z, r1, c6
dp4 r2.x, r1, c4
dp4 r2.y, r1, c5
mad r2.xyz, r2, c10.w, -v0
mov r1, c4
dp4 r4.x, c9, r1
dp3 o2.y, r2, r3
dp3 o3.y, r3, r4
dp3 o2.z, v2, r2
dp3 o2.x, r2, v1
dp3 o3.z, v2, r4
dp3 o3.x, v1, r4
mad o1.xy, v3, c11, c11.zwzw
dp4 o0.w, v0, c3
dp4 o0.z, v0, c2
dp4 o0.y, v0, c1
dp4 o0.x, v0, c0
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
aaaagaamgabcbcaabcaaaaaaaaaagabiaaaaccaaaaaaaaaaafpifaaaaaaaagii
aaaaaaaaafpieaaaaaaaagiiaaaaaaaaafpibaaaaaaaaoiiaaaaaaaaafpiaaaa
aaaaapmiaaaaaaaamiapaaacaabliiaakbafafaamiapaaacaamgiiaaklafaeac
miapaaacaalbdejeklafadacmiapiadoaagmaadeklafacacmiahaaagaamamgma
alaiaaajmiahaaacaaleblaacbajabaamiahaaacaamamgleclaiabacmiahaaad
aalogfaaobabaeaamiahaaagaalelbleclahaaagmiahaaagaamagmleclagaaag
miahaaadabgflomaolabaeadmiahaaacaalelbleclahabacmiahaaacaamagmle
clagabacmiahaaadaamablaaobadaeaamiahaaafabmablmaklagakafmiabiaab
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
00009c6c005d200d8186c0836041fffc00011c6c00400e0c0106c0836041dffc
00001c6c005d300c0186c0836041dffc401f9c6c011d0808010400d740619f9c
401f9c6c01d0300d8106c0c360403f80401f9c6c01d0200d8106c0c360405f80
401f9c6c01d0100d8106c0c360409f80401f9c6c01d0000d8106c0c360411f80
00019c6c01d0600d8286c0c360405ffc00019c6c01d0500d8286c0c360409ffc
00019c6c01d0400d8286c0c360411ffc00009c6c0190600c0086c0c360405ffc
00009c6c0190500c0086c0c360409ffc00009c6c0190400c0086c0c360411ffc
00001c6c00800243011842436041dffc00001c6c01000230812182630021dffc
00009c6c011d100c02bfc0e30041dffc401f9c6c0140020c0106034360405fa4
401f9c6c01400e0c0106034360411fa400001c6c00800e0c00bfc0836041dffc
401f9c6c0140020c0106014360405fa0401f9c6c01400e0c0286008360411fa0
401f9c6c0140000c0086034360409fa4401f9c6c0140000c0286004360409fa1
"
}

SubProgram "d3d11 " {
Keywords { "DIRECTIONAL" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 112 // 112 used size, 10 vars
Vector 96 [_MainTex_ST] 4
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
eefiecedcdgdmnaagdmahjcbgfdgehfnlmopgnpkabaaaaaahiafaaaaadaaaaaa
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
peadaaaaeaaaabaapnaaaaaafjaaaaaeegiocaaaaaaaaaaaahaaaaaafjaaaaae
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
adaaaaaaegiacaaaaaaaaaaaagaaaaaaogikcaaaaaaaaaaaagaaaaaadiaaaaah
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
varying highp vec3 xlv_TEXCOORD1;
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
  highp vec3 tmpvar_7;
  tmpvar_7 = (tmpvar_6 * (_World2Object * _WorldSpaceLightPos0).xyz);
  tmpvar_3 = tmpvar_7;
  highp vec4 tmpvar_8;
  tmpvar_8.w = 1.0;
  tmpvar_8.xyz = _WorldSpaceCameraPos;
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = (tmpvar_6 * (((_World2Object * tmpvar_8).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = tmpvar_3;
}



#endif
#ifdef FRAGMENT

varying mediump vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp float _FresnelPower;
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
  mediump float tmpvar_5;
  lowp vec4 tmpvar_6;
  tmpvar_6 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_7;
  tmpvar_7 = (tmpvar_6.xyz * _Color.xyz);
  tmpvar_3 = tmpvar_7;
  lowp vec4 tmpvar_8;
  tmpvar_8 = texture2D (_SpecMap, xlv_TEXCOORD0);
  mediump vec3 tmpvar_9;
  tmpvar_9 = ((tmpvar_6.w * tmpvar_8.xyz) * _Gloss);
  lowp vec3 tmpvar_10;
  tmpvar_10 = ((texture2D (_BumpMap, xlv_TEXCOORD0).xyz * 2.0) - 1.0);
  tmpvar_4 = tmpvar_10;
  mediump vec3 tmpvar_11;
  tmpvar_11 = normalize(tmpvar_4);
  highp float tmpvar_12;
  tmpvar_12 = max ((0.20373 + (0.79627 * pow (clamp ((1.0 - max (dot (normalize(xlv_TEXCOORD1), tmpvar_11), 0.0)), 0.0, 1.0), _FresnelPower))), 0.0);
  tmpvar_5 = tmpvar_12;
  lightDir_2 = xlv_TEXCOORD2;
  highp vec3 tmpvar_13;
  tmpvar_13 = normalize(xlv_TEXCOORD1);
  mediump vec3 lightDir_14;
  lightDir_14 = lightDir_2;
  mediump vec3 viewDir_15;
  viewDir_15 = tmpvar_13;
  mediump vec4 c_16;
  mediump vec3 specCol_17;
  highp float nh_18;
  mediump float tmpvar_19;
  tmpvar_19 = max (0.0, dot (tmpvar_4, normalize((lightDir_14 + viewDir_15))));
  nh_18 = tmpvar_19;
  mediump float arg1_20;
  arg1_20 = (32.0 * _Shininess);
  highp vec3 tmpvar_21;
  tmpvar_21 = (pow (nh_18, arg1_20) * tmpvar_9);
  specCol_17 = tmpvar_21;
  c_16.xyz = ((((tmpvar_3 * _LightColor0.xyz) * max (0.0, dot (tmpvar_4, lightDir_14))) + (_LightColor0.xyz * specCol_17)) * 2.0);
  c_16.w = tmpvar_5;
  c_1.xyz = c_16.xyz;
  c_1.w = tmpvar_5;
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
  highp vec3 tmpvar_7;
  tmpvar_7 = (tmpvar_6 * (_World2Object * _WorldSpaceLightPos0).xyz);
  tmpvar_3 = tmpvar_7;
  highp vec4 tmpvar_8;
  tmpvar_8.w = 1.0;
  tmpvar_8.xyz = _WorldSpaceCameraPos;
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = (tmpvar_6 * (((_World2Object * tmpvar_8).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = tmpvar_3;
}



#endif
#ifdef FRAGMENT

varying mediump vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp float _FresnelPower;
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
  mediump float tmpvar_5;
  lowp vec4 tmpvar_6;
  tmpvar_6 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_7;
  tmpvar_7 = (tmpvar_6.xyz * _Color.xyz);
  tmpvar_3 = tmpvar_7;
  lowp vec4 tmpvar_8;
  tmpvar_8 = texture2D (_SpecMap, xlv_TEXCOORD0);
  mediump vec3 tmpvar_9;
  tmpvar_9 = ((tmpvar_6.w * tmpvar_8.xyz) * _Gloss);
  lowp vec3 normal_10;
  normal_10.xy = ((texture2D (_BumpMap, xlv_TEXCOORD0).wy * 2.0) - 1.0);
  normal_10.z = sqrt(((1.0 - (normal_10.x * normal_10.x)) - (normal_10.y * normal_10.y)));
  tmpvar_4 = normal_10;
  mediump vec3 tmpvar_11;
  tmpvar_11 = normalize(tmpvar_4);
  highp float tmpvar_12;
  tmpvar_12 = max ((0.20373 + (0.79627 * pow (clamp ((1.0 - max (dot (normalize(xlv_TEXCOORD1), tmpvar_11), 0.0)), 0.0, 1.0), _FresnelPower))), 0.0);
  tmpvar_5 = tmpvar_12;
  lightDir_2 = xlv_TEXCOORD2;
  highp vec3 tmpvar_13;
  tmpvar_13 = normalize(xlv_TEXCOORD1);
  mediump vec3 lightDir_14;
  lightDir_14 = lightDir_2;
  mediump vec3 viewDir_15;
  viewDir_15 = tmpvar_13;
  mediump vec4 c_16;
  mediump vec3 specCol_17;
  highp float nh_18;
  mediump float tmpvar_19;
  tmpvar_19 = max (0.0, dot (tmpvar_4, normalize((lightDir_14 + viewDir_15))));
  nh_18 = tmpvar_19;
  mediump float arg1_20;
  arg1_20 = (32.0 * _Shininess);
  highp vec3 tmpvar_21;
  tmpvar_21 = (pow (nh_18, arg1_20) * tmpvar_9);
  specCol_17 = tmpvar_21;
  c_16.xyz = ((((tmpvar_3 * _LightColor0.xyz) * max (0.0, dot (tmpvar_4, lightDir_14))) + (_LightColor0.xyz * specCol_17)) * 2.0);
  c_16.w = tmpvar_5;
  c_1.xyz = c_16.xyz;
  c_1.w = tmpvar_5;
  gl_FragData[0] = c_1;
}



#endif"
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
"3.0-!!ARBvp1.0
# 34 ALU
PARAM c[21] = { { 1 },
		state.matrix.mvp,
		program.local[5..20] };
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
"vs_3_0
; 37 ALU
dcl_position o0
dcl_texcoord0 o1
dcl_texcoord1 o2
dcl_texcoord2 o3
dcl_texcoord3 o4
def c20, 1.00000000, 0, 0, 0
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
mov r1.w, c20.x
dp4 r0.w, v0, c7
dp3 o3.y, r2, r0
dp3 o3.z, v2, r0
dp3 o3.x, v1, r0
dp4 r0.z, v0, c6
dp4 r0.x, v0, c4
dp4 r0.y, v0, c5
dp4 r3.z, r1, c10
dp4 r3.x, r1, c8
dp4 r3.y, r1, c9
mad r1.xyz, r3, c18.w, -v0
dp3 o2.y, r1, r2
dp3 o2.z, v2, r1
dp3 o2.x, r1, v1
dp4 o4.w, r0, c15
dp4 o4.z, r0, c14
dp4 o4.y, r0, c13
dp4 o4.x, r0, c12
mad o1.xy, v3, c19, c19.zwzw
dp4 o0.w, v0, c3
dp4 o0.z, v0, c2
dp4 o0.y, v0, c1
dp4 o0.x, v0, c0
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
bcaaaaaaaaaadacfaaaaccaaaaaaaaaaafpidaaaaaaaagiiaaaaaaaaafpifaaa
aaaaagiiaaaaaaaaafpicaaaaaaaaoiiaaaaaaaaafpiaaaaaaaaapmiaaaaaaaa
miapaaabaabliiaakbadafaamiapaaabaamgiiaakladaeabmiapaaabaalbdeje
kladadabmiapiadoaagmaadekladacabmiahaaabaaleblaacbanabaamiahaaag
aamamgmaalamaaanmiahaaaeaalogfaaobacafaamiahaaagaalelbleclalaaag
miahaaahaamamgleclamababmiapaaabaabliiaakbadajaamiapaaabaamgiiaa
kladaiabmiahaaahaalelbleclalabahmiahaaagaamagmleclakaaagmiahaaae
abgflomaolacafaemiahaaaeaamablaaobaeafaamiahaaagabmablmaklagaoad
miahaaahaamagmleclakabahmiapaaabaalbdejekladahabmiapaaabaagmnaje
kladagabmiahaaadabmablmaklahaoadmiabiaabaaloloaapaagafaamiaciaab
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
00009c6c005d200d8186c0836041fffc00019c6c00400e0c0106c0836041dffc
00011c6c005d300c0186c0836041dffc401f9c6c011d0808010400d740619f9c
401f9c6c01d0300d8106c0c360403f80401f9c6c01d0200d8106c0c360405f80
401f9c6c01d0100d8106c0c360409f80401f9c6c01d0000d8106c0c360411f80
00001c6c01d0700d8106c0c360403ffc00001c6c01d0600d8106c0c360405ffc
00001c6c01d0500d8106c0c360409ffc00001c6c01d0400d8106c0c360411ffc
00021c6c01d0a00d8286c0c360405ffc00021c6c01d0900d8286c0c360409ffc
00021c6c01d0800d8286c0c360411ffc00009c6c0190a00c0486c0c360405ffc
00009c6c0190900c0486c0c360409ffc00009c6c0190800c0486c0c360411ffc
00011c6c00800243011843436041dffc00011c6c01000230812183630121dffc
401f9c6c01d0f00d8086c0c360403fa8401f9c6c01d0e00d8086c0c360405fa8
401f9c6c01d0d00d8086c0c360409fa8401f9c6c01d0c00d8086c0c360411fa8
00001c6c011d100c08bfc0e30041dffc00009c6c011d100c02bfc0e30041dffc
401f9c6c0140020c0106004360405fa4401f9c6c01400e0c0106004360411fa4
00011c6c00800e0c04bfc0836041dffc401f9c6c0140020c0106014360405fa0
401f9c6c01400e0c0286008360411fa0401f9c6c0140000c0486004360409fa4
401f9c6c0140000c0286024360409fa1
"
}

SubProgram "d3d11 " {
Keywords { "SPOT" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 176 // 176 used size, 11 vars
Matrix 48 [_LightMatrix0] 4
Vector 160 [_MainTex_ST] 4
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
eefiecedafjngjilpbigffcofhlkincdfhjlnicnabaaaaaapiagaaaaadaaaaaa
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
fhabaaaafjaaaaaeegiocaaaaaaaaaaaalaaaaaafjaaaaaeegiocaaaabaaaaaa
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
egbabaaaadaaaaaaegiacaaaaaaaaaaaakaaaaaaogikcaaaaaaaaaaaakaaaaaa
diaaaaahhcaabaaaaaaaaaaajgbebaaaabaaaaaacgbjbaaaacaaaaaadcaaaaak
hcaabaaaaaaaaaaajgbebaaaacaaaaaacgbjbaaaabaaaaaaegacbaiaebaaaaaa
aaaaaaaadiaaaaahhcaabaaaaaaaaaaaegacbaaaaaaaaaaapgbpbaaaabaaaaaa
diaaaaajhcaabaaaabaaaaaafgifcaaaabaaaaaaaeaaaaaaegiccaaaadaaaaaa
bbaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaa
abaaaaaaaeaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaa
adaaaaaabcaaaaaakgikcaaaabaaaaaaaeaaaaaaegacbaaaabaaaaaaaaaaaaai
hcaabaaaabaaaaaaegacbaaaabaaaaaaegiccaaaadaaaaaabdaaaaaadcaaaaal
hcaabaaaabaaaaaaegacbaaaabaaaaaapgipcaaaadaaaaaabeaaaaaaegbcbaia
ebaaaaaaaaaaaaaabaaaaaahcccabaaaacaaaaaaegacbaaaaaaaaaaaegacbaaa
abaaaaaabaaaaaahbccabaaaacaaaaaaegbcbaaaabaaaaaaegacbaaaabaaaaaa
baaaaaaheccabaaaacaaaaaaegbcbaaaacaaaaaaegacbaaaabaaaaaadiaaaaaj
hcaabaaaabaaaaaafgifcaaaacaaaaaaaaaaaaaaegiccaaaadaaaaaabbaaaaaa
dcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaaacaaaaaa
aaaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaa
bcaaaaaakgikcaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaa
abaaaaaaegiccaaaadaaaaaabdaaaaaapgipcaaaacaaaaaaaaaaaaaaegacbaaa
abaaaaaadcaaaaalhcaabaaaabaaaaaaegacbaaaabaaaaaapgipcaaaadaaaaaa
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
varying highp vec3 xlv_TEXCOORD1;
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
  highp vec3 tmpvar_7;
  tmpvar_7 = (tmpvar_6 * (((_World2Object * _WorldSpaceLightPos0).xyz * unity_Scale.w) - _glesVertex.xyz));
  tmpvar_3 = tmpvar_7;
  highp vec4 tmpvar_8;
  tmpvar_8.w = 1.0;
  tmpvar_8.xyz = _WorldSpaceCameraPos;
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = (tmpvar_6 * (((_World2Object * tmpvar_8).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = tmpvar_3;
  xlv_TEXCOORD3 = (_LightMatrix0 * (_Object2World * _glesVertex));
}



#endif
#ifdef FRAGMENT

varying highp vec4 xlv_TEXCOORD3;
varying mediump vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp float _FresnelPower;
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
  mediump float tmpvar_5;
  lowp vec4 tmpvar_6;
  tmpvar_6 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_7;
  tmpvar_7 = (tmpvar_6.xyz * _Color.xyz);
  tmpvar_3 = tmpvar_7;
  lowp vec4 tmpvar_8;
  tmpvar_8 = texture2D (_SpecMap, xlv_TEXCOORD0);
  mediump vec3 tmpvar_9;
  tmpvar_9 = ((tmpvar_6.w * tmpvar_8.xyz) * _Gloss);
  lowp vec3 tmpvar_10;
  tmpvar_10 = ((texture2D (_BumpMap, xlv_TEXCOORD0).xyz * 2.0) - 1.0);
  tmpvar_4 = tmpvar_10;
  mediump vec3 tmpvar_11;
  tmpvar_11 = normalize(tmpvar_4);
  highp float tmpvar_12;
  tmpvar_12 = max ((0.20373 + (0.79627 * pow (clamp ((1.0 - max (dot (normalize(xlv_TEXCOORD1), tmpvar_11), 0.0)), 0.0, 1.0), _FresnelPower))), 0.0);
  tmpvar_5 = tmpvar_12;
  mediump vec3 tmpvar_13;
  tmpvar_13 = normalize(xlv_TEXCOORD2);
  lightDir_2 = tmpvar_13;
  highp vec3 tmpvar_14;
  tmpvar_14 = normalize(xlv_TEXCOORD1);
  lowp vec4 tmpvar_15;
  highp vec2 P_16;
  P_16 = ((xlv_TEXCOORD3.xy / xlv_TEXCOORD3.w) + 0.5);
  tmpvar_15 = texture2D (_LightTexture0, P_16);
  highp float tmpvar_17;
  tmpvar_17 = dot (xlv_TEXCOORD3.xyz, xlv_TEXCOORD3.xyz);
  lowp vec4 tmpvar_18;
  tmpvar_18 = texture2D (_LightTextureB0, vec2(tmpvar_17));
  mediump vec3 lightDir_19;
  lightDir_19 = lightDir_2;
  mediump vec3 viewDir_20;
  viewDir_20 = tmpvar_14;
  mediump float atten_21;
  atten_21 = ((float((xlv_TEXCOORD3.z > 0.0)) * tmpvar_15.w) * tmpvar_18.w);
  mediump vec4 c_22;
  mediump vec3 specCol_23;
  highp float nh_24;
  mediump float tmpvar_25;
  tmpvar_25 = max (0.0, dot (tmpvar_4, normalize((lightDir_19 + viewDir_20))));
  nh_24 = tmpvar_25;
  mediump float arg1_26;
  arg1_26 = (32.0 * _Shininess);
  highp vec3 tmpvar_27;
  tmpvar_27 = (pow (nh_24, arg1_26) * tmpvar_9);
  specCol_23 = tmpvar_27;
  c_22.xyz = ((((tmpvar_3 * _LightColor0.xyz) * max (0.0, dot (tmpvar_4, lightDir_19))) + (_LightColor0.xyz * specCol_23)) * (atten_21 * 2.0));
  c_22.w = tmpvar_5;
  c_1.xyz = c_22.xyz;
  c_1.w = tmpvar_5;
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
  highp vec3 tmpvar_7;
  tmpvar_7 = (tmpvar_6 * (((_World2Object * _WorldSpaceLightPos0).xyz * unity_Scale.w) - _glesVertex.xyz));
  tmpvar_3 = tmpvar_7;
  highp vec4 tmpvar_8;
  tmpvar_8.w = 1.0;
  tmpvar_8.xyz = _WorldSpaceCameraPos;
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = (tmpvar_6 * (((_World2Object * tmpvar_8).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = tmpvar_3;
  xlv_TEXCOORD3 = (_LightMatrix0 * (_Object2World * _glesVertex));
}



#endif
#ifdef FRAGMENT

varying highp vec4 xlv_TEXCOORD3;
varying mediump vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp float _FresnelPower;
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
  mediump float tmpvar_5;
  lowp vec4 tmpvar_6;
  tmpvar_6 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_7;
  tmpvar_7 = (tmpvar_6.xyz * _Color.xyz);
  tmpvar_3 = tmpvar_7;
  lowp vec4 tmpvar_8;
  tmpvar_8 = texture2D (_SpecMap, xlv_TEXCOORD0);
  mediump vec3 tmpvar_9;
  tmpvar_9 = ((tmpvar_6.w * tmpvar_8.xyz) * _Gloss);
  lowp vec3 normal_10;
  normal_10.xy = ((texture2D (_BumpMap, xlv_TEXCOORD0).wy * 2.0) - 1.0);
  normal_10.z = sqrt(((1.0 - (normal_10.x * normal_10.x)) - (normal_10.y * normal_10.y)));
  tmpvar_4 = normal_10;
  mediump vec3 tmpvar_11;
  tmpvar_11 = normalize(tmpvar_4);
  highp float tmpvar_12;
  tmpvar_12 = max ((0.20373 + (0.79627 * pow (clamp ((1.0 - max (dot (normalize(xlv_TEXCOORD1), tmpvar_11), 0.0)), 0.0, 1.0), _FresnelPower))), 0.0);
  tmpvar_5 = tmpvar_12;
  mediump vec3 tmpvar_13;
  tmpvar_13 = normalize(xlv_TEXCOORD2);
  lightDir_2 = tmpvar_13;
  highp vec3 tmpvar_14;
  tmpvar_14 = normalize(xlv_TEXCOORD1);
  lowp vec4 tmpvar_15;
  highp vec2 P_16;
  P_16 = ((xlv_TEXCOORD3.xy / xlv_TEXCOORD3.w) + 0.5);
  tmpvar_15 = texture2D (_LightTexture0, P_16);
  highp float tmpvar_17;
  tmpvar_17 = dot (xlv_TEXCOORD3.xyz, xlv_TEXCOORD3.xyz);
  lowp vec4 tmpvar_18;
  tmpvar_18 = texture2D (_LightTextureB0, vec2(tmpvar_17));
  mediump vec3 lightDir_19;
  lightDir_19 = lightDir_2;
  mediump vec3 viewDir_20;
  viewDir_20 = tmpvar_14;
  mediump float atten_21;
  atten_21 = ((float((xlv_TEXCOORD3.z > 0.0)) * tmpvar_15.w) * tmpvar_18.w);
  mediump vec4 c_22;
  mediump vec3 specCol_23;
  highp float nh_24;
  mediump float tmpvar_25;
  tmpvar_25 = max (0.0, dot (tmpvar_4, normalize((lightDir_19 + viewDir_20))));
  nh_24 = tmpvar_25;
  mediump float arg1_26;
  arg1_26 = (32.0 * _Shininess);
  highp vec3 tmpvar_27;
  tmpvar_27 = (pow (nh_24, arg1_26) * tmpvar_9);
  specCol_23 = tmpvar_27;
  c_22.xyz = ((((tmpvar_3 * _LightColor0.xyz) * max (0.0, dot (tmpvar_4, lightDir_19))) + (_LightColor0.xyz * specCol_23)) * (atten_21 * 2.0));
  c_22.w = tmpvar_5;
  c_1.xyz = c_22.xyz;
  c_1.w = tmpvar_5;
  gl_FragData[0] = c_1;
}



#endif"
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
"3.0-!!ARBvp1.0
# 33 ALU
PARAM c[21] = { { 1 },
		state.matrix.mvp,
		program.local[5..20] };
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
"vs_3_0
; 36 ALU
dcl_position o0
dcl_texcoord0 o1
dcl_texcoord1 o2
dcl_texcoord2 o3
dcl_texcoord3 o4
def c20, 1.00000000, 0, 0, 0
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
mov r1.w, c20.x
dp3 o3.y, r2, r0
dp3 o3.z, v2, r0
dp3 o3.x, v1, r0
dp4 r0.w, v0, c7
dp4 r0.z, v0, c6
dp4 r0.x, v0, c4
dp4 r0.y, v0, c5
dp4 r3.z, r1, c10
dp4 r3.x, r1, c8
dp4 r3.y, r1, c9
mad r1.xyz, r3, c18.w, -v0
dp3 o2.y, r1, r2
dp3 o2.z, v2, r1
dp3 o2.x, r1, v1
dp4 o4.z, r0, c14
dp4 o4.y, r0, c13
dp4 o4.x, r0, c12
mad o1.xy, v3, c19, c19.zwzw
dp4 o0.w, v0, c3
dp4 o0.z, v0, c2
dp4 o0.y, v0, c1
dp4 o0.x, v0, c0
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
bcaaaaaaaaaadacfaaaaccaaaaaaaaaaafpidaaaaaaaagiiaaaaaaaaafpifaaa
aaaaagiiaaaaaaaaafpicaaaaaaaaoiiaaaaaaaaafpibaaaaaaaapmiaaaaaaaa
miapaaaaaabliiaakbadafaamiapaaaaaamgiiaakladaeaamiapaaaaaalbdeje
kladadaamiapiadoaagmaadekladacaamiahaaaaaaleblaacbanabaamiahaaag
aamamgmaalamaaanmiahaaaeaalogfaaobacafaamiahaaagaalelbleclalaaag
miahaaahaamamgleclamabaamiapaaaaaabliiaakbadajaamiapaaaaaamgiiaa
kladaiaamiahaaahaalelbleclalabahmiahaaagaamagmleclakaaagmiahaaae
abgflomaolacafaemiahaaaeaamablaaobaeafaamiahaaagabmablmaklagaoad
miahaaahaamagmleclakabahmiapaaaaaalbdejekladahaamiapaaaaaagmejhk
kladagaamiahaaadabmablmaklahaoadmiabiaabaaloloaapaagafaamiaciaab
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
00009c6c005d200d8186c0836041fffc00019c6c00400e0c0106c0836041dffc
00011c6c005d300c0186c0836041dffc401f9c6c011d0808010400d740619f9c
401f9c6c01d0300d8106c0c360403f80401f9c6c01d0200d8106c0c360405f80
401f9c6c01d0100d8106c0c360409f80401f9c6c01d0000d8106c0c360411f80
00001c6c01d0700d8106c0c360403ffc00001c6c01d0600d8106c0c360405ffc
00001c6c01d0500d8106c0c360409ffc00001c6c01d0400d8106c0c360411ffc
00021c6c01d0a00d8286c0c360405ffc00021c6c01d0900d8286c0c360409ffc
00021c6c01d0800d8286c0c360411ffc00009c6c0190a00c0486c0c360405ffc
00009c6c0190900c0486c0c360409ffc00009c6c0190800c0486c0c360411ffc
00011c6c00800243011843436041dffc00011c6c01000230812183630121dffc
401f9c6c01d0e00d8086c0c360405fa8401f9c6c01d0d00d8086c0c360409fa8
401f9c6c01d0c00d8086c0c360411fa800001c6c011d100c08bfc0e30041dffc
00009c6c011d100c02bfc0e30041dffc401f9c6c0140020c0106004360405fa4
401f9c6c01400e0c0106004360411fa400011c6c00800e0c04bfc0836041dffc
401f9c6c0140020c0106014360405fa0401f9c6c01400e0c0286008360411fa0
401f9c6c0140000c0486004360409fa4401f9c6c0140000c0286024360409fa1
"
}

SubProgram "d3d11 " {
Keywords { "POINT_COOKIE" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 176 // 176 used size, 11 vars
Matrix 48 [_LightMatrix0] 4
Vector 160 [_MainTex_ST] 4
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
eefiecedkhafdghledbaandiffiakgoaiikhgdafabaaaaaapiagaaaaadaaaaaa
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
fhabaaaafjaaaaaeegiocaaaaaaaaaaaalaaaaaafjaaaaaeegiocaaaabaaaaaa
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
egbabaaaadaaaaaaegiacaaaaaaaaaaaakaaaaaaogikcaaaaaaaaaaaakaaaaaa
diaaaaahhcaabaaaaaaaaaaajgbebaaaabaaaaaacgbjbaaaacaaaaaadcaaaaak
hcaabaaaaaaaaaaajgbebaaaacaaaaaacgbjbaaaabaaaaaaegacbaiaebaaaaaa
aaaaaaaadiaaaaahhcaabaaaaaaaaaaaegacbaaaaaaaaaaapgbpbaaaabaaaaaa
diaaaaajhcaabaaaabaaaaaafgifcaaaabaaaaaaaeaaaaaaegiccaaaadaaaaaa
bbaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaa
abaaaaaaaeaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaa
adaaaaaabcaaaaaakgikcaaaabaaaaaaaeaaaaaaegacbaaaabaaaaaaaaaaaaai
hcaabaaaabaaaaaaegacbaaaabaaaaaaegiccaaaadaaaaaabdaaaaaadcaaaaal
hcaabaaaabaaaaaaegacbaaaabaaaaaapgipcaaaadaaaaaabeaaaaaaegbcbaia
ebaaaaaaaaaaaaaabaaaaaahcccabaaaacaaaaaaegacbaaaaaaaaaaaegacbaaa
abaaaaaabaaaaaahbccabaaaacaaaaaaegbcbaaaabaaaaaaegacbaaaabaaaaaa
baaaaaaheccabaaaacaaaaaaegbcbaaaacaaaaaaegacbaaaabaaaaaadiaaaaaj
hcaabaaaabaaaaaafgifcaaaacaaaaaaaaaaaaaaegiccaaaadaaaaaabbaaaaaa
dcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaaacaaaaaa
aaaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaa
bcaaaaaakgikcaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaa
abaaaaaaegiccaaaadaaaaaabdaaaaaapgipcaaaacaaaaaaaaaaaaaaegacbaaa
abaaaaaadcaaaaalhcaabaaaabaaaaaaegacbaaaabaaaaaapgipcaaaadaaaaaa
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
varying highp vec3 xlv_TEXCOORD1;
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
  highp vec3 tmpvar_7;
  tmpvar_7 = (tmpvar_6 * (((_World2Object * _WorldSpaceLightPos0).xyz * unity_Scale.w) - _glesVertex.xyz));
  tmpvar_3 = tmpvar_7;
  highp vec4 tmpvar_8;
  tmpvar_8.w = 1.0;
  tmpvar_8.xyz = _WorldSpaceCameraPos;
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = (tmpvar_6 * (((_World2Object * tmpvar_8).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = tmpvar_3;
  xlv_TEXCOORD3 = (_LightMatrix0 * (_Object2World * _glesVertex)).xyz;
}



#endif
#ifdef FRAGMENT

varying highp vec3 xlv_TEXCOORD3;
varying mediump vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp float _FresnelPower;
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
  mediump float tmpvar_5;
  lowp vec4 tmpvar_6;
  tmpvar_6 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_7;
  tmpvar_7 = (tmpvar_6.xyz * _Color.xyz);
  tmpvar_3 = tmpvar_7;
  lowp vec4 tmpvar_8;
  tmpvar_8 = texture2D (_SpecMap, xlv_TEXCOORD0);
  mediump vec3 tmpvar_9;
  tmpvar_9 = ((tmpvar_6.w * tmpvar_8.xyz) * _Gloss);
  lowp vec3 tmpvar_10;
  tmpvar_10 = ((texture2D (_BumpMap, xlv_TEXCOORD0).xyz * 2.0) - 1.0);
  tmpvar_4 = tmpvar_10;
  mediump vec3 tmpvar_11;
  tmpvar_11 = normalize(tmpvar_4);
  highp float tmpvar_12;
  tmpvar_12 = max ((0.20373 + (0.79627 * pow (clamp ((1.0 - max (dot (normalize(xlv_TEXCOORD1), tmpvar_11), 0.0)), 0.0, 1.0), _FresnelPower))), 0.0);
  tmpvar_5 = tmpvar_12;
  mediump vec3 tmpvar_13;
  tmpvar_13 = normalize(xlv_TEXCOORD2);
  lightDir_2 = tmpvar_13;
  highp vec3 tmpvar_14;
  tmpvar_14 = normalize(xlv_TEXCOORD1);
  highp float tmpvar_15;
  tmpvar_15 = dot (xlv_TEXCOORD3, xlv_TEXCOORD3);
  lowp vec4 tmpvar_16;
  tmpvar_16 = texture2D (_LightTextureB0, vec2(tmpvar_15));
  lowp vec4 tmpvar_17;
  tmpvar_17 = textureCube (_LightTexture0, xlv_TEXCOORD3);
  mediump vec3 lightDir_18;
  lightDir_18 = lightDir_2;
  mediump vec3 viewDir_19;
  viewDir_19 = tmpvar_14;
  mediump float atten_20;
  atten_20 = (tmpvar_16.w * tmpvar_17.w);
  mediump vec4 c_21;
  mediump vec3 specCol_22;
  highp float nh_23;
  mediump float tmpvar_24;
  tmpvar_24 = max (0.0, dot (tmpvar_4, normalize((lightDir_18 + viewDir_19))));
  nh_23 = tmpvar_24;
  mediump float arg1_25;
  arg1_25 = (32.0 * _Shininess);
  highp vec3 tmpvar_26;
  tmpvar_26 = (pow (nh_23, arg1_25) * tmpvar_9);
  specCol_22 = tmpvar_26;
  c_21.xyz = ((((tmpvar_3 * _LightColor0.xyz) * max (0.0, dot (tmpvar_4, lightDir_18))) + (_LightColor0.xyz * specCol_22)) * (atten_20 * 2.0));
  c_21.w = tmpvar_5;
  c_1.xyz = c_21.xyz;
  c_1.w = tmpvar_5;
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
  highp vec3 tmpvar_7;
  tmpvar_7 = (tmpvar_6 * (((_World2Object * _WorldSpaceLightPos0).xyz * unity_Scale.w) - _glesVertex.xyz));
  tmpvar_3 = tmpvar_7;
  highp vec4 tmpvar_8;
  tmpvar_8.w = 1.0;
  tmpvar_8.xyz = _WorldSpaceCameraPos;
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = (tmpvar_6 * (((_World2Object * tmpvar_8).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = tmpvar_3;
  xlv_TEXCOORD3 = (_LightMatrix0 * (_Object2World * _glesVertex)).xyz;
}



#endif
#ifdef FRAGMENT

varying highp vec3 xlv_TEXCOORD3;
varying mediump vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp float _FresnelPower;
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
  mediump float tmpvar_5;
  lowp vec4 tmpvar_6;
  tmpvar_6 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_7;
  tmpvar_7 = (tmpvar_6.xyz * _Color.xyz);
  tmpvar_3 = tmpvar_7;
  lowp vec4 tmpvar_8;
  tmpvar_8 = texture2D (_SpecMap, xlv_TEXCOORD0);
  mediump vec3 tmpvar_9;
  tmpvar_9 = ((tmpvar_6.w * tmpvar_8.xyz) * _Gloss);
  lowp vec3 normal_10;
  normal_10.xy = ((texture2D (_BumpMap, xlv_TEXCOORD0).wy * 2.0) - 1.0);
  normal_10.z = sqrt(((1.0 - (normal_10.x * normal_10.x)) - (normal_10.y * normal_10.y)));
  tmpvar_4 = normal_10;
  mediump vec3 tmpvar_11;
  tmpvar_11 = normalize(tmpvar_4);
  highp float tmpvar_12;
  tmpvar_12 = max ((0.20373 + (0.79627 * pow (clamp ((1.0 - max (dot (normalize(xlv_TEXCOORD1), tmpvar_11), 0.0)), 0.0, 1.0), _FresnelPower))), 0.0);
  tmpvar_5 = tmpvar_12;
  mediump vec3 tmpvar_13;
  tmpvar_13 = normalize(xlv_TEXCOORD2);
  lightDir_2 = tmpvar_13;
  highp vec3 tmpvar_14;
  tmpvar_14 = normalize(xlv_TEXCOORD1);
  highp float tmpvar_15;
  tmpvar_15 = dot (xlv_TEXCOORD3, xlv_TEXCOORD3);
  lowp vec4 tmpvar_16;
  tmpvar_16 = texture2D (_LightTextureB0, vec2(tmpvar_15));
  lowp vec4 tmpvar_17;
  tmpvar_17 = textureCube (_LightTexture0, xlv_TEXCOORD3);
  mediump vec3 lightDir_18;
  lightDir_18 = lightDir_2;
  mediump vec3 viewDir_19;
  viewDir_19 = tmpvar_14;
  mediump float atten_20;
  atten_20 = (tmpvar_16.w * tmpvar_17.w);
  mediump vec4 c_21;
  mediump vec3 specCol_22;
  highp float nh_23;
  mediump float tmpvar_24;
  tmpvar_24 = max (0.0, dot (tmpvar_4, normalize((lightDir_18 + viewDir_19))));
  nh_23 = tmpvar_24;
  mediump float arg1_25;
  arg1_25 = (32.0 * _Shininess);
  highp vec3 tmpvar_26;
  tmpvar_26 = (pow (nh_23, arg1_25) * tmpvar_9);
  specCol_22 = tmpvar_26;
  c_21.xyz = ((((tmpvar_3 * _LightColor0.xyz) * max (0.0, dot (tmpvar_4, lightDir_18))) + (_LightColor0.xyz * specCol_22)) * (atten_20 * 2.0));
  c_21.w = tmpvar_5;
  c_1.xyz = c_21.xyz;
  c_1.w = tmpvar_5;
  gl_FragData[0] = c_1;
}



#endif"
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
"3.0-!!ARBvp1.0
# 31 ALU
PARAM c[21] = { { 1 },
		state.matrix.mvp,
		program.local[5..20] };
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
"vs_3_0
; 34 ALU
dcl_position o0
dcl_texcoord0 o1
dcl_texcoord1 o2
dcl_texcoord2 o3
dcl_texcoord3 o4
def c20, 1.00000000, 0, 0, 0
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
mov r1.w, c20.x
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
dp3 o2.y, r2, r3
dp3 o3.y, r3, r4
dp3 o2.z, v2, r2
dp3 o2.x, r2, v1
dp3 o3.z, v2, r4
dp3 o3.x, v1, r4
dp4 o4.y, r0, c13
dp4 o4.x, r0, c12
mad o1.xy, v3, c19, c19.zwzw
dp4 o0.w, v0, c3
dp4 o0.z, v0, c2
dp4 o0.y, v0, c1
dp4 o0.x, v0, c0
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
klahadaamiapiadoaagmaadeklahacaamiahaaaaaamamgmaalamaaanmiahaaad
aaleblaacbanabaamiahaaadaamamgleclamabadmiahaaaeaalogfaaobacafaa
miahaaagaalelbleclalaaaamiapaaaaaabliiaakbahajaamiapaaaaaamgiiaa
klahaiaamiahaaagaamagmleclakaaagmiahaaaeabgflomaolacafaemiahaaad
aalelbleclalabadmiahaaadaamagmleclakabadmiahaaaeaamablaaobaeafaa
miahaaagabmablmaklagaoahmiapaaaaaalbdejeklahahaamiapaaaaaagmojkk
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
00009c6c005d200d8186c0836041fffc00019c6c00400e0c0106c0836041dffc
00011c6c005d300c0186c0836041dffc401f9c6c011d0808010400d740619f9c
401f9c6c01d0300d8106c0c360403f80401f9c6c01d0200d8106c0c360405f80
401f9c6c01d0100d8106c0c360409f80401f9c6c01d0000d8106c0c360411f80
00001c6c01d0700d8106c0c360403ffc00001c6c01d0600d8106c0c360405ffc
00001c6c01d0500d8106c0c360409ffc00001c6c01d0400d8106c0c360411ffc
00021c6c01d0a00d8286c0c360405ffc00021c6c01d0900d8286c0c360409ffc
00021c6c01d0800d8286c0c360411ffc00009c6c0190a00c0486c0c360405ffc
00009c6c0190900c0486c0c360409ffc00009c6c0190800c0486c0c360411ffc
00011c6c00800243011843436041dffc00011c6c01000230812183630121dffc
401f9c6c01d0d00d8086c0c360409fa8401f9c6c01d0c00d8086c0c360411fa8
00001c6c011d100c02bfc0e30041dffc401f9c6c0140020c0106044360405fa4
401f9c6c01400e0c0106044360411fa400009c6c00800e0c04bfc0836041dffc
401f9c6c0140020c0106004360405fa0401f9c6c01400e0c0086008360411fa0
401f9c6c0140000c0286044360409fa4401f9c6c0140000c0086014360409fa1
"
}

SubProgram "d3d11 " {
Keywords { "DIRECTIONAL_COOKIE" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "color" Color
ConstBuffer "$Globals" 176 // 176 used size, 11 vars
Matrix 48 [_LightMatrix0] 4
Vector 160 [_MainTex_ST] 4
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
eefiecedbdkhacpbpkppficoknmibaoblkcgllidabaaaaaammagaaaaadaaaaaa
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
emabaaaafjaaaaaeegiocaaaaaaaaaaaalaaaaaafjaaaaaeegiocaaaabaaaaaa
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
akaaaaaaogikcaaaaaaaaaaaakaaaaaadiaaaaahhcaabaaaaaaaaaaajgbebaaa
abaaaaaacgbjbaaaacaaaaaadcaaaaakhcaabaaaaaaaaaaajgbebaaaacaaaaaa
cgbjbaaaabaaaaaaegacbaiaebaaaaaaaaaaaaaadiaaaaahhcaabaaaaaaaaaaa
egacbaaaaaaaaaaapgbpbaaaabaaaaaadiaaaaajhcaabaaaabaaaaaafgifcaaa
abaaaaaaaeaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaaabaaaaaa
egiccaaaadaaaaaabaaaaaaaagiacaaaabaaaaaaaeaaaaaaegacbaaaabaaaaaa
dcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaaabaaaaaa
aeaaaaaaegacbaaaabaaaaaaaaaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaa
egiccaaaadaaaaaabdaaaaaadcaaaaalhcaabaaaabaaaaaaegacbaaaabaaaaaa
pgipcaaaadaaaaaabeaaaaaaegbcbaiaebaaaaaaaaaaaaaabaaaaaahcccabaaa
acaaaaaaegacbaaaaaaaaaaaegacbaaaabaaaaaabaaaaaahbccabaaaacaaaaaa
egbcbaaaabaaaaaaegacbaaaabaaaaaabaaaaaaheccabaaaacaaaaaaegbcbaaa
acaaaaaaegacbaaaabaaaaaadiaaaaajhcaabaaaabaaaaaafgifcaaaacaaaaaa
aaaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaa
adaaaaaabaaaaaaaagiacaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaadcaaaaal
hcaabaaaabaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaaacaaaaaaaaaaaaaa
egacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabdaaaaaa
pgipcaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaabaaaaaahcccabaaaadaaaaaa
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
varying highp vec3 xlv_TEXCOORD1;
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
  highp vec3 tmpvar_7;
  tmpvar_7 = (tmpvar_6 * (_World2Object * _WorldSpaceLightPos0).xyz);
  tmpvar_3 = tmpvar_7;
  highp vec4 tmpvar_8;
  tmpvar_8.w = 1.0;
  tmpvar_8.xyz = _WorldSpaceCameraPos;
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = (tmpvar_6 * (((_World2Object * tmpvar_8).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = tmpvar_3;
  xlv_TEXCOORD3 = (_LightMatrix0 * (_Object2World * _glesVertex)).xy;
}



#endif
#ifdef FRAGMENT

varying highp vec2 xlv_TEXCOORD3;
varying mediump vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp float _FresnelPower;
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
  mediump float tmpvar_5;
  lowp vec4 tmpvar_6;
  tmpvar_6 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_7;
  tmpvar_7 = (tmpvar_6.xyz * _Color.xyz);
  tmpvar_3 = tmpvar_7;
  lowp vec4 tmpvar_8;
  tmpvar_8 = texture2D (_SpecMap, xlv_TEXCOORD0);
  mediump vec3 tmpvar_9;
  tmpvar_9 = ((tmpvar_6.w * tmpvar_8.xyz) * _Gloss);
  lowp vec3 tmpvar_10;
  tmpvar_10 = ((texture2D (_BumpMap, xlv_TEXCOORD0).xyz * 2.0) - 1.0);
  tmpvar_4 = tmpvar_10;
  mediump vec3 tmpvar_11;
  tmpvar_11 = normalize(tmpvar_4);
  highp float tmpvar_12;
  tmpvar_12 = max ((0.20373 + (0.79627 * pow (clamp ((1.0 - max (dot (normalize(xlv_TEXCOORD1), tmpvar_11), 0.0)), 0.0, 1.0), _FresnelPower))), 0.0);
  tmpvar_5 = tmpvar_12;
  lightDir_2 = xlv_TEXCOORD2;
  highp vec3 tmpvar_13;
  tmpvar_13 = normalize(xlv_TEXCOORD1);
  lowp vec4 tmpvar_14;
  tmpvar_14 = texture2D (_LightTexture0, xlv_TEXCOORD3);
  mediump vec3 lightDir_15;
  lightDir_15 = lightDir_2;
  mediump vec3 viewDir_16;
  viewDir_16 = tmpvar_13;
  mediump float atten_17;
  atten_17 = tmpvar_14.w;
  mediump vec4 c_18;
  mediump vec3 specCol_19;
  highp float nh_20;
  mediump float tmpvar_21;
  tmpvar_21 = max (0.0, dot (tmpvar_4, normalize((lightDir_15 + viewDir_16))));
  nh_20 = tmpvar_21;
  mediump float arg1_22;
  arg1_22 = (32.0 * _Shininess);
  highp vec3 tmpvar_23;
  tmpvar_23 = (pow (nh_20, arg1_22) * tmpvar_9);
  specCol_19 = tmpvar_23;
  c_18.xyz = ((((tmpvar_3 * _LightColor0.xyz) * max (0.0, dot (tmpvar_4, lightDir_15))) + (_LightColor0.xyz * specCol_19)) * (atten_17 * 2.0));
  c_18.w = tmpvar_5;
  c_1.xyz = c_18.xyz;
  c_1.w = tmpvar_5;
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
  highp vec3 tmpvar_7;
  tmpvar_7 = (tmpvar_6 * (_World2Object * _WorldSpaceLightPos0).xyz);
  tmpvar_3 = tmpvar_7;
  highp vec4 tmpvar_8;
  tmpvar_8.w = 1.0;
  tmpvar_8.xyz = _WorldSpaceCameraPos;
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = (tmpvar_6 * (((_World2Object * tmpvar_8).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD2 = tmpvar_3;
  xlv_TEXCOORD3 = (_LightMatrix0 * (_Object2World * _glesVertex)).xy;
}



#endif
#ifdef FRAGMENT

varying highp vec2 xlv_TEXCOORD3;
varying mediump vec3 xlv_TEXCOORD2;
varying highp vec3 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp float _FresnelPower;
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
  mediump float tmpvar_5;
  lowp vec4 tmpvar_6;
  tmpvar_6 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_7;
  tmpvar_7 = (tmpvar_6.xyz * _Color.xyz);
  tmpvar_3 = tmpvar_7;
  lowp vec4 tmpvar_8;
  tmpvar_8 = texture2D (_SpecMap, xlv_TEXCOORD0);
  mediump vec3 tmpvar_9;
  tmpvar_9 = ((tmpvar_6.w * tmpvar_8.xyz) * _Gloss);
  lowp vec3 normal_10;
  normal_10.xy = ((texture2D (_BumpMap, xlv_TEXCOORD0).wy * 2.0) - 1.0);
  normal_10.z = sqrt(((1.0 - (normal_10.x * normal_10.x)) - (normal_10.y * normal_10.y)));
  tmpvar_4 = normal_10;
  mediump vec3 tmpvar_11;
  tmpvar_11 = normalize(tmpvar_4);
  highp float tmpvar_12;
  tmpvar_12 = max ((0.20373 + (0.79627 * pow (clamp ((1.0 - max (dot (normalize(xlv_TEXCOORD1), tmpvar_11), 0.0)), 0.0, 1.0), _FresnelPower))), 0.0);
  tmpvar_5 = tmpvar_12;
  lightDir_2 = xlv_TEXCOORD2;
  highp vec3 tmpvar_13;
  tmpvar_13 = normalize(xlv_TEXCOORD1);
  lowp vec4 tmpvar_14;
  tmpvar_14 = texture2D (_LightTexture0, xlv_TEXCOORD3);
  mediump vec3 lightDir_15;
  lightDir_15 = lightDir_2;
  mediump vec3 viewDir_16;
  viewDir_16 = tmpvar_13;
  mediump float atten_17;
  atten_17 = tmpvar_14.w;
  mediump vec4 c_18;
  mediump vec3 specCol_19;
  highp float nh_20;
  mediump float tmpvar_21;
  tmpvar_21 = max (0.0, dot (tmpvar_4, normalize((lightDir_15 + viewDir_16))));
  nh_20 = tmpvar_21;
  mediump float arg1_22;
  arg1_22 = (32.0 * _Shininess);
  highp vec3 tmpvar_23;
  tmpvar_23 = (pow (nh_20, arg1_22) * tmpvar_9);
  specCol_19 = tmpvar_23;
  c_18.xyz = ((((tmpvar_3 * _LightColor0.xyz) * max (0.0, dot (tmpvar_4, lightDir_15))) + (_LightColor0.xyz * specCol_19)) * (atten_17 * 2.0));
  c_18.w = tmpvar_5;
  c_1.xyz = c_18.xyz;
  c_1.w = tmpvar_5;
  gl_FragData[0] = c_1;
}



#endif"
}

}
Program "fp" {
// Fragment combos: 5
//   opengl - ALU: 44 to 55, TEX: 3 to 5
//   d3d9 - ALU: 46 to 55, TEX: 3 to 5
//   d3d11 - ALU: 32 to 42, TEX: 3 to 5, FLOW: 1 to 1
SubProgram "opengl " {
Keywords { "POINT" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
Float 4 [_FresnelPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_LightTexture0] 2D
"3.0-!!ARBfp1.0
OPTION ARB_precision_hint_fastest;
# 49 ALU, 4 TEX
PARAM c[7] = { program.local[0..4],
		{ 2, 1, 0, 0.79627001 },
		{ 0.20373, 32 } };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
TEMP R4;
TEX R0.yw, fragment.texcoord[0], texture[2], 2D;
MAD R1.xy, R0.wyzw, c[5].x, -c[5].y;
MUL R0.x, R1.y, R1.y;
MAD R0.x, -R1, R1, -R0;
ADD R0.x, R0, c[5].y;
RSQ R0.x, R0.x;
RCP R1.z, R0.x;
DP3 R0.y, fragment.texcoord[2], fragment.texcoord[2];
RSQ R0.x, R0.y;
MUL R3.xyz, R0.x, fragment.texcoord[2];
DP3 R0.y, fragment.texcoord[1], fragment.texcoord[1];
RSQ R0.x, R0.y;
MAD R4.xyz, fragment.texcoord[1], R0.x, R3;
DP3 R0.y, R1, R1;
RSQ R0.y, R0.y;
DP3 R0.x, fragment.texcoord[1], fragment.texcoord[1];
MUL R2.xyz, R0.y, R1;
RSQ R0.x, R0.x;
MUL R0.xyz, R0.x, fragment.texcoord[1];
DP3 R0.x, R0, R2;
MAX R0.x, R0, c[5].z;
DP3 R1.w, R1, R3;
DP3 R0.y, R4, R4;
ADD_SAT R0.w, -R0.x, c[5].y;
RSQ R0.y, R0.y;
MUL R0.xyz, R0.y, R4;
DP3 R0.x, R1, R0;
MAX R2.x, R1.w, c[5].z;
TEX R1, fragment.texcoord[0], texture[0], 2D;
MAX R2.y, R0.x, c[5].z;
TEX R0.xyz, fragment.texcoord[0], texture[1], 2D;
MUL R0.xyz, R1.w, R0;
MOV R2.z, c[6].y;
MUL R1.w, R2.z, c[2].x;
MUL R1.xyz, R1, c[1];
MUL R0.xyz, R0, c[3].x;
POW R1.w, R2.y, R1.w;
MUL R0.xyz, R1.w, R0;
MUL R0.xyz, R0, c[0];
MUL R1.xyz, R1, c[0];
MAD R1.xyz, R1, R2.x, R0;
POW R0.x, R0.w, c[4].x;
DP3 R0.y, fragment.texcoord[3], fragment.texcoord[3];
MUL R0.x, R0, c[5].w;
TEX R0.w, R0.y, texture[3], 2D;
MUL R1.xyz, R0.w, R1;
ADD R0.x, R0, c[6];
MUL result.color.xyz, R1, c[5].x;
MAX result.color.w, R0.x, c[5].z;
END
# 49 instructions, 5 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "POINT" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
Float 4 [_FresnelPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_LightTexture0] 2D
"ps_3_0
; 50 ALU, 4 TEX
dcl_2d s0
dcl_2d s1
dcl_2d s2
dcl_2d s3
def c5, 2.00000000, -1.00000000, 1.00000000, 0.00000000
def c6, 0.79627001, 0.20373000, 32.00000000, 0
dcl_texcoord0 v0.xy
dcl_texcoord1 v1.xyz
dcl_texcoord2 v2.xyz
dcl_texcoord3 v3.xyz
texld r0.yw, v0, s2
mad_pp r1.xy, r0.wyzw, c5.x, c5.y
mul_pp r0.x, r1.y, r1.y
mad_pp r0.x, -r1, r1, -r0
add_pp r0.x, r0, c5.z
rsq_pp r0.x, r0.x
rcp_pp r1.z, r0.x
dp3_pp r0.y, r1, r1
rsq_pp r0.y, r0.y
dp3 r0.x, v1, v1
mul_pp r2.xyz, r0.y, r1
rsq r0.x, r0.x
mul r0.xyz, r0.x, v1
dp3 r0.x, r0, r2
max r0.x, r0, c5.w
add_sat r1.w, -r0.x, c5.z
dp3_pp r0.y, v2, v2
rsq_pp r0.x, r0.y
mul_pp r2.xyz, r0.x, v2
dp3_pp r0.y, v1, v1
rsq_pp r0.x, r0.y
mad_pp r3.xyz, v1, r0.x, r2
pow r0, r1.w, c4.x
dp3_pp r0.y, r1, r2
dp3_pp r0.z, r3, r3
rsq_pp r0.z, r0.z
mul_pp r2.xyz, r0.z, r3
dp3_pp r0.z, r1, r2
texld r1, v0, s0
mov_pp r0.w, c2.x
texld r2.xyz, v0, s1
mul_pp r2.xyz, r1.w, r2
mul_pp r1.xyz, r1, c1
mul_pp r1.xyz, r1, c0
max_pp r0.y, r0, c5.w
mul_pp r0.w, c6.z, r0
max_pp r0.z, r0, c5.w
pow r3, r0.z, r0.w
mov r0.z, r3.x
mul_pp r2.xyz, r2, c3.x
mul r2.xyz, r0.z, r2
mul_pp r2.xyz, r2, c0
mad_pp r2.xyz, r1, r0.y, r2
dp3 r1.x, v3, v3
mov r0.y, r0.x
texld r0.x, r1.x, s3
mul_pp r1.xyz, r0.x, r2
mad r0.x, r0.y, c6, c6.y
mul_pp oC0.xyz, r1, c5.x
max oC0.w, r0.x, c5
"
}

SubProgram "xbox360 " {
Keywords { "POINT" }
Vector 1 [_Color]
Float 4 [_FresnelPower]
Float 3 [_Gloss]
Vector 0 [_LightColor0]
Float 2 [_Shininess]
SetTexture 0 [_LightTexture0] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_SpecMap] 2D
// Shader Timing Estimate, in Cycles/64 pixel vector:
// ALU: 36.00 (27 instructions), vertex: 0, texture: 16,
//   sequencer: 14, interpolator: 16;    8 GPRs, 24 threads,
// Performance (if enough threads): ~36 cycles per vector
// * Texture cycle estimates are assuming an 8bit/component texture with no
//     aniso or trilinear filtering.

"ps_360
backbbaaaaaaabpiaaaaabpaaaaaaaaaaaaaaaceaaaaabkaaaaaabmiaaaaaaaa
aaaaaaaaaaaaabhiaaaaaabmaaaaabglppppadaaaaaaaaajaaaaaabmaaaaaaaa
aaaaabgeaaaaaanaaaadaaacaaabaaaaaaaaaanmaaaaaaaaaaaaaaomaaacaaab
aaabaaaaaaaaaapeaaaaaaaaaaaaabaeaaacaaaeaaabaaaaaaaaabbeaaaaaaaa
aaaaabceaaacaaadaaabaaaaaaaaabbeaaaaaaaaaaaaabclaaacaaaaaaabaaaa
aaaaaapeaaaaaaaaaaaaabdiaaadaaaaaaabaaaaaaaaaanmaaaaaaaaaaaaabeh
aaadaaabaaabaaaaaaaaaanmaaaaaaaaaaaaabfaaaacaaacaaabaaaaaaaaabbe
aaaaaaaaaaaaabflaaadaaadaaabaaaaaaaaaanmaaaaaaaafpechfgnhaengbha
aaklklklaaaeaaamaaabaaabaaabaaaaaaaaaaaafpedgpgmgphcaaklaaabaaad
aaabaaaeaaabaaaaaaaaaaaafpeghcgfhdgogfgmfagphhgfhcaaklklaaaaaaad
aaabaaabaaabaaaaaaaaaaaafpehgmgphdhdaafpemgjghgiheedgpgmgphcdaaa
fpemgjghgihefegfhihehfhcgfdaaafpengbgjgofegfhiaafpfdgigjgogjgogf
hdhdaafpfdhagfgdengbhaaahahdfpddfpdaaadccodacodcdadddfddcodaaakl
aaaaaaaaaaaaaaabaaaaaaaaaaaaaaaaaaaaaabeabpmaabaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaeaaaaaablabaaaahaaaaaaaaaeaaaaaaaaaaaacmie
aaapaaapaaaaaaabaaaadafaaaaahbfbaaaahcfcaaaahdfdaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaeaaaaaaadpiaaaaa
ecaaaaaaaaaaaaaaaaaaaaaadofajojjlpiaaaaadpelnifkabfefaaeaaaabcaa
meaaaaaaaaaagaajgaapbcaabcaaaaaaaaaagabfgablbcaabcaaaaaaaaaacacb
aaaaccaaaaaaaaaamiaeaaaaaaloloaapaadadaakiaifaabbpbppodpaaaaeaaa
badidaabbpbppoiiaaaaeaaababieaabbpbppefiaaaaeaaabaciaaabbpbpppnj
aaaaeaaamiaiaaabaaloloaapaababaabeaiaaaaaalologmnaacacacmiadaaah
aagngmmgilaapoppambhaeaaaamplomgibaeabpofiioacadaalbpmblobaeadia
miaiaaaaaegngnlbnbahahpofiioabacaablpmblobacacibaibhafagaablmagm
obababahaicoafaeaaabpmlboaacagahkmbdacabaamemfeboaafafadkaeeahab
aamdmdblpaaeaeiafieiabaaaagmblmgoaabaaibfiioahaeaaabmgblobaeabia
kmcbacafaamdloecpaacahadkmecacafaamdloedpaaeahadkichaeadaabamleb
mbahahaakieeaeafaaloloicnaagadaakiioaeadaakggmmaicafppaaebbeadab
aelblbblkaadpoideaepabaaaaaamemgobaeadabdiieababaamggmgmkbabaeaa
diehabacaamablmgobacababmiaeaaabaamgbllbilabppppmiaiiaaaaamggmaa
kcabppaamiahaaaaaamamabfklacaaaamiahiaaaaalbmaaaobabaaaaaaaaaaaa
aaaaaaaaaaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "POINT" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
Float 4 [_FresnelPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_LightTexture0] 2D
"sce_fp_rsx // 59 instructions using 5 registers
[Configuration]
24
ffffffff0003c020000ffff1000000000000840005000000
[Offsets]
5
_LightColor0 2 0
0000031000000130
_Color 1 0
00000020
_Shininess 1 0
00000170
_Gloss 1 0
00000080
_FresnelPower 1 0
00000330
[Microcode]
944
9e001700c8011c9dc8000001c8003fe10e840240c8001c9dc8020001c8000001
0000000000000000000000000000000094041704c8011c9dc8000001c8003fe1
ae8e3940c8011c9dc8000029c800bfe1068c0440ce081c9daa02000054020001
00000000000040000000bf8000000000108c0240c8001c9d00020000c8000001
000000000000000000000000000000008e041702c8011c9dc8000001c8003fe1
0e800240ff181c9dc8080001c800000110800240ab181c9cab180000c8000001
1090044001181c9e01180000c9000003ee040100c8011c9dc8000001c8003fe1
10080500c8081c9dc8080001c800000110900340c9201c9d00020000c8000001
00003f80000000000000000000000000088c3b40ff203c9dff200001c8000001
0e840240c9081c9dc8020001c800000100000000000000000000000000000000
ce903940c8011c9dc8000029c800bfe110800540c9181c9dc9200001c8000001
1082014000021c9cc8000001c800000100000000000000000000000000000000
a8000500c8011c9dc8010001c800bfe10e080340c9201c9dc91c0001c8000001
0e863940c8101c9dc8000029c800000108820540c9181c9dc90c0001c8000001
10840900c9001c9d00020000c800000100000000000000000000000000000000
0e8a3940c9181c9dc8000029c800000110880240c9041c9d00020000c8000001
000042000000000000000000000000000e840240c9081c9dff080001c8000001
ae063b00c8011c9d54000001c800bfe11002090055041c9d00020000c8000001
0000000000000000000000000000000008021d00fe041c9dc8000001c8000001
1000020054041c9dc9100001c80000010e8a0100c9141c9dc8000001c8000001
08000500c80c1c9dc9140001c80000010206090054001c9d00020000c8000001
0000000000000000000000000000000008001c00fe001c9dc8000001c8000001
02068300c80c1c9f00020000c800000100003f80000000000000000000000000
0e80020054001c9dc9000001c800000108021d00c80c1c9dc8000001c8000001
0e800440c9001c9dc8020001c908000100000000000000000000000000000000
1000020054041c9d00020000c800000100000000000000000000000000000000
08001c00fe001c9dc8000001c800000102041706fe101c9dc8000001c8000001
0e80024000081c9cc9001001c80000010202040054001c9d00020000aa020000
d85a3f4b9e993e5000000000000000001081090000041c9c00020000c8000001
00000000000000000000000000000000
"
}

SubProgram "d3d11 " {
Keywords { "POINT" }
ConstBuffer "$Globals" 176 // 160 used size, 11 vars
Vector 16 [_LightColor0] 4
Vector 112 [_Color] 4
Float 148 [_Shininess]
Float 152 [_Gloss]
Float 156 [_FresnelPower]
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 1
SetTexture 1 [_SpecMap] 2D 3
SetTexture 2 [_BumpMap] 2D 2
SetTexture 3 [_LightTexture0] 2D 0
// 48 instructions, 6 temp regs, 0 temp arrays:
// ALU 37 float, 0 int, 0 uint
// TEX 4 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0
eefiecedjlfmacimihobggonhcnigochnekcdoddabaaaaaaeaahaaaaadaaaaaa
cmaaaaaammaaaaaaaaabaaaaejfdeheojiaaaaaaafaaaaaaaiaaaaaaiaaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaadadaaaaimaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaahahaaaaimaaaaaa
adaaaaaaaaaaaaaaadaaaaaaaeaaaaaaahahaaaafdfgfpfaepfdejfeejepeoaa
feeffiedepepfceeaaklklklepfdeheocmaaaaaaabaaaaaaaiaaaaaacaaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaafdfgfpfegbhcghgfheaaklkl
fdeieefcdiagaaaaeaaaaaaaioabaaaafjaaaaaeegiocaaaaaaaaaaaakaaaaaa
fkaaaaadaagabaaaaaaaaaaafkaaaaadaagabaaaabaaaaaafkaaaaadaagabaaa
acaaaaaafkaaaaadaagabaaaadaaaaaafibiaaaeaahabaaaaaaaaaaaffffaaaa
fibiaaaeaahabaaaabaaaaaaffffaaaafibiaaaeaahabaaaacaaaaaaffffaaaa
fibiaaaeaahabaaaadaaaaaaffffaaaagcbaaaaddcbabaaaabaaaaaagcbaaaad
hcbabaaaacaaaaaagcbaaaadhcbabaaaadaaaaaagcbaaaadhcbabaaaaeaaaaaa
gfaaaaadpccabaaaaaaaaaaagiaaaaacagaaaaaaefaaaaajpcaabaaaaaaaaaaa
egbabaaaabaaaaaaeghobaaaabaaaaaaaagabaaaadaaaaaaefaaaaajpcaabaaa
abaaaaaaegbabaaaabaaaaaaeghobaaaaaaaaaaaaagabaaaabaaaaaadiaaaaah
hcaabaaaaaaaaaaaegacbaaaaaaaaaaapgapbaaaabaaaaaadiaaaaaihcaabaaa
abaaaaaaegacbaaaabaaaaaaegiccaaaaaaaaaaaahaaaaaadiaaaaaihcaabaaa
abaaaaaaegacbaaaabaaaaaaegiccaaaaaaaaaaaabaaaaaadiaaaaaihcaabaaa
aaaaaaaaegacbaaaaaaaaaaakgikcaaaaaaaaaaaajaaaaaadiaaaaaiicaabaaa
aaaaaaaabkiacaaaaaaaaaaaajaaaaaaabeaaaaaaaaaaaecbaaaaaahicaabaaa
abaaaaaaegbcbaaaacaaaaaaegbcbaaaacaaaaaaeeaaaaaficaabaaaabaaaaaa
dkaabaaaabaaaaaadiaaaaahhcaabaaaacaaaaaapgapbaaaabaaaaaaegbcbaaa
acaaaaaabaaaaaahicaabaaaabaaaaaaegbcbaaaadaaaaaaegbcbaaaadaaaaaa
eeaaaaaficaabaaaabaaaaaadkaabaaaabaaaaaadcaaaaajhcaabaaaadaaaaaa
egbcbaaaadaaaaaapgapbaaaabaaaaaaegacbaaaacaaaaaadiaaaaahhcaabaaa
aeaaaaaapgapbaaaabaaaaaaegbcbaaaadaaaaaabaaaaaahicaabaaaabaaaaaa
egacbaaaadaaaaaaegacbaaaadaaaaaaeeaaaaaficaabaaaabaaaaaadkaabaaa
abaaaaaadiaaaaahhcaabaaaadaaaaaapgapbaaaabaaaaaaegacbaaaadaaaaaa
efaaaaajpcaabaaaafaaaaaaegbabaaaabaaaaaaeghobaaaacaaaaaaaagabaaa
acaaaaaadcaaaaapdcaabaaaafaaaaaahgapbaaaafaaaaaaaceaaaaaaaaaaaea
aaaaaaeaaaaaaaaaaaaaaaaaaceaaaaaaaaaialpaaaaialpaaaaaaaaaaaaaaaa
dcaaaaakicaabaaaabaaaaaaakaabaiaebaaaaaaafaaaaaaakaabaaaafaaaaaa
abeaaaaaaaaaiadpdcaaaaakicaabaaaabaaaaaabkaabaiaebaaaaaaafaaaaaa
bkaabaaaafaaaaaadkaabaaaabaaaaaaelaaaaafecaabaaaafaaaaaadkaabaaa
abaaaaaabaaaaaahicaabaaaabaaaaaaegacbaaaafaaaaaaegacbaaaadaaaaaa
deaaaaahicaabaaaabaaaaaadkaabaaaabaaaaaaabeaaaaaaaaaaaaacpaaaaaf
icaabaaaabaaaaaadkaabaaaabaaaaaadiaaaaahicaabaaaaaaaaaaadkaabaaa
aaaaaaaadkaabaaaabaaaaaabjaaaaaficaabaaaaaaaaaaadkaabaaaaaaaaaaa
diaaaaahhcaabaaaaaaaaaaaegacbaaaaaaaaaaapgapbaaaaaaaaaaadiaaaaai
hcaabaaaaaaaaaaaegacbaaaaaaaaaaaegiccaaaaaaaaaaaabaaaaaabaaaaaah
icaabaaaaaaaaaaaegacbaaaafaaaaaaegacbaaaaeaaaaaadeaaaaahicaabaaa
aaaaaaaadkaabaaaaaaaaaaaabeaaaaaaaaaaaaadcaaaaajhcaabaaaaaaaaaaa
egacbaaaabaaaaaapgapbaaaaaaaaaaaegacbaaaaaaaaaaabaaaaaahicaabaaa
aaaaaaaaegbcbaaaaeaaaaaaegbcbaaaaeaaaaaaefaaaaajpcaabaaaabaaaaaa
pgapbaaaaaaaaaaaeghobaaaadaaaaaaaagabaaaaaaaaaaaaaaaaaahicaabaaa
aaaaaaaaakaabaaaabaaaaaaakaabaaaabaaaaaadiaaaaahhccabaaaaaaaaaaa
pgapbaaaaaaaaaaaegacbaaaaaaaaaaabaaaaaahbcaabaaaaaaaaaaaegacbaaa
afaaaaaaegacbaaaafaaaaaaeeaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaa
diaaaaahhcaabaaaaaaaaaaaagaabaaaaaaaaaaaegacbaaaafaaaaaabaaaaaah
bcaabaaaaaaaaaaaegacbaaaacaaaaaaegacbaaaaaaaaaaadeaaaaahbcaabaaa
aaaaaaaaakaabaaaaaaaaaaaabeaaaaaaaaaaaaaaaaaaaaibcaabaaaaaaaaaaa
akaabaiaebaaaaaaaaaaaaaaabeaaaaaaaaaiadpdeaaaaahbcaabaaaaaaaaaaa
akaabaaaaaaaaaaaabeaaaaaaaaaaaaacpaaaaafbcaabaaaaaaaaaaaakaabaaa
aaaaaaaadiaaaaaibcaabaaaaaaaaaaaakaabaaaaaaaaaaadkiacaaaaaaaaaaa
ajaaaaaabjaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaadcaaaaajiccabaaa
aaaaaaaaakaabaaaaaaaaaaaabeaaaaafknieldpabeaaaaajjjofadodoaaaaab
"
}

SubProgram "gles " {
Keywords { "POINT" }
"!!GLES"
}

SubProgram "glesdesktop " {
Keywords { "POINT" }
"!!GLES"
}

SubProgram "opengl " {
Keywords { "DIRECTIONAL" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
Float 4 [_FresnelPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
"3.0-!!ARBfp1.0
OPTION ARB_precision_hint_fastest;
# 44 ALU, 3 TEX
PARAM c[7] = { program.local[0..4],
		{ 2, 1, 0, 0.79627001 },
		{ 0.20373, 32 } };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
TEX R0.yw, fragment.texcoord[0], texture[2], 2D;
MAD R1.xy, R0.wyzw, c[5].x, -c[5].y;
MUL R0.x, R1.y, R1.y;
MAD R0.x, -R1, R1, -R0;
ADD R0.x, R0, c[5].y;
RSQ R0.x, R0.x;
RCP R1.z, R0.x;
DP3 R0.y, fragment.texcoord[1], fragment.texcoord[1];
DP3 R0.x, fragment.texcoord[1], fragment.texcoord[1];
MOV R2.xyz, fragment.texcoord[2];
RSQ R0.y, R0.y;
MAD R3.xyz, fragment.texcoord[1], R0.y, R2;
DP3 R0.y, R1, R1;
RSQ R0.y, R0.y;
MUL R2.xyz, R0.y, R1;
RSQ R0.x, R0.x;
MUL R0.xyz, R0.x, fragment.texcoord[1];
DP3 R0.x, R0, R2;
DP3 R0.w, R3, R3;
RSQ R0.y, R0.w;
MUL R2.xyz, R0.y, R3;
DP3 R0.y, R1, R2;
DP3 R1.x, R1, fragment.texcoord[2];
MAX R0.x, R0, c[5].z;
ADD_SAT R0.x, -R0, c[5].y;
POW R1.w, R0.x, c[4].x;
MAX R2.w, R0.y, c[5].z;
TEX R0, fragment.texcoord[0], texture[0], 2D;
TEX R2.xyz, fragment.texcoord[0], texture[1], 2D;
MUL R2.xyz, R0.w, R2;
MOV R3.x, c[6].y;
MUL R0.w, R3.x, c[2].x;
MUL R0.xyz, R0, c[1];
POW R0.w, R2.w, R0.w;
MUL R2.xyz, R2, c[3].x;
MUL R2.xyz, R0.w, R2;
MUL R0.xyz, R0, c[0];
MUL R0.w, R1, c[5];
MUL R2.xyz, R2, c[0];
MAX R1.x, R1, c[5].z;
MAD R1.xyz, R0, R1.x, R2;
ADD R0.x, R0.w, c[6];
MUL result.color.xyz, R1, c[5].x;
MAX result.color.w, R0.x, c[5].z;
END
# 44 instructions, 4 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "DIRECTIONAL" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
Float 4 [_FresnelPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
"ps_3_0
; 46 ALU, 3 TEX
dcl_2d s0
dcl_2d s1
dcl_2d s2
def c5, 2.00000000, -1.00000000, 1.00000000, 0.00000000
def c6, 0.79627001, 0.20373000, 32.00000000, 0
dcl_texcoord0 v0.xy
dcl_texcoord1 v1.xyz
dcl_texcoord2 v2.xyz
texld r0.yw, v0, s2
mad_pp r2.xy, r0.wyzw, c5.x, c5.y
mul_pp r0.x, r2.y, r2.y
mad_pp r0.x, -r2, r2, -r0
add_pp r0.x, r0, c5.z
rsq_pp r0.x, r0.x
rcp_pp r2.z, r0.x
dp3_pp r0.y, r2, r2
rsq_pp r0.y, r0.y
dp3 r0.x, v1, v1
dp3_pp r1.w, r2, v2
mul_pp r1.xyz, r0.y, r2
rsq r0.x, r0.x
mul r0.xyz, r0.x, v1
dp3 r0.x, r0, r1
max r0.w, r0.x, c5
add_sat r0.w, -r0, c5.z
pow r4, r0.w, c4.x
dp3_pp r1.x, v1, v1
rsq_pp r1.x, r1.x
mov_pp r0.xyz, v2
mad_pp r0.xyz, v1, r1.x, r0
dp3_pp r1.x, r0, r0
rsq_pp r0.w, r1.x
mul_pp r0.xyz, r0.w, r0
dp3_pp r0.x, r2, r0
mov_pp r0.w, c2.x
mul_pp r0.y, c6.z, r0.w
max_pp r0.x, r0, c5.w
pow r3, r0.x, r0.y
texld r0, v0, s0
texld r1.xyz, v0, s1
mul_pp r1.xyz, r0.w, r1
mul_pp r0.xyz, r0, c1
mov r0.w, r3.x
mul_pp r1.xyz, r1, c3.x
mul r1.xyz, r0.w, r1
mul_pp r0.xyz, r0, c0
mov r0.w, r4.x
mul_pp r1.xyz, r1, c0
max_pp r1.w, r1, c5
mad_pp r1.xyz, r0, r1.w, r1
mad r0.x, r0.w, c6, c6.y
mul_pp oC0.xyz, r1, c5.x
max oC0.w, r0.x, c5
"
}

SubProgram "xbox360 " {
Keywords { "DIRECTIONAL" }
Vector 1 [_Color]
Float 4 [_FresnelPower]
Float 3 [_Gloss]
Vector 0 [_LightColor0]
Float 2 [_Shininess]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_BumpMap] 2D
SetTexture 2 [_SpecMap] 2D
// Shader Timing Estimate, in Cycles/64 pixel vector:
// ALU: 32.00 (24 instructions), vertex: 0, texture: 12,
//   sequencer: 12, interpolator: 12;    6 GPRs, 30 threads,
// Performance (if enough threads): ~32 cycles per vector
// * Texture cycle estimates are assuming an 8bit/component texture with no
//     aniso or trilinear filtering.

"ps_360
backbbaaaaaaabnaaaaaableaaaaaaaaaaaaaaceaaaaabhmaaaaabkeaaaaaaaa
aaaaaaaaaaaaabfeaaaaaabmaaaaabeippppadaaaaaaaaaiaaaaaabmaaaaaaaa
aaaaabebaaaaaalmaaadaaabaaabaaaaaaaaaamiaaaaaaaaaaaaaaniaaacaaab
aaabaaaaaaaaaaoaaaaaaaaaaaaaaapaaaacaaaeaaabaaaaaaaaabaaaaaaaaaa
aaaaabbaaaacaaadaaabaaaaaaaaabaaaaaaaaaaaaaaabbhaaacaaaaaaabaaaa
aaaaaaoaaaaaaaaaaaaaabceaaadaaaaaaabaaaaaaaaaamiaaaaaaaaaaaaabcn
aaacaaacaaabaaaaaaaaabaaaaaaaaaaaaaaabdiaaadaaacaaabaaaaaaaaaami
aaaaaaaafpechfgnhaengbhaaaklklklaaaeaaamaaabaaabaaabaaaaaaaaaaaa
fpedgpgmgphcaaklaaabaaadaaabaaaeaaabaaaaaaaaaaaafpeghcgfhdgogfgm
fagphhgfhcaaklklaaaaaaadaaabaaabaaabaaaaaaaaaaaafpehgmgphdhdaafp
emgjghgiheedgpgmgphcdaaafpengbgjgofegfhiaafpfdgigjgogjgogfhdhdaa
fpfdhagfgdengbhaaahahdfpddfpdaaadccodacodcdadddfddcodaaaaaaaaaaa
aaaaaaabaaaaaaaaaaaaaaaaaaaaaabeabpmaabaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaeaaaaaabhebaaaafaaaaaaaaaeaaaaaaaaaaaacagdaaahaaah
aaaaaaabaaaadafaaaaahbfbaaaahcfcaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaeaaaaaaadpiaaaaaecaaaaaaaaaaaaaa
aaaaaaaadofajojjlpiaaaaadpelnifkaabfdaadaaaabcaameaaaaaaaaaagaag
gaambcaabcaaaaaaaaaagabcgabibcaaccaaaaaabacieaabbpbppoiiaaaaeaaa
babiaaabbpbppghpaaaaeaaabaaidaabbpbppgiiaaaaeaaamiabaaaaaaloloaa
paababaamiahaaadaamamaaakbadabaamiadaaafaamhgmmgilaapoppfiioabaa
aablpmgmobadaeiamiabaaaaaegngnlbnbafafpoaiehafaeaamamagmkbadaaaf
aiihafadaablmalbobababafaaioacabaapmpmmloaadacafkaibafabaamdmdgm
paababiafibbabaaaablgmgmoaacaaibfiehafabaabfgmgmobababiabeacaaab
aalolpgmnaabafacamieaeabaalplomgnaafacpokiboacacaapmagebmbafafad
kicbacabaalomdecnaadacadkiehacadaalogmedicabppadebibadabaelblbmg
kaadpoideabpabaaaaaacmgmobaeadabdibiababaagmgmblkbabaeaadiihabab
aamagmblobacababmiaiaaabaablbllbilabppppmiaiiaaaaablgmaakcabppaa
miahaaaaaamamamaklabaaaamiahiaaaaamamaaaoaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "DIRECTIONAL" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
Float 4 [_FresnelPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
"sce_fp_rsx // 56 instructions using 4 registers
[Configuration]
24
ffffffff0001c0200007fff9000000000000840004000000
[Offsets]
5
_LightColor0 2 0
0000033000000200
_Color 1 0
00000110
_Shininess 1 0
00000050
_Gloss 1 0
000001b0
_FresnelPower 1 0
000002f0
[Microcode]
896
94061704c8011c9dc8000001c8003fe1ae803940c8011c9dc8000029c800bfe1
068e0440ce0c1c9daa0200005402000100000000000040000000bf8000000000
108c014000021c9cc8000001c800000100000000000000000000000000000000
b0020500c8011c9dc8010001c800bfe110800240ab1c1c9cab1c0000c8000001
1e7e7d00c8001c9dc8000001c8000001108a0440011c1c9e011c0000c9000003
ce8c0140c8011c9dc8000001c8003fe10e040340c9181c9dc9000001c8000001
9e001700c8011c9dc8000001c8003fe10e883940c8081c9dc8000029c8000001
10880340c9141c9dc8020001c800000100000000000000000000000000003f80
0e800240c8001c9dc8020001c800000100000000000000000000000000000000
088e3b40ff103c9dff100001c800000106040100c8081c9dc8000001c8000001
108e0540c91c1c9dc9100001c8000001108c0240c9181c9d00020000c8000001
0000420000000000000000000000000010040900c91c1c9d00020000c8000001
00000000000000000000000000000000ae043b00c8011c9dfe040001c800bfe1
10800240c8001c9d00020000c800000100000000000000000000000000000000
10820540c91c1c9dc9180001c80000010e823940c91c1c9dc8000029c8000001
10041d00fe081c9dc8000001c80000010e800240c9001c9dc8020001c8000001
0000000000000000000000000000000008000500c8081c9dc9040001c8000001
10820900c9041c9d00020000c800000100000000000000000000000000000000
10040200c8081c9dc9180001c80000010204090054001c9d00020000c8000001
000000000000000000000000000000008e021702c8011c9dc8000001c8003fe1
08001c00fe081c9dc8000001c80000010e840240ff001c9dc8040001c8000001
02048300c8081c9f00020000c800000100003f80000000000000000000000000
0e800240c9001c9dff040001c800000110001d00c8081c9dc8000001c8000001
10020200c8001c9d00020000c800000100000000000000000000000000000000
0e84020054001c9dc9080001c800000108001c00fe041c9dc8000001c8000001
0e800440c9081c9dc8021001c900000100000000000000000000000000000000
0204040054001c9d00020000aa020000d85a3f4b9e993e500000000000000000
1081090000081c9c00020000c800000100000000000000000000000000000000
"
}

SubProgram "d3d11 " {
Keywords { "DIRECTIONAL" }
ConstBuffer "$Globals" 112 // 96 used size, 10 vars
Vector 16 [_LightColor0] 4
Vector 48 [_Color] 4
Float 84 [_Shininess]
Float 88 [_Gloss]
Float 92 [_FresnelPower]
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 0
SetTexture 1 [_SpecMap] 2D 2
SetTexture 2 [_BumpMap] 2D 1
// 42 instructions, 5 temp regs, 0 temp arrays:
// ALU 32 float, 0 int, 0 uint
// TEX 3 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0
eefiecedikmdcehbbokfgapfdjhbmabepmcelfdbabaaaaaafiagaaaaadaaaaaa
cmaaaaaaleaaaaaaoiaaaaaaejfdeheoiaaaaaaaaeaaaaaaaiaaaaaagiaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaheaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaadadaaaaheaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaaheaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaahahaaaafdfgfpfa
epfdejfeejepeoaafeeffiedepepfceeaaklklklepfdeheocmaaaaaaabaaaaaa
aiaaaaaacaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaafdfgfpfe
gbhcghgfheaaklklfdeieefcgiafaaaaeaaaaaaafkabaaaafjaaaaaeegiocaaa
aaaaaaaaagaaaaaafkaaaaadaagabaaaaaaaaaaafkaaaaadaagabaaaabaaaaaa
fkaaaaadaagabaaaacaaaaaafibiaaaeaahabaaaaaaaaaaaffffaaaafibiaaae
aahabaaaabaaaaaaffffaaaafibiaaaeaahabaaaacaaaaaaffffaaaagcbaaaad
dcbabaaaabaaaaaagcbaaaadhcbabaaaacaaaaaagcbaaaadhcbabaaaadaaaaaa
gfaaaaadpccabaaaaaaaaaaagiaaaaacafaaaaaaefaaaaajpcaabaaaaaaaaaaa
egbabaaaabaaaaaaeghobaaaabaaaaaaaagabaaaacaaaaaaefaaaaajpcaabaaa
abaaaaaaegbabaaaabaaaaaaeghobaaaaaaaaaaaaagabaaaaaaaaaaadiaaaaah
hcaabaaaaaaaaaaaegacbaaaaaaaaaaapgapbaaaabaaaaaadiaaaaaihcaabaaa
abaaaaaaegacbaaaabaaaaaaegiccaaaaaaaaaaaadaaaaaadiaaaaaihcaabaaa
abaaaaaaegacbaaaabaaaaaaegiccaaaaaaaaaaaabaaaaaadiaaaaaihcaabaaa
aaaaaaaaegacbaaaaaaaaaaakgikcaaaaaaaaaaaafaaaaaadiaaaaaiicaabaaa
aaaaaaaabkiacaaaaaaaaaaaafaaaaaaabeaaaaaaaaaaaecbaaaaaahicaabaaa
abaaaaaaegbcbaaaacaaaaaaegbcbaaaacaaaaaaeeaaaaaficaabaaaabaaaaaa
dkaabaaaabaaaaaadcaaaaajhcaabaaaacaaaaaaegbcbaaaacaaaaaapgapbaaa
abaaaaaaegbcbaaaadaaaaaadiaaaaahhcaabaaaadaaaaaapgapbaaaabaaaaaa
egbcbaaaacaaaaaabaaaaaahicaabaaaabaaaaaaegacbaaaacaaaaaaegacbaaa
acaaaaaaeeaaaaaficaabaaaabaaaaaadkaabaaaabaaaaaadiaaaaahhcaabaaa
acaaaaaapgapbaaaabaaaaaaegacbaaaacaaaaaaefaaaaajpcaabaaaaeaaaaaa
egbabaaaabaaaaaaeghobaaaacaaaaaaaagabaaaabaaaaaadcaaaaapdcaabaaa
aeaaaaaahgapbaaaaeaaaaaaaceaaaaaaaaaaaeaaaaaaaeaaaaaaaaaaaaaaaaa
aceaaaaaaaaaialpaaaaialpaaaaaaaaaaaaaaaadcaaaaakicaabaaaabaaaaaa
akaabaiaebaaaaaaaeaaaaaaakaabaaaaeaaaaaaabeaaaaaaaaaiadpdcaaaaak
icaabaaaabaaaaaabkaabaiaebaaaaaaaeaaaaaabkaabaaaaeaaaaaadkaabaaa
abaaaaaaelaaaaafecaabaaaaeaaaaaadkaabaaaabaaaaaabaaaaaahicaabaaa
abaaaaaaegacbaaaaeaaaaaaegacbaaaacaaaaaadeaaaaahicaabaaaabaaaaaa
dkaabaaaabaaaaaaabeaaaaaaaaaaaaacpaaaaaficaabaaaabaaaaaadkaabaaa
abaaaaaadiaaaaahicaabaaaaaaaaaaadkaabaaaaaaaaaaadkaabaaaabaaaaaa
bjaaaaaficaabaaaaaaaaaaadkaabaaaaaaaaaaadiaaaaahhcaabaaaaaaaaaaa
egacbaaaaaaaaaaapgapbaaaaaaaaaaadiaaaaaihcaabaaaaaaaaaaaegacbaaa
aaaaaaaaegiccaaaaaaaaaaaabaaaaaabaaaaaahicaabaaaaaaaaaaaegacbaaa
aeaaaaaaegbcbaaaadaaaaaadeaaaaahicaabaaaaaaaaaaadkaabaaaaaaaaaaa
abeaaaaaaaaaaaaadcaaaaajhcaabaaaaaaaaaaaegacbaaaabaaaaaapgapbaaa
aaaaaaaaegacbaaaaaaaaaaaaaaaaaahhccabaaaaaaaaaaaegacbaaaaaaaaaaa
egacbaaaaaaaaaaabaaaaaahbcaabaaaaaaaaaaaegacbaaaaeaaaaaaegacbaaa
aeaaaaaaeeaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaadiaaaaahhcaabaaa
aaaaaaaaagaabaaaaaaaaaaaegacbaaaaeaaaaaabaaaaaahbcaabaaaaaaaaaaa
egacbaaaadaaaaaaegacbaaaaaaaaaaadeaaaaahbcaabaaaaaaaaaaaakaabaaa
aaaaaaaaabeaaaaaaaaaaaaaaaaaaaaibcaabaaaaaaaaaaaakaabaiaebaaaaaa
aaaaaaaaabeaaaaaaaaaiadpdeaaaaahbcaabaaaaaaaaaaaakaabaaaaaaaaaaa
abeaaaaaaaaaaaaacpaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaadiaaaaai
bcaabaaaaaaaaaaaakaabaaaaaaaaaaadkiacaaaaaaaaaaaafaaaaaabjaaaaaf
bcaabaaaaaaaaaaaakaabaaaaaaaaaaadcaaaaajiccabaaaaaaaaaaaakaabaaa
aaaaaaaaabeaaaaafknieldpabeaaaaajjjofadodoaaaaab"
}

SubProgram "gles " {
Keywords { "DIRECTIONAL" }
"!!GLES"
}

SubProgram "glesdesktop " {
Keywords { "DIRECTIONAL" }
"!!GLES"
}

SubProgram "opengl " {
Keywords { "SPOT" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
Float 4 [_FresnelPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_LightTexture0] 2D
SetTexture 4 [_LightTextureB0] 2D
"3.0-!!ARBfp1.0
OPTION ARB_precision_hint_fastest;
# 55 ALU, 5 TEX
PARAM c[7] = { program.local[0..4],
		{ 2, 1, 0, 0.79627001 },
		{ 0.20373, 0.5, 32 } };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
TEMP R4;
TEX R0.yw, fragment.texcoord[0], texture[2], 2D;
MAD R1.xy, R0.wyzw, c[5].x, -c[5].y;
MUL R0.x, R1.y, R1.y;
MAD R0.x, -R1, R1, -R0;
ADD R0.x, R0, c[5].y;
RSQ R0.x, R0.x;
RCP R1.z, R0.x;
DP3 R0.y, fragment.texcoord[2], fragment.texcoord[2];
RSQ R0.x, R0.y;
MUL R3.xyz, R0.x, fragment.texcoord[2];
DP3 R0.y, fragment.texcoord[1], fragment.texcoord[1];
RSQ R0.x, R0.y;
MAD R4.xyz, fragment.texcoord[1], R0.x, R3;
DP3 R0.y, R1, R1;
RSQ R0.y, R0.y;
DP3 R0.x, fragment.texcoord[1], fragment.texcoord[1];
MUL R2.xyz, R0.y, R1;
RSQ R0.x, R0.x;
MUL R0.xyz, R0.x, fragment.texcoord[1];
DP3 R0.x, R0, R2;
MAX R0.x, R0, c[5].z;
ADD_SAT R0.w, -R0.x, c[5].y;
DP3 R1.w, R1, R3;
DP3 R0.y, R4, R4;
RSQ R0.y, R0.y;
MUL R0.xyz, R0.y, R4;
DP3 R0.x, R1, R0;
MAX R2.x, R1.w, c[5].z;
MAX R2.y, R0.x, c[5].z;
TEX R1, fragment.texcoord[0], texture[0], 2D;
TEX R0.xyz, fragment.texcoord[0], texture[1], 2D;
MUL R0.xyz, R1.w, R0;
MOV R2.z, c[6];
MUL R1.w, R2.z, c[2].x;
MUL R1.xyz, R1, c[1];
MUL R1.xyz, R1, c[0];
POW R1.w, R2.y, R1.w;
MUL R0.xyz, R0, c[3].x;
MUL R0.xyz, R1.w, R0;
MUL R0.xyz, R0, c[0];
MAD R0.xyz, R1, R2.x, R0;
POW R0.w, R0.w, c[4].x;
RCP R1.y, fragment.texcoord[3].w;
MAD R1.zw, fragment.texcoord[3].xyxy, R1.y, c[6].y;
MUL R1.x, R0.w, c[5].w;
DP3 R1.y, fragment.texcoord[3], fragment.texcoord[3];
TEX R0.w, R1.zwzw, texture[3], 2D;
TEX R1.w, R1.y, texture[4], 2D;
SLT R1.y, c[5].z, fragment.texcoord[3].z;
MUL R0.w, R1.y, R0;
MUL R0.w, R0, R1;
MUL R2.xyz, R0.w, R0;
ADD R0.x, R1, c[6];
MUL result.color.xyz, R2, c[5].x;
MAX result.color.w, R0.x, c[5].z;
END
# 55 instructions, 5 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "SPOT" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
Float 4 [_FresnelPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_LightTexture0] 2D
SetTexture 4 [_LightTextureB0] 2D
"ps_3_0
; 55 ALU, 5 TEX
dcl_2d s0
dcl_2d s1
dcl_2d s2
dcl_2d s3
dcl_2d s4
def c5, 2.00000000, -1.00000000, 1.00000000, 0.00000000
def c6, 0.79627001, 0.20373000, 0.50000000, 32.00000000
dcl_texcoord0 v0.xy
dcl_texcoord1 v1.xyz
dcl_texcoord2 v2.xyz
dcl_texcoord3 v3
texld r0.yw, v0, s2
mad_pp r1.xy, r0.wyzw, c5.x, c5.y
mul_pp r0.x, r1.y, r1.y
mad_pp r0.x, -r1, r1, -r0
add_pp r0.x, r0, c5.z
rsq_pp r0.x, r0.x
rcp_pp r1.z, r0.x
dp3_pp r0.y, r1, r1
rsq_pp r0.y, r0.y
dp3 r0.x, v1, v1
mul_pp r2.xyz, r0.y, r1
rsq r0.x, r0.x
mul r0.xyz, r0.x, v1
dp3 r0.x, r0, r2
max r0.x, r0, c5.w
add_sat r1.w, -r0.x, c5.z
dp3_pp r0.y, v2, v2
rsq_pp r0.x, r0.y
mul_pp r2.xyz, r0.x, v2
dp3_pp r0.y, v1, v1
rsq_pp r0.x, r0.y
mad_pp r3.xyz, v1, r0.x, r2
pow r0, r1.w, c4.x
dp3_pp r0.y, r1, r2
dp3_pp r0.z, r3, r3
rsq_pp r0.z, r0.z
mul_pp r2.xyz, r0.z, r3
dp3_pp r0.z, r1, r2
texld r1, v0, s0
mov_pp r0.w, c2.x
texld r2.xyz, v0, s1
mul_pp r2.xyz, r1.w, r2
mul_pp r1.xyz, r1, c1
max_pp r0.y, r0, c5.w
mul_pp r0.w, c6, r0
max_pp r0.z, r0, c5.w
pow r3, r0.z, r0.w
mov r0.z, r3.x
mul_pp r2.xyz, r2, c3.x
mul r2.xyz, r0.z, r2
mul_pp r2.xyz, r2, c0
mul_pp r1.xyz, r1, c0
mad_pp r1.xyz, r1, r0.y, r2
rcp r0.z, v3.w
mad r2.xy, v3, r0.z, c6.z
mov r0.y, r0.x
dp3 r0.x, v3, v3
texld r0.w, r2, s3
cmp r0.z, -v3, c5.w, c5
texld r0.x, r0.x, s4
mul_pp r0.z, r0, r0.w
mul_pp r0.x, r0.z, r0
mul_pp r1.xyz, r0.x, r1
mad r0.x, r0.y, c6, c6.y
mul_pp oC0.xyz, r1, c5.x
max oC0.w, r0.x, c5
"
}

SubProgram "xbox360 " {
Keywords { "SPOT" }
Vector 1 [_Color]
Float 4 [_FresnelPower]
Float 3 [_Gloss]
Vector 0 [_LightColor0]
Float 2 [_Shininess]
SetTexture 0 [_LightTexture0] 2D
SetTexture 1 [_LightTextureB0] 2D
SetTexture 2 [_MainTex] 2D
SetTexture 3 [_BumpMap] 2D
SetTexture 4 [_SpecMap] 2D
// Shader Timing Estimate, in Cycles/64 pixel vector:
// ALU: 41.33 (31 instructions), vertex: 0, texture: 20,
//   sequencer: 16, interpolator: 16;    8 GPRs, 24 threads,
// Performance (if enough threads): ~41 cycles per vector
// * Texture cycle estimates are assuming an 8bit/component texture with no
//     aniso or trilinear filtering.

"ps_360
backbbaaaaaaacbmaaaaaccmaaaaaaaaaaaaaaceaaaaabmeaaaaabomaaaaaaaa
aaaaaaaaaaaaabjmaaaaaabmaaaaabipppppadaaaaaaaaakaaaaaabmaaaaaaaa
aaaaabiiaaaaaaoeaaadaaadaaabaaaaaaaaaapaaaaaaaaaaaaaabaaaaacaaab
aaabaaaaaaaaabaiaaaaaaaaaaaaabbiaaacaaaeaaabaaaaaaaaabciaaaaaaaa
aaaaabdiaaacaaadaaabaaaaaaaaabciaaaaaaaaaaaaabdpaaacaaaaaaabaaaa
aaaaabaiaaaaaaaaaaaaabemaaadaaaaaaabaaaaaaaaaapaaaaaaaaaaaaaabfl
aaadaaabaaabaaaaaaaaaapaaaaaaaaaaaaaabglaaadaaacaaabaaaaaaaaaapa
aaaaaaaaaaaaabheaaacaaacaaabaaaaaaaaabciaaaaaaaaaaaaabhpaaadaaae
aaabaaaaaaaaaapaaaaaaaaafpechfgnhaengbhaaaklklklaaaeaaamaaabaaab
aaabaaaaaaaaaaaafpedgpgmgphcaaklaaabaaadaaabaaaeaaabaaaaaaaaaaaa
fpeghcgfhdgogfgmfagphhgfhcaaklklaaaaaaadaaabaaabaaabaaaaaaaaaaaa
fpehgmgphdhdaafpemgjghgiheedgpgmgphcdaaafpemgjghgihefegfhihehfhc
gfdaaafpemgjghgihefegfhihehfhcgfecdaaafpengbgjgofegfhiaafpfdgigj
gogjgogfhdhdaafpfdhagfgdengbhaaahahdfpddfpdaaadccodacodcdadddfdd
codaaaklaaaaaaaaaaaaaaabaaaaaaaaaaaaaaaaaaaaaabeabpmaabaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaeaaaaaabombaaaahaaaaaaaaaeaaaaaaaa
aaaadaieaaapaaapaaaaaaabaaaadafaaaaahbfbaaaahcfcaaaapdfdaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaadpaaaaaa
dpiaaaaaecaaaaaaeaaaaaaaaaaaaaaadofajojjlpiaaaaadpelnifkaffagaae
baakbcaabcaaaaabaaaaaaaagaalmeaabcaaaaaaaaaagabbgabhbcaabcaaaaaa
aaaagabnfacdbcaaccaaaaaaemeiaaabaaloloblpaadadadmiamaaaaaamgkmgm
mlaaadpopmbicacbbpbppbppaaaaeaaaliaihaabbpbpppplaaaaeaaabadifaab
bpbpppnjaaaaeaaabacieaabbpbppgiiaaaaeaaabaeiaaabbpbppeehaaaaeaaa
miabaaaaaaloloaapaacacaafiioaeaaaablabgmobaeaaiamiabaaaaaaloloaa
paababaamiadaaagaagnblmgilafpoppmiahaaaeaamamaaakbaeabaamiaiaaab
aegngnlbnbagagpofibhaaacaablmagmobaeaciamiahaaafaagmmaaaobaaabaa
miaoaaahaapmpmaaoaacafaacabbadaaaamdmdmgpaahahadficdadabaalalagm
obagagiamiapaaadaaaalaaaobahadaamiaeaaabaagmblaaobadacaamiadaaab
aamemfaaoaababaakaebagaaaagmblbloaababibfiioagaeaapmpmgmkbaeaaia
beabaaadaamdlogmnaadagacambcaeadaalolomgnaacagpokiboacacaadmnleb
mbagagadkiceacadaalomdecnaafacadkieoacadaakggmedicadppadebbeadab
aelblbmgkaadpoideaepabaaaaaabimgobaeadabdiieababaamggmgmkbabaeaa
diehabacaamablmgobacababmiaeaaabaamgbllbilabppppmiaiiaaaaamggmaa
kcabppaamiahaaaaaamamabfklacaaaamiahiaaaaalbmaaaobabaaaaaaaaaaaa
aaaaaaaaaaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "SPOT" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
Float 4 [_FresnelPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_LightTexture0] 2D
SetTexture 4 [_LightTextureB0] 2D
"sce_fp_rsx // 69 instructions using 5 registers
[Configuration]
24
ffffffff0003c020000ffff1000000000000840005000000
[Offsets]
5
_LightColor0 2 0
000003b0000002b0
_Color 1 0
00000050
_Shininess 1 0
000000b0
_Gloss 1 0
00000020
_FresnelPower 1 0
00000380
[Microcode]
1104
9e041700c8011c9dc8000001c8003fe108820240fe081c9d00020000c8000001
00000000000000000000000000000000fe060100c8011c9dc8000001c8003fe1
0e8a0240c8081c9dc8020001c800000100000000000000000000000000000000
8e001702c8011c9dc8000001c8003fe1ce843940c8011c9dc8000029c800bfe1
0e82024055041c9dc8000001c800000106003a00c80c1c9dfe0c0001c8000001
1088014000021c9cc8000001c800000100000000000000000000000000000000
06000300c8001c9d00020000c800000100003f00000000000000000000000000
10880240c9101c9daa020000c800000100000000000042000000000000000000
10021706c8001c9dc8000001c8000001ae803940c8011c9dc8000029c800bfe1
1e7e7d00c8001c9dc8000001c80000010e080340c9081c9dc9000001c8000001
02000500c80c1c9dc80c0001c800000110800d00540c1c9d00020000c8000001
0000000000000000000000000000000094061704c8011c9dc8000001c8003fe1
0e903940c8101c9dc8000029c8000001068c0440ce0c1c9d00020000aa020000
000040000000bf8000000000000000001e7e7d00c8001c9dc8000001c8000001
10800240c9001c9dc8040001c8000001a8060500c8011c9dc8010001c800bfe1
08800240ab181c9cab180000c80000010880044001181c9e01180000c9000003
08800340c9001c9d00020000c800000100003f80000000000000000000000000
088c3b40c9003c9d55000001c800000108800540c9181c9dc9080001c8000001
0e883940c9181c9dc8000029c8000001028c0540c9181c9dc9200001c8000001
1002090001181c9c00020000c800000100000000000000000000000000000000
ae063b00c8011c9d540c0001c800bfe110061d00fe041c9dc8000001c8000001
0e840240c9141c9dc8020001c800000100000000000000000000000000000000
10020200c80c1c9dc9100001c800000108020500c80c1c9dc9100001c8000001
0204090054041c9d00020000c800000100000000000000000000000000000000
08021c00fe041c9dc8000001c800000102048300c8081c9f00020000c8000001
00003f800000000000000000000000001084090055001c9daa020000c8000001
000000000000000000000000000000000e840240c9081c9dff080001c8000001
10021d00c8081c9dc8000001c800000110020200c8041c9d00020000c8000001
000000000000000000000000000000000e82020054041c9dc9040001c8000001
0e820440c9041c9dc8020001c908000100000000000000000000000000000000
02021c00fe041c9dc8000001c80000010200170800001c9cc8000001c8000001
10820240c9001c9d00000000c800000102000400c8041c9d00020000aa020000
d85a3f4b9e993e5000000000000000001e7e7e00c8001c9dc8000001c8000001
1080090000001c9c00020000c800000100000000000000000000000000000000
0e810240ff041c9dc9041001c8000001
"
}

SubProgram "d3d11 " {
Keywords { "SPOT" }
ConstBuffer "$Globals" 176 // 160 used size, 11 vars
Vector 16 [_LightColor0] 4
Vector 112 [_Color] 4
Float 148 [_Shininess]
Float 152 [_Gloss]
Float 156 [_FresnelPower]
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 2
SetTexture 1 [_SpecMap] 2D 4
SetTexture 2 [_BumpMap] 2D 3
SetTexture 3 [_LightTexture0] 2D 0
SetTexture 4 [_LightTextureB0] 2D 1
// 54 instructions, 6 temp regs, 0 temp arrays:
// ALU 41 float, 0 int, 1 uint
// TEX 5 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0
eefiecedbgobooallbdhfefodaogkhigifndpmgcabaaaaaabiaiaaaaadaaaaaa
cmaaaaaammaaaaaaaaabaaaaejfdeheojiaaaaaaafaaaaaaaiaaaaaaiaaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaadadaaaaimaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaahahaaaaimaaaaaa
adaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapapaaaafdfgfpfaepfdejfeejepeoaa
feeffiedepepfceeaaklklklepfdeheocmaaaaaaabaaaaaaaiaaaaaacaaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaafdfgfpfegbhcghgfheaaklkl
fdeieefcbaahaaaaeaaaaaaameabaaaafjaaaaaeegiocaaaaaaaaaaaakaaaaaa
fkaaaaadaagabaaaaaaaaaaafkaaaaadaagabaaaabaaaaaafkaaaaadaagabaaa
acaaaaaafkaaaaadaagabaaaadaaaaaafkaaaaadaagabaaaaeaaaaaafibiaaae
aahabaaaaaaaaaaaffffaaaafibiaaaeaahabaaaabaaaaaaffffaaaafibiaaae
aahabaaaacaaaaaaffffaaaafibiaaaeaahabaaaadaaaaaaffffaaaafibiaaae
aahabaaaaeaaaaaaffffaaaagcbaaaaddcbabaaaabaaaaaagcbaaaadhcbabaaa
acaaaaaagcbaaaadhcbabaaaadaaaaaagcbaaaadpcbabaaaaeaaaaaagfaaaaad
pccabaaaaaaaaaaagiaaaaacagaaaaaaaoaaaaahdcaabaaaaaaaaaaaegbabaaa
aeaaaaaapgbpbaaaaeaaaaaaaaaaaaakdcaabaaaaaaaaaaaegaabaaaaaaaaaaa
aceaaaaaaaaaaadpaaaaaadpaaaaaaaaaaaaaaaaefaaaaajpcaabaaaaaaaaaaa
egaabaaaaaaaaaaaeghobaaaadaaaaaaaagabaaaaaaaaaaadbaaaaahbcaabaaa
aaaaaaaaabeaaaaaaaaaaaaackbabaaaaeaaaaaaabaaaaahbcaabaaaaaaaaaaa
akaabaaaaaaaaaaaabeaaaaaaaaaiadpdiaaaaahbcaabaaaaaaaaaaadkaabaaa
aaaaaaaaakaabaaaaaaaaaaabaaaaaahccaabaaaaaaaaaaaegbcbaaaaeaaaaaa
egbcbaaaaeaaaaaaefaaaaajpcaabaaaabaaaaaafgafbaaaaaaaaaaaeghobaaa
aeaaaaaaaagabaaaabaaaaaaapaaaaahbcaabaaaaaaaaaaaagaabaaaaaaaaaaa
agaabaaaabaaaaaaefaaaaajpcaabaaaabaaaaaaegbabaaaabaaaaaaeghobaaa
abaaaaaaaagabaaaaeaaaaaaefaaaaajpcaabaaaacaaaaaaegbabaaaabaaaaaa
eghobaaaaaaaaaaaaagabaaaacaaaaaadiaaaaahocaabaaaaaaaaaaaagajbaaa
abaaaaaapgapbaaaacaaaaaadiaaaaaihcaabaaaabaaaaaaegacbaaaacaaaaaa
egiccaaaaaaaaaaaahaaaaaadiaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaa
egiccaaaaaaaaaaaabaaaaaadiaaaaaiocaabaaaaaaaaaaafgaobaaaaaaaaaaa
kgikcaaaaaaaaaaaajaaaaaadiaaaaaiicaabaaaabaaaaaabkiacaaaaaaaaaaa
ajaaaaaaabeaaaaaaaaaaaecbaaaaaahbcaabaaaacaaaaaaegbcbaaaacaaaaaa
egbcbaaaacaaaaaaeeaaaaafbcaabaaaacaaaaaaakaabaaaacaaaaaadiaaaaah
hcaabaaaacaaaaaaagaabaaaacaaaaaaegbcbaaaacaaaaaabaaaaaahicaabaaa
acaaaaaaegbcbaaaadaaaaaaegbcbaaaadaaaaaaeeaaaaaficaabaaaacaaaaaa
dkaabaaaacaaaaaadcaaaaajhcaabaaaadaaaaaaegbcbaaaadaaaaaapgapbaaa
acaaaaaaegacbaaaacaaaaaadiaaaaahhcaabaaaaeaaaaaapgapbaaaacaaaaaa
egbcbaaaadaaaaaabaaaaaahicaabaaaacaaaaaaegacbaaaadaaaaaaegacbaaa
adaaaaaaeeaaaaaficaabaaaacaaaaaadkaabaaaacaaaaaadiaaaaahhcaabaaa
adaaaaaapgapbaaaacaaaaaaegacbaaaadaaaaaaefaaaaajpcaabaaaafaaaaaa
egbabaaaabaaaaaaeghobaaaacaaaaaaaagabaaaadaaaaaadcaaaaapdcaabaaa
afaaaaaahgapbaaaafaaaaaaaceaaaaaaaaaaaeaaaaaaaeaaaaaaaaaaaaaaaaa
aceaaaaaaaaaialpaaaaialpaaaaaaaaaaaaaaaadcaaaaakicaabaaaacaaaaaa
akaabaiaebaaaaaaafaaaaaaakaabaaaafaaaaaaabeaaaaaaaaaiadpdcaaaaak
icaabaaaacaaaaaabkaabaiaebaaaaaaafaaaaaabkaabaaaafaaaaaadkaabaaa
acaaaaaaelaaaaafecaabaaaafaaaaaadkaabaaaacaaaaaabaaaaaahicaabaaa
acaaaaaaegacbaaaafaaaaaaegacbaaaadaaaaaadeaaaaahicaabaaaacaaaaaa
dkaabaaaacaaaaaaabeaaaaaaaaaaaaacpaaaaaficaabaaaacaaaaaadkaabaaa
acaaaaaadiaaaaahicaabaaaabaaaaaadkaabaaaabaaaaaadkaabaaaacaaaaaa
bjaaaaaficaabaaaabaaaaaadkaabaaaabaaaaaadiaaaaahocaabaaaaaaaaaaa
fgaobaaaaaaaaaaapgapbaaaabaaaaaadiaaaaaiocaabaaaaaaaaaaafgaobaaa
aaaaaaaaagijcaaaaaaaaaaaabaaaaaabaaaaaahicaabaaaabaaaaaaegacbaaa
afaaaaaaegacbaaaaeaaaaaadeaaaaahicaabaaaabaaaaaadkaabaaaabaaaaaa
abeaaaaaaaaaaaaadcaaaaajocaabaaaaaaaaaaaagajbaaaabaaaaaapgapbaaa
abaaaaaafgaobaaaaaaaaaaadiaaaaahhccabaaaaaaaaaaaagaabaaaaaaaaaaa
jgahbaaaaaaaaaaabaaaaaahbcaabaaaaaaaaaaaegacbaaaafaaaaaaegacbaaa
afaaaaaaeeaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaadiaaaaahhcaabaaa
aaaaaaaaagaabaaaaaaaaaaaegacbaaaafaaaaaabaaaaaahbcaabaaaaaaaaaaa
egacbaaaacaaaaaaegacbaaaaaaaaaaadeaaaaahbcaabaaaaaaaaaaaakaabaaa
aaaaaaaaabeaaaaaaaaaaaaaaaaaaaaibcaabaaaaaaaaaaaakaabaiaebaaaaaa
aaaaaaaaabeaaaaaaaaaiadpdeaaaaahbcaabaaaaaaaaaaaakaabaaaaaaaaaaa
abeaaaaaaaaaaaaacpaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaadiaaaaai
bcaabaaaaaaaaaaaakaabaaaaaaaaaaadkiacaaaaaaaaaaaajaaaaaabjaaaaaf
bcaabaaaaaaaaaaaakaabaaaaaaaaaaadcaaaaajiccabaaaaaaaaaaaakaabaaa
aaaaaaaaabeaaaaafknieldpabeaaaaajjjofadodoaaaaab"
}

SubProgram "gles " {
Keywords { "SPOT" }
"!!GLES"
}

SubProgram "glesdesktop " {
Keywords { "SPOT" }
"!!GLES"
}

SubProgram "opengl " {
Keywords { "POINT_COOKIE" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
Float 4 [_FresnelPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_LightTextureB0] 2D
SetTexture 4 [_LightTexture0] CUBE
"3.0-!!ARBfp1.0
OPTION ARB_precision_hint_fastest;
# 51 ALU, 5 TEX
PARAM c[7] = { program.local[0..4],
		{ 2, 1, 0, 0.79627001 },
		{ 0.20373, 32 } };
TEMP R0;
TEMP R1;
TEMP R2;
TEX R0.yw, fragment.texcoord[0], texture[2], 2D;
MAD R1.xy, R0.wyzw, c[5].x, -c[5].y;
MUL R0.x, R1.y, R1.y;
MAD R0.x, -R1, R1, -R0;
ADD R0.x, R0, c[5].y;
RSQ R0.x, R0.x;
RCP R1.z, R0.x;
DP3 R0.y, R1, R1;
RSQ R0.y, R0.y;
DP3 R0.x, fragment.texcoord[1], fragment.texcoord[1];
MUL R2.xyz, R0.y, R1;
RSQ R0.x, R0.x;
MUL R0.xyz, R0.x, fragment.texcoord[1];
DP3 R0.w, R0, R2;
DP3 R1.w, fragment.texcoord[2], fragment.texcoord[2];
RSQ R0.x, R1.w;
DP3 R1.w, fragment.texcoord[1], fragment.texcoord[1];
MAX R0.w, R0, c[5].z;
ADD_SAT R0.w, -R0, c[5].y;
MUL R0.xyz, R0.x, fragment.texcoord[2];
RSQ R1.w, R1.w;
MAD R2.xyz, fragment.texcoord[1], R1.w, R0;
DP3 R1.w, R2, R2;
RSQ R1.w, R1.w;
MUL R2.xyz, R1.w, R2;
DP3 R0.x, R1, R0;
DP3 R0.y, R1, R2;
TEX R1, fragment.texcoord[0], texture[0], 2D;
MUL R1.xyz, R1, c[1];
MUL R1.xyz, R1, c[0];
MAX R2.x, R0, c[5].z;
MAX R2.y, R0, c[5].z;
TEX R0.xyz, fragment.texcoord[0], texture[1], 2D;
MUL R0.xyz, R1.w, R0;
MOV R2.z, c[6].y;
MUL R1.w, R2.z, c[2].x;
POW R1.w, R2.y, R1.w;
MUL R0.xyz, R0, c[3].x;
MUL R0.xyz, R1.w, R0;
MUL R0.xyz, R0, c[0];
MAD R0.xyz, R1, R2.x, R0;
POW R0.w, R0.w, c[4].x;
MUL R1.x, R0.w, c[5].w;
DP3 R1.y, fragment.texcoord[3], fragment.texcoord[3];
TEX R0.w, fragment.texcoord[3], texture[4], CUBE;
TEX R1.w, R1.y, texture[3], 2D;
MUL R0.w, R1, R0;
MUL R2.xyz, R0.w, R0;
ADD R0.x, R1, c[6];
MUL result.color.xyz, R2, c[5].x;
MAX result.color.w, R0.x, c[5].z;
END
# 51 instructions, 3 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "POINT_COOKIE" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
Float 4 [_FresnelPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_LightTextureB0] 2D
SetTexture 4 [_LightTexture0] CUBE
"ps_3_0
; 51 ALU, 5 TEX
dcl_2d s0
dcl_2d s1
dcl_2d s2
dcl_2d s3
dcl_cube s4
def c5, 2.00000000, -1.00000000, 1.00000000, 0.00000000
def c6, 0.79627001, 0.20373000, 32.00000000, 0
dcl_texcoord0 v0.xy
dcl_texcoord1 v1.xyz
dcl_texcoord2 v2.xyz
dcl_texcoord3 v3.xyz
texld r0.yw, v0, s2
mad_pp r1.xy, r0.wyzw, c5.x, c5.y
mul_pp r0.x, r1.y, r1.y
mad_pp r0.x, -r1, r1, -r0
add_pp r0.x, r0, c5.z
rsq_pp r0.x, r0.x
rcp_pp r1.z, r0.x
dp3_pp r0.y, r1, r1
rsq_pp r0.y, r0.y
dp3 r0.x, v1, v1
mul_pp r2.xyz, r0.y, r1
rsq r0.x, r0.x
mul r0.xyz, r0.x, v1
dp3 r0.x, r0, r2
max r0.x, r0, c5.w
add_sat r1.w, -r0.x, c5.z
pow r0, r1.w, c4.x
dp3_pp r2.x, v2, v2
rsq_pp r0.y, r2.x
mul_pp r2.xyz, r0.y, v2
dp3_pp r0.z, v1, v1
rsq_pp r0.y, r0.z
mad_pp r3.xyz, v1, r0.y, r2
dp3_pp r0.y, r1, r2
dp3_pp r0.z, r3, r3
rsq_pp r0.z, r0.z
mul_pp r2.xyz, r0.z, r3
dp3_pp r0.z, r1, r2
texld r1, v0, s0
mov_pp r0.w, c2.x
texld r2.xyz, v0, s1
mul_pp r2.xyz, r1.w, r2
mul_pp r1.xyz, r1, c1
max_pp r0.y, r0, c5.w
mul_pp r0.w, c6.z, r0
max_pp r0.z, r0, c5.w
pow r3, r0.z, r0.w
mul_pp r2.xyz, r2, c3.x
mov r0.z, r3.x
mul r2.xyz, r0.z, r2
mul_pp r2.xyz, r2, c0
mul_pp r1.xyz, r1, c0
mad_pp r1.xyz, r1, r0.y, r2
mov r0.y, r0.x
dp3 r0.x, v3, v3
texld r0.w, v3, s4
texld r0.x, r0.x, s3
mul r0.x, r0, r0.w
mul_pp r1.xyz, r0.x, r1
mad r0.x, r0.y, c6, c6.y
mul_pp oC0.xyz, r1, c5.x
max oC0.w, r0.x, c5
"
}

SubProgram "xbox360 " {
Keywords { "POINT_COOKIE" }
Vector 1 [_Color]
Float 4 [_FresnelPower]
Float 3 [_Gloss]
Vector 0 [_LightColor0]
Float 2 [_Shininess]
SetTexture 0 [_LightTexture0] CUBE
SetTexture 1 [_LightTextureB0] 2D
SetTexture 2 [_MainTex] 2D
SetTexture 3 [_BumpMap] 2D
SetTexture 4 [_SpecMap] 2D
// Shader Timing Estimate, in Cycles/64 pixel vector:
// ALU: 41.33 (31 instructions), vertex: 0, texture: 20,
//   sequencer: 16, interpolator: 16;    8 GPRs, 24 threads,
// Performance (if enough threads): ~41 cycles per vector
// * Texture cycle estimates are assuming an 8bit/component texture with no
//     aniso or trilinear filtering.

"ps_360
backbbaaaaaaaccmaaaaaccmaaaaaaaaaaaaaaceaaaaabneaaaaabpmaaaaaaaa
aaaaaaaaaaaaabkmaaaaaabmaaaaabkappppadaaaaaaaaakaaaaaabmaaaaaaaa
aaaaabjjaaaaaaoeaaadaaadaaabaaaaaaaaaapaaaaaaaaaaaaaabaaaaacaaab
aaabaaaaaaaaabaiaaaaaaaaaaaaabbiaaacaaaeaaabaaaaaaaaabciaaaaaaaa
aaaaabdiaaacaaadaaabaaaaaaaaabciaaaaaaaaaaaaabdpaaacaaaaaaabaaaa
aaaaabaiaaaaaaaaaaaaabemaaadaaaaaaabaaaaaaaaabfmaaaaaaaaaaaaabgm
aaadaaabaaabaaaaaaaaaapaaaaaaaaaaaaaabhmaaadaaacaaabaaaaaaaaaapa
aaaaaaaaaaaaabifaaacaaacaaabaaaaaaaaabciaaaaaaaaaaaaabjaaaadaaae
aaabaaaaaaaaaapaaaaaaaaafpechfgnhaengbhaaaklklklaaaeaaamaaabaaab
aaabaaaaaaaaaaaafpedgpgmgphcaaklaaabaaadaaabaaaeaaabaaaaaaaaaaaa
fpeghcgfhdgogfgmfagphhgfhcaaklklaaaaaaadaaabaaabaaabaaaaaaaaaaaa
fpehgmgphdhdaafpemgjghgiheedgpgmgphcdaaafpemgjghgihefegfhihehfhc
gfdaaaklaaaeaaaoaaabaaabaaabaaaaaaaaaaaafpemgjghgihefegfhihehfhc
gfecdaaafpengbgjgofegfhiaafpfdgigjgogjgogfhdhdaafpfdhagfgdengbha
aahahdfpddfpdaaadccodacodcdadddfddcodaaaaaaaaaaaaaaaaaabaaaaaaaa
aaaaaaaaaaaaaabeabpmaabaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaea
aaaaabombaaaahaaaaaaaaaeaaaaaaaaaaaacmieaaapaaapaaaaaaabaaaadafa
aaaahbfbaaaahcfcaaaahdfdaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaeaaaaaaadpmaaaaadpiaaaaaecaaaaaaaaaaaaaa
dofajojjlpiaaaaadpelnifkafaagaaedaakbcaabcaaaabfaaaaaaaagaanmeaa
bcaaaaaaaaaagabdgabjbcaabcaaaaaaaaaagabpdacfbcaaccaaaaaamiaiaaaa
aaloloaapaadadaamiapaaadaakgmnaapcadadaaemeeaaaeaablblmgocadadid
miadaaaeaagnmglbmladaapobaeidaabbpbppoiiaaaaeaaabacifaabbpbppeal
aaaaeaaabadiaaabbpbpppnjaaaaeaaajaaiaaibbpbppoppaaaamaaapmbicaab
bpbppbppaaaaeaaamiabaaaeaagmblaacbacpoaamiaiaaabaaloloaapaababaa
beaiaaaaaalolomgpaacacaamiadaaahaagngmmgilaapoppamehafaaaalhlobl
kbafabacfiioacadaagmpmblobafadiamiaiaaaaaegngnmgnbahahpofiioabae
aablpmblobacacibaibhafagaablmagmobababahaicoafacaaabpmlboaaeagah
kmbdacabaamemfeboaafafadkaeeahabaamdmdblpaacaciafieiabaaaagmblmg
oaabaaibfiioahacaaabmgblobacabiakmcbacafaamdloecpaacahadkmecacaf
aamdloedpaaeahadkichaeadaabamlebmbahahaakieeaeafaaloloicnaagadaa
kiioaeadaakggmmaicafppaaebbeadabaelbmgmgkaadpoideaepabaaaaaabimg
obaeadabdiieababaamggmgmkbabaeaadiehabacaamablmgobacababmiaeaaab
aamgbllbilabppppmiaiiaaaaamggmaakcabppaamiahaaaaaamamabfklacaaaa
miahiaaaaalbmaaaobabaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "POINT_COOKIE" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
Float 4 [_FresnelPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_LightTextureB0] 2D
SetTexture 4 [_LightTexture0] CUBE
"sce_fp_rsx // 61 instructions using 4 registers
[Configuration]
24
ffffffff0003c020000ffff1000000000000840004000000
[Offsets]
5
_LightColor0 2 0
00000350000001f0
_Color 1 0
00000100
_Shininess 1 0
00000140
_Gloss 1 0
00000090
_FresnelPower 1 0
00000320
[Microcode]
976
94021704c8011c9dc8000001c8003fe106860440ce041c9d00020000aa020000
000040000000bf800000000000000000ee000100c8011c9dc8000001c8003fe1
10060500c8001c9dc8000001c800000110800240ab0c1c9cab0c0000c8000001
9e041700c8011c9dc8000001c8003fe1ce823940c8011c9dc8000029c800bfe1
10820240c8081c9d00020000c800000100000000000000000000000000000000
8e061702c8011c9dc8000001c8003fe10e840240ff041c9dc80c0001c8000001
10840440010c1c9e010c0000c900000310840340c9081c9d00020000c8000001
00003f800000000000000000000000000e880240c8081c9dc8020001c8000001
00000000000000000000000000000000ae803940c8011c9dc8000029c800bfe1
0e060340c9041c9dc9000001c80000011080014000021c9cc8000001c8000001
0000000000000000000000000000000008863b40ff083c9dff080001c8000001
10840240c9001c9d00020000c800000100004200000000000000000000000000
02001706fe0c1c9dc8000001c80000010e8a3940c80c1c9dc8000029c8000001
10880540c90c1c9dc9140001c8000001a4000500c8011c9dc8010001c800bfe1
108a0540c90c1c9dc9040001c80000010e8a3940c90c1c9dc8000029c8000001
0e860240c9101c9dc8020001c800000100000000000000000000000000000000
10860900c9141c9dc8020001c800000100000000000000000000000000000000
ae063b00c8011c9daa000000c800bfe110060900c9101c9d00020000c8000001
0000000000000000000000000000000004041d00fe0c1c9dc8000001c8000001
02040500c80c1c9dc9140001c80000011000090000081c9c00020000c8000001
0000000000000000000000000000000004040200c8081c9dff080001c8000001
04041c00aa081c9cc8000001c800000102048300fe001c9f00020000c8000001
00003f800000000000000000000000000e860240c90c1c9dff0c0001c8000001
04001d00c8081c9dc8000001c80000010e840200aa081c9cc9080001c8000001
f0001708c8011c9dc8000001c8003fe110040200aa001c9c00020000c8000001
000000000000000000000000000000001e7e7d00c8001c9dc8000001c8000001
0e840440c9081c9dc8020001c90c000100000000000000000000000000000000
1082020000001c9cc8000001c800000108021c00fe081c9dc8000001c8000001
0200040054041c9d00020000aa020000d85a3f4b9e993e500000000000000000
1080090000001c9c00020000c800000100000000000000000000000000000000
0e810240ff041c9dc9081001c8000001
"
}

SubProgram "d3d11 " {
Keywords { "POINT_COOKIE" }
ConstBuffer "$Globals" 176 // 160 used size, 11 vars
Vector 16 [_LightColor0] 4
Vector 112 [_Color] 4
Float 148 [_Shininess]
Float 152 [_Gloss]
Float 156 [_FresnelPower]
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 2
SetTexture 1 [_SpecMap] 2D 4
SetTexture 2 [_BumpMap] 2D 3
SetTexture 3 [_LightTextureB0] 2D 1
SetTexture 4 [_LightTexture0] CUBE 0
// 49 instructions, 6 temp regs, 0 temp arrays:
// ALU 37 float, 0 int, 0 uint
// TEX 5 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0
eefiecedbeiadpkbbkmkgfnnchdfcgiboiccnkdbabaaaaaaiaahaaaaadaaaaaa
cmaaaaaammaaaaaaaaabaaaaejfdeheojiaaaaaaafaaaaaaaiaaaaaaiaaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaadadaaaaimaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaahahaaaaimaaaaaa
adaaaaaaaaaaaaaaadaaaaaaaeaaaaaaahahaaaafdfgfpfaepfdejfeejepeoaa
feeffiedepepfceeaaklklklepfdeheocmaaaaaaabaaaaaaaiaaaaaacaaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaafdfgfpfegbhcghgfheaaklkl
fdeieefchiagaaaaeaaaaaaajoabaaaafjaaaaaeegiocaaaaaaaaaaaakaaaaaa
fkaaaaadaagabaaaaaaaaaaafkaaaaadaagabaaaabaaaaaafkaaaaadaagabaaa
acaaaaaafkaaaaadaagabaaaadaaaaaafkaaaaadaagabaaaaeaaaaaafibiaaae
aahabaaaaaaaaaaaffffaaaafibiaaaeaahabaaaabaaaaaaffffaaaafibiaaae
aahabaaaacaaaaaaffffaaaafibiaaaeaahabaaaadaaaaaaffffaaaafidaaaae
aahabaaaaeaaaaaaffffaaaagcbaaaaddcbabaaaabaaaaaagcbaaaadhcbabaaa
acaaaaaagcbaaaadhcbabaaaadaaaaaagcbaaaadhcbabaaaaeaaaaaagfaaaaad
pccabaaaaaaaaaaagiaaaaacagaaaaaaefaaaaajpcaabaaaaaaaaaaaegbabaaa
abaaaaaaeghobaaaabaaaaaaaagabaaaaeaaaaaaefaaaaajpcaabaaaabaaaaaa
egbabaaaabaaaaaaeghobaaaaaaaaaaaaagabaaaacaaaaaadiaaaaahhcaabaaa
aaaaaaaaegacbaaaaaaaaaaapgapbaaaabaaaaaadiaaaaaihcaabaaaabaaaaaa
egacbaaaabaaaaaaegiccaaaaaaaaaaaahaaaaaadiaaaaaihcaabaaaabaaaaaa
egacbaaaabaaaaaaegiccaaaaaaaaaaaabaaaaaadiaaaaaihcaabaaaaaaaaaaa
egacbaaaaaaaaaaakgikcaaaaaaaaaaaajaaaaaadiaaaaaiicaabaaaaaaaaaaa
bkiacaaaaaaaaaaaajaaaaaaabeaaaaaaaaaaaecbaaaaaahicaabaaaabaaaaaa
egbcbaaaacaaaaaaegbcbaaaacaaaaaaeeaaaaaficaabaaaabaaaaaadkaabaaa
abaaaaaadiaaaaahhcaabaaaacaaaaaapgapbaaaabaaaaaaegbcbaaaacaaaaaa
baaaaaahicaabaaaabaaaaaaegbcbaaaadaaaaaaegbcbaaaadaaaaaaeeaaaaaf
icaabaaaabaaaaaadkaabaaaabaaaaaadcaaaaajhcaabaaaadaaaaaaegbcbaaa
adaaaaaapgapbaaaabaaaaaaegacbaaaacaaaaaadiaaaaahhcaabaaaaeaaaaaa
pgapbaaaabaaaaaaegbcbaaaadaaaaaabaaaaaahicaabaaaabaaaaaaegacbaaa
adaaaaaaegacbaaaadaaaaaaeeaaaaaficaabaaaabaaaaaadkaabaaaabaaaaaa
diaaaaahhcaabaaaadaaaaaapgapbaaaabaaaaaaegacbaaaadaaaaaaefaaaaaj
pcaabaaaafaaaaaaegbabaaaabaaaaaaeghobaaaacaaaaaaaagabaaaadaaaaaa
dcaaaaapdcaabaaaafaaaaaahgapbaaaafaaaaaaaceaaaaaaaaaaaeaaaaaaaea
aaaaaaaaaaaaaaaaaceaaaaaaaaaialpaaaaialpaaaaaaaaaaaaaaaadcaaaaak
icaabaaaabaaaaaaakaabaiaebaaaaaaafaaaaaaakaabaaaafaaaaaaabeaaaaa
aaaaiadpdcaaaaakicaabaaaabaaaaaabkaabaiaebaaaaaaafaaaaaabkaabaaa
afaaaaaadkaabaaaabaaaaaaelaaaaafecaabaaaafaaaaaadkaabaaaabaaaaaa
baaaaaahicaabaaaabaaaaaaegacbaaaafaaaaaaegacbaaaadaaaaaadeaaaaah
icaabaaaabaaaaaadkaabaaaabaaaaaaabeaaaaaaaaaaaaacpaaaaaficaabaaa
abaaaaaadkaabaaaabaaaaaadiaaaaahicaabaaaaaaaaaaadkaabaaaaaaaaaaa
dkaabaaaabaaaaaabjaaaaaficaabaaaaaaaaaaadkaabaaaaaaaaaaadiaaaaah
hcaabaaaaaaaaaaaegacbaaaaaaaaaaapgapbaaaaaaaaaaadiaaaaaihcaabaaa
aaaaaaaaegacbaaaaaaaaaaaegiccaaaaaaaaaaaabaaaaaabaaaaaahicaabaaa
aaaaaaaaegacbaaaafaaaaaaegacbaaaaeaaaaaadeaaaaahicaabaaaaaaaaaaa
dkaabaaaaaaaaaaaabeaaaaaaaaaaaaadcaaaaajhcaabaaaaaaaaaaaegacbaaa
abaaaaaapgapbaaaaaaaaaaaegacbaaaaaaaaaaabaaaaaahicaabaaaaaaaaaaa
egbcbaaaaeaaaaaaegbcbaaaaeaaaaaaefaaaaajpcaabaaaabaaaaaapgapbaaa
aaaaaaaaeghobaaaadaaaaaaaagabaaaabaaaaaaefaaaaajpcaabaaaadaaaaaa
egbcbaaaaeaaaaaaeghobaaaaeaaaaaaaagabaaaaaaaaaaaapaaaaahicaabaaa
aaaaaaaaagaabaaaabaaaaaapgapbaaaadaaaaaadiaaaaahhccabaaaaaaaaaaa
pgapbaaaaaaaaaaaegacbaaaaaaaaaaabaaaaaahbcaabaaaaaaaaaaaegacbaaa
afaaaaaaegacbaaaafaaaaaaeeaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaa
diaaaaahhcaabaaaaaaaaaaaagaabaaaaaaaaaaaegacbaaaafaaaaaabaaaaaah
bcaabaaaaaaaaaaaegacbaaaacaaaaaaegacbaaaaaaaaaaadeaaaaahbcaabaaa
aaaaaaaaakaabaaaaaaaaaaaabeaaaaaaaaaaaaaaaaaaaaibcaabaaaaaaaaaaa
akaabaiaebaaaaaaaaaaaaaaabeaaaaaaaaaiadpdeaaaaahbcaabaaaaaaaaaaa
akaabaaaaaaaaaaaabeaaaaaaaaaaaaacpaaaaafbcaabaaaaaaaaaaaakaabaaa
aaaaaaaadiaaaaaibcaabaaaaaaaaaaaakaabaaaaaaaaaaadkiacaaaaaaaaaaa
ajaaaaaabjaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaadcaaaaajiccabaaa
aaaaaaaaakaabaaaaaaaaaaaabeaaaaafknieldpabeaaaaajjjofadodoaaaaab
"
}

SubProgram "gles " {
Keywords { "POINT_COOKIE" }
"!!GLES"
}

SubProgram "glesdesktop " {
Keywords { "POINT_COOKIE" }
"!!GLES"
}

SubProgram "opengl " {
Keywords { "DIRECTIONAL_COOKIE" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
Float 4 [_FresnelPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_LightTexture0] 2D
"3.0-!!ARBfp1.0
OPTION ARB_precision_hint_fastest;
# 46 ALU, 4 TEX
PARAM c[7] = { program.local[0..4],
		{ 2, 1, 0, 0.79627001 },
		{ 0.20373, 32 } };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
TEX R0.yw, fragment.texcoord[0], texture[2], 2D;
MAD R1.xy, R0.wyzw, c[5].x, -c[5].y;
MUL R0.x, R1.y, R1.y;
MAD R0.x, -R1, R1, -R0;
ADD R0.x, R0, c[5].y;
RSQ R0.x, R0.x;
RCP R1.z, R0.x;
DP3 R0.y, fragment.texcoord[1], fragment.texcoord[1];
DP3 R0.x, fragment.texcoord[1], fragment.texcoord[1];
MOV R2.xyz, fragment.texcoord[2];
RSQ R0.y, R0.y;
MAD R3.xyz, fragment.texcoord[1], R0.y, R2;
DP3 R0.y, R3, R3;
RSQ R0.w, R0.y;
DP3 R0.y, R1, R1;
RSQ R0.y, R0.y;
MUL R2.xyz, R0.y, R1;
RSQ R0.x, R0.x;
MUL R0.xyz, R0.x, fragment.texcoord[1];
DP3 R0.x, R0, R2;
MUL R2.xyz, R0.w, R3;
DP3 R0.y, R1, R2;
DP3 R1.x, R1, fragment.texcoord[2];
MAX R0.x, R0, c[5].z;
ADD_SAT R1.w, -R0.x, c[5].y;
MAX R2.w, R0.y, c[5].z;
TEX R0, fragment.texcoord[0], texture[0], 2D;
TEX R2.xyz, fragment.texcoord[0], texture[1], 2D;
MUL R2.xyz, R0.w, R2;
MOV R3.x, c[6].y;
MUL R0.w, R3.x, c[2].x;
MUL R0.xyz, R0, c[1];
POW R0.w, R2.w, R0.w;
MUL R2.xyz, R2, c[3].x;
MUL R2.xyz, R0.w, R2;
POW R0.w, R1.w, c[4].x;
MUL R0.xyz, R0, c[0];
MUL R2.xyz, R2, c[0];
MAX R1.x, R1, c[5].z;
MAD R1.xyz, R0, R1.x, R2;
MUL R0.x, R0.w, c[5].w;
TEX R0.w, fragment.texcoord[3], texture[3], 2D;
MUL R1.xyz, R0.w, R1;
ADD R0.x, R0, c[6];
MUL result.color.xyz, R1, c[5].x;
MAX result.color.w, R0.x, c[5].z;
END
# 46 instructions, 4 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "DIRECTIONAL_COOKIE" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
Float 4 [_FresnelPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_LightTexture0] 2D
"ps_3_0
; 47 ALU, 4 TEX
dcl_2d s0
dcl_2d s1
dcl_2d s2
dcl_2d s3
def c5, 2.00000000, -1.00000000, 1.00000000, 0.00000000
def c6, 0.79627001, 0.20373000, 32.00000000, 0
dcl_texcoord0 v0.xy
dcl_texcoord1 v1.xyz
dcl_texcoord2 v2.xyz
dcl_texcoord3 v3.xy
texld r0.yw, v0, s2
mad_pp r2.xy, r0.wyzw, c5.x, c5.y
mul_pp r0.x, r2.y, r2.y
mad_pp r0.x, -r2, r2, -r0
add_pp r0.x, r0, c5.z
rsq_pp r0.x, r0.x
rcp_pp r2.z, r0.x
dp3 r0.x, v1, v1
rsq r0.w, r0.x
dp3_pp r0.y, r2, r2
rsq_pp r0.y, r0.y
mul r1.xyz, r0.w, v1
mul_pp r0.xyz, r0.y, r2
dp3 r0.w, r1, r0
dp3_pp r1.x, v1, v1
max r0.w, r0, c5
add_sat r0.w, -r0, c5.z
pow r4, r0.w, c4.x
rsq_pp r1.x, r1.x
mov_pp r0.xyz, v2
mad_pp r0.xyz, v1, r1.x, r0
dp3_pp r1.x, r0, r0
rsq_pp r1.x, r1.x
mul_pp r0.xyz, r1.x, r0
dp3_pp r0.x, r2, r0
mov_pp r0.w, c2.x
mul_pp r0.y, c6.z, r0.w
max_pp r0.x, r0, c5.w
pow r3, r0.x, r0.y
texld r0, v0, s0
texld r1.xyz, v0, s1
mul_pp r1.xyz, r0.w, r1
mul_pp r0.xyz, r0, c1
mov r0.w, r3.x
mul_pp r1.xyz, r1, c3.x
mul r1.xyz, r0.w, r1
dp3_pp r0.w, r2, v2
mul_pp r0.xyz, r0, c0
max_pp r0.w, r0, c5
mul_pp r1.xyz, r1, c0
mad_pp r1.xyz, r0, r0.w, r1
mov r0.x, r4
texld r0.w, v3, s3
mul_pp r1.xyz, r0.w, r1
mad r0.x, r0, c6, c6.y
mul_pp oC0.xyz, r1, c5.x
max oC0.w, r0.x, c5
"
}

SubProgram "xbox360 " {
Keywords { "DIRECTIONAL_COOKIE" }
Vector 1 [_Color]
Float 4 [_FresnelPower]
Float 3 [_Gloss]
Vector 0 [_LightColor0]
Float 2 [_Shininess]
SetTexture 0 [_LightTexture0] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_SpecMap] 2D
// Shader Timing Estimate, in Cycles/64 pixel vector:
// ALU: 32.00 (24 instructions), vertex: 0, texture: 16,
//   sequencer: 12, interpolator: 16;    8 GPRs, 24 threads,
// Performance (if enough threads): ~32 cycles per vector
// * Texture cycle estimates are assuming an 8bit/component texture with no
//     aniso or trilinear filtering.

"ps_360
backbbaaaaaaabpiaaaaabmaaaaaaaaaaaaaaaceaaaaabkaaaaaabmiaaaaaaaa
aaaaaaaaaaaaabhiaaaaaabmaaaaabglppppadaaaaaaaaajaaaaaabmaaaaaaaa
aaaaabgeaaaaaanaaaadaaacaaabaaaaaaaaaanmaaaaaaaaaaaaaaomaaacaaab
aaabaaaaaaaaaapeaaaaaaaaaaaaabaeaaacaaaeaaabaaaaaaaaabbeaaaaaaaa
aaaaabceaaacaaadaaabaaaaaaaaabbeaaaaaaaaaaaaabclaaacaaaaaaabaaaa
aaaaaapeaaaaaaaaaaaaabdiaaadaaaaaaabaaaaaaaaaanmaaaaaaaaaaaaabeh
aaadaaabaaabaaaaaaaaaanmaaaaaaaaaaaaabfaaaacaaacaaabaaaaaaaaabbe
aaaaaaaaaaaaabflaaadaaadaaabaaaaaaaaaanmaaaaaaaafpechfgnhaengbha
aaklklklaaaeaaamaaabaaabaaabaaaaaaaaaaaafpedgpgmgphcaaklaaabaaad
aaabaaaeaaabaaaaaaaaaaaafpeghcgfhdgogfgmfagphhgfhcaaklklaaaaaaad
aaabaaabaaabaaaaaaaaaaaafpehgmgphdhdaafpemgjghgiheedgpgmgphcdaaa
fpemgjghgihefegfhihehfhcgfdaaafpengbgjgofegfhiaafpfdgigjgogjgogf
hdhdaafpfdhagfgdengbhaaahahdfpddfpdaaadccodacodcdadddfddcodaaakl
aaaaaaaaaaaaaaabaaaaaaaaaaaaaaaaaaaaaabeabpmaabaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaeaaaaaabiabaaaahaaaaaaaaaeaaaaaaaaaaaaciie
aaapaaapaaaaaaabaaaadafaaaaahbfbaaaahcfcaaaaddfdaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaeaaaaaaadpiaaaaa
ecaaaaaaaaaaaaaaaaaaaaaadofajojjlpiaaaaadpelnifkaaffeaadaaaabcaa
meaaaaaaaaaagaahgaanbcaabcaaaaaaaaaagabdgabjbcaaccaaaaaabaaifagb
bpbpphppaaaaeaaabadidaabbpbppoiiaaaaeaaababieaabbpbppgiiaaaaeaaa
baciaaabbpbpppnjaaaaeaaabeaiaaaaaalologmnaababacmiadaaahaagngmmg
ilaapoppambhafaaaalolomgibaeabpofiioabadaablpmblobaeadiamiaiaaaa
aegngnlbnbahahpoaichafagaablmagmobababahaieoafaeaapmpmlboaagacah
kmbdaeabaabjbkeboaafafadkaieahabaamdmdblpaaeaeiafieiabaaaagmblmg
oaabaaibfieoahafaaabmgblobaeabiakmcbaeacaalploecpaahacadkmecaeac
aamdlpedpaafahadkichafadaamabgebmbahahaakieeafacaaloloicnaagadaa
kiioafacaakggmmaicacppaaebbeacabaelblbblkaacpoiceaepabaaaaaamemg
obafacabdiieababaamggmgmkbabaeaadiehabacaamablmgobaeababmiaeaaab
aamgbllbilabppppmiaiiaaaaamggmaakcabppaamiahaaaaaamamabfklacaaaa
miahiaaaaalbmaaaobabaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "DIRECTIONAL_COOKIE" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 2 [_Shininess]
Float 3 [_Gloss]
Float 4 [_FresnelPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_LightTexture0] 2D
"sce_fp_rsx // 57 instructions using 4 registers
[Configuration]
24
ffffffff0003c020000ffff9000000000000840004000000
[Offsets]
5
_LightColor0 2 0
0000033000000240
_Color 1 0
000000f0
_Shininess 1 0
00000040
_Gloss 1 0
00000090
_FresnelPower 1 0
00000300
[Microcode]
912
94001704c8011c9dc8000001c8003fe106860440ce001c9daa02000054020001
00000000000040000000bf80000000001084014000021c9cc8000001c8000001
000000000000000000000000000000009e041700c8011c9dc8000001c8003fe1
ae823940c8011c9dc8000029c800bfe102800240ab0c1c9cab0c0000c8000001
10820240c8081c9d00020000c800000100000000000000000000000000000000
8e061702c8011c9dc8000001c8003fe110800440010c1c9e010c000001000002
0e800240ff041c9dc80c0001c8000001b0060500c8011c9dc8010001c800bfe1
0e880240c8081c9dc8020001c800000100000000000000000000000000000000
10880240c9081c9dc8020001c800000100000000000000000000000000004200
ce8c0140c8011c9dc8000001c8003fe110800340c9001c9d00020000c8000001
00003f8000000000000000000000000008863b40ff003c9dff000001c8000001
10800540c90c1c9dc9180001c80000010e843940c90c1c9dc8000029c8000001
0e060340c9181c9dc9040001c800000110840900c9001c9d00020000c8000001
00000000000000000000000000000000f0001706c8011c9dc8000001c8003fe1
0e8a3940c80c1c9dc8000029c800000102820540c90c1c9dc9140001c8000001
1002090001041c9c00020000c800000100000000000000000000000000000000
ae063b00c8011c9dfe0c0001c800bfe110061d00fe041c9dc8000001c8000001
08000500c80c1c9dc9080001c80000010e860240c9101c9dc8020001c8000001
0000000000000000000000000000000010880100c9101c9dc8000001c8000001
10040200c80c1c9dc9100001c80000010204090054001c9d00020000c8000001
0000000000000000000000000000000008001c00fe081c9dc8000001c8000001
02048300c8081c9f00020000c800000100003f80000000000000000000000000
0e80020054001c9dc9000001c80000010e840240c90c1c9dff080001c8000001
08021d00c8081c9dc8000001c80000011002020054041c9d00020000c8000001
0000000000000000000000000000000008001c00fe041c9dc8000001c8000001
0e800440c9001c9dc8020001c908000100000000000000000000000000000000
0202040054001c9d00020000aa020000d85a3f4b9e993e500000000000000000
1080090000041c9c00020000c800000100000000000000000000000000000000
0e810240fe001c9dc9001001c8000001
"
}

SubProgram "d3d11 " {
Keywords { "DIRECTIONAL_COOKIE" }
ConstBuffer "$Globals" 176 // 160 used size, 11 vars
Vector 16 [_LightColor0] 4
Vector 112 [_Color] 4
Float 148 [_Shininess]
Float 152 [_Gloss]
Float 156 [_FresnelPower]
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 1
SetTexture 1 [_SpecMap] 2D 3
SetTexture 2 [_BumpMap] 2D 2
SetTexture 3 [_LightTexture0] 2D 0
// 44 instructions, 5 temp regs, 0 temp arrays:
// ALU 33 float, 0 int, 0 uint
// TEX 4 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0
eefiecedidfcbfejfifdkdkhmilpnmkpblknnhcjabaaaaaaniagaaaaadaaaaaa
cmaaaaaammaaaaaaaaabaaaaejfdeheojiaaaaaaafaaaaaaaiaaaaaaiaaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaadadaaaaimaaaaaaadaaaaaaaaaaaaaaadaaaaaaabaaaaaa
amamaaaaimaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaaahahaaaaimaaaaaa
acaaaaaaaaaaaaaaadaaaaaaadaaaaaaahahaaaafdfgfpfaepfdejfeejepeoaa
feeffiedepepfceeaaklklklepfdeheocmaaaaaaabaaaaaaaiaaaaaacaaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaafdfgfpfegbhcghgfheaaklkl
fdeieefcnaafaaaaeaaaaaaaheabaaaafjaaaaaeegiocaaaaaaaaaaaakaaaaaa
fkaaaaadaagabaaaaaaaaaaafkaaaaadaagabaaaabaaaaaafkaaaaadaagabaaa
acaaaaaafkaaaaadaagabaaaadaaaaaafibiaaaeaahabaaaaaaaaaaaffffaaaa
fibiaaaeaahabaaaabaaaaaaffffaaaafibiaaaeaahabaaaacaaaaaaffffaaaa
fibiaaaeaahabaaaadaaaaaaffffaaaagcbaaaaddcbabaaaabaaaaaagcbaaaad
mcbabaaaabaaaaaagcbaaaadhcbabaaaacaaaaaagcbaaaadhcbabaaaadaaaaaa
gfaaaaadpccabaaaaaaaaaaagiaaaaacafaaaaaaefaaaaajpcaabaaaaaaaaaaa
egbabaaaabaaaaaaeghobaaaabaaaaaaaagabaaaadaaaaaaefaaaaajpcaabaaa
abaaaaaaegbabaaaabaaaaaaeghobaaaaaaaaaaaaagabaaaabaaaaaadiaaaaah
hcaabaaaaaaaaaaaegacbaaaaaaaaaaapgapbaaaabaaaaaadiaaaaaihcaabaaa
abaaaaaaegacbaaaabaaaaaaegiccaaaaaaaaaaaahaaaaaadiaaaaaihcaabaaa
abaaaaaaegacbaaaabaaaaaaegiccaaaaaaaaaaaabaaaaaadiaaaaaihcaabaaa
aaaaaaaaegacbaaaaaaaaaaakgikcaaaaaaaaaaaajaaaaaadiaaaaaiicaabaaa
aaaaaaaabkiacaaaaaaaaaaaajaaaaaaabeaaaaaaaaaaaecbaaaaaahicaabaaa
abaaaaaaegbcbaaaacaaaaaaegbcbaaaacaaaaaaeeaaaaaficaabaaaabaaaaaa
dkaabaaaabaaaaaadcaaaaajhcaabaaaacaaaaaaegbcbaaaacaaaaaapgapbaaa
abaaaaaaegbcbaaaadaaaaaadiaaaaahhcaabaaaadaaaaaapgapbaaaabaaaaaa
egbcbaaaacaaaaaabaaaaaahicaabaaaabaaaaaaegacbaaaacaaaaaaegacbaaa
acaaaaaaeeaaaaaficaabaaaabaaaaaadkaabaaaabaaaaaadiaaaaahhcaabaaa
acaaaaaapgapbaaaabaaaaaaegacbaaaacaaaaaaefaaaaajpcaabaaaaeaaaaaa
egbabaaaabaaaaaaeghobaaaacaaaaaaaagabaaaacaaaaaadcaaaaapdcaabaaa
aeaaaaaahgapbaaaaeaaaaaaaceaaaaaaaaaaaeaaaaaaaeaaaaaaaaaaaaaaaaa
aceaaaaaaaaaialpaaaaialpaaaaaaaaaaaaaaaadcaaaaakicaabaaaabaaaaaa
akaabaiaebaaaaaaaeaaaaaaakaabaaaaeaaaaaaabeaaaaaaaaaiadpdcaaaaak
icaabaaaabaaaaaabkaabaiaebaaaaaaaeaaaaaabkaabaaaaeaaaaaadkaabaaa
abaaaaaaelaaaaafecaabaaaaeaaaaaadkaabaaaabaaaaaabaaaaaahicaabaaa
abaaaaaaegacbaaaaeaaaaaaegacbaaaacaaaaaadeaaaaahicaabaaaabaaaaaa
dkaabaaaabaaaaaaabeaaaaaaaaaaaaacpaaaaaficaabaaaabaaaaaadkaabaaa
abaaaaaadiaaaaahicaabaaaaaaaaaaadkaabaaaaaaaaaaadkaabaaaabaaaaaa
bjaaaaaficaabaaaaaaaaaaadkaabaaaaaaaaaaadiaaaaahhcaabaaaaaaaaaaa
egacbaaaaaaaaaaapgapbaaaaaaaaaaadiaaaaaihcaabaaaaaaaaaaaegacbaaa
aaaaaaaaegiccaaaaaaaaaaaabaaaaaabaaaaaahicaabaaaaaaaaaaaegacbaaa
aeaaaaaaegbcbaaaadaaaaaadeaaaaahicaabaaaaaaaaaaadkaabaaaaaaaaaaa
abeaaaaaaaaaaaaadcaaaaajhcaabaaaaaaaaaaaegacbaaaabaaaaaapgapbaaa
aaaaaaaaegacbaaaaaaaaaaaefaaaaajpcaabaaaabaaaaaaogbkbaaaabaaaaaa
eghobaaaadaaaaaaaagabaaaaaaaaaaaaaaaaaahicaabaaaaaaaaaaadkaabaaa
abaaaaaadkaabaaaabaaaaaadiaaaaahhccabaaaaaaaaaaapgapbaaaaaaaaaaa
egacbaaaaaaaaaaabaaaaaahbcaabaaaaaaaaaaaegacbaaaaeaaaaaaegacbaaa
aeaaaaaaeeaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaadiaaaaahhcaabaaa
aaaaaaaaagaabaaaaaaaaaaaegacbaaaaeaaaaaabaaaaaahbcaabaaaaaaaaaaa
egacbaaaadaaaaaaegacbaaaaaaaaaaadeaaaaahbcaabaaaaaaaaaaaakaabaaa
aaaaaaaaabeaaaaaaaaaaaaaaaaaaaaibcaabaaaaaaaaaaaakaabaiaebaaaaaa
aaaaaaaaabeaaaaaaaaaiadpdeaaaaahbcaabaaaaaaaaaaaakaabaaaaaaaaaaa
abeaaaaaaaaaaaaacpaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaadiaaaaai
bcaabaaaaaaaaaaaakaabaaaaaaaaaaadkiacaaaaaaaaaaaajaaaaaabjaaaaaf
bcaabaaaaaaaaaaaakaabaaaaaaaaaaadcaaaaajiccabaaaaaaaaaaaakaabaaa
aaaaaaaaabeaaaaafknieldpabeaaaaajjjofadodoaaaaab"
}

SubProgram "gles " {
Keywords { "DIRECTIONAL_COOKIE" }
"!!GLES"
}

SubProgram "glesdesktop " {
Keywords { "DIRECTIONAL_COOKIE" }
"!!GLES"
}

}
	}

#LINE 102

	}
	Fallback "Reflective/Bumped Diffuse"
}
