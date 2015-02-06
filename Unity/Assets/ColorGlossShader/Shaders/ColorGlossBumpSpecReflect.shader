Shader "ColorGloss/Bumped Specular Reflective" {
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
	}
	SubShader {
		Tags { "RenderType" = "Opaque" }
		LOD 400

			
	Pass {
		Name "FORWARD"
		Tags { "LightMode" = "ForwardBase" }
Program "vp" {
// Vertex combos: 9
//   opengl - ALU: 31 to 94
//   d3d9 - ALU: 32 to 97
//   d3d11 - ALU: 18 to 52, TEX: 0 to 0, FLOW: 1 to 1
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
ADD result.texcoord[5].xyz, R0, R1;
MOV R1.xyz, c[13];
MOV R1.w, c[0].x;
DP4 R2.z, R1, c[11];
DP4 R2.y, R1, c[10];
DP4 R2.x, R1, c[9];
MAD R3.xyz, R2, c[22].w, -vertex.position;
MOV R0.xyz, vertex.attrib[14];
MUL R1.xyz, vertex.normal.zxyw, R0.yzxw;
MAD R1.xyz, vertex.normal.yzxw, R0.zxyw, -R1;
MUL R2.xyz, vertex.attrib[14].w, R1;
MOV R0, c[14];
DP4 R1.z, R0, c[11];
DP4 R1.x, R0, c[9];
DP4 R1.y, R0, c[10];
DP3 R0.y, R2, c[5];
DP3 R0.w, -R3, c[5];
DP3 R0.x, vertex.attrib[14], c[5];
DP3 R0.z, vertex.normal, c[5];
MUL result.texcoord[1], R0, c[22].w;
DP3 R0.y, R2, c[6];
DP3 R0.w, -R3, c[6];
DP3 R0.x, vertex.attrib[14], c[6];
DP3 R0.z, vertex.normal, c[6];
MUL result.texcoord[2], R0, c[22].w;
DP3 R0.y, R2, c[7];
DP3 R0.w, -R3, c[7];
DP3 R0.x, vertex.attrib[14], c[7];
DP3 R0.z, vertex.normal, c[7];
DP3 result.texcoord[4].y, R2, R1;
DP3 result.texcoord[6].y, R2, R3;
MUL result.texcoord[3], R0, c[22].w;
DP3 result.texcoord[4].z, vertex.normal, R1;
DP3 result.texcoord[4].x, vertex.attrib[14], R1;
DP3 result.texcoord[6].z, vertex.normal, R3;
DP3 result.texcoord[6].x, vertex.attrib[14], R3;
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
add o6.xyz, r0, r1
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
mul r2.xyz, v1.w, r1
mov r0, c10
dp4 r4.z, c13, r0
mov r0, c9
dp4 r4.y, c13, r0
mov r1, c8
dp4 r4.x, c13, r1
dp3 r0.y, r2, c4
dp3 r0.w, -r3, c4
dp3 r0.x, v1, c4
dp3 r0.z, v2, c4
mul o2, r0, c21.w
dp3 r0.y, r2, c5
dp3 r0.w, -r3, c5
dp3 r0.x, v1, c5
dp3 r0.z, v2, c5
mul o3, r0, c21.w
dp3 r0.y, r2, c6
dp3 r0.w, -r3, c6
dp3 r0.x, v1, c6
dp3 r0.z, v2, c6
dp3 o5.y, r2, r4
dp3 o7.y, r2, r3
mul o4, r0, c21.w
dp3 o5.z, v2, r4
dp3 o5.x, v1, r4
dp3 o7.z, v2, r3
dp3 o7.x, v1, r3
mad o1.xy, v3, c22, c22.zwzw
dp4 o0.w, v0, c3
dp4 o0.z, v0, c2
dp4 o0.y, v0, c1
dp4 o0.x, v0, c0
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
aaaaaaalaaaaacjaaabaaaahaaaagaaiaaaadaajaacafaakaaaadafaaaabpbfb
aaacpcfcaaadpdfdaaaehefeaaahhfffaaaihgfgaaaabadaaaaabadnaaaabado
aaaabadpaaaaaackaaaaaaclaaaabacmaaaabaebaaaaaacnaaaaaacoaaaabacp
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
kiccadafaalolomanaalahaokiihadahaelbgciaibacaoaomiabiaaeaaloloaa
paakabaamiaciaaeaagcloaapaaaakaamiaeiaaeaaloloaapaakaiaamiabiaag
aaloloaapaacabaamiaciaagaagcloaapaaaacaamiaeiaagaaloloaapaacaiaa
miadiaaaaabjlabkilaebfbfmiamaaabaalbigdbklaaanadmiahaaahaegmlole
klacanahaiboaiaaaemghgggkbacapajaicbaiacaadoanmbgpacajajaiecaiac
aadoanlbgpadajajaiieaiacaadoanlmgpaeajajaibbabaaaakhkhgmkpaiafaj
aiciabadaagmlbmgoaahaaajbeacaaaaaakhkhmgkpaiagabaeciadaeaamgmggm
oaahaaagbeaeaaaaaakhkhblkpaiahabaeciaeafaalbbllboaahaaagmiapiaab
aaaablaakbafbeaamiapiaacaaaablaakbaebeaamiapiaadaaaablaakbadbeaa
geihaaaaaalologboaacaaabmiahiaafaablmagfklaaaiaaaaaaaaaaaaaaaaaa
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
"sce_vp_rsx // 56 instructions using 8 registers
[Configuration]
8
0000003841050800
[Microcode]
896
00019c6c005d200d8186c0836041fffc00021c6c00400e0c0106c0836041dffc
00029c6c005d300c0186c0836041dffc00039c6c009ca20c013fc0c36041dffc
401f9c6c011c9808010400d740619f9c401f9c6c01d0300d8106c0c360403f80
401f9c6c01d0200d8106c0c360405f80401f9c6c01d0100d8106c0c360409f80
401f9c6c01d0000d8106c0c360411f8000001c6c01506e0c010600c360411ffc
00001c6c0150620c010600c360405ffc00009c6c01505e0c010600c360411ffc
00009c6c0150520c010600c360405ffc00011c6c01504e0c010600c360411ffc
00011c6c0150420c010600c360405ffc00031c6c01d0a00d8686c0c360405ffc
00031c6c01d0900d8686c0c360409ffc00031c6c01d0800d8686c0c360411ffc
00019c6c0150400c0e8600c360411ffc00019c6c0150600c0e8600c360405ffc
00001c6c0150500c0e8600c360409ffc00039c6c0190a00c0a86c0c360405ffc
00039c6c0190900c0a86c0c360409ffc00039c6c0190800c0a86c0c360411ffc
00029c6c00800243011844436041dffc00021c6c010002308121846302a1dffc
00039c6c011ca00c0ebfc0e30041dffc401f9c6c0140020c0106064360405fac
401f9c6c01400e0c0106064360411fac00009c6c0080002a8095404360409ffc
00019c6c0040002a8086c08360409ffc00029c6c00800e7f810604436041dffc
401f9c6c0140020c0106074360405fb4401f9c6c01400e0c0106074360411fb4
00001c6c0150608c0e8600c360403ffc00009c6c0150508c0e8600c360403ffc
00011c6c0150408c0e8600c360403ffc00021c6c019cf00c0686c0c360405ffc
00021c6c019d000c0686c0c360409ffc00021c6c019d100c0686c0c360411ffc
00021c6c010000000680036aa0a03ffc00019c6c0080000d069a03436041fffc
401f9c6c0140000c0a86064360409fac401f9c6c0140000c0a86074360409fb4
00001c6c0150600c0a8600c360409ffc00009c6c0150500c0a8600c360409ffc
00011c6c0150400c0a8600c360409ffc00029c6c01dcc00d8686c0c360405ffc
00029c6c01dcd00d8686c0c360409ffc00029c6c01dce00d8686c0c360411ffc
00019c6c00c0000c0886c08302a1dffc00021c6c009cb07f888600c36041dffc
401f9c6c00c0000c0886c08301a1dfb0401f9c6c009ca00d84bfc0c36041ffa0
401f9c6c009ca00d82bfc0c36041ffa4401f9c6c009ca00d80bfc0c36041ffa9
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
// 66 instructions, 5 temp regs, 0 temp arrays:
// ALU 36 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0
eefiecedjjhefipiobgidmcjiocjmnokffieiajhabaaaaaaoiakaaaaadaaaaaa
cmaaaaaapeaaaaaanmabaaaaejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaa
abaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaljaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfc
enebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheooaaaaaaaaiaaaaaa
aiaaaaaamiaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaneaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaabaaaaaaadamaaaaneaaaaaaabaaaaaaaaaaaaaa
adaaaaaaacaaaaaaapaaaaaaneaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaa
apaaaaaaneaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaneaaaaaa
aeaaaaaaaaaaaaaaadaaaaaaafaaaaaaahaiaaaaneaaaaaaafaaaaaaaaaaaaaa
adaaaaaaagaaaaaaahaiaaaaneaaaaaaagaaaaaaaaaaaaaaadaaaaaaahaaaaaa
ahaiaaaafdfgfpfaepfdejfeejepeoaafeeffiedepepfceeaaklklklfdeieefc
aeajaaaaeaaaabaaebacaaaafjaaaaaeegiocaaaaaaaaaaaahaaaaaafjaaaaae
egiocaaaabaaaaaaafaaaaaafjaaaaaeegiocaaaacaaaaaabjaaaaaafjaaaaae
egiocaaaadaaaaaabfaaaaaafpaaaaadpcbabaaaaaaaaaaafpaaaaadpcbabaaa
abaaaaaafpaaaaadhcbabaaaacaaaaaafpaaaaaddcbabaaaadaaaaaaghaaaaae
pccabaaaaaaaaaaaabaaaaaagfaaaaaddccabaaaabaaaaaagfaaaaadpccabaaa
acaaaaaagfaaaaadpccabaaaadaaaaaagfaaaaadpccabaaaaeaaaaaagfaaaaad
hccabaaaafaaaaaagfaaaaadhccabaaaagaaaaaagfaaaaadhccabaaaahaaaaaa
giaaaaacafaaaaaadiaaaaaipcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaa
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
adaaaaaabeaaaaaaegbcbaiaebaaaaaaaaaaaaaadiaaaaajhcaabaaaabaaaaaa
fgafbaiaebaaaaaaaaaaaaaaegiccaaaadaaaaaaanaaaaaadcaaaaalhcaabaaa
abaaaaaaegiccaaaadaaaaaaamaaaaaaagaabaiaebaaaaaaaaaaaaaaegacbaaa
abaaaaaadcaaaaallcaabaaaabaaaaaaegiicaaaadaaaaaaaoaaaaaakgakbaia
ebaaaaaaaaaaaaaaegaibaaaabaaaaaadgaaaaaficaabaaaacaaaaaaakaabaaa
abaaaaaadiaaaaahhcaabaaaadaaaaaajgbebaaaabaaaaaacgbjbaaaacaaaaaa
dcaaaaakhcaabaaaadaaaaaajgbebaaaacaaaaaacgbjbaaaabaaaaaaegacbaia
ebaaaaaaadaaaaaadiaaaaahhcaabaaaadaaaaaaegacbaaaadaaaaaapgbpbaaa
abaaaaaadgaaaaagbcaabaaaaeaaaaaaakiacaaaadaaaaaaamaaaaaadgaaaaag
ccaabaaaaeaaaaaaakiacaaaadaaaaaaanaaaaaadgaaaaagecaabaaaaeaaaaaa
akiacaaaadaaaaaaaoaaaaaabaaaaaahccaabaaaacaaaaaaegacbaaaadaaaaaa
egacbaaaaeaaaaaabaaaaaahbcaabaaaacaaaaaaegbcbaaaabaaaaaaegacbaaa
aeaaaaaabaaaaaahecaabaaaacaaaaaaegbcbaaaacaaaaaaegacbaaaaeaaaaaa
diaaaaaipccabaaaacaaaaaaegaobaaaacaaaaaapgipcaaaadaaaaaabeaaaaaa
dgaaaaaficaabaaaacaaaaaabkaabaaaabaaaaaadgaaaaagbcaabaaaaeaaaaaa
bkiacaaaadaaaaaaamaaaaaadgaaaaagccaabaaaaeaaaaaabkiacaaaadaaaaaa
anaaaaaadgaaaaagecaabaaaaeaaaaaabkiacaaaadaaaaaaaoaaaaaabaaaaaah
ccaabaaaacaaaaaaegacbaaaadaaaaaaegacbaaaaeaaaaaabaaaaaahbcaabaaa
acaaaaaaegbcbaaaabaaaaaaegacbaaaaeaaaaaabaaaaaahecaabaaaacaaaaaa
egbcbaaaacaaaaaaegacbaaaaeaaaaaadiaaaaaipccabaaaadaaaaaaegaobaaa
acaaaaaapgipcaaaadaaaaaabeaaaaaadgaaaaagbcaabaaaacaaaaaackiacaaa
adaaaaaaamaaaaaadgaaaaagccaabaaaacaaaaaackiacaaaadaaaaaaanaaaaaa
dgaaaaagecaabaaaacaaaaaackiacaaaadaaaaaaaoaaaaaabaaaaaahccaabaaa
abaaaaaaegacbaaaadaaaaaaegacbaaaacaaaaaabaaaaaahbcaabaaaabaaaaaa
egbcbaaaabaaaaaaegacbaaaacaaaaaabaaaaaahecaabaaaabaaaaaaegbcbaaa
acaaaaaaegacbaaaacaaaaaadiaaaaaipccabaaaaeaaaaaaegaobaaaabaaaaaa
pgipcaaaadaaaaaabeaaaaaadiaaaaajhcaabaaaabaaaaaafgifcaaaacaaaaaa
aaaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaa
adaaaaaabaaaaaaaagiacaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaadcaaaaal
hcaabaaaabaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaaacaaaaaaaaaaaaaa
egacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabdaaaaaa
pgipcaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaabaaaaaahcccabaaaafaaaaaa
egacbaaaadaaaaaaegacbaaaabaaaaaabaaaaaahcccabaaaahaaaaaaegacbaaa
adaaaaaaegacbaaaaaaaaaaabaaaaaahbccabaaaafaaaaaaegbcbaaaabaaaaaa
egacbaaaabaaaaaabaaaaaaheccabaaaafaaaaaaegbcbaaaacaaaaaaegacbaaa
abaaaaaadiaaaaaihcaabaaaabaaaaaaegbcbaaaacaaaaaapgipcaaaadaaaaaa
beaaaaaadiaaaaaihcaabaaaacaaaaaafgafbaaaabaaaaaaegiccaaaadaaaaaa
anaaaaaadcaaaaaklcaabaaaabaaaaaaegiicaaaadaaaaaaamaaaaaaagaabaaa
abaaaaaaegaibaaaacaaaaaadcaaaaakhcaabaaaabaaaaaaegiccaaaadaaaaaa
aoaaaaaakgakbaaaabaaaaaaegadbaaaabaaaaaadgaaaaaficaabaaaabaaaaaa
abeaaaaaaaaaiadpbbaaaaaibcaabaaaacaaaaaaegiocaaaacaaaaaabcaaaaaa
egaobaaaabaaaaaabbaaaaaiccaabaaaacaaaaaaegiocaaaacaaaaaabdaaaaaa
egaobaaaabaaaaaabbaaaaaiecaabaaaacaaaaaaegiocaaaacaaaaaabeaaaaaa
egaobaaaabaaaaaadiaaaaahpcaabaaaadaaaaaajgacbaaaabaaaaaaegakbaaa
abaaaaaabbaaaaaibcaabaaaaeaaaaaaegiocaaaacaaaaaabfaaaaaaegaobaaa
adaaaaaabbaaaaaiccaabaaaaeaaaaaaegiocaaaacaaaaaabgaaaaaaegaobaaa
adaaaaaabbaaaaaiecaabaaaaeaaaaaaegiocaaaacaaaaaabhaaaaaaegaobaaa
adaaaaaaaaaaaaahhcaabaaaacaaaaaaegacbaaaacaaaaaaegacbaaaaeaaaaaa
diaaaaahicaabaaaaaaaaaaabkaabaaaabaaaaaabkaabaaaabaaaaaadcaaaaak
icaabaaaaaaaaaaaakaabaaaabaaaaaaakaabaaaabaaaaaadkaabaiaebaaaaaa
aaaaaaaadcaaaaakhccabaaaagaaaaaaegiccaaaacaaaaaabiaaaaaapgapbaaa
aaaaaaaaegacbaaaacaaaaaabaaaaaahbccabaaaahaaaaaaegbcbaaaabaaaaaa
egacbaaaaaaaaaaabaaaaaaheccabaaaahaaaaaaegbcbaaaacaaaaaaegacbaaa
aaaaaaaadoaaaaab"
}

SubProgram "gles " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
"!!GLES
#define SHADER_API_GLES 1
#define tex2D texture2D


#ifdef VERTEX
#define gl_ModelViewProjectionMatrix glstate_matrix_mvp
uniform mat4 glstate_matrix_mvp;

varying highp vec3 xlv_TEXCOORD6;
varying lowp vec3 xlv_TEXCOORD5;
varying lowp vec3 xlv_TEXCOORD4;
varying lowp vec4 xlv_TEXCOORD3;
varying lowp vec4 xlv_TEXCOORD2;
varying lowp vec4 xlv_TEXCOORD1;
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
  xlv_TEXCOORD1 = tmpvar_4;
  xlv_TEXCOORD2 = tmpvar_5;
  xlv_TEXCOORD3 = tmpvar_6;
  xlv_TEXCOORD4 = tmpvar_7;
  xlv_TEXCOORD5 = tmpvar_8;
  xlv_TEXCOORD6 = (tmpvar_14 * (((_World2Object * tmpvar_26).xyz * unity_Scale.w) - _glesVertex.xyz));
}



#endif
#ifdef FRAGMENT

varying highp vec3 xlv_TEXCOORD6;
varying lowp vec3 xlv_TEXCOORD5;
varying lowp vec3 xlv_TEXCOORD4;
varying lowp vec4 xlv_TEXCOORD3;
varying lowp vec4 xlv_TEXCOORD2;
varying lowp vec4 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
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
  tmpvar_6.x = xlv_TEXCOORD1.w;
  tmpvar_6.y = xlv_TEXCOORD2.w;
  tmpvar_6.z = xlv_TEXCOORD3.w;
  tmpvar_2 = tmpvar_6;
  lowp vec3 tmpvar_7;
  tmpvar_7 = xlv_TEXCOORD1.xyz;
  tmpvar_3 = tmpvar_7;
  lowp vec3 tmpvar_8;
  tmpvar_8 = xlv_TEXCOORD2.xyz;
  tmpvar_4 = tmpvar_8;
  lowp vec3 tmpvar_9;
  tmpvar_9 = xlv_TEXCOORD3.xyz;
  tmpvar_5 = tmpvar_9;
  mediump vec3 tmpvar_10;
  mediump vec3 tmpvar_11;
  mediump float tmpvar_12;
  mediump vec4 spec_13;
  lowp vec4 tmpvar_14;
  tmpvar_14 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_15;
  tmpvar_15 = (tmpvar_14.xyz * _Color.xyz);
  tmpvar_10 = tmpvar_15;
  lowp vec4 tmpvar_16;
  tmpvar_16 = texture2D (_SpecMap, xlv_TEXCOORD0);
  spec_13 = tmpvar_16;
  mediump vec3 tmpvar_17;
  tmpvar_17 = (spec_13.xyz * _Gloss);
  lowp vec3 tmpvar_18;
  tmpvar_18 = ((texture2D (_BumpMap, xlv_TEXCOORD0).xyz * 2.0) - 1.0);
  tmpvar_11 = tmpvar_18;
  mediump vec3 tmpvar_19;
  tmpvar_19.x = dot (tmpvar_3, tmpvar_11);
  tmpvar_19.y = dot (tmpvar_4, tmpvar_11);
  tmpvar_19.z = dot (tmpvar_5, tmpvar_11);
  highp vec3 tmpvar_20;
  tmpvar_20 = (tmpvar_2 - (2.0 * (dot (tmpvar_19, tmpvar_2) * tmpvar_19)));
  lowp vec4 tmpvar_21;
  tmpvar_21 = (textureCube (_Cube, tmpvar_20) * tmpvar_14.w);
  lowp float tmpvar_22;
  tmpvar_22 = (tmpvar_21.w * _ReflectColor.w);
  tmpvar_12 = tmpvar_22;
  highp vec3 tmpvar_23;
  tmpvar_23 = normalize(xlv_TEXCOORD6);
  mediump vec3 lightDir_24;
  lightDir_24 = xlv_TEXCOORD4;
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
  tmpvar_31 = (pow (nh_28, arg1_30) * tmpvar_17);
  specCol_27 = tmpvar_31;
  c_26.xyz = ((((tmpvar_10 * _LightColor0.xyz) * max (0.0, dot (tmpvar_11, lightDir_24))) + (_LightColor0.xyz * specCol_27)) * 2.0);
  c_26.w = tmpvar_12;
  c_1 = c_26;
  mediump vec3 tmpvar_32;
  tmpvar_32 = (c_1.xyz + (tmpvar_10 * xlv_TEXCOORD5));
  c_1.xyz = tmpvar_32;
  mediump vec3 tmpvar_33;
  tmpvar_33 = (c_1.xyz + ((tmpvar_21.xyz * _ReflectColor.xyz) * _ReflectPower));
  c_1.xyz = tmpvar_33;
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

varying highp vec3 xlv_TEXCOORD6;
varying lowp vec3 xlv_TEXCOORD5;
varying lowp vec3 xlv_TEXCOORD4;
varying lowp vec4 xlv_TEXCOORD3;
varying lowp vec4 xlv_TEXCOORD2;
varying lowp vec4 xlv_TEXCOORD1;
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
  xlv_TEXCOORD1 = tmpvar_4;
  xlv_TEXCOORD2 = tmpvar_5;
  xlv_TEXCOORD3 = tmpvar_6;
  xlv_TEXCOORD4 = tmpvar_7;
  xlv_TEXCOORD5 = tmpvar_8;
  xlv_TEXCOORD6 = (tmpvar_14 * (((_World2Object * tmpvar_26).xyz * unity_Scale.w) - _glesVertex.xyz));
}



#endif
#ifdef FRAGMENT

varying highp vec3 xlv_TEXCOORD6;
varying lowp vec3 xlv_TEXCOORD5;
varying lowp vec3 xlv_TEXCOORD4;
varying lowp vec4 xlv_TEXCOORD3;
varying lowp vec4 xlv_TEXCOORD2;
varying lowp vec4 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
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
  tmpvar_6.x = xlv_TEXCOORD1.w;
  tmpvar_6.y = xlv_TEXCOORD2.w;
  tmpvar_6.z = xlv_TEXCOORD3.w;
  tmpvar_2 = tmpvar_6;
  lowp vec3 tmpvar_7;
  tmpvar_7 = xlv_TEXCOORD1.xyz;
  tmpvar_3 = tmpvar_7;
  lowp vec3 tmpvar_8;
  tmpvar_8 = xlv_TEXCOORD2.xyz;
  tmpvar_4 = tmpvar_8;
  lowp vec3 tmpvar_9;
  tmpvar_9 = xlv_TEXCOORD3.xyz;
  tmpvar_5 = tmpvar_9;
  mediump vec3 tmpvar_10;
  mediump vec3 tmpvar_11;
  mediump float tmpvar_12;
  mediump vec4 spec_13;
  lowp vec4 tmpvar_14;
  tmpvar_14 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_15;
  tmpvar_15 = (tmpvar_14.xyz * _Color.xyz);
  tmpvar_10 = tmpvar_15;
  lowp vec4 tmpvar_16;
  tmpvar_16 = texture2D (_SpecMap, xlv_TEXCOORD0);
  spec_13 = tmpvar_16;
  mediump vec3 tmpvar_17;
  tmpvar_17 = (spec_13.xyz * _Gloss);
  lowp vec3 normal_18;
  normal_18.xy = ((texture2D (_BumpMap, xlv_TEXCOORD0).wy * 2.0) - 1.0);
  normal_18.z = sqrt(((1.0 - (normal_18.x * normal_18.x)) - (normal_18.y * normal_18.y)));
  tmpvar_11 = normal_18;
  mediump vec3 tmpvar_19;
  tmpvar_19.x = dot (tmpvar_3, tmpvar_11);
  tmpvar_19.y = dot (tmpvar_4, tmpvar_11);
  tmpvar_19.z = dot (tmpvar_5, tmpvar_11);
  highp vec3 tmpvar_20;
  tmpvar_20 = (tmpvar_2 - (2.0 * (dot (tmpvar_19, tmpvar_2) * tmpvar_19)));
  lowp vec4 tmpvar_21;
  tmpvar_21 = (textureCube (_Cube, tmpvar_20) * tmpvar_14.w);
  lowp float tmpvar_22;
  tmpvar_22 = (tmpvar_21.w * _ReflectColor.w);
  tmpvar_12 = tmpvar_22;
  highp vec3 tmpvar_23;
  tmpvar_23 = normalize(xlv_TEXCOORD6);
  mediump vec3 lightDir_24;
  lightDir_24 = xlv_TEXCOORD4;
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
  tmpvar_31 = (pow (nh_28, arg1_30) * tmpvar_17);
  specCol_27 = tmpvar_31;
  c_26.xyz = ((((tmpvar_10 * _LightColor0.xyz) * max (0.0, dot (tmpvar_11, lightDir_24))) + (_LightColor0.xyz * specCol_27)) * 2.0);
  c_26.w = tmpvar_12;
  c_1 = c_26;
  mediump vec3 tmpvar_32;
  tmpvar_32 = (c_1.xyz + (tmpvar_10 * xlv_TEXCOORD5));
  c_1.xyz = tmpvar_32;
  mediump vec3 tmpvar_33;
  tmpvar_33 = (c_1.xyz + ((tmpvar_21.xyz * _ReflectColor.xyz) * _ReflectPower));
  c_1.xyz = tmpvar_33;
  gl_FragData[0] = c_1;
}



#endif"
}

SubProgram "opengl " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
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
# 31 ALU
PARAM c[18] = { { 1 },
		state.matrix.mvp,
		program.local[5..17] };
TEMP R0;
TEMP R1;
TEMP R2;
MOV R0.xyz, vertex.attrib[14];
MUL R1.xyz, vertex.normal.zxyw, R0.yzxw;
MAD R0.xyz, vertex.normal.yzxw, R0.zxyw, -R1;
MUL R1.xyz, vertex.attrib[14].w, R0;
MOV R0.xyz, c[13];
MOV R0.w, c[0].x;
DP4 R2.z, R0, c[11];
DP4 R2.x, R0, c[9];
DP4 R2.y, R0, c[10];
MAD R2.xyz, R2, c[15].w, -vertex.position;
DP3 R0.y, R1, c[5];
DP3 R0.w, -R2, c[5];
DP3 R0.x, vertex.attrib[14], c[5];
DP3 R0.z, vertex.normal, c[5];
MUL result.texcoord[1], R0, c[15].w;
DP3 R0.y, R1, c[6];
DP3 R0.w, -R2, c[6];
DP3 R0.x, vertex.attrib[14], c[6];
DP3 R0.z, vertex.normal, c[6];
MUL result.texcoord[2], R0, c[15].w;
DP3 R0.y, R1, c[7];
DP3 R0.w, -R2, c[7];
DP3 R0.x, vertex.attrib[14], c[7];
DP3 R0.z, vertex.normal, c[7];
MUL result.texcoord[3], R0, c[15].w;
MAD result.texcoord[0].xy, vertex.texcoord[0], c[17], c[17].zwzw;
MAD result.texcoord[4].xy, vertex.texcoord[1], c[16], c[16].zwzw;
DP4 result.position.w, vertex.position, c[4];
DP4 result.position.z, vertex.position, c[3];
DP4 result.position.y, vertex.position, c[2];
DP4 result.position.x, vertex.position, c[1];
END
# 31 instructions, 3 R-regs
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
Matrix 4 [_Object2World]
Matrix 8 [_World2Object]
Vector 13 [unity_Scale]
Vector 14 [unity_LightmapST]
Vector 15 [_MainTex_ST]
"vs_3_0
; 32 ALU
dcl_position o0
dcl_texcoord0 o1
dcl_texcoord1 o2
dcl_texcoord2 o3
dcl_texcoord3 o4
dcl_texcoord4 o5
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
mul r1.xyz, v1.w, r0
mov r0.xyz, c12
mov r0.w, c16.x
dp4 r2.z, r0, c10
dp4 r2.x, r0, c8
dp4 r2.y, r0, c9
mad r2.xyz, r2, c13.w, -v0
dp3 r0.y, r1, c4
dp3 r0.w, -r2, c4
dp3 r0.x, v1, c4
dp3 r0.z, v2, c4
mul o2, r0, c13.w
dp3 r0.y, r1, c5
dp3 r0.w, -r2, c5
dp3 r0.x, v1, c5
dp3 r0.z, v2, c5
mul o3, r0, c13.w
dp3 r0.y, r1, c6
dp3 r0.w, -r2, c6
dp3 r0.x, v1, c6
dp3 r0.z, v2, c6
mul o4, r0, c13.w
mad o1.xy, v3, c15, c15.zwzw
mad o5.xy, v4, c14, c14.zwzw
dp4 o0.w, v0, c3
dp4 o0.z, v0, c2
dp4 o0.y, v0, c1
dp4 o0.x, v0, c0
"
}

SubProgram "xbox360 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
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
// ALU: 46.67 (35 instructions), vertex: 64, texture: 0,
//   sequencer: 20,  11 GPRs, 15 threads,
// Performance (if enough threads): ~64 cycles per vector
// * Vertex cycle estimates are assuming 3 vfetch_minis for every vfetch_full,
//     with <= 32 bytes per vfetch_full group.

"vs_360
backbbabaaaaaboiaaaaacciaaaaaaaaaaaaaaceaaaaaaaaaaaaabieaaaaaaaa
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
codaaaklaaaaaaaaaaaaacciaaebaaakaaaaaaaaaaaaaaaaaaaaeakfaaaaaaab
aaaaaaafaaaaaaafaaaaacjaaabaaaafaaaagaagaaaadaahaaaafaaiaacbfaaj
aaaadafaaaabpbfbaaacpcfcaaadpdfdaaaedefeaaaabaccaaaabackaaaabacl
aaaabacmaaaabacdpbfffaafaaaabcabmcaaaaaaaaaaeaakaaaabcaameaaaaaa
aaaagaaogabebcaabcaaaaaaaaaagabkgacabcaabcaaaaaaaaaagacgbacmbcaa
ccaaaaaaafpifaaaaaaaagiiaaaaaaaaafpieaaaaaaaagiiaaaaaaaaafpiaaaa
aaaaacihaaaaaaaaafpidaaaaaaaadmhaaaaaaaaafpicaaaaaaaadmhaaaaaaaa
miapaaabaabliiaakbafaeaamiapaaabaamgiiaaklafadabmiapaaabaalbdeje
klafacabmiapiadoaagmaadeklafababmiahaaagaamamgmaalakaaalbeccabai
aablgmblkbaeagaekmbeaiaiaablgmebibaeahafkibhabajaamggcmdibaeahag
kichabakaalbgcedibaeagagmiahaaagaalelbleclajaaagkiehabahaabcgfid
mbaaaeagmiahaaahablhlomaolaaaeahmiahaaagaamagmleclaiaaagmialaaab
aalbgcleklaaafabmiahaaaeaagmloleklaeafakbealaaaaaamggcgmkbaaahae
aebeababaagmgmgmoaabaaajbeaeaaacaalblbmgoaabaaaeaebbacadaalbmglb
oaaeajajmialaaagabgfblgfklagamafbeahaaaaaamnblblobahaeabaeedadae
aamglcblkbaaahaakichabafaelbgcmaibagahagkiihabagaegmgciaibagagag
miadiaaaaabjlabkiladaoaomiadiaaeaabjlabkilacananmiahaaagaebllole
klagafagmiadaaaaaalblcbjklaaafabbeaiaaabaagmgmgmoaagafaaaeciabac
aamglbgmoaagafaebeacaaadaalololbpaaiahaaaeciacadaalbmglboaagafae
miapiaabaaaablaakbadamaamiapiaacaaaablaakbacamaamiapiaadaaaablaa
kbabamaaaaaaaaaaaaaaaaaaaaaaaaaa"
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
Matrix 260 [_Object2World]
Matrix 264 [_World2Object]
Vector 466 [unity_Scale]
Vector 465 [unity_LightmapST]
Vector 464 [_MainTex_ST]
"sce_vp_rsx // 30 instructions using 6 registers
[Configuration]
8
0000001e43050600
[Microcode]
480
00021c6c00400e0c0106c0836041dffc00029c6c005d300c0186c0836041dffc
401f9c6c011d0808010400d740619f9c401f9c6c011d1908010400d740619fac
401f9c6c01d0300d8106c0c360403f80401f9c6c01d0200d8106c0c360405f80
401f9c6c01d0100d8106c0c360409f80401f9c6c01d0000d8106c0c360411f80
00001c6c01506e0c010600c360411ffc00001c6c0150620c010600c360405ffc
00009c6c01505e0c010600c360411ffc00009c6c0150520c010600c360405ffc
00011c6c01504e0c010600c360411ffc00011c6c0150420c010600c360405ffc
00019c6c0190a00c0a86c0c360405ffc00019c6c0190900c0a86c0c360409ffc
00019c6c0190800c0a86c0c360411ffc00029c6c00800243011844436041dffc
00019c6c011d200c06bfc0e30041dffc00021c6c010002308121846302a1dffc
00001c6c0150608c068600c360403ffc00009c6c0150508c068600c360403ffc
00011c6c0150408c068600c360403ffc00019c6c00800e7f810604436041dffc
00001c6c0150600c068600c360409ffc00011c6c0150400c068600c360409ffc
00009c6c0150500c068600c360409ffc401f9c6c009d200d84bfc0c36041ffa0
401f9c6c009d200d82bfc0c36041ffa4401f9c6c009d200d80bfc0c36041ffa9
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
// 41 instructions, 4 temp regs, 0 temp arrays:
// ALU 18 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0
eefiecedghiamkacnmllifeiogglfkliodcjoajcabaaaaaahaahaaaaadaaaaaa
cmaaaaaapeaaaaaakmabaaaaejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaa
abaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapadaaaaljaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfc
enebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheolaaaaaaaagaaaaaa
aiaaaaaajiaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaakeaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaabaaaaaaadamaaaakeaaaaaaaeaaaaaaaaaaaaaa
adaaaaaaabaaaaaaamadaaaakeaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaa
apaaaaaakeaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaapaaaaaakeaaaaaa
adaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaafdfgfpfaepfdejfeejepeoaa
feeffiedepepfceeaaklklklfdeieefclmafaaaaeaaaabaagpabaaaafjaaaaae
egiocaaaaaaaaaaaaiaaaaaafjaaaaaeegiocaaaabaaaaaaafaaaaaafjaaaaae
egiocaaaacaaaaaabfaaaaaafpaaaaadpcbabaaaaaaaaaaafpaaaaadpcbabaaa
abaaaaaafpaaaaadhcbabaaaacaaaaaafpaaaaaddcbabaaaadaaaaaafpaaaaad
dcbabaaaaeaaaaaaghaaaaaepccabaaaaaaaaaaaabaaaaaagfaaaaaddccabaaa
abaaaaaagfaaaaadmccabaaaabaaaaaagfaaaaadpccabaaaacaaaaaagfaaaaad
pccabaaaadaaaaaagfaaaaadpccabaaaaeaaaaaagiaaaaacaeaaaaaadiaaaaai
pcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaacaaaaaaabaaaaaadcaaaaak
pcaabaaaaaaaaaaaegiocaaaacaaaaaaaaaaaaaaagbabaaaaaaaaaaaegaobaaa
aaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaacaaaaaaacaaaaaakgbkbaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaakpccabaaaaaaaaaaaegiocaaaacaaaaaa
adaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaaldccabaaaabaaaaaa
egbabaaaadaaaaaaegiacaaaaaaaaaaaahaaaaaaogikcaaaaaaaaaaaahaaaaaa
dcaaaaalmccabaaaabaaaaaaagbebaaaaeaaaaaaagiecaaaaaaaaaaaagaaaaaa
kgiocaaaaaaaaaaaagaaaaaadiaaaaajhcaabaaaaaaaaaaafgifcaaaabaaaaaa
aeaaaaaaegiccaaaacaaaaaabbaaaaaadcaaaaalhcaabaaaaaaaaaaaegiccaaa
acaaaaaabaaaaaaaagiacaaaabaaaaaaaeaaaaaaegacbaaaaaaaaaaadcaaaaal
hcaabaaaaaaaaaaaegiccaaaacaaaaaabcaaaaaakgikcaaaabaaaaaaaeaaaaaa
egacbaaaaaaaaaaaaaaaaaaihcaabaaaaaaaaaaaegacbaaaaaaaaaaaegiccaaa
acaaaaaabdaaaaaadcaaaaalhcaabaaaaaaaaaaaegacbaaaaaaaaaaapgipcaaa
acaaaaaabeaaaaaaegbcbaiaebaaaaaaaaaaaaaadiaaaaajhcaabaaaabaaaaaa
fgafbaiaebaaaaaaaaaaaaaaegiccaaaacaaaaaaanaaaaaadcaaaaallcaabaaa
aaaaaaaaegiicaaaacaaaaaaamaaaaaaagaabaiaebaaaaaaaaaaaaaaegaibaaa
abaaaaaadcaaaaallcaabaaaaaaaaaaaegiicaaaacaaaaaaaoaaaaaakgakbaia
ebaaaaaaaaaaaaaaegambaaaaaaaaaaadgaaaaaficaabaaaabaaaaaaakaabaaa
aaaaaaaadiaaaaahhcaabaaaacaaaaaajgbebaaaabaaaaaacgbjbaaaacaaaaaa
dcaaaaakhcaabaaaacaaaaaajgbebaaaacaaaaaacgbjbaaaabaaaaaaegacbaia
ebaaaaaaacaaaaaadiaaaaahhcaabaaaacaaaaaaegacbaaaacaaaaaapgbpbaaa
abaaaaaadgaaaaagbcaabaaaadaaaaaaakiacaaaacaaaaaaamaaaaaadgaaaaag
ccaabaaaadaaaaaaakiacaaaacaaaaaaanaaaaaadgaaaaagecaabaaaadaaaaaa
akiacaaaacaaaaaaaoaaaaaabaaaaaahccaabaaaabaaaaaaegacbaaaacaaaaaa
egacbaaaadaaaaaabaaaaaahbcaabaaaabaaaaaaegbcbaaaabaaaaaaegacbaaa
adaaaaaabaaaaaahecaabaaaabaaaaaaegbcbaaaacaaaaaaegacbaaaadaaaaaa
diaaaaaipccabaaaacaaaaaaegaobaaaabaaaaaapgipcaaaacaaaaaabeaaaaaa
dgaaaaaficaabaaaabaaaaaabkaabaaaaaaaaaaadgaaaaagbcaabaaaadaaaaaa
bkiacaaaacaaaaaaamaaaaaadgaaaaagccaabaaaadaaaaaabkiacaaaacaaaaaa
anaaaaaadgaaaaagecaabaaaadaaaaaabkiacaaaacaaaaaaaoaaaaaabaaaaaah
ccaabaaaabaaaaaaegacbaaaacaaaaaaegacbaaaadaaaaaabaaaaaahbcaabaaa
abaaaaaaegbcbaaaabaaaaaaegacbaaaadaaaaaabaaaaaahecaabaaaabaaaaaa
egbcbaaaacaaaaaaegacbaaaadaaaaaadiaaaaaipccabaaaadaaaaaaegaobaaa
abaaaaaapgipcaaaacaaaaaabeaaaaaadgaaaaagbcaabaaaabaaaaaackiacaaa
acaaaaaaamaaaaaadgaaaaagccaabaaaabaaaaaackiacaaaacaaaaaaanaaaaaa
dgaaaaagecaabaaaabaaaaaackiacaaaacaaaaaaaoaaaaaabaaaaaahccaabaaa
aaaaaaaaegacbaaaacaaaaaaegacbaaaabaaaaaabaaaaaahbcaabaaaaaaaaaaa
egbcbaaaabaaaaaaegacbaaaabaaaaaabaaaaaahecaabaaaaaaaaaaaegbcbaaa
acaaaaaaegacbaaaabaaaaaadiaaaaaipccabaaaaeaaaaaaegaobaaaaaaaaaaa
pgipcaaaacaaaaaabeaaaaaadoaaaaab"
}

SubProgram "gles " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
"!!GLES
#define SHADER_API_GLES 1
#define tex2D texture2D


#ifdef VERTEX
#define gl_ModelViewProjectionMatrix glstate_matrix_mvp
uniform mat4 glstate_matrix_mvp;

varying highp vec2 xlv_TEXCOORD4;
varying lowp vec4 xlv_TEXCOORD3;
varying lowp vec4 xlv_TEXCOORD2;
varying lowp vec4 xlv_TEXCOORD1;
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
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = tmpvar_3;
  xlv_TEXCOORD2 = tmpvar_4;
  xlv_TEXCOORD3 = tmpvar_5;
  xlv_TEXCOORD4 = ((_glesMultiTexCoord1.xy * unity_LightmapST.xy) + unity_LightmapST.zw);
}



#endif
#ifdef FRAGMENT

varying highp vec2 xlv_TEXCOORD4;
varying lowp vec4 xlv_TEXCOORD3;
varying lowp vec4 xlv_TEXCOORD2;
varying lowp vec4 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform sampler2D unity_Lightmap;
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
  tmpvar_6.x = xlv_TEXCOORD1.w;
  tmpvar_6.y = xlv_TEXCOORD2.w;
  tmpvar_6.z = xlv_TEXCOORD3.w;
  tmpvar_2 = tmpvar_6;
  lowp vec3 tmpvar_7;
  tmpvar_7 = xlv_TEXCOORD1.xyz;
  tmpvar_3 = tmpvar_7;
  lowp vec3 tmpvar_8;
  tmpvar_8 = xlv_TEXCOORD2.xyz;
  tmpvar_4 = tmpvar_8;
  lowp vec3 tmpvar_9;
  tmpvar_9 = xlv_TEXCOORD3.xyz;
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
  lowp vec4 tmpvar_18;
  tmpvar_18 = (textureCube (_Cube, tmpvar_17) * tmpvar_13.w);
  lowp float tmpvar_19;
  tmpvar_19 = (tmpvar_18.w * _ReflectColor.w);
  tmpvar_12 = tmpvar_19;
  lowp vec3 tmpvar_20;
  tmpvar_20 = (2.0 * texture2D (unity_Lightmap, xlv_TEXCOORD4).xyz);
  mediump vec3 tmpvar_21;
  tmpvar_21 = (tmpvar_10 * tmpvar_20);
  c_1.xyz = tmpvar_21;
  c_1.w = tmpvar_12;
  mediump vec3 tmpvar_22;
  tmpvar_22 = (c_1.xyz + ((tmpvar_18.xyz * _ReflectColor.xyz) * _ReflectPower));
  c_1.xyz = tmpvar_22;
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

varying highp vec2 xlv_TEXCOORD4;
varying lowp vec4 xlv_TEXCOORD3;
varying lowp vec4 xlv_TEXCOORD2;
varying lowp vec4 xlv_TEXCOORD1;
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
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = tmpvar_3;
  xlv_TEXCOORD2 = tmpvar_4;
  xlv_TEXCOORD3 = tmpvar_5;
  xlv_TEXCOORD4 = ((_glesMultiTexCoord1.xy * unity_LightmapST.xy) + unity_LightmapST.zw);
}



#endif
#ifdef FRAGMENT

varying highp vec2 xlv_TEXCOORD4;
varying lowp vec4 xlv_TEXCOORD3;
varying lowp vec4 xlv_TEXCOORD2;
varying lowp vec4 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform sampler2D unity_Lightmap;
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
  tmpvar_6.x = xlv_TEXCOORD1.w;
  tmpvar_6.y = xlv_TEXCOORD2.w;
  tmpvar_6.z = xlv_TEXCOORD3.w;
  tmpvar_2 = tmpvar_6;
  lowp vec3 tmpvar_7;
  tmpvar_7 = xlv_TEXCOORD1.xyz;
  tmpvar_3 = tmpvar_7;
  lowp vec3 tmpvar_8;
  tmpvar_8 = xlv_TEXCOORD2.xyz;
  tmpvar_4 = tmpvar_8;
  lowp vec3 tmpvar_9;
  tmpvar_9 = xlv_TEXCOORD3.xyz;
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
  lowp vec4 tmpvar_18;
  tmpvar_18 = (textureCube (_Cube, tmpvar_17) * tmpvar_13.w);
  lowp float tmpvar_19;
  tmpvar_19 = (tmpvar_18.w * _ReflectColor.w);
  tmpvar_12 = tmpvar_19;
  lowp vec4 tmpvar_20;
  tmpvar_20 = texture2D (unity_Lightmap, xlv_TEXCOORD4);
  lowp vec3 tmpvar_21;
  tmpvar_21 = ((8.0 * tmpvar_20.w) * tmpvar_20.xyz);
  mediump vec3 tmpvar_22;
  tmpvar_22 = (tmpvar_10 * tmpvar_21);
  c_1.xyz = tmpvar_22;
  c_1.w = tmpvar_12;
  mediump vec3 tmpvar_23;
  tmpvar_23 = (c_1.xyz + ((tmpvar_18.xyz * _ReflectColor.xyz) * _ReflectPower));
  c_1.xyz = tmpvar_23;
  gl_FragData[0] = c_1;
}



#endif"
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
"3.0-!!ARBvp1.0
# 63 ALU
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
ADD result.texcoord[5].xyz, R0, R1;
MOV R1.xyz, c[13];
MOV R1.w, c[0].x;
DP4 R2.z, R1, c[11];
DP4 R2.y, R1, c[10];
DP4 R2.x, R1, c[9];
MAD R3.xyz, R2, c[23].w, -vertex.position;
MOV R0.xyz, vertex.attrib[14];
MUL R1.xyz, vertex.normal.zxyw, R0.yzxw;
MAD R1.xyz, vertex.normal.yzxw, R0.zxyw, -R1;
MUL R2.xyz, vertex.attrib[14].w, R1;
MOV R0, c[15];
DP4 R1.z, R0, c[11];
DP4 R1.x, R0, c[9];
DP4 R1.y, R0, c[10];
DP3 R0.y, R2, c[5];
DP3 R0.w, -R3, c[5];
DP3 R0.x, vertex.attrib[14], c[5];
DP3 R0.z, vertex.normal, c[5];
MUL result.texcoord[1], R0, c[23].w;
DP3 R0.y, R2, c[6];
DP3 R0.w, -R3, c[6];
DP3 R0.x, vertex.attrib[14], c[6];
DP3 R0.z, vertex.normal, c[6];
MUL result.texcoord[2], R0, c[23].w;
DP3 R0.y, R2, c[7];
DP3 R0.w, -R3, c[7];
DP3 R0.x, vertex.attrib[14], c[7];
DP3 R0.z, vertex.normal, c[7];
MUL result.texcoord[3], R0, c[23].w;
DP4 R0.w, vertex.position, c[4];
DP4 R0.z, vertex.position, c[3];
DP4 R0.x, vertex.position, c[1];
DP4 R0.y, vertex.position, c[2];
DP3 result.texcoord[4].y, R2, R1;
DP3 result.texcoord[4].z, vertex.normal, R1;
DP3 result.texcoord[4].x, vertex.attrib[14], R1;
MUL R1.xyz, R0.xyww, c[0].y;
MUL R1.y, R1, c[14].x;
DP3 result.texcoord[6].y, R2, R3;
DP3 result.texcoord[6].z, vertex.normal, R3;
DP3 result.texcoord[6].x, vertex.attrib[14], R3;
ADD result.texcoord[7].xy, R1, R1.z;
MOV result.position, R0;
MOV result.texcoord[7].zw, R0;
MAD result.texcoord[0].xy, vertex.texcoord[0], c[24], c[24].zwzw;
END
# 63 instructions, 4 R-regs
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
"vs_3_0
; 66 ALU
dcl_position o0
dcl_texcoord0 o1
dcl_texcoord1 o2
dcl_texcoord2 o3
dcl_texcoord3 o4
dcl_texcoord4 o5
dcl_texcoord5 o6
dcl_texcoord6 o7
dcl_texcoord7 o8
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
add o6.xyz, r0, r1
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
mul r2.xyz, v1.w, r1
mov r0, c10
dp4 r4.z, c15, r0
mov r0, c9
dp4 r4.y, c15, r0
mov r1, c8
dp4 r4.x, c15, r1
dp3 r0.y, r2, c4
dp3 r0.w, -r3, c4
dp3 r0.x, v1, c4
dp3 r0.z, v2, c4
mul o2, r0, c23.w
dp3 r0.y, r2, c5
dp3 r0.w, -r3, c5
dp3 r0.x, v1, c5
dp3 r0.z, v2, c5
mul o3, r0, c23.w
dp3 r0.y, r2, c6
dp3 r0.w, -r3, c6
dp3 r0.x, v1, c6
dp3 r0.z, v2, c6
mul o4, r0, c23.w
dp4 r0.w, v0, c3
dp4 r0.z, v0, c2
dp4 r0.x, v0, c0
dp4 r0.y, v0, c1
mul r1.xyz, r0.xyww, c25.y
mul r1.y, r1, c13.x
dp3 o5.y, r2, r4
dp3 o7.y, r2, r3
dp3 o5.z, v2, r4
dp3 o5.x, v1, r4
dp3 o7.z, v2, r3
dp3 o7.x, v1, r3
mad o8.xy, r1.z, c14.zwzw, r1
mov o0, r0
mov o8.zw, r0
mad o1.xy, v3, c24, c24.zwzw
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
// ALU: 78.67 (59 instructions), vertex: 32, texture: 0,
//   sequencer: 26,  14 GPRs, 12 threads,
// Performance (if enough threads): ~78 cycles per vector
// * Vertex cycle estimates are assuming 3 vfetch_minis for every vfetch_full,
//     with <= 32 bytes per vfetch_full group.

"vs_360
backbbabaaaaadfmaaaaadjeaaaaaaaaaaaaaaceaaaaackiaaaaacnaaaaaaaaa
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
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaeaaaaaadfeaahbaaanaaaaaaaa
aaaaaaaaaaaagnaiaaaaaaabaaaaaaaeaaaaaaanaaaaacjaaabaaaahaaaagaai
aaaadaajaacafaakaaaadafaaaabpbfbaaacpcfcaaadpdfdaaaehefeaaahhfff
aaaihgfgaaalphfhaaaabadgaaaabaebaaaabaecaaaabaedaaaaaadaaaaaaadb
aaaabadcaaaabaefaaaaaaddaaaaaadeaaaabadfaaaaaacpaaaabaeaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaadpaaaaaaaaaaaaaaaaaaaaaaaaaaaaaapaffeaah
aaaabcaamcaaaaaaaaaafaalaaaabcaameaaaaaaaaaagabagabgbcaabcaaaaaa
aaaagabmgaccbcaabcaaaaaaaaaagacigacobcaabcaaaaaaaaaagadegadkbcaa
bcaaaaaaaaaagaeaaaaaccaaaaaaaaaaafpiiaaaaaaaagiiaaaaaaaaafpibaaa
aaaaagiiaaaaaaaaafpijaaaaaaaaeehaaaaaaaaafpieaaaaaaaadmhaaaaaaaa
kmepahaaaabliiedibaiaobbmiapaaaaaamgnapiklaianaamiapaaaaaalbdepi
klaiamaamiapaaanaagmnajeklaialaamiapiadoaananaaaocananaamiahaaac
aamamgmaalbeaabfceibakahaablgmgmkbabapiabechaaakaaleblblcbbfadab
kicnahadaablieebibajbbbabebhaaafaamglomgkbajbaabkichadagaalbgcma
ibabbabbmialaaaaaamamgleclbeadakmiahaaakaalelbleclbdaaackmehaaac
aamdgficmbajabbbmiahaaacablklomaolajabacmiahaaakaamagmleclbcaaak
miahaaalaalelbliclbdadaamiahaaamaagmloleklabapagmiahaaafaalbloma
klajapafkmieacaeaagmmgecmaafadbbbealaaaaaalkblgmkbajbgamaebeaeaf
aamgbllboaafadadbeaoaaagaalbimmgkbaabbamaebbafagaalbblmgoaamacaa
miahaaamaamagmleclbcadalmiahaaalabmablmaklakbgaimiahaaaiaagmlebf
klaabaagbeahaaaaaamnbllbobacabafaeemagadaamgiggmkbaabbadmiahaaak
aabllemaklaaapaikibcadagaalolomanaahacbakichadacaelbgciaibalbaba
miakaaafaalbmbgbklaaapadmiahaaaiaegmloleklalapacaibhadahaemggcgm
kbalbbakaibhajacaamagmggkbanppakmiamiaahaanlnlaaocananaamiabiaae
aaloloaapaamabaamiaciaaeaagcloaapaaaamaamiaeiaaeaalomdaapaamajaa
miabiaagaaloloaapaalabaamiaciaagaagcloaapaaaalaamiaeiaagaalomdaa
paalajaamiadiaaaaabjlabkilaebhbhaicbajabaadoanmbgpaeakakaiecajab
aadoanlbgpafakakaiieajabaadoanlmgpagakakaicbadaaaakhkhmgkpajahak
kiiiacaeaagmgmeboaaiahabbeacaaaaaakhkhblkpajaiafaeciaeafaamglbmg
oaaiahadbeaeaaaaaakhkhlbkpajajafaeciafagaalbmgbloaaiahadmiadiaah
aamgbkbiklacacacmiapiaabaaaablaakbagbgaamiapiaacaaaablaakbafbgaa
miapiaadaaaablaakbaebgaageihaaaaaalologboaabaaadmiahiaafaablmagf
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
"sce_vp_rsx // 61 instructions using 9 registers
[Configuration]
8
0000003d41050900
[Defaults]
1
455 1
3f000000
[Microcode]
976
00019c6c005d100d8186c0836041fffc00029c6c00400e0c0106c0836041dffc
00039c6c005d300c0186c0836041dffc00041c6c009c920c013fc0c36041dffc
401f9c6c011c8808010400d740619f9c00001c6c01506e0c010600c360411ffc
00001c6c0150620c010600c360405ffc00009c6c01505e0c010600c360411ffc
00009c6c0150520c010600c360405ffc00011c6c01504e0c010600c360411ffc
00011c6c0150420c010600c360405ffc00021c6c01d0300d8106c0c360403ffc
00021c6c01d0200d8106c0c360405ffc00021c6c01d0100d8106c0c360409ffc
00021c6c01d0000d8106c0c360411ffc00031c6c01d0a00d8686c0c360405ffc
00031c6c01d0900d8686c0c360409ffc00031c6c01d0800d8686c0c360411ffc
00019c6c0150400c108600c360411ffc00019c6c0150600c108600c360405ffc
00001c6c0150500c108600c360409ffc00041c6c0190a00c0e86c0c360405ffc
00041c6c0190900c0e86c0c360409ffc00041c6c0190800c0e86c0c360411ffc
00039c6c00800243011845436041dffc00029c6c010002308121856303a1dffc
401f9c6c0040000d8886c0836041ff80401f9c6c004000558886c08360407fb8
00039c6c011c900c10bfc0e30041dffc00021c6c009c700e088000c36041dffc
401f9c6c0140020c0106064360405fac401f9c6c01400e0c0106064360411fac
00021c6c009d202a888000c360409ffc00009c6c0080002a8095404360409ffc
00019c6c0040002a8086c08360409ffc401f9c6c00c000080886c09542219fb8
00029c6c00800e7f810605436041dffc401f9c6c0140020c0106074360405fb4
401f9c6c01400e0c0106074360411fb400001c6c0150608c0e8600c360403ffc
00009c6c0150508c0e8600c360403ffc00011c6c0150408c0e8600c360403ffc
00021c6c019ce00c0686c0c360405ffc00021c6c019cf00c0686c0c360409ffc
00021c6c019d000c0686c0c360411ffc00021c6c010000000680036aa0a03ffc
00019c6c0080000d069a03436041fffc401f9c6c0140000c0a86064360409fac
401f9c6c0140000c0a86074360409fb400001c6c0150600c0a8600c360409ffc
00009c6c0150500c0a8600c360409ffc00011c6c0150400c0a8600c360409ffc
00029c6c01dcb00d8686c0c360405ffc00029c6c01dcc00d8686c0c360409ffc
00029c6c01dcd00d8686c0c360411ffc00019c6c00c0000c0886c08302a1dffc
00021c6c009ca07f888600c36041dffc401f9c6c00c0000c0886c08301a1dfb0
401f9c6c009c900d84bfc0c36041ffa0401f9c6c009c900d82bfc0c36041ffa4
401f9c6c009c900d80bfc0c36041ffa9
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
Vector 160 [_MainTex_ST] 4
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
// 71 instructions, 6 temp regs, 0 temp arrays:
// ALU 39 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0
eefiecedmlaiodflmanbichlcihbigiikhhaklfhabaaaaaajialaaaaadaaaaaa
cmaaaaaapeaaaaaapeabaaaaejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaa
abaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaljaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfc
enebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheopiaaaaaaajaaaaaa
aiaaaaaaoaaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaomaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaabaaaaaaadamaaaaomaaaaaaabaaaaaaaaaaaaaa
adaaaaaaacaaaaaaapaaaaaaomaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaa
apaaaaaaomaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaomaaaaaa
aeaaaaaaaaaaaaaaadaaaaaaafaaaaaaahaiaaaaomaaaaaaafaaaaaaaaaaaaaa
adaaaaaaagaaaaaaahaiaaaaomaaaaaaagaaaaaaaaaaaaaaadaaaaaaahaaaaaa
ahaiaaaaomaaaaaaahaaaaaaaaaaaaaaadaaaaaaaiaaaaaaapaaaaaafdfgfpfa
epfdejfeejepeoaafeeffiedepepfceeaaklklklfdeieefcjmajaaaaeaaaabaa
ghacaaaafjaaaaaeegiocaaaaaaaaaaaalaaaaaafjaaaaaeegiocaaaabaaaaaa
agaaaaaafjaaaaaeegiocaaaacaaaaaabjaaaaaafjaaaaaeegiocaaaadaaaaaa
bfaaaaaafpaaaaadpcbabaaaaaaaaaaafpaaaaadpcbabaaaabaaaaaafpaaaaad
hcbabaaaacaaaaaafpaaaaaddcbabaaaadaaaaaaghaaaaaepccabaaaaaaaaaaa
abaaaaaagfaaaaaddccabaaaabaaaaaagfaaaaadpccabaaaacaaaaaagfaaaaad
pccabaaaadaaaaaagfaaaaadpccabaaaaeaaaaaagfaaaaadhccabaaaafaaaaaa
gfaaaaadhccabaaaagaaaaaagfaaaaadhccabaaaahaaaaaagfaaaaadpccabaaa
aiaaaaaagiaaaaacagaaaaaadiaaaaaipcaabaaaaaaaaaaafgbfbaaaaaaaaaaa
egiocaaaadaaaaaaabaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaa
aaaaaaaaagbabaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaa
egiocaaaadaaaaaaacaaaaaakgbkbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaak
pcaabaaaaaaaaaaaegiocaaaadaaaaaaadaaaaaapgbpbaaaaaaaaaaaegaobaaa
aaaaaaaadgaaaaafpccabaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaaldccabaaa
abaaaaaaegbabaaaadaaaaaaegiacaaaaaaaaaaaakaaaaaaogikcaaaaaaaaaaa
akaaaaaadiaaaaajhcaabaaaabaaaaaafgifcaaaabaaaaaaaeaaaaaaegiccaaa
adaaaaaabbaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabaaaaaaa
agiacaaaabaaaaaaaeaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaa
egiccaaaadaaaaaabcaaaaaakgikcaaaabaaaaaaaeaaaaaaegacbaaaabaaaaaa
aaaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaaegiccaaaadaaaaaabdaaaaaa
dcaaaaalhcaabaaaabaaaaaaegacbaaaabaaaaaapgipcaaaadaaaaaabeaaaaaa
egbcbaiaebaaaaaaaaaaaaaadiaaaaajhcaabaaaacaaaaaafgafbaiaebaaaaaa
abaaaaaaegiccaaaadaaaaaaanaaaaaadcaaaaalhcaabaaaacaaaaaaegiccaaa
adaaaaaaamaaaaaaagaabaiaebaaaaaaabaaaaaaegacbaaaacaaaaaadcaaaaal
lcaabaaaacaaaaaaegiicaaaadaaaaaaaoaaaaaakgakbaiaebaaaaaaabaaaaaa
egaibaaaacaaaaaadgaaaaaficaabaaaadaaaaaaakaabaaaacaaaaaadiaaaaah
hcaabaaaaeaaaaaajgbebaaaabaaaaaacgbjbaaaacaaaaaadcaaaaakhcaabaaa
aeaaaaaajgbebaaaacaaaaaacgbjbaaaabaaaaaaegacbaiaebaaaaaaaeaaaaaa
diaaaaahhcaabaaaaeaaaaaaegacbaaaaeaaaaaapgbpbaaaabaaaaaadgaaaaag
bcaabaaaafaaaaaaakiacaaaadaaaaaaamaaaaaadgaaaaagccaabaaaafaaaaaa
akiacaaaadaaaaaaanaaaaaadgaaaaagecaabaaaafaaaaaaakiacaaaadaaaaaa
aoaaaaaabaaaaaahccaabaaaadaaaaaaegacbaaaaeaaaaaaegacbaaaafaaaaaa
baaaaaahbcaabaaaadaaaaaaegbcbaaaabaaaaaaegacbaaaafaaaaaabaaaaaah
ecaabaaaadaaaaaaegbcbaaaacaaaaaaegacbaaaafaaaaaadiaaaaaipccabaaa
acaaaaaaegaobaaaadaaaaaapgipcaaaadaaaaaabeaaaaaadgaaaaaficaabaaa
adaaaaaabkaabaaaacaaaaaadgaaaaagbcaabaaaafaaaaaabkiacaaaadaaaaaa
amaaaaaadgaaaaagccaabaaaafaaaaaabkiacaaaadaaaaaaanaaaaaadgaaaaag
ecaabaaaafaaaaaabkiacaaaadaaaaaaaoaaaaaabaaaaaahccaabaaaadaaaaaa
egacbaaaaeaaaaaaegacbaaaafaaaaaabaaaaaahbcaabaaaadaaaaaaegbcbaaa
abaaaaaaegacbaaaafaaaaaabaaaaaahecaabaaaadaaaaaaegbcbaaaacaaaaaa
egacbaaaafaaaaaadiaaaaaipccabaaaadaaaaaaegaobaaaadaaaaaapgipcaaa
adaaaaaabeaaaaaadgaaaaagbcaabaaaadaaaaaackiacaaaadaaaaaaamaaaaaa
dgaaaaagccaabaaaadaaaaaackiacaaaadaaaaaaanaaaaaadgaaaaagecaabaaa
adaaaaaackiacaaaadaaaaaaaoaaaaaabaaaaaahccaabaaaacaaaaaaegacbaaa
aeaaaaaaegacbaaaadaaaaaabaaaaaahbcaabaaaacaaaaaaegbcbaaaabaaaaaa
egacbaaaadaaaaaabaaaaaahecaabaaaacaaaaaaegbcbaaaacaaaaaaegacbaaa
adaaaaaadiaaaaaipccabaaaaeaaaaaaegaobaaaacaaaaaapgipcaaaadaaaaaa
beaaaaaadiaaaaajhcaabaaaacaaaaaafgifcaaaacaaaaaaaaaaaaaaegiccaaa
adaaaaaabbaaaaaadcaaaaalhcaabaaaacaaaaaaegiccaaaadaaaaaabaaaaaaa
agiacaaaacaaaaaaaaaaaaaaegacbaaaacaaaaaadcaaaaalhcaabaaaacaaaaaa
egiccaaaadaaaaaabcaaaaaakgikcaaaacaaaaaaaaaaaaaaegacbaaaacaaaaaa
dcaaaaalhcaabaaaacaaaaaaegiccaaaadaaaaaabdaaaaaapgipcaaaacaaaaaa
aaaaaaaaegacbaaaacaaaaaabaaaaaahcccabaaaafaaaaaaegacbaaaaeaaaaaa
egacbaaaacaaaaaabaaaaaahcccabaaaahaaaaaaegacbaaaaeaaaaaaegacbaaa
abaaaaaabaaaaaahbccabaaaafaaaaaaegbcbaaaabaaaaaaegacbaaaacaaaaaa
baaaaaaheccabaaaafaaaaaaegbcbaaaacaaaaaaegacbaaaacaaaaaadiaaaaai
hcaabaaaacaaaaaaegbcbaaaacaaaaaapgipcaaaadaaaaaabeaaaaaadiaaaaai
hcaabaaaadaaaaaafgafbaaaacaaaaaaegiccaaaadaaaaaaanaaaaaadcaaaaak
lcaabaaaacaaaaaaegiicaaaadaaaaaaamaaaaaaagaabaaaacaaaaaaegaibaaa
adaaaaaadcaaaaakhcaabaaaacaaaaaaegiccaaaadaaaaaaaoaaaaaakgakbaaa
acaaaaaaegadbaaaacaaaaaadgaaaaaficaabaaaacaaaaaaabeaaaaaaaaaiadp
bbaaaaaibcaabaaaadaaaaaaegiocaaaacaaaaaabcaaaaaaegaobaaaacaaaaaa
bbaaaaaiccaabaaaadaaaaaaegiocaaaacaaaaaabdaaaaaaegaobaaaacaaaaaa
bbaaaaaiecaabaaaadaaaaaaegiocaaaacaaaaaabeaaaaaaegaobaaaacaaaaaa
diaaaaahpcaabaaaaeaaaaaajgacbaaaacaaaaaaegakbaaaacaaaaaabbaaaaai
bcaabaaaafaaaaaaegiocaaaacaaaaaabfaaaaaaegaobaaaaeaaaaaabbaaaaai
ccaabaaaafaaaaaaegiocaaaacaaaaaabgaaaaaaegaobaaaaeaaaaaabbaaaaai
ecaabaaaafaaaaaaegiocaaaacaaaaaabhaaaaaaegaobaaaaeaaaaaaaaaaaaah
hcaabaaaadaaaaaaegacbaaaadaaaaaaegacbaaaafaaaaaadiaaaaahicaabaaa
abaaaaaabkaabaaaacaaaaaabkaabaaaacaaaaaadcaaaaakicaabaaaabaaaaaa
akaabaaaacaaaaaaakaabaaaacaaaaaadkaabaiaebaaaaaaabaaaaaadcaaaaak
hccabaaaagaaaaaaegiccaaaacaaaaaabiaaaaaapgapbaaaabaaaaaaegacbaaa
adaaaaaabaaaaaahbccabaaaahaaaaaaegbcbaaaabaaaaaaegacbaaaabaaaaaa
baaaaaaheccabaaaahaaaaaaegbcbaaaacaaaaaaegacbaaaabaaaaaadiaaaaai
ccaabaaaaaaaaaaabkaabaaaaaaaaaaaakiacaaaabaaaaaaafaaaaaadiaaaaak
ncaabaaaabaaaaaaagahbaaaaaaaaaaaaceaaaaaaaaaaadpaaaaaaaaaaaaaadp
aaaaaadpdgaaaaafmccabaaaaiaaaaaakgaobaaaaaaaaaaaaaaaaaahdccabaaa
aiaaaaaakgakbaaaabaaaaaamgaabaaaabaaaaaadoaaaaab"
}

SubProgram "gles " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
"!!GLES
#define SHADER_API_GLES 1
#define tex2D texture2D


#ifdef VERTEX
#define gl_ModelViewProjectionMatrix glstate_matrix_mvp
uniform mat4 glstate_matrix_mvp;

varying highp vec4 xlv_TEXCOORD7;
varying highp vec3 xlv_TEXCOORD6;
varying lowp vec3 xlv_TEXCOORD5;
varying lowp vec3 xlv_TEXCOORD4;
varying lowp vec4 xlv_TEXCOORD3;
varying lowp vec4 xlv_TEXCOORD2;
varying lowp vec4 xlv_TEXCOORD1;
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
  xlv_TEXCOORD1 = tmpvar_4;
  xlv_TEXCOORD2 = tmpvar_5;
  xlv_TEXCOORD3 = tmpvar_6;
  xlv_TEXCOORD4 = tmpvar_7;
  xlv_TEXCOORD5 = tmpvar_8;
  xlv_TEXCOORD6 = (tmpvar_14 * (((_World2Object * tmpvar_26).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD7 = (unity_World2Shadow[0] * (_Object2World * _glesVertex));
}



#endif
#ifdef FRAGMENT

varying highp vec4 xlv_TEXCOORD7;
varying highp vec3 xlv_TEXCOORD6;
varying lowp vec3 xlv_TEXCOORD5;
varying lowp vec3 xlv_TEXCOORD4;
varying lowp vec4 xlv_TEXCOORD3;
varying lowp vec4 xlv_TEXCOORD2;
varying lowp vec4 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform mediump float _Gloss;
uniform mediump float _Shininess;
uniform mediump float _ReflectPower;
uniform lowp vec4 _ReflectColor;
uniform lowp vec4 _Color;
uniform samplerCube _Cube;
uniform sampler2D _SpecMap;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform sampler2D _ShadowMapTexture;
uniform lowp vec4 _LightColor0;
uniform highp vec4 _LightShadowData;
void main ()
{
  lowp vec4 c_1;
  highp vec3 tmpvar_2;
  mediump vec3 tmpvar_3;
  mediump vec3 tmpvar_4;
  mediump vec3 tmpvar_5;
  lowp vec3 tmpvar_6;
  tmpvar_6.x = xlv_TEXCOORD1.w;
  tmpvar_6.y = xlv_TEXCOORD2.w;
  tmpvar_6.z = xlv_TEXCOORD3.w;
  tmpvar_2 = tmpvar_6;
  lowp vec3 tmpvar_7;
  tmpvar_7 = xlv_TEXCOORD1.xyz;
  tmpvar_3 = tmpvar_7;
  lowp vec3 tmpvar_8;
  tmpvar_8 = xlv_TEXCOORD2.xyz;
  tmpvar_4 = tmpvar_8;
  lowp vec3 tmpvar_9;
  tmpvar_9 = xlv_TEXCOORD3.xyz;
  tmpvar_5 = tmpvar_9;
  mediump vec3 tmpvar_10;
  mediump vec3 tmpvar_11;
  mediump float tmpvar_12;
  mediump vec4 spec_13;
  lowp vec4 tmpvar_14;
  tmpvar_14 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_15;
  tmpvar_15 = (tmpvar_14.xyz * _Color.xyz);
  tmpvar_10 = tmpvar_15;
  lowp vec4 tmpvar_16;
  tmpvar_16 = texture2D (_SpecMap, xlv_TEXCOORD0);
  spec_13 = tmpvar_16;
  mediump vec3 tmpvar_17;
  tmpvar_17 = (spec_13.xyz * _Gloss);
  lowp vec3 tmpvar_18;
  tmpvar_18 = ((texture2D (_BumpMap, xlv_TEXCOORD0).xyz * 2.0) - 1.0);
  tmpvar_11 = tmpvar_18;
  mediump vec3 tmpvar_19;
  tmpvar_19.x = dot (tmpvar_3, tmpvar_11);
  tmpvar_19.y = dot (tmpvar_4, tmpvar_11);
  tmpvar_19.z = dot (tmpvar_5, tmpvar_11);
  highp vec3 tmpvar_20;
  tmpvar_20 = (tmpvar_2 - (2.0 * (dot (tmpvar_19, tmpvar_2) * tmpvar_19)));
  lowp vec4 tmpvar_21;
  tmpvar_21 = (textureCube (_Cube, tmpvar_20) * tmpvar_14.w);
  lowp float tmpvar_22;
  tmpvar_22 = (tmpvar_21.w * _ReflectColor.w);
  tmpvar_12 = tmpvar_22;
  lowp float tmpvar_23;
  mediump float lightShadowDataX_24;
  highp float dist_25;
  lowp float tmpvar_26;
  tmpvar_26 = texture2DProj (_ShadowMapTexture, xlv_TEXCOORD7).x;
  dist_25 = tmpvar_26;
  highp float tmpvar_27;
  tmpvar_27 = _LightShadowData.x;
  lightShadowDataX_24 = tmpvar_27;
  highp float tmpvar_28;
  tmpvar_28 = max (float((dist_25 > (xlv_TEXCOORD7.z / xlv_TEXCOORD7.w))), lightShadowDataX_24);
  tmpvar_23 = tmpvar_28;
  highp vec3 tmpvar_29;
  tmpvar_29 = normalize(xlv_TEXCOORD6);
  mediump vec3 lightDir_30;
  lightDir_30 = xlv_TEXCOORD4;
  mediump vec3 viewDir_31;
  viewDir_31 = tmpvar_29;
  mediump float atten_32;
  atten_32 = tmpvar_23;
  mediump vec4 c_33;
  mediump vec3 specCol_34;
  highp float nh_35;
  mediump float tmpvar_36;
  tmpvar_36 = max (0.0, dot (tmpvar_11, normalize((lightDir_30 + viewDir_31))));
  nh_35 = tmpvar_36;
  mediump float arg1_37;
  arg1_37 = (32.0 * _Shininess);
  highp vec3 tmpvar_38;
  tmpvar_38 = (pow (nh_35, arg1_37) * tmpvar_17);
  specCol_34 = tmpvar_38;
  c_33.xyz = ((((tmpvar_10 * _LightColor0.xyz) * max (0.0, dot (tmpvar_11, lightDir_30))) + (_LightColor0.xyz * specCol_34)) * (atten_32 * 2.0));
  c_33.w = tmpvar_12;
  c_1 = c_33;
  mediump vec3 tmpvar_39;
  tmpvar_39 = (c_1.xyz + (tmpvar_10 * xlv_TEXCOORD5));
  c_1.xyz = tmpvar_39;
  mediump vec3 tmpvar_40;
  tmpvar_40 = (c_1.xyz + ((tmpvar_21.xyz * _ReflectColor.xyz) * _ReflectPower));
  c_1.xyz = tmpvar_40;
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

varying highp vec4 xlv_TEXCOORD7;
varying highp vec3 xlv_TEXCOORD6;
varying lowp vec3 xlv_TEXCOORD5;
varying lowp vec3 xlv_TEXCOORD4;
varying lowp vec4 xlv_TEXCOORD3;
varying lowp vec4 xlv_TEXCOORD2;
varying lowp vec4 xlv_TEXCOORD1;
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
  lowp vec4 tmpvar_4;
  lowp vec4 tmpvar_5;
  lowp vec4 tmpvar_6;
  lowp vec3 tmpvar_7;
  lowp vec3 tmpvar_8;
  highp vec4 tmpvar_9;
  tmpvar_9 = (gl_ModelViewProjectionMatrix * _glesVertex);
  highp vec4 tmpvar_10;
  tmpvar_10.w = 1.0;
  tmpvar_10.xyz = _WorldSpaceCameraPos;
  mat3 tmpvar_11;
  tmpvar_11[0] = _Object2World[0].xyz;
  tmpvar_11[1] = _Object2World[1].xyz;
  tmpvar_11[2] = _Object2World[2].xyz;
  highp vec3 tmpvar_12;
  tmpvar_12 = (tmpvar_11 * (_glesVertex.xyz - ((_World2Object * tmpvar_10).xyz * unity_Scale.w)));
  highp vec3 tmpvar_13;
  highp vec3 tmpvar_14;
  tmpvar_13 = tmpvar_1.xyz;
  tmpvar_14 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_15;
  tmpvar_15[0].x = tmpvar_13.x;
  tmpvar_15[0].y = tmpvar_14.x;
  tmpvar_15[0].z = tmpvar_2.x;
  tmpvar_15[1].x = tmpvar_13.y;
  tmpvar_15[1].y = tmpvar_14.y;
  tmpvar_15[1].z = tmpvar_2.y;
  tmpvar_15[2].x = tmpvar_13.z;
  tmpvar_15[2].y = tmpvar_14.z;
  tmpvar_15[2].z = tmpvar_2.z;
  vec4 v_16;
  v_16.x = _Object2World[0].x;
  v_16.y = _Object2World[1].x;
  v_16.z = _Object2World[2].x;
  v_16.w = _Object2World[3].x;
  highp vec4 tmpvar_17;
  tmpvar_17.xyz = (tmpvar_15 * v_16.xyz);
  tmpvar_17.w = tmpvar_12.x;
  highp vec4 tmpvar_18;
  tmpvar_18 = (tmpvar_17 * unity_Scale.w);
  tmpvar_4 = tmpvar_18;
  vec4 v_19;
  v_19.x = _Object2World[0].y;
  v_19.y = _Object2World[1].y;
  v_19.z = _Object2World[2].y;
  v_19.w = _Object2World[3].y;
  highp vec4 tmpvar_20;
  tmpvar_20.xyz = (tmpvar_15 * v_19.xyz);
  tmpvar_20.w = tmpvar_12.y;
  highp vec4 tmpvar_21;
  tmpvar_21 = (tmpvar_20 * unity_Scale.w);
  tmpvar_5 = tmpvar_21;
  vec4 v_22;
  v_22.x = _Object2World[0].z;
  v_22.y = _Object2World[1].z;
  v_22.z = _Object2World[2].z;
  v_22.w = _Object2World[3].z;
  highp vec4 tmpvar_23;
  tmpvar_23.xyz = (tmpvar_15 * v_22.xyz);
  tmpvar_23.w = tmpvar_12.z;
  highp vec4 tmpvar_24;
  tmpvar_24 = (tmpvar_23 * unity_Scale.w);
  tmpvar_6 = tmpvar_24;
  mat3 tmpvar_25;
  tmpvar_25[0] = _Object2World[0].xyz;
  tmpvar_25[1] = _Object2World[1].xyz;
  tmpvar_25[2] = _Object2World[2].xyz;
  highp vec3 tmpvar_26;
  tmpvar_26 = (tmpvar_15 * (_World2Object * _WorldSpaceLightPos0).xyz);
  tmpvar_7 = tmpvar_26;
  highp vec4 tmpvar_27;
  tmpvar_27.w = 1.0;
  tmpvar_27.xyz = _WorldSpaceCameraPos;
  highp vec4 tmpvar_28;
  tmpvar_28.w = 1.0;
  tmpvar_28.xyz = (tmpvar_25 * (tmpvar_2 * unity_Scale.w));
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
  highp vec4 o_44;
  highp vec4 tmpvar_45;
  tmpvar_45 = (tmpvar_9 * 0.5);
  highp vec2 tmpvar_46;
  tmpvar_46.x = tmpvar_45.x;
  tmpvar_46.y = (tmpvar_45.y * _ProjectionParams.x);
  o_44.xy = (tmpvar_46 + tmpvar_45.w);
  o_44.zw = tmpvar_9.zw;
  gl_Position = tmpvar_9;
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = tmpvar_4;
  xlv_TEXCOORD2 = tmpvar_5;
  xlv_TEXCOORD3 = tmpvar_6;
  xlv_TEXCOORD4 = tmpvar_7;
  xlv_TEXCOORD5 = tmpvar_8;
  xlv_TEXCOORD6 = (tmpvar_15 * (((_World2Object * tmpvar_27).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD7 = o_44;
}



#endif
#ifdef FRAGMENT

varying highp vec4 xlv_TEXCOORD7;
varying highp vec3 xlv_TEXCOORD6;
varying lowp vec3 xlv_TEXCOORD5;
varying lowp vec3 xlv_TEXCOORD4;
varying lowp vec4 xlv_TEXCOORD3;
varying lowp vec4 xlv_TEXCOORD2;
varying lowp vec4 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform mediump float _Gloss;
uniform mediump float _Shininess;
uniform mediump float _ReflectPower;
uniform lowp vec4 _ReflectColor;
uniform lowp vec4 _Color;
uniform samplerCube _Cube;
uniform sampler2D _SpecMap;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform sampler2D _ShadowMapTexture;
uniform lowp vec4 _LightColor0;
void main ()
{
  lowp vec4 c_1;
  highp vec3 tmpvar_2;
  mediump vec3 tmpvar_3;
  mediump vec3 tmpvar_4;
  mediump vec3 tmpvar_5;
  lowp vec3 tmpvar_6;
  tmpvar_6.x = xlv_TEXCOORD1.w;
  tmpvar_6.y = xlv_TEXCOORD2.w;
  tmpvar_6.z = xlv_TEXCOORD3.w;
  tmpvar_2 = tmpvar_6;
  lowp vec3 tmpvar_7;
  tmpvar_7 = xlv_TEXCOORD1.xyz;
  tmpvar_3 = tmpvar_7;
  lowp vec3 tmpvar_8;
  tmpvar_8 = xlv_TEXCOORD2.xyz;
  tmpvar_4 = tmpvar_8;
  lowp vec3 tmpvar_9;
  tmpvar_9 = xlv_TEXCOORD3.xyz;
  tmpvar_5 = tmpvar_9;
  mediump vec3 tmpvar_10;
  mediump vec3 tmpvar_11;
  mediump float tmpvar_12;
  mediump vec4 spec_13;
  lowp vec4 tmpvar_14;
  tmpvar_14 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_15;
  tmpvar_15 = (tmpvar_14.xyz * _Color.xyz);
  tmpvar_10 = tmpvar_15;
  lowp vec4 tmpvar_16;
  tmpvar_16 = texture2D (_SpecMap, xlv_TEXCOORD0);
  spec_13 = tmpvar_16;
  mediump vec3 tmpvar_17;
  tmpvar_17 = (spec_13.xyz * _Gloss);
  lowp vec3 normal_18;
  normal_18.xy = ((texture2D (_BumpMap, xlv_TEXCOORD0).wy * 2.0) - 1.0);
  normal_18.z = sqrt(((1.0 - (normal_18.x * normal_18.x)) - (normal_18.y * normal_18.y)));
  tmpvar_11 = normal_18;
  mediump vec3 tmpvar_19;
  tmpvar_19.x = dot (tmpvar_3, tmpvar_11);
  tmpvar_19.y = dot (tmpvar_4, tmpvar_11);
  tmpvar_19.z = dot (tmpvar_5, tmpvar_11);
  highp vec3 tmpvar_20;
  tmpvar_20 = (tmpvar_2 - (2.0 * (dot (tmpvar_19, tmpvar_2) * tmpvar_19)));
  lowp vec4 tmpvar_21;
  tmpvar_21 = (textureCube (_Cube, tmpvar_20) * tmpvar_14.w);
  lowp float tmpvar_22;
  tmpvar_22 = (tmpvar_21.w * _ReflectColor.w);
  tmpvar_12 = tmpvar_22;
  lowp float tmpvar_23;
  tmpvar_23 = texture2DProj (_ShadowMapTexture, xlv_TEXCOORD7).x;
  highp vec3 tmpvar_24;
  tmpvar_24 = normalize(xlv_TEXCOORD6);
  mediump vec3 lightDir_25;
  lightDir_25 = xlv_TEXCOORD4;
  mediump vec3 viewDir_26;
  viewDir_26 = tmpvar_24;
  mediump float atten_27;
  atten_27 = tmpvar_23;
  mediump vec4 c_28;
  mediump vec3 specCol_29;
  highp float nh_30;
  mediump float tmpvar_31;
  tmpvar_31 = max (0.0, dot (tmpvar_11, normalize((lightDir_25 + viewDir_26))));
  nh_30 = tmpvar_31;
  mediump float arg1_32;
  arg1_32 = (32.0 * _Shininess);
  highp vec3 tmpvar_33;
  tmpvar_33 = (pow (nh_30, arg1_32) * tmpvar_17);
  specCol_29 = tmpvar_33;
  c_28.xyz = ((((tmpvar_10 * _LightColor0.xyz) * max (0.0, dot (tmpvar_11, lightDir_25))) + (_LightColor0.xyz * specCol_29)) * (atten_27 * 2.0));
  c_28.w = tmpvar_12;
  c_1 = c_28;
  mediump vec3 tmpvar_34;
  tmpvar_34 = (c_1.xyz + (tmpvar_10 * xlv_TEXCOORD5));
  c_1.xyz = tmpvar_34;
  mediump vec3 tmpvar_35;
  tmpvar_35 = (c_1.xyz + ((tmpvar_21.xyz * _ReflectColor.xyz) * _ReflectPower));
  c_1.xyz = tmpvar_35;
  gl_FragData[0] = c_1;
}



#endif"
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
Matrix 5 [_Object2World]
Matrix 9 [_World2Object]
Vector 16 [unity_Scale]
Vector 17 [unity_LightmapST]
Vector 18 [_MainTex_ST]
"3.0-!!ARBvp1.0
# 37 ALU
PARAM c[19] = { { 1, 0.5 },
		state.matrix.mvp,
		program.local[5..18] };
TEMP R0;
TEMP R1;
TEMP R2;
MOV R0.xyz, vertex.attrib[14];
MUL R1.xyz, vertex.normal.zxyw, R0.yzxw;
MAD R0.xyz, vertex.normal.yzxw, R0.zxyw, -R1;
MUL R1.xyz, vertex.attrib[14].w, R0;
MOV R0.xyz, c[13];
MOV R0.w, c[0].x;
DP4 R2.z, R0, c[11];
DP4 R2.x, R0, c[9];
DP4 R2.y, R0, c[10];
MAD R2.xyz, R2, c[16].w, -vertex.position;
DP3 R0.y, R1, c[5];
DP3 R0.w, -R2, c[5];
DP4 R1.w, vertex.position, c[4];
DP3 R0.x, vertex.attrib[14], c[5];
DP3 R0.z, vertex.normal, c[5];
MUL result.texcoord[1], R0, c[16].w;
DP3 R0.y, R1, c[6];
DP3 R0.w, -R2, c[6];
DP3 R0.x, vertex.attrib[14], c[6];
DP3 R0.z, vertex.normal, c[6];
MUL result.texcoord[2], R0, c[16].w;
DP3 R0.y, R1, c[7];
DP4 R1.z, vertex.position, c[3];
DP3 R0.w, -R2, c[7];
DP4 R1.x, vertex.position, c[1];
DP4 R1.y, vertex.position, c[2];
MUL R2.xyz, R1.xyww, c[0].y;
DP3 R0.x, vertex.attrib[14], c[7];
DP3 R0.z, vertex.normal, c[7];
MUL result.texcoord[3], R0, c[16].w;
MOV R0.x, R2;
MUL R0.y, R2, c[14].x;
ADD result.texcoord[5].xy, R0, R2.z;
MOV result.position, R1;
MOV result.texcoord[5].zw, R1;
MAD result.texcoord[0].xy, vertex.texcoord[0], c[18], c[18].zwzw;
MAD result.texcoord[4].xy, vertex.texcoord[1], c[17], c[17].zwzw;
END
# 37 instructions, 3 R-regs
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
Matrix 4 [_Object2World]
Matrix 8 [_World2Object]
Vector 15 [unity_Scale]
Vector 16 [unity_LightmapST]
Vector 17 [_MainTex_ST]
"vs_3_0
; 38 ALU
dcl_position o0
dcl_texcoord0 o1
dcl_texcoord1 o2
dcl_texcoord2 o3
dcl_texcoord3 o4
dcl_texcoord4 o5
dcl_texcoord5 o6
def c18, 1.00000000, 0.50000000, 0, 0
dcl_position0 v0
dcl_tangent0 v1
dcl_normal0 v2
dcl_texcoord0 v3
dcl_texcoord1 v4
mov r0.xyz, v1
mul r1.xyz, v2.zxyw, r0.yzxw
mov r0.xyz, v1
mad r0.xyz, v2.yzxw, r0.zxyw, -r1
mul r1.xyz, v1.w, r0
mov r0.xyz, c12
mov r0.w, c18.x
dp4 r2.z, r0, c10
dp4 r2.x, r0, c8
dp4 r2.y, r0, c9
mad r2.xyz, r2, c15.w, -v0
dp3 r0.y, r1, c4
dp3 r0.w, -r2, c4
dp4 r1.w, v0, c3
dp3 r0.x, v1, c4
dp3 r0.z, v2, c4
mul o2, r0, c15.w
dp3 r0.y, r1, c5
dp3 r0.w, -r2, c5
dp3 r0.x, v1, c5
dp3 r0.z, v2, c5
mul o3, r0, c15.w
dp3 r0.y, r1, c6
dp4 r1.z, v0, c2
dp3 r0.w, -r2, c6
dp4 r1.x, v0, c0
dp4 r1.y, v0, c1
mul r2.xyz, r1.xyww, c18.y
dp3 r0.x, v1, c6
dp3 r0.z, v2, c6
mul o4, r0, c15.w
mov r0.x, r2
mul r0.y, r2, c13.x
mad o6.xy, r2.z, c14.zwzw, r0
mov o0, r1
mov o6.zw, r1
mad o1.xy, v3, c17, c17.zwzw
mad o5.xy, v4, c16, c16.zwzw
"
}

SubProgram "xbox360 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
Vector 16 [_MainTex_ST]
Matrix 7 [_Object2World] 3
Vector 1 [_ProjectionParams]
Vector 2 [_ScreenParams]
Matrix 10 [_World2Object] 4
Vector 0 [_WorldSpaceCameraPos]
Matrix 3 [glstate_matrix_mvp] 4
Vector 15 [unity_LightmapST]
Vector 14 [unity_Scale]
// Shader Timing Estimate, in Cycles/64 vertex vector:
// ALU: 52.00 (39 instructions), vertex: 64, texture: 0,
//   sequencer: 20,  11 GPRs, 15 threads,
// Performance (if enough threads): ~64 cycles per vector
// * Vertex cycle estimates are assuming 3 vfetch_minis for every vfetch_full,
//     with <= 32 bytes per vfetch_full group.

"vs_360
backbbabaaaaacgeaaaaacjiaaaaaaaaaaaaaaceaaaaabmmaaaaabpeaaaaaaaa
aaaaaaaaaaaaabkeaaaaaabmaaaaabjhpppoadaaaaaaaaajaaaaaabmaaaaaaaa
aaaaabjaaaaaaanaaaacaabaaaabaaaaaaaaaanmaaaaaaaaaaaaaaomaaacaaah
aaadaaaaaaaaaapmaaaaaaaaaaaaabamaaacaaabaaabaaaaaaaaaanmaaaaaaaa
aaaaabboaaacaaacaaabaaaaaaaaaanmaaaaaaaaaaaaabcmaaacaaakaaaeaaaa
aaaaaapmaaaaaaaaaaaaabdkaaacaaaaaaabaaaaaaaaabfaaaaaaaaaaaaaabga
aaacaaadaaaeaaaaaaaaaapmaaaaaaaaaaaaabhdaaacaaapaaabaaaaaaaaaanm
aaaaaaaaaaaaabieaaacaaaoaaabaaaaaaaaaanmaaaaaaaafpengbgjgofegfhi
fpfdfeaaaaabaaadaaabaaaeaaabaaaaaaaaaaaafpepgcgkgfgdhedcfhgphcgm
geaaklklaaadaaadaaaeaaaeaaabaaaaaaaaaaaafpfahcgpgkgfgdhegjgpgofa
gbhcgbgnhdaafpfdgdhcgfgfgofagbhcgbgnhdaafpfhgphcgmgedcepgcgkgfgd
heaafpfhgphcgmgefdhagbgdgfedgbgngfhcgbfagphdaaklaaabaaadaaabaaad
aaabaaaaaaaaaaaaghgmhdhegbhegffpgngbhehcgjhifpgnhghaaahfgogjhehj
fpemgjghgihegngbhafdfeaahfgogjhehjfpfdgdgbgmgfaahghdfpddfpdaaadc
codacodcdadddfddcodaaaklaaaaaaaaaaaaaaabaaaaaaaaaaaaaaaaaaaaaabe
aapmaabaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaeaaaaaacfiaafbaaak
aaaaaaaaaaaaaaaaaaaafamgaaaaaaabaaaaaaafaaaaaaahaaaaacjaaabaaaaf
aaaagaagaaaadaahaaaafaaiaacbfaajaaaadafaaaabpbfbaaacpcfcaaadpdfd
aaaedefeaaafpfffaaaabacgaaaabacoaaaabacpaaaabadaaaaabachaaaaaacf
aaaabacnaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaadpaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaapbfffaafaaaabcabmcaaaaaaaaaafaakaaaabcaameaaaaaaaaaagaap
gabfbcaabcaaaaaaaaaagablgacbbcaabcaaaaaaaaaagacheacnbcaaccaaaaaa
afpidaaaaaaaaeedaaaaaaaaafpicaaaaaaaaefiaaaaaaaaafpiaaaaaaaaacih
aaaaaaaaafpiiaaaaaaaacdpaaaaaaaaafpiiaaaaaaaapmiaaaaaaaamiapaaab
aagmiiaakbadagaamiapaaabaablnapikladafabmiapaaabaamgdepikladaeab
miapaaajaalbnajekladadabmiapiadoaananaaaocajajaamiahaaaeaamamgma
alamaaanbeccabahaalbgmlbkbacaiackmbeahahaalbgmebibacajahkibhabaf
aablgcmdibacajaikichabakaamggcedibacaiaimiahaaaeaalelbleclalaaae
kiehabagaabcgkidmbaaacaimiahaaagablhmpmaolaaacagmiahaaaeaamagmle
clakaaaemialaaabaalbgcleklaaahabmianaaacaagmiekoklacahakbealaaaa
aamggcmgkbaaajacaebeababaagmgmgmoaabaaafbeaeaaacaalblbbloaabaaac
aebbacadaagmmglboaacafafmiahaaaeabmablbfklaeaoadbeahaaaaaalolbbl
obagacabaeekadacaamglgblkbaaaiaakichabafaelbgcmaibaeaiajmiahaaaf
aegmloleklaeahafmiakaaadaalblgbbklaaahackicoacaaaapmgmiaibajppaj
miamiaafaanlnlaaocajajaamiadiaaaaabklabkilaibabamiadiaaeaalalabk
ilaiapapkibhaaaeaemggcecibaeajabbeaiaaabaagmgmlboaafaeadaeciabac
aamglblboaafaeabbeacaaadaaloloblpaahagadaeciacadaalbmglboaafaeac
miadiaafaablbkgnklaaacaamiapiaabaaaablaakbadaoaamiapiaacaaaablaa
kbacaoaamiapiaadaaaablaakbabaoaaaaaaaaaaaaaaaaaaaaaaaaaa"
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
Matrix 260 [_Object2World]
Matrix 264 [_World2Object]
Vector 465 [unity_Scale]
Vector 464 [unity_LightmapST]
Vector 463 [_MainTex_ST]
"sce_vp_rsx // 35 instructions using 7 registers
[Configuration]
8
0000002343050700
[Defaults]
1
462 1
3f000000
[Microcode]
560
00029c6c00400e0c0106c0836041dffc00031c6c005d300c0186c0836041dffc
401f9c6c011cf808010400d740619f9c401f9c6c011d0908010400d740619fac
00001c6c01506e0c010600c360411ffc00001c6c0150620c010600c360405ffc
00009c6c01505e0c010600c360411ffc00009c6c0150520c010600c360405ffc
00011c6c01504e0c010600c360411ffc00011c6c0150420c010600c360405ffc
00019c6c01d0300d8106c0c360403ffc00019c6c01d0200d8106c0c360405ffc
00019c6c01d0100d8106c0c360409ffc00019c6c01d0000d8106c0c360411ffc
00021c6c0190a00c0c86c0c360405ffc00021c6c0190900c0c86c0c360409ffc
00021c6c0190800c0c86c0c360411ffc00031c6c00800243011845436041dffc
00029c6c01000230812185630321dffc401f9c6c0040000d8686c0836041ff80
401f9c6c004000558686c08360407fb000021c6c011d100c08bfc0e30041dffc
00019c6c009ce00e068000c36041dffc00019c6c009d202a868000c360409ffc
401f9c6c00c000080686c09541a19fb000001c6c0150608c088600c360403ffc
00009c6c0150508c088600c360403ffc00011c6c0150408c088600c360403ffc
00019c6c00800e7f810605436041dffc00001c6c0150600c068600c360409ffc
00011c6c0150400c068600c360409ffc00009c6c0150500c068600c360409ffc
401f9c6c009d100d84bfc0c36041ffa0401f9c6c009d100d82bfc0c36041ffa4
401f9c6c009d100d80bfc0c36041ffa9
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
Vector 160 [unity_LightmapST] 4
Vector 176 [_MainTex_ST] 4
ConstBuffer "UnityPerCamera" 128 // 96 used size, 8 vars
Vector 64 [_WorldSpaceCameraPos] 3
Vector 80 [_ProjectionParams] 4
ConstBuffer "UnityPerDraw" 336 // 336 used size, 6 vars
Matrix 0 [glstate_matrix_mvp] 4
Matrix 192 [_Object2World] 4
Matrix 256 [_World2Object] 4
Vector 320 [unity_Scale] 4
BindCB "$Globals" 0
BindCB "UnityPerCamera" 1
BindCB "UnityPerDraw" 2
// 46 instructions, 5 temp regs, 0 temp arrays:
// ALU 21 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0
eefiecedhadoolebjojmipjfceafnjpeolednicfabaaaaaacaaiaaaaadaaaaaa
cmaaaaaapeaaaaaameabaaaaejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaa
abaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapadaaaaljaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfc
enebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheomiaaaaaaahaaaaaa
aiaaaaaalaaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaalmaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaabaaaaaaadamaaaalmaaaaaaaeaaaaaaaaaaaaaa
adaaaaaaabaaaaaaamadaaaalmaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaa
apaaaaaalmaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaapaaaaaalmaaaaaa
adaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaalmaaaaaaafaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaafdfgfpfaepfdejfeejepeoaafeeffiedepepfcee
aaklklklfdeieefcfeagaaaaeaaaabaajfabaaaafjaaaaaeegiocaaaaaaaaaaa
amaaaaaafjaaaaaeegiocaaaabaaaaaaagaaaaaafjaaaaaeegiocaaaacaaaaaa
bfaaaaaafpaaaaadpcbabaaaaaaaaaaafpaaaaadpcbabaaaabaaaaaafpaaaaad
hcbabaaaacaaaaaafpaaaaaddcbabaaaadaaaaaafpaaaaaddcbabaaaaeaaaaaa
ghaaaaaepccabaaaaaaaaaaaabaaaaaagfaaaaaddccabaaaabaaaaaagfaaaaad
mccabaaaabaaaaaagfaaaaadpccabaaaacaaaaaagfaaaaadpccabaaaadaaaaaa
gfaaaaadpccabaaaaeaaaaaagfaaaaadpccabaaaafaaaaaagiaaaaacafaaaaaa
diaaaaaipcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaacaaaaaaabaaaaaa
dcaaaaakpcaabaaaaaaaaaaaegiocaaaacaaaaaaaaaaaaaaagbabaaaaaaaaaaa
egaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaacaaaaaaacaaaaaa
kgbkbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaa
acaaaaaaadaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadgaaaaafpccabaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaaldccabaaaabaaaaaaegbabaaaadaaaaaa
egiacaaaaaaaaaaaalaaaaaaogikcaaaaaaaaaaaalaaaaaadcaaaaalmccabaaa
abaaaaaaagbebaaaaeaaaaaaagiecaaaaaaaaaaaakaaaaaakgiocaaaaaaaaaaa
akaaaaaadiaaaaajhcaabaaaabaaaaaafgifcaaaabaaaaaaaeaaaaaaegiccaaa
acaaaaaabbaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaacaaaaaabaaaaaaa
agiacaaaabaaaaaaaeaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaa
egiccaaaacaaaaaabcaaaaaakgikcaaaabaaaaaaaeaaaaaaegacbaaaabaaaaaa
aaaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaaegiccaaaacaaaaaabdaaaaaa
dcaaaaalhcaabaaaabaaaaaaegacbaaaabaaaaaapgipcaaaacaaaaaabeaaaaaa
egbcbaiaebaaaaaaaaaaaaaadiaaaaajhcaabaaaacaaaaaafgafbaiaebaaaaaa
abaaaaaaegiccaaaacaaaaaaanaaaaaadcaaaaallcaabaaaabaaaaaaegiicaaa
acaaaaaaamaaaaaaagaabaiaebaaaaaaabaaaaaaegaibaaaacaaaaaadcaaaaal
lcaabaaaabaaaaaaegiicaaaacaaaaaaaoaaaaaakgakbaiaebaaaaaaabaaaaaa
egambaaaabaaaaaadgaaaaaficaabaaaacaaaaaaakaabaaaabaaaaaadiaaaaah
hcaabaaaadaaaaaajgbebaaaabaaaaaacgbjbaaaacaaaaaadcaaaaakhcaabaaa
adaaaaaajgbebaaaacaaaaaacgbjbaaaabaaaaaaegacbaiaebaaaaaaadaaaaaa
diaaaaahhcaabaaaadaaaaaaegacbaaaadaaaaaapgbpbaaaabaaaaaadgaaaaag
bcaabaaaaeaaaaaaakiacaaaacaaaaaaamaaaaaadgaaaaagccaabaaaaeaaaaaa
akiacaaaacaaaaaaanaaaaaadgaaaaagecaabaaaaeaaaaaaakiacaaaacaaaaaa
aoaaaaaabaaaaaahccaabaaaacaaaaaaegacbaaaadaaaaaaegacbaaaaeaaaaaa
baaaaaahbcaabaaaacaaaaaaegbcbaaaabaaaaaaegacbaaaaeaaaaaabaaaaaah
ecaabaaaacaaaaaaegbcbaaaacaaaaaaegacbaaaaeaaaaaadiaaaaaipccabaaa
acaaaaaaegaobaaaacaaaaaapgipcaaaacaaaaaabeaaaaaadgaaaaaficaabaaa
acaaaaaabkaabaaaabaaaaaadgaaaaagbcaabaaaaeaaaaaabkiacaaaacaaaaaa
amaaaaaadgaaaaagccaabaaaaeaaaaaabkiacaaaacaaaaaaanaaaaaadgaaaaag
ecaabaaaaeaaaaaabkiacaaaacaaaaaaaoaaaaaabaaaaaahccaabaaaacaaaaaa
egacbaaaadaaaaaaegacbaaaaeaaaaaabaaaaaahbcaabaaaacaaaaaaegbcbaaa
abaaaaaaegacbaaaaeaaaaaabaaaaaahecaabaaaacaaaaaaegbcbaaaacaaaaaa
egacbaaaaeaaaaaadiaaaaaipccabaaaadaaaaaaegaobaaaacaaaaaapgipcaaa
acaaaaaabeaaaaaadgaaaaagbcaabaaaacaaaaaackiacaaaacaaaaaaamaaaaaa
dgaaaaagccaabaaaacaaaaaackiacaaaacaaaaaaanaaaaaadgaaaaagecaabaaa
acaaaaaackiacaaaacaaaaaaaoaaaaaabaaaaaahccaabaaaabaaaaaaegacbaaa
adaaaaaaegacbaaaacaaaaaabaaaaaahbcaabaaaabaaaaaaegbcbaaaabaaaaaa
egacbaaaacaaaaaabaaaaaahecaabaaaabaaaaaaegbcbaaaacaaaaaaegacbaaa
acaaaaaadiaaaaaipccabaaaaeaaaaaaegaobaaaabaaaaaapgipcaaaacaaaaaa
beaaaaaadiaaaaaiccaabaaaaaaaaaaabkaabaaaaaaaaaaaakiacaaaabaaaaaa
afaaaaaadiaaaaakncaabaaaabaaaaaaagahbaaaaaaaaaaaaceaaaaaaaaaaadp
aaaaaaaaaaaaaadpaaaaaadpdgaaaaafmccabaaaafaaaaaakgaobaaaaaaaaaaa
aaaaaaahdccabaaaafaaaaaakgakbaaaabaaaaaamgaabaaaabaaaaaadoaaaaab
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

varying highp vec4 xlv_TEXCOORD5;
varying highp vec2 xlv_TEXCOORD4;
varying lowp vec4 xlv_TEXCOORD3;
varying lowp vec4 xlv_TEXCOORD2;
varying lowp vec4 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
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
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = tmpvar_3;
  xlv_TEXCOORD2 = tmpvar_4;
  xlv_TEXCOORD3 = tmpvar_5;
  xlv_TEXCOORD4 = ((_glesMultiTexCoord1.xy * unity_LightmapST.xy) + unity_LightmapST.zw);
  xlv_TEXCOORD5 = (unity_World2Shadow[0] * (_Object2World * _glesVertex));
}



#endif
#ifdef FRAGMENT

varying highp vec4 xlv_TEXCOORD5;
varying highp vec2 xlv_TEXCOORD4;
varying lowp vec4 xlv_TEXCOORD3;
varying lowp vec4 xlv_TEXCOORD2;
varying lowp vec4 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform sampler2D unity_Lightmap;
uniform mediump float _ReflectPower;
uniform lowp vec4 _ReflectColor;
uniform lowp vec4 _Color;
uniform samplerCube _Cube;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform sampler2D _ShadowMapTexture;
uniform highp vec4 _LightShadowData;
void main ()
{
  lowp vec4 c_1;
  highp vec3 tmpvar_2;
  mediump vec3 tmpvar_3;
  mediump vec3 tmpvar_4;
  mediump vec3 tmpvar_5;
  lowp vec3 tmpvar_6;
  tmpvar_6.x = xlv_TEXCOORD1.w;
  tmpvar_6.y = xlv_TEXCOORD2.w;
  tmpvar_6.z = xlv_TEXCOORD3.w;
  tmpvar_2 = tmpvar_6;
  lowp vec3 tmpvar_7;
  tmpvar_7 = xlv_TEXCOORD1.xyz;
  tmpvar_3 = tmpvar_7;
  lowp vec3 tmpvar_8;
  tmpvar_8 = xlv_TEXCOORD2.xyz;
  tmpvar_4 = tmpvar_8;
  lowp vec3 tmpvar_9;
  tmpvar_9 = xlv_TEXCOORD3.xyz;
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
  lowp vec4 tmpvar_18;
  tmpvar_18 = (textureCube (_Cube, tmpvar_17) * tmpvar_13.w);
  lowp float tmpvar_19;
  tmpvar_19 = (tmpvar_18.w * _ReflectColor.w);
  tmpvar_12 = tmpvar_19;
  lowp float tmpvar_20;
  mediump float lightShadowDataX_21;
  highp float dist_22;
  lowp float tmpvar_23;
  tmpvar_23 = texture2DProj (_ShadowMapTexture, xlv_TEXCOORD5).x;
  dist_22 = tmpvar_23;
  highp float tmpvar_24;
  tmpvar_24 = _LightShadowData.x;
  lightShadowDataX_21 = tmpvar_24;
  highp float tmpvar_25;
  tmpvar_25 = max (float((dist_22 > (xlv_TEXCOORD5.z / xlv_TEXCOORD5.w))), lightShadowDataX_21);
  tmpvar_20 = tmpvar_25;
  lowp vec3 tmpvar_26;
  tmpvar_26 = min ((2.0 * texture2D (unity_Lightmap, xlv_TEXCOORD4).xyz), vec3((tmpvar_20 * 2.0)));
  mediump vec3 tmpvar_27;
  tmpvar_27 = (tmpvar_10 * tmpvar_26);
  c_1.xyz = tmpvar_27;
  c_1.w = tmpvar_12;
  mediump vec3 tmpvar_28;
  tmpvar_28 = (c_1.xyz + ((tmpvar_18.xyz * _ReflectColor.xyz) * _ReflectPower));
  c_1.xyz = tmpvar_28;
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

varying highp vec4 xlv_TEXCOORD5;
varying highp vec2 xlv_TEXCOORD4;
varying lowp vec4 xlv_TEXCOORD3;
varying lowp vec4 xlv_TEXCOORD2;
varying lowp vec4 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp vec4 _MainTex_ST;
uniform highp vec4 unity_LightmapST;
uniform highp vec4 unity_Scale;
uniform highp mat4 _World2Object;
uniform highp mat4 _Object2World;

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
  lowp vec4 tmpvar_3;
  lowp vec4 tmpvar_4;
  lowp vec4 tmpvar_5;
  highp vec4 tmpvar_6;
  tmpvar_6 = (gl_ModelViewProjectionMatrix * _glesVertex);
  highp vec4 tmpvar_7;
  tmpvar_7.w = 1.0;
  tmpvar_7.xyz = _WorldSpaceCameraPos;
  mat3 tmpvar_8;
  tmpvar_8[0] = _Object2World[0].xyz;
  tmpvar_8[1] = _Object2World[1].xyz;
  tmpvar_8[2] = _Object2World[2].xyz;
  highp vec3 tmpvar_9;
  tmpvar_9 = (tmpvar_8 * (_glesVertex.xyz - ((_World2Object * tmpvar_7).xyz * unity_Scale.w)));
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
  vec4 v_13;
  v_13.x = _Object2World[0].x;
  v_13.y = _Object2World[1].x;
  v_13.z = _Object2World[2].x;
  v_13.w = _Object2World[3].x;
  highp vec4 tmpvar_14;
  tmpvar_14.xyz = (tmpvar_12 * v_13.xyz);
  tmpvar_14.w = tmpvar_9.x;
  highp vec4 tmpvar_15;
  tmpvar_15 = (tmpvar_14 * unity_Scale.w);
  tmpvar_3 = tmpvar_15;
  vec4 v_16;
  v_16.x = _Object2World[0].y;
  v_16.y = _Object2World[1].y;
  v_16.z = _Object2World[2].y;
  v_16.w = _Object2World[3].y;
  highp vec4 tmpvar_17;
  tmpvar_17.xyz = (tmpvar_12 * v_16.xyz);
  tmpvar_17.w = tmpvar_9.y;
  highp vec4 tmpvar_18;
  tmpvar_18 = (tmpvar_17 * unity_Scale.w);
  tmpvar_4 = tmpvar_18;
  vec4 v_19;
  v_19.x = _Object2World[0].z;
  v_19.y = _Object2World[1].z;
  v_19.z = _Object2World[2].z;
  v_19.w = _Object2World[3].z;
  highp vec4 tmpvar_20;
  tmpvar_20.xyz = (tmpvar_12 * v_19.xyz);
  tmpvar_20.w = tmpvar_9.z;
  highp vec4 tmpvar_21;
  tmpvar_21 = (tmpvar_20 * unity_Scale.w);
  tmpvar_5 = tmpvar_21;
  highp vec4 o_22;
  highp vec4 tmpvar_23;
  tmpvar_23 = (tmpvar_6 * 0.5);
  highp vec2 tmpvar_24;
  tmpvar_24.x = tmpvar_23.x;
  tmpvar_24.y = (tmpvar_23.y * _ProjectionParams.x);
  o_22.xy = (tmpvar_24 + tmpvar_23.w);
  o_22.zw = tmpvar_6.zw;
  gl_Position = tmpvar_6;
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = tmpvar_3;
  xlv_TEXCOORD2 = tmpvar_4;
  xlv_TEXCOORD3 = tmpvar_5;
  xlv_TEXCOORD4 = ((_glesMultiTexCoord1.xy * unity_LightmapST.xy) + unity_LightmapST.zw);
  xlv_TEXCOORD5 = o_22;
}



#endif
#ifdef FRAGMENT

varying highp vec4 xlv_TEXCOORD5;
varying highp vec2 xlv_TEXCOORD4;
varying lowp vec4 xlv_TEXCOORD3;
varying lowp vec4 xlv_TEXCOORD2;
varying lowp vec4 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform sampler2D unity_Lightmap;
uniform mediump float _ReflectPower;
uniform lowp vec4 _ReflectColor;
uniform lowp vec4 _Color;
uniform samplerCube _Cube;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform sampler2D _ShadowMapTexture;
void main ()
{
  lowp vec4 c_1;
  highp vec3 tmpvar_2;
  mediump vec3 tmpvar_3;
  mediump vec3 tmpvar_4;
  mediump vec3 tmpvar_5;
  lowp vec3 tmpvar_6;
  tmpvar_6.x = xlv_TEXCOORD1.w;
  tmpvar_6.y = xlv_TEXCOORD2.w;
  tmpvar_6.z = xlv_TEXCOORD3.w;
  tmpvar_2 = tmpvar_6;
  lowp vec3 tmpvar_7;
  tmpvar_7 = xlv_TEXCOORD1.xyz;
  tmpvar_3 = tmpvar_7;
  lowp vec3 tmpvar_8;
  tmpvar_8 = xlv_TEXCOORD2.xyz;
  tmpvar_4 = tmpvar_8;
  lowp vec3 tmpvar_9;
  tmpvar_9 = xlv_TEXCOORD3.xyz;
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
  lowp vec4 tmpvar_18;
  tmpvar_18 = (textureCube (_Cube, tmpvar_17) * tmpvar_13.w);
  lowp float tmpvar_19;
  tmpvar_19 = (tmpvar_18.w * _ReflectColor.w);
  tmpvar_12 = tmpvar_19;
  lowp vec4 tmpvar_20;
  tmpvar_20 = texture2DProj (_ShadowMapTexture, xlv_TEXCOORD5);
  lowp vec4 tmpvar_21;
  tmpvar_21 = texture2D (unity_Lightmap, xlv_TEXCOORD4);
  lowp vec3 tmpvar_22;
  tmpvar_22 = ((8.0 * tmpvar_21.w) * tmpvar_21.xyz);
  lowp vec3 tmpvar_23;
  tmpvar_23 = max (min (tmpvar_22, ((tmpvar_20.x * 2.0) * tmpvar_21.xyz)), (tmpvar_22 * tmpvar_20.x));
  mediump vec3 tmpvar_24;
  tmpvar_24 = (tmpvar_10 * tmpvar_23);
  c_1.xyz = tmpvar_24;
  c_1.w = tmpvar_12;
  mediump vec3 tmpvar_25;
  tmpvar_25 = (c_1.xyz + ((tmpvar_18.xyz * _ReflectColor.xyz) * _ReflectPower));
  c_1.xyz = tmpvar_25;
  gl_FragData[0] = c_1;
}



#endif"
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
ADD result.texcoord[5].xyz, R0, R1;
MOV R1.xyz, c[13];
DP4 R2.z, R1, c[11];
DP4 R2.y, R1, c[10];
DP4 R2.x, R1, c[9];
MAD R3.xyz, R2, c[30].w, -vertex.position;
MOV R0.xyz, vertex.attrib[14];
MUL R1.xyz, vertex.normal.zxyw, R0.yzxw;
MAD R1.xyz, vertex.normal.yzxw, R0.zxyw, -R1;
MUL R2.xyz, vertex.attrib[14].w, R1;
MOV R0, c[14];
DP4 R1.z, R0, c[11];
DP4 R1.x, R0, c[9];
DP4 R1.y, R0, c[10];
DP3 R0.y, R2, c[5];
DP3 R0.w, -R3, c[5];
DP3 R0.x, vertex.attrib[14], c[5];
DP3 R0.z, vertex.normal, c[5];
MUL result.texcoord[1], R0, c[30].w;
DP3 R0.y, R2, c[6];
DP3 R0.w, -R3, c[6];
DP3 R0.x, vertex.attrib[14], c[6];
DP3 R0.z, vertex.normal, c[6];
MUL result.texcoord[2], R0, c[30].w;
DP3 R0.y, R2, c[7];
DP3 R0.w, -R3, c[7];
DP3 R0.x, vertex.attrib[14], c[7];
DP3 R0.z, vertex.normal, c[7];
DP3 result.texcoord[4].y, R2, R1;
DP3 result.texcoord[6].y, R2, R3;
MUL result.texcoord[3], R0, c[30].w;
DP3 result.texcoord[4].z, vertex.normal, R1;
DP3 result.texcoord[4].x, vertex.attrib[14], R1;
DP3 result.texcoord[6].z, vertex.normal, R3;
DP3 result.texcoord[6].x, vertex.attrib[14], R3;
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
add r0.xyz, r2, r0
add o6.xyz, r0, r1
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
mul r2.xyz, v1.w, r1
mov r0, c10
dp4 r4.z, c13, r0
mov r0, c9
dp4 r4.y, c13, r0
mov r1, c8
dp4 r4.x, c13, r1
dp3 r0.y, r2, c4
dp3 r0.w, -r3, c4
dp3 r0.x, v1, c4
dp3 r0.z, v2, c4
mul o2, r0, c29.w
dp3 r0.y, r2, c5
dp3 r0.w, -r3, c5
dp3 r0.x, v1, c5
dp3 r0.z, v2, c5
mul o3, r0, c29.w
dp3 r0.y, r2, c6
dp3 r0.w, -r3, c6
dp3 r0.x, v1, c6
dp3 r0.z, v2, c6
dp3 o5.y, r2, r4
dp3 o7.y, r2, r3
mul o4, r0, c29.w
dp3 o5.z, v2, r4
dp3 o5.x, v1, r4
dp3 o7.z, v2, r3
dp3 o7.x, v1, r3
mad o1.xy, v3, c30, c30.zwzw
dp4 o0.w, v0, c3
dp4 o0.z, v0, c2
dp4 o0.y, v0, c1
dp4 o0.x, v0, c0
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
aabaaaajaaaagaakaaaadaalaacafaamaaaadafaaaabpbfbaaacpcfcaaadpdfd
aaaehefeaaahhfffaaaihgfgaaaabadgaaaabaeiaaaabaejaaaabaekaaaaaada
aaaaaadbaaaabadcaaaabagcaaaaaaddaaaaaadeaaaabadfaaaaaaaaaaaaaaaa
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
kibhadaeaelbgcmaibakbgbgmiahaaacaablmaleklaebfacmiabiaaeaaloloaa
paamabaamiaciaaeaagdloaapaaaamaamiaeiaaeaaloloaapaamafaamiabiaag
aaloloaapaakabaamiaciaagaagdloaapaaaakaamiaeiaagaaloloaapaakafaa
miadiaaaaabjlabkiladbobokicpadafaegmaaiaiaacacbgmiahaaamaegmlole
klakbfaemiadaaaaaalblclaklaabfadaibpadakaelbaagmkaacadagaibpanac
aemgaaggkaacaeagaicbanaeaadoanmbgpakagagaiecanaeaadoanlbgpalagag
aiieanaeaadoanlmgpamagagaicbadabaakhkhmgkpananagbeacaaabaakhkhgm
kpanaoaaaeciahahaagmgmmgoaamaladbeaeaaabaakhkhlbkpanapaaaeciaiai
aamglbmgoaamalaabeapaaaaaapipilbobacacamaeipajacaapilbmgobacagal
miapaaaaaajejepiolakakaamiapaaacaajemgpiolakagacmiapiaabaaaablaa
kbajbnaamiapiaacaaaablaakbaibnaamiapiaadaaaablaakbahbnaamiapaaac
aajegmaaolafagacmiapaaaaaaaaaapiolafafaageihababaalologboaaeabad
miahaaabaabllemnklabbaabmiapaaaeaapipigmilaaafppfibaaaaaaaaaaagm
ocaaaaiaficaaaaaaaaaaalbocaaaaiafieaaaaaaaaaaamgocaaaaiafiiaaaaa
aaaaaablocaaaaiamiapaaaaaapiaaaaobacaaaaemipaaadaapilbmgkcaappae
emecacaaaamgblgmobadaaaeemciacacaagmmgblobadacaeembbaaacaabllblb
obadacaemiaeaaaaaalbgmaaobadaaaakibhacaeaalmmaecibacaiajkiciacae
aamgblicmbaeadajkieoacafaabgpmmaibacagajbeahaaaaaabbmalbkbaaahaf
ambiafaaaamgmggmobaaadadbeahaaaaaabebamgoaafaaacamihacaaaamabalb
oaaaaeadmiahaaaaaamabaaaoaaaacaamiahiaafaalemaaaoaabaaaaaaaaaaaa
aaaaaaaaaaaaaaaa"
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
00049c6c011c200c06bfc0e30041dffc401f9c6c0140020c01060a4360405fac
401f9c6c01400e0c01060a4360411fac00019c6c0080007f8086c7436041fffc
00039c6c0080000d8e86c7436041fffc00009c6c0080007f80bfc04360409ffc
00021c6c0040007f8086c08360409ffc00021c6c0040002a8086c08360405ffc
00019c6c010000000886c64361a1fffc00031c6c0100000d8c86c64363a1fffc
00039c6c00800e7f810608436041dffc401f9c6c0140020c0106094360405fb4
401f9c6c01400e0c0106094360411fb400001c6c0150608c128600c360403ffc
00009c6c0150508c128600c360403ffc00011c6c0150408c128600c360403ffc
00041c6c019c700c0886c0c360405ffc00041c6c019c800c0886c0c360409ffc
00041c6c019c900c0886c0c360411ffc00009c6c010000000880046aa0a09ffc
00021c6c0080000d089a04436041fffc401f9c6c0140000c0e860a4360409fac
00019c6c0100002a8086c54361a1fffc00029c6c0100000d8a86c5436321fffc
401f9c6c0140000c0e86094360409fb400031c6c01dc400d8886c0c360405ffc
00031c6c01dc500d8886c0c360409ffc00031c6c01dc600d8886c0c360411ffc
00021c6c00c0000c1086c0830321dffc00031c6c009c302a828600c36041dffc
00041c6c00c0000c0c86c0830221dffc00001c6c2150600c0e8600c002b0827c
00011c6c2150400c0e8600caa2a8827c00031c6c209ce00d8a86c0d542a5e27c
00031c6c00dc002a8186c0836321fffc00009c6c2150500c0e8600dfe2a2827c
401f9c6c109c200d84bfc0c00331e2a0401f9c6c109c200d82bfc0caa329e2a4
401f9c6c109c200d80bfc0d54325e2a800001c6c1080000d8686c45fe323e2fc
00001c6c029c000d808000c36041fffc00001c6c0080000d8086c5436041fffc
00009c6c009cc02a808600c36041dffc00009c6c011cd000008600c300a1dffc
00001c6c011cb055008600c300a1dffc00001c6c011ca07f808600c30021dffc
401f9c6c00c0000c1086c0830021dfb1
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
// 90 instructions, 7 temp regs, 0 temp arrays:
// ALU 49 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0
eefiecedignpdhdnafnjcilhejllamfmeghcbfliabaaaaaadiaoaaaaadaaaaaa
cmaaaaaapeaaaaaanmabaaaaejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaa
abaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaljaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfc
enebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheooaaaaaaaaiaaaaaa
aiaaaaaamiaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaneaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaabaaaaaaadamaaaaneaaaaaaabaaaaaaaaaaaaaa
adaaaaaaacaaaaaaapaaaaaaneaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaa
apaaaaaaneaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaneaaaaaa
aeaaaaaaaaaaaaaaadaaaaaaafaaaaaaahaiaaaaneaaaaaaafaaaaaaaaaaaaaa
adaaaaaaagaaaaaaahaiaaaaneaaaaaaagaaaaaaaaaaaaaaadaaaaaaahaaaaaa
ahaiaaaafdfgfpfaepfdejfeejepeoaafeeffiedepepfceeaaklklklfdeieefc
feamaaaaeaaaabaabfadaaaafjaaaaaeegiocaaaaaaaaaaaahaaaaaafjaaaaae
egiocaaaabaaaaaaafaaaaaafjaaaaaeegiocaaaacaaaaaabjaaaaaafjaaaaae
egiocaaaadaaaaaabfaaaaaafpaaaaadpcbabaaaaaaaaaaafpaaaaadpcbabaaa
abaaaaaafpaaaaadhcbabaaaacaaaaaafpaaaaaddcbabaaaadaaaaaaghaaaaae
pccabaaaaaaaaaaaabaaaaaagfaaaaaddccabaaaabaaaaaagfaaaaadpccabaaa
acaaaaaagfaaaaadpccabaaaadaaaaaagfaaaaadpccabaaaaeaaaaaagfaaaaad
hccabaaaafaaaaaagfaaaaadhccabaaaagaaaaaagfaaaaadhccabaaaahaaaaaa
giaaaaacahaaaaaadiaaaaaipcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaa
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
adaaaaaabeaaaaaaegbcbaiaebaaaaaaaaaaaaaadiaaaaajhcaabaaaabaaaaaa
fgafbaiaebaaaaaaaaaaaaaaegiccaaaadaaaaaaanaaaaaadcaaaaalhcaabaaa
abaaaaaaegiccaaaadaaaaaaamaaaaaaagaabaiaebaaaaaaaaaaaaaaegacbaaa
abaaaaaadcaaaaallcaabaaaabaaaaaaegiicaaaadaaaaaaaoaaaaaakgakbaia
ebaaaaaaaaaaaaaaegaibaaaabaaaaaadgaaaaaficaabaaaacaaaaaaakaabaaa
abaaaaaadiaaaaahhcaabaaaadaaaaaajgbebaaaabaaaaaacgbjbaaaacaaaaaa
dcaaaaakhcaabaaaadaaaaaajgbebaaaacaaaaaacgbjbaaaabaaaaaaegacbaia
ebaaaaaaadaaaaaadiaaaaahhcaabaaaadaaaaaaegacbaaaadaaaaaapgbpbaaa
abaaaaaadgaaaaagbcaabaaaaeaaaaaaakiacaaaadaaaaaaamaaaaaadgaaaaag
ccaabaaaaeaaaaaaakiacaaaadaaaaaaanaaaaaadgaaaaagecaabaaaaeaaaaaa
akiacaaaadaaaaaaaoaaaaaabaaaaaahccaabaaaacaaaaaaegacbaaaadaaaaaa
egacbaaaaeaaaaaabaaaaaahbcaabaaaacaaaaaaegbcbaaaabaaaaaaegacbaaa
aeaaaaaabaaaaaahecaabaaaacaaaaaaegbcbaaaacaaaaaaegacbaaaaeaaaaaa
diaaaaaipccabaaaacaaaaaaegaobaaaacaaaaaapgipcaaaadaaaaaabeaaaaaa
dgaaaaaficaabaaaacaaaaaabkaabaaaabaaaaaadgaaaaagbcaabaaaaeaaaaaa
bkiacaaaadaaaaaaamaaaaaadgaaaaagccaabaaaaeaaaaaabkiacaaaadaaaaaa
anaaaaaadgaaaaagecaabaaaaeaaaaaabkiacaaaadaaaaaaaoaaaaaabaaaaaah
ccaabaaaacaaaaaaegacbaaaadaaaaaaegacbaaaaeaaaaaabaaaaaahbcaabaaa
acaaaaaaegbcbaaaabaaaaaaegacbaaaaeaaaaaabaaaaaahecaabaaaacaaaaaa
egbcbaaaacaaaaaaegacbaaaaeaaaaaadiaaaaaipccabaaaadaaaaaaegaobaaa
acaaaaaapgipcaaaadaaaaaabeaaaaaadgaaaaagbcaabaaaacaaaaaackiacaaa
adaaaaaaamaaaaaadgaaaaagccaabaaaacaaaaaackiacaaaadaaaaaaanaaaaaa
dgaaaaagecaabaaaacaaaaaackiacaaaadaaaaaaaoaaaaaabaaaaaahccaabaaa
abaaaaaaegacbaaaadaaaaaaegacbaaaacaaaaaabaaaaaahbcaabaaaabaaaaaa
egbcbaaaabaaaaaaegacbaaaacaaaaaabaaaaaahecaabaaaabaaaaaaegbcbaaa
acaaaaaaegacbaaaacaaaaaadiaaaaaipccabaaaaeaaaaaaegaobaaaabaaaaaa
pgipcaaaadaaaaaabeaaaaaadiaaaaajhcaabaaaabaaaaaafgifcaaaacaaaaaa
aaaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaa
adaaaaaabaaaaaaaagiacaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaadcaaaaal
hcaabaaaabaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaaacaaaaaaaaaaaaaa
egacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabdaaaaaa
pgipcaaaacaaaaaaaaaaaaaaegacbaaaabaaaaaabaaaaaahcccabaaaafaaaaaa
egacbaaaadaaaaaaegacbaaaabaaaaaabaaaaaahcccabaaaahaaaaaaegacbaaa
adaaaaaaegacbaaaaaaaaaaabaaaaaahbccabaaaafaaaaaaegbcbaaaabaaaaaa
egacbaaaabaaaaaabaaaaaaheccabaaaafaaaaaaegbcbaaaacaaaaaaegacbaaa
abaaaaaadgaaaaaficaabaaaabaaaaaaabeaaaaaaaaaiadpdiaaaaaihcaabaaa
acaaaaaaegbcbaaaacaaaaaapgipcaaaadaaaaaabeaaaaaadiaaaaaihcaabaaa
adaaaaaafgafbaaaacaaaaaaegiccaaaadaaaaaaanaaaaaadcaaaaaklcaabaaa
acaaaaaaegiicaaaadaaaaaaamaaaaaaagaabaaaacaaaaaaegaibaaaadaaaaaa
dcaaaaakhcaabaaaabaaaaaaegiccaaaadaaaaaaaoaaaaaakgakbaaaacaaaaaa
egadbaaaacaaaaaabbaaaaaibcaabaaaacaaaaaaegiocaaaacaaaaaabcaaaaaa
egaobaaaabaaaaaabbaaaaaiccaabaaaacaaaaaaegiocaaaacaaaaaabdaaaaaa
egaobaaaabaaaaaabbaaaaaiecaabaaaacaaaaaaegiocaaaacaaaaaabeaaaaaa
egaobaaaabaaaaaadiaaaaahpcaabaaaadaaaaaajgacbaaaabaaaaaaegakbaaa
abaaaaaabbaaaaaibcaabaaaaeaaaaaaegiocaaaacaaaaaabfaaaaaaegaobaaa
adaaaaaabbaaaaaiccaabaaaaeaaaaaaegiocaaaacaaaaaabgaaaaaaegaobaaa
adaaaaaabbaaaaaiecaabaaaaeaaaaaaegiocaaaacaaaaaabhaaaaaaegaobaaa
adaaaaaaaaaaaaahhcaabaaaacaaaaaaegacbaaaacaaaaaaegacbaaaaeaaaaaa
diaaaaahicaabaaaaaaaaaaabkaabaaaabaaaaaabkaabaaaabaaaaaadcaaaaak
icaabaaaaaaaaaaaakaabaaaabaaaaaaakaabaaaabaaaaaadkaabaiaebaaaaaa
aaaaaaaadcaaaaakhcaabaaaacaaaaaaegiccaaaacaaaaaabiaaaaaapgapbaaa
aaaaaaaaegacbaaaacaaaaaadiaaaaaihcaabaaaadaaaaaafgbfbaaaaaaaaaaa
egiccaaaadaaaaaaanaaaaaadcaaaaakhcaabaaaadaaaaaaegiccaaaadaaaaaa
amaaaaaaagbabaaaaaaaaaaaegacbaaaadaaaaaadcaaaaakhcaabaaaadaaaaaa
egiccaaaadaaaaaaaoaaaaaakgbkbaaaaaaaaaaaegacbaaaadaaaaaadcaaaaak
hcaabaaaadaaaaaaegiccaaaadaaaaaaapaaaaaapgbpbaaaaaaaaaaaegacbaaa
adaaaaaaaaaaaaajpcaabaaaaeaaaaaafgafbaiaebaaaaaaadaaaaaaegiocaaa
acaaaaaaadaaaaaadiaaaaahpcaabaaaafaaaaaafgafbaaaabaaaaaaegaobaaa
aeaaaaaadiaaaaahpcaabaaaaeaaaaaaegaobaaaaeaaaaaaegaobaaaaeaaaaaa
aaaaaaajpcaabaaaagaaaaaaagaabaiaebaaaaaaadaaaaaaegiocaaaacaaaaaa
acaaaaaaaaaaaaajpcaabaaaadaaaaaakgakbaiaebaaaaaaadaaaaaaegiocaaa
acaaaaaaaeaaaaaadcaaaaajpcaabaaaafaaaaaaegaobaaaagaaaaaaagaabaaa
abaaaaaaegaobaaaafaaaaaadcaaaaajpcaabaaaabaaaaaaegaobaaaadaaaaaa
kgakbaaaabaaaaaaegaobaaaafaaaaaadcaaaaajpcaabaaaaeaaaaaaegaobaaa
agaaaaaaegaobaaaagaaaaaaegaobaaaaeaaaaaadcaaaaajpcaabaaaadaaaaaa
egaobaaaadaaaaaaegaobaaaadaaaaaaegaobaaaaeaaaaaaeeaaaaafpcaabaaa
aeaaaaaaegaobaaaadaaaaaadcaaaaanpcaabaaaadaaaaaaegaobaaaadaaaaaa
egiocaaaacaaaaaaafaaaaaaaceaaaaaaaaaiadpaaaaiadpaaaaiadpaaaaiadp
aoaaaaakpcaabaaaadaaaaaaaceaaaaaaaaaiadpaaaaiadpaaaaiadpaaaaiadp
egaobaaaadaaaaaadiaaaaahpcaabaaaabaaaaaaegaobaaaabaaaaaaegaobaaa
aeaaaaaadeaaaaakpcaabaaaabaaaaaaegaobaaaabaaaaaaaceaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaadiaaaaahpcaabaaaabaaaaaaegaobaaaadaaaaaa
egaobaaaabaaaaaadiaaaaaihcaabaaaadaaaaaafgafbaaaabaaaaaaegiccaaa
acaaaaaaahaaaaaadcaaaaakhcaabaaaadaaaaaaegiccaaaacaaaaaaagaaaaaa
agaabaaaabaaaaaaegacbaaaadaaaaaadcaaaaakhcaabaaaabaaaaaaegiccaaa
acaaaaaaaiaaaaaakgakbaaaabaaaaaaegacbaaaadaaaaaadcaaaaakhcaabaaa
abaaaaaaegiccaaaacaaaaaaajaaaaaapgapbaaaabaaaaaaegacbaaaabaaaaaa
aaaaaaahhccabaaaagaaaaaaegacbaaaabaaaaaaegacbaaaacaaaaaabaaaaaah
bccabaaaahaaaaaaegbcbaaaabaaaaaaegacbaaaaaaaaaaabaaaaaaheccabaaa
ahaaaaaaegbcbaaaacaaaaaaegacbaaaaaaaaaaadoaaaaab"
}

SubProgram "gles " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" "VERTEXLIGHT_ON" }
"!!GLES
#define SHADER_API_GLES 1
#define tex2D texture2D


#ifdef VERTEX
#define gl_ModelViewProjectionMatrix glstate_matrix_mvp
uniform mat4 glstate_matrix_mvp;

varying highp vec3 xlv_TEXCOORD6;
varying lowp vec3 xlv_TEXCOORD5;
varying lowp vec3 xlv_TEXCOORD4;
varying lowp vec4 xlv_TEXCOORD3;
varying lowp vec4 xlv_TEXCOORD2;
varying lowp vec4 xlv_TEXCOORD1;
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
  xlv_TEXCOORD1 = tmpvar_4;
  xlv_TEXCOORD2 = tmpvar_5;
  xlv_TEXCOORD3 = tmpvar_6;
  xlv_TEXCOORD4 = tmpvar_7;
  xlv_TEXCOORD5 = tmpvar_8;
  xlv_TEXCOORD6 = (tmpvar_14 * (((_World2Object * tmpvar_27).xyz * unity_Scale.w) - _glesVertex.xyz));
}



#endif
#ifdef FRAGMENT

varying highp vec3 xlv_TEXCOORD6;
varying lowp vec3 xlv_TEXCOORD5;
varying lowp vec3 xlv_TEXCOORD4;
varying lowp vec4 xlv_TEXCOORD3;
varying lowp vec4 xlv_TEXCOORD2;
varying lowp vec4 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
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
  tmpvar_6.x = xlv_TEXCOORD1.w;
  tmpvar_6.y = xlv_TEXCOORD2.w;
  tmpvar_6.z = xlv_TEXCOORD3.w;
  tmpvar_2 = tmpvar_6;
  lowp vec3 tmpvar_7;
  tmpvar_7 = xlv_TEXCOORD1.xyz;
  tmpvar_3 = tmpvar_7;
  lowp vec3 tmpvar_8;
  tmpvar_8 = xlv_TEXCOORD2.xyz;
  tmpvar_4 = tmpvar_8;
  lowp vec3 tmpvar_9;
  tmpvar_9 = xlv_TEXCOORD3.xyz;
  tmpvar_5 = tmpvar_9;
  mediump vec3 tmpvar_10;
  mediump vec3 tmpvar_11;
  mediump float tmpvar_12;
  mediump vec4 spec_13;
  lowp vec4 tmpvar_14;
  tmpvar_14 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_15;
  tmpvar_15 = (tmpvar_14.xyz * _Color.xyz);
  tmpvar_10 = tmpvar_15;
  lowp vec4 tmpvar_16;
  tmpvar_16 = texture2D (_SpecMap, xlv_TEXCOORD0);
  spec_13 = tmpvar_16;
  mediump vec3 tmpvar_17;
  tmpvar_17 = (spec_13.xyz * _Gloss);
  lowp vec3 tmpvar_18;
  tmpvar_18 = ((texture2D (_BumpMap, xlv_TEXCOORD0).xyz * 2.0) - 1.0);
  tmpvar_11 = tmpvar_18;
  mediump vec3 tmpvar_19;
  tmpvar_19.x = dot (tmpvar_3, tmpvar_11);
  tmpvar_19.y = dot (tmpvar_4, tmpvar_11);
  tmpvar_19.z = dot (tmpvar_5, tmpvar_11);
  highp vec3 tmpvar_20;
  tmpvar_20 = (tmpvar_2 - (2.0 * (dot (tmpvar_19, tmpvar_2) * tmpvar_19)));
  lowp vec4 tmpvar_21;
  tmpvar_21 = (textureCube (_Cube, tmpvar_20) * tmpvar_14.w);
  lowp float tmpvar_22;
  tmpvar_22 = (tmpvar_21.w * _ReflectColor.w);
  tmpvar_12 = tmpvar_22;
  highp vec3 tmpvar_23;
  tmpvar_23 = normalize(xlv_TEXCOORD6);
  mediump vec3 lightDir_24;
  lightDir_24 = xlv_TEXCOORD4;
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
  tmpvar_31 = (pow (nh_28, arg1_30) * tmpvar_17);
  specCol_27 = tmpvar_31;
  c_26.xyz = ((((tmpvar_10 * _LightColor0.xyz) * max (0.0, dot (tmpvar_11, lightDir_24))) + (_LightColor0.xyz * specCol_27)) * 2.0);
  c_26.w = tmpvar_12;
  c_1 = c_26;
  mediump vec3 tmpvar_32;
  tmpvar_32 = (c_1.xyz + (tmpvar_10 * xlv_TEXCOORD5));
  c_1.xyz = tmpvar_32;
  mediump vec3 tmpvar_33;
  tmpvar_33 = (c_1.xyz + ((tmpvar_21.xyz * _ReflectColor.xyz) * _ReflectPower));
  c_1.xyz = tmpvar_33;
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

varying highp vec3 xlv_TEXCOORD6;
varying lowp vec3 xlv_TEXCOORD5;
varying lowp vec3 xlv_TEXCOORD4;
varying lowp vec4 xlv_TEXCOORD3;
varying lowp vec4 xlv_TEXCOORD2;
varying lowp vec4 xlv_TEXCOORD1;
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
  xlv_TEXCOORD1 = tmpvar_4;
  xlv_TEXCOORD2 = tmpvar_5;
  xlv_TEXCOORD3 = tmpvar_6;
  xlv_TEXCOORD4 = tmpvar_7;
  xlv_TEXCOORD5 = tmpvar_8;
  xlv_TEXCOORD6 = (tmpvar_14 * (((_World2Object * tmpvar_27).xyz * unity_Scale.w) - _glesVertex.xyz));
}



#endif
#ifdef FRAGMENT

varying highp vec3 xlv_TEXCOORD6;
varying lowp vec3 xlv_TEXCOORD5;
varying lowp vec3 xlv_TEXCOORD4;
varying lowp vec4 xlv_TEXCOORD3;
varying lowp vec4 xlv_TEXCOORD2;
varying lowp vec4 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
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
  tmpvar_6.x = xlv_TEXCOORD1.w;
  tmpvar_6.y = xlv_TEXCOORD2.w;
  tmpvar_6.z = xlv_TEXCOORD3.w;
  tmpvar_2 = tmpvar_6;
  lowp vec3 tmpvar_7;
  tmpvar_7 = xlv_TEXCOORD1.xyz;
  tmpvar_3 = tmpvar_7;
  lowp vec3 tmpvar_8;
  tmpvar_8 = xlv_TEXCOORD2.xyz;
  tmpvar_4 = tmpvar_8;
  lowp vec3 tmpvar_9;
  tmpvar_9 = xlv_TEXCOORD3.xyz;
  tmpvar_5 = tmpvar_9;
  mediump vec3 tmpvar_10;
  mediump vec3 tmpvar_11;
  mediump float tmpvar_12;
  mediump vec4 spec_13;
  lowp vec4 tmpvar_14;
  tmpvar_14 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_15;
  tmpvar_15 = (tmpvar_14.xyz * _Color.xyz);
  tmpvar_10 = tmpvar_15;
  lowp vec4 tmpvar_16;
  tmpvar_16 = texture2D (_SpecMap, xlv_TEXCOORD0);
  spec_13 = tmpvar_16;
  mediump vec3 tmpvar_17;
  tmpvar_17 = (spec_13.xyz * _Gloss);
  lowp vec3 normal_18;
  normal_18.xy = ((texture2D (_BumpMap, xlv_TEXCOORD0).wy * 2.0) - 1.0);
  normal_18.z = sqrt(((1.0 - (normal_18.x * normal_18.x)) - (normal_18.y * normal_18.y)));
  tmpvar_11 = normal_18;
  mediump vec3 tmpvar_19;
  tmpvar_19.x = dot (tmpvar_3, tmpvar_11);
  tmpvar_19.y = dot (tmpvar_4, tmpvar_11);
  tmpvar_19.z = dot (tmpvar_5, tmpvar_11);
  highp vec3 tmpvar_20;
  tmpvar_20 = (tmpvar_2 - (2.0 * (dot (tmpvar_19, tmpvar_2) * tmpvar_19)));
  lowp vec4 tmpvar_21;
  tmpvar_21 = (textureCube (_Cube, tmpvar_20) * tmpvar_14.w);
  lowp float tmpvar_22;
  tmpvar_22 = (tmpvar_21.w * _ReflectColor.w);
  tmpvar_12 = tmpvar_22;
  highp vec3 tmpvar_23;
  tmpvar_23 = normalize(xlv_TEXCOORD6);
  mediump vec3 lightDir_24;
  lightDir_24 = xlv_TEXCOORD4;
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
  tmpvar_31 = (pow (nh_28, arg1_30) * tmpvar_17);
  specCol_27 = tmpvar_31;
  c_26.xyz = ((((tmpvar_10 * _LightColor0.xyz) * max (0.0, dot (tmpvar_11, lightDir_24))) + (_LightColor0.xyz * specCol_27)) * 2.0);
  c_26.w = tmpvar_12;
  c_1 = c_26;
  mediump vec3 tmpvar_32;
  tmpvar_32 = (c_1.xyz + (tmpvar_10 * xlv_TEXCOORD5));
  c_1.xyz = tmpvar_32;
  mediump vec3 tmpvar_33;
  tmpvar_33 = (c_1.xyz + ((tmpvar_21.xyz * _ReflectColor.xyz) * _ReflectPower));
  c_1.xyz = tmpvar_33;
  gl_FragData[0] = c_1;
}



#endif"
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
"3.0-!!ARBvp1.0
# 94 ALU
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
DP4 R2.z, R4, c[26];
DP4 R2.y, R4, c[25];
DP4 R2.x, R4, c[24];
ADD R2.xyz, R2, R3;
ADD R0.xyz, R2, R0;
ADD result.texcoord[5].xyz, R0, R1;
MOV R1.xyz, c[13];
DP4 R2.z, R1, c[11];
DP4 R2.y, R1, c[10];
DP4 R2.x, R1, c[9];
MAD R3.xyz, R2, c[31].w, -vertex.position;
MOV R0.xyz, vertex.attrib[14];
MUL R1.xyz, vertex.normal.zxyw, R0.yzxw;
MAD R1.xyz, vertex.normal.yzxw, R0.zxyw, -R1;
MUL R2.xyz, vertex.attrib[14].w, R1;
MOV R0, c[15];
DP4 R1.z, R0, c[11];
DP4 R1.x, R0, c[9];
DP4 R1.y, R0, c[10];
DP3 R0.y, R2, c[5];
DP3 R0.w, -R3, c[5];
DP3 R0.x, vertex.attrib[14], c[5];
DP3 R0.z, vertex.normal, c[5];
MUL result.texcoord[1], R0, c[31].w;
DP3 R0.y, R2, c[6];
DP3 R0.w, -R3, c[6];
DP3 R0.x, vertex.attrib[14], c[6];
DP3 R0.z, vertex.normal, c[6];
MUL result.texcoord[2], R0, c[31].w;
DP3 R0.y, R2, c[7];
DP3 R0.w, -R3, c[7];
DP3 R0.x, vertex.attrib[14], c[7];
DP3 R0.z, vertex.normal, c[7];
MUL result.texcoord[3], R0, c[31].w;
DP4 R0.w, vertex.position, c[4];
DP4 R0.z, vertex.position, c[3];
DP4 R0.x, vertex.position, c[1];
DP4 R0.y, vertex.position, c[2];
DP3 result.texcoord[4].y, R2, R1;
DP3 result.texcoord[4].z, vertex.normal, R1;
DP3 result.texcoord[4].x, vertex.attrib[14], R1;
MUL R1.xyz, R0.xyww, c[0].z;
MUL R1.y, R1, c[14].x;
DP3 result.texcoord[6].y, R2, R3;
DP3 result.texcoord[6].z, vertex.normal, R3;
DP3 result.texcoord[6].x, vertex.attrib[14], R3;
ADD result.texcoord[7].xy, R1, R1.z;
MOV result.position, R0;
MOV result.texcoord[7].zw, R0;
MAD result.texcoord[0].xy, vertex.texcoord[0], c[32], c[32].zwzw;
END
# 94 instructions, 5 R-regs
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
"vs_3_0
; 97 ALU
dcl_position o0
dcl_texcoord0 o1
dcl_texcoord1 o2
dcl_texcoord2 o3
dcl_texcoord3 o4
dcl_texcoord4 o5
dcl_texcoord5 o6
dcl_texcoord6 o7
dcl_texcoord7 o8
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
add o6.xyz, r0, r1
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
mul r2.xyz, v1.w, r1
mov r0, c10
dp4 r4.z, c15, r0
mov r0, c9
dp4 r4.y, c15, r0
mov r1, c8
dp4 r4.x, c15, r1
dp3 r0.y, r2, c4
dp3 r0.w, -r3, c4
dp3 r0.x, v1, c4
dp3 r0.z, v2, c4
mul o2, r0, c31.w
dp3 r0.y, r2, c5
dp3 r0.w, -r3, c5
dp3 r0.x, v1, c5
dp3 r0.z, v2, c5
mul o3, r0, c31.w
dp3 r0.y, r2, c6
dp3 r0.w, -r3, c6
dp3 r0.x, v1, c6
dp3 r0.z, v2, c6
mul o4, r0, c31.w
dp4 r0.w, v0, c3
dp4 r0.z, v0, c2
dp4 r0.x, v0, c0
dp4 r0.y, v0, c1
mul r1.xyz, r0.xyww, c33.z
mul r1.y, r1, c13.x
dp3 o5.y, r2, r4
dp3 o7.y, r2, r3
dp3 o5.z, v2, r4
dp3 o5.x, v1, r4
dp3 o7.z, v2, r3
dp3 o7.x, v1, r3
mad o8.xy, r1.z, c14.zwzw, r1
mov o0, r0
mov o8.zw, r0
mad o1.xy, v3, c32, c32.zwzw
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
// ALU: 120.00 (90 instructions), vertex: 32, texture: 0,
//   sequencer: 38,  19 GPRs, 9 threads,
// Performance (if enough threads): ~120 cycles per vector
// * Vertex cycle estimates are assuming 3 vfetch_minis for every vfetch_full,
//     with <= 32 bytes per vfetch_full group.
// * Warning: high GPR count may result in poorer than estimated performance.

"vs_360
backbbabaaaaaecmaaaaafcmaaaaaaaaaaaaaaceaaaaadhiaaaaadkaaaaaaaaa
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
aaaaaaeaaaaaaeomaahbaabcaaaaaaaaaaaaaaaaaaaagnaiaaaaaaabaaaaaaae
aaaaaaanaaaaacjaaabaaaakaaaagaalaaaadaamaacafaanaaaadafaaaabpbfb
aaacpcfcaaadpdfdaaaehefeaaahhfffaaaihgfgaaalphfhaaaabadpaaaabaen
aaaabaeoaaaabaepaaaaaadjaaaaaadkaaaabadlaaaabaghaaaaaadmaaaaaadn
aaaabadoaaaaaadiaaaabaemaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaadpaaaaaa
aaaaaaaadpiaaaaaaaaaaaaapaffeaakaaaabcaamcaaaaaaaaaafaaoaaaabcaa
meaaaaaaaaaagabdgabjbcaabcaaaaaaaaaagabpgacfbcaabcaaaaaaaaaagacl
gadbbcaabcaaaaaaaaaagadhgadnbcaabcaaaaaaaaaagaedgaejbcaabcaaaaaa
aaaagaepgaffbcaabcaaaaaaaaaagaflgagbbcaabcaaaaaaaaaabaghaaaaccaa
aaaaaaaaafpifaaaaaaaaanbaaaaaaaaafpicaaaaaaaagiiaaaaaaaaafpjaaaa
aaaaaoiiaaaaaaaaafpieaaaaaaaapmiaaaaaaaakiepaoaaaamgiiedkbafbgbj
miapaaaaaalbnapiklafbfaamiapaaaaaagmdepiklafbeaamiapaaabaablnaje
klafbdaamiapiadoaananaaaocababaamiahaaakaamamgmaalbnaaboceibahao
aablgmgmkbacbhiabechaaahaaleblblcbboadackiclaoagaagfblebibbabpbi
beblaaadaalblomgkbbabiackiehadaiaalbgcmaibacbibjkiihaaaaaalogfic
obbaacbjmiahaaahaamamgleclbnadahkiehaeajaamgleeckbafbkbjmiahaaak
aalelbleclbmaaakmianaaakaapagmieclblaaakmiahaaamaalbmaleklafbjaj
miahaaanaalelbleclbmadahmiahaaahabgflomaolbaacaamialaaadaagmgcli
klbabhadmialaaaiaagmloleklacbhaibeahaaaaaamggcgmkbbabjaiaebeaiai
aagmgmmgoaadaaadbeahaaalaalbleblkbagbjaiaebeajajaalblbbloaadaaaa
beacaaakaalololbpaaoahaimiahaabcaamagmleclbladanmiahaaadaagmlele
klafbiammiahaaagaagmlemaklagbialmiahaabbabbebllpklakbpafaeblakaa
aamnblmgobahacaebeahaaamaemggcblkbbbbjadmiahaaahaabllemaklagbhag
aeehakafaelbgcmgkbbbbiaamialaaadaablgfgcklafbhadkiipadagaeblaamd
iaadaebjkibpadalaegmaamaiaadafbikicpadanaelbaaiaiaadagbimiahaaao
aegmloleklbbbhafmiadaaafaalblclaklaabhadkiedadadaamemeidmbahahbj
beepaaapaalehcgmobahahabmiamiaahaanlnlaaocababaamiabiaaeaaloloaa
pabcacaamiaciaaeaagdloaapaaabcaamiaeiaaeaaloloaapabcbaaamiabiaag
aaloloaapabbacaamiaciaagaagdloaapaaabbaamiaeiaagaaloloaapabbbaaa
miadiaaaaalalabkilaecacakibbaaaeaadoanecepamahppkmccaaaeaadoaneb
epanahppkmeeaaaeaadoanecepaoahppkiibaaabaakhkhebipapapabbeacaaab
aakhkhgmkpapbaafaeciaiaiaagmgmbloaaoamadbeaeaaabaakhkhlbkpapbbaf
aeciajajaamglbmgoaaoamadbeapaaafaapipilbobananaoaeipakacaapilbmg
obanahammiapaaafaajejepiolalalafmiapaaacaajemgpiolalahacmiadiaah
aamgbkbiklaaacaamiapiaabaaaablaakbakbpaamiapiaacaaaablaakbajbpaa
miapiaadaaaablaakbaibpaamiapaaacaajegmaaolagahacmiapaaaaaaaaaapi
olagagafgeihababaalologboaaeabadmiahaaabaabllemnklabbcabmiapaaae
aapipimgilaaahppfibaaaaaaaaaaagmocaaaaiaficaaaaaaaaaaalbocaaaaia
fieaaaaaaaaaaamgocaaaaiafiiaaaaaaaaaaablocaaaaiamiapaaaaaapiaaaa
obacaaaaemipaaadaapilbmgkcaappaeemecacaaaamgblgmobadaaaeemciacac
aagmmgblobadacaeembbaaacaabllblbobadacaemiaeaaaaaalbgmaaobadaaaa
kibhacaeaalmmaecibacakalkiciacaeaamgblicmbaeadalkieoacafaabgpmma
ibacaialbeahaaaaaabbmalbkbaaajafambiafaaaamgmggmobaaadadbeahaaaa
aabebamgoaafaaacamihacaaaamabalboaaaaeadmiahaaaaaamabaaaoaaaacaa
miahiaafaalemaaaoaabaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
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
"sce_vp_rsx // 84 instructions using 12 registers
[Configuration]
8
0000005441050c00
[Defaults]
1
447 3
000000003f8000003f000000
[Microcode]
1344
00021c6c005d100d8186c0836041fffc00041c6c00400e0c0106c0836041dffc
00029c6c005d300c0186c0836041dffc00031c6c009c120c013fc0c36041dffc
401f9c6c011c0808010400d740619f9c00001c6c01506e0c010600c360411ffc
00001c6c0150620c010600c360405ffc00009c6c01505e0c010600c360411ffc
00009c6c0150520c010600c360405ffc00011c6c01504e0c010600c360411ffc
00011c6c0150420c010600c360405ffc00019c6c01d0300d8106c0c360403ffc
00019c6c01d0200d8106c0c360405ffc00019c6c01d0100d8106c0c360409ffc
00019c6c01d0000d8106c0c360411ffc00009c6c01d0500d8106c0c360409ffc
00009c6c01d0400d8106c0c360403ffc00011c6c01d0600d8106c0c360409ffc
00051c6c01d0a00d8886c0c360405ffc00051c6c01d0900d8886c0c360409ffc
00051c6c01d0800d8886c0c360411ffc00021c6c0150400c0c8600c360411ffc
00001c6c0150600c0c8600c360409ffc00001c6c0150500c0c8600c360403ffc
00049c6c0190a00c0a86c0c360405ffc00049c6c0190900c0a86c0c360409ffc
00049c6c0190800c0a86c0c360411ffc00029c6c00dce00d8186c0aaa121fffc
00031c6c00dd000d8186c0bfe0a1fffc00039c6c00dcf00d8186c0aaa0a1fffc
00059c6c00800243011848436041dffc00041c6c010002308121886305a1dffc
401f9c6c0040000d8686c0836041ff80401f9c6c004000558686c08360407fb8
00049c6c011c100c12bfc0e30041dffc00059c6c009bf00e06aa80c36041dffc
401f9c6c0140020c01060a4360405fac401f9c6c01400e0c01060a4360411fac
00019c6c0080007f8086c7436041fffc00059c6c009d202a968000c360409ffc
00039c6c0080000d8e86c7436041fffc00009c6c0080007f80bfc04360409ffc
00021c6c0040007f8086c08360409ffc00021c6c0040002a8086c08360405ffc
00019c6c010000000886c64361a1fffc00031c6c0100000d8c86c64363a1fffc
401f9c6c00c000081686c09545a19fb800039c6c00800e7f810608436041dffc
401f9c6c0140020c0106094360405fb4401f9c6c01400e0c0106094360411fb4
00001c6c0150608c128600c360403ffc00009c6c0150508c128600c360403ffc
00011c6c0150408c128600c360403ffc00041c6c019c600c0886c0c360405ffc
00041c6c019c700c0886c0c360409ffc00041c6c019c800c0886c0c360411ffc
00009c6c010000000880046aa0a09ffc00021c6c0080000d089a04436041fffc
401f9c6c0140000c0e860a4360409fac00019c6c0100002a8086c54361a1fffc
00029c6c0100000d8a86c5436321fffc401f9c6c0140000c0e86094360409fb4
00031c6c01dc300d8886c0c360405ffc00031c6c01dc400d8886c0c360409ffc
00031c6c01dc500d8886c0c360411ffc00021c6c00c0000c1086c0830321dffc
00031c6c009c202a828600c36041dffc00041c6c00c0000c0c86c0830221dffc
00001c6c2150600c0e8600c002b0827c00011c6c2150400c0e8600caa2a8827c
00031c6c209cd00d8a86c0d542a5e27c00031c6c00dbf02a8186c0836321fffc
00009c6c2150500c0e8600dfe2a2827c401f9c6c109c100d84bfc0c00331e2a0
401f9c6c109c100d82bfc0caa329e2a4401f9c6c109c100d80bfc0d54325e2a8
00001c6c1080000d8686c45fe323e2fc00001c6c029bf00d808000c36041fffc
00001c6c0080000d8086c5436041fffc00009c6c009cb02a808600c36041dffc
00009c6c011cc000008600c300a1dffc00001c6c011ca055008600c300a1dffc
00001c6c011c907f808600c30021dffc401f9c6c00c0000c1086c0830021dfb1
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
Vector 160 [_MainTex_ST] 4
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
// 95 instructions, 8 temp regs, 0 temp arrays:
// ALU 52 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0
eefiecedldnfkpbjdjinoknjjdoaidancohopaliabaaaaaaoiaoaaaaadaaaaaa
cmaaaaaapeaaaaaapeabaaaaejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaa
abaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaljaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfc
enebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheopiaaaaaaajaaaaaa
aiaaaaaaoaaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaomaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaabaaaaaaadamaaaaomaaaaaaabaaaaaaaaaaaaaa
adaaaaaaacaaaaaaapaaaaaaomaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaa
apaaaaaaomaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaomaaaaaa
aeaaaaaaaaaaaaaaadaaaaaaafaaaaaaahaiaaaaomaaaaaaafaaaaaaaaaaaaaa
adaaaaaaagaaaaaaahaiaaaaomaaaaaaagaaaaaaaaaaaaaaadaaaaaaahaaaaaa
ahaiaaaaomaaaaaaahaaaaaaaaaaaaaaadaaaaaaaiaaaaaaapaaaaaafdfgfpfa
epfdejfeejepeoaafeeffiedepepfceeaaklklklfdeieefcomamaaaaeaaaabaa
dladaaaafjaaaaaeegiocaaaaaaaaaaaalaaaaaafjaaaaaeegiocaaaabaaaaaa
agaaaaaafjaaaaaeegiocaaaacaaaaaabjaaaaaafjaaaaaeegiocaaaadaaaaaa
bfaaaaaafpaaaaadpcbabaaaaaaaaaaafpaaaaadpcbabaaaabaaaaaafpaaaaad
hcbabaaaacaaaaaafpaaaaaddcbabaaaadaaaaaaghaaaaaepccabaaaaaaaaaaa
abaaaaaagfaaaaaddccabaaaabaaaaaagfaaaaadpccabaaaacaaaaaagfaaaaad
pccabaaaadaaaaaagfaaaaadpccabaaaaeaaaaaagfaaaaadhccabaaaafaaaaaa
gfaaaaadhccabaaaagaaaaaagfaaaaadhccabaaaahaaaaaagfaaaaadpccabaaa
aiaaaaaagiaaaaacaiaaaaaadiaaaaaipcaabaaaaaaaaaaafgbfbaaaaaaaaaaa
egiocaaaadaaaaaaabaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaa
aaaaaaaaagbabaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaa
egiocaaaadaaaaaaacaaaaaakgbkbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaak
pcaabaaaaaaaaaaaegiocaaaadaaaaaaadaaaaaapgbpbaaaaaaaaaaaegaobaaa
aaaaaaaadgaaaaafpccabaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaaldccabaaa
abaaaaaaegbabaaaadaaaaaaegiacaaaaaaaaaaaakaaaaaaogikcaaaaaaaaaaa
akaaaaaadiaaaaajhcaabaaaabaaaaaafgifcaaaabaaaaaaaeaaaaaaegiccaaa
adaaaaaabbaaaaaadcaaaaalhcaabaaaabaaaaaaegiccaaaadaaaaaabaaaaaaa
agiacaaaabaaaaaaaeaaaaaaegacbaaaabaaaaaadcaaaaalhcaabaaaabaaaaaa
egiccaaaadaaaaaabcaaaaaakgikcaaaabaaaaaaaeaaaaaaegacbaaaabaaaaaa
aaaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaaegiccaaaadaaaaaabdaaaaaa
dcaaaaalhcaabaaaabaaaaaaegacbaaaabaaaaaapgipcaaaadaaaaaabeaaaaaa
egbcbaiaebaaaaaaaaaaaaaadiaaaaajhcaabaaaacaaaaaafgafbaiaebaaaaaa
abaaaaaaegiccaaaadaaaaaaanaaaaaadcaaaaalhcaabaaaacaaaaaaegiccaaa
adaaaaaaamaaaaaaagaabaiaebaaaaaaabaaaaaaegacbaaaacaaaaaadcaaaaal
lcaabaaaacaaaaaaegiicaaaadaaaaaaaoaaaaaakgakbaiaebaaaaaaabaaaaaa
egaibaaaacaaaaaadgaaaaaficaabaaaadaaaaaaakaabaaaacaaaaaadiaaaaah
hcaabaaaaeaaaaaajgbebaaaabaaaaaacgbjbaaaacaaaaaadcaaaaakhcaabaaa
aeaaaaaajgbebaaaacaaaaaacgbjbaaaabaaaaaaegacbaiaebaaaaaaaeaaaaaa
diaaaaahhcaabaaaaeaaaaaaegacbaaaaeaaaaaapgbpbaaaabaaaaaadgaaaaag
bcaabaaaafaaaaaaakiacaaaadaaaaaaamaaaaaadgaaaaagccaabaaaafaaaaaa
akiacaaaadaaaaaaanaaaaaadgaaaaagecaabaaaafaaaaaaakiacaaaadaaaaaa
aoaaaaaabaaaaaahccaabaaaadaaaaaaegacbaaaaeaaaaaaegacbaaaafaaaaaa
baaaaaahbcaabaaaadaaaaaaegbcbaaaabaaaaaaegacbaaaafaaaaaabaaaaaah
ecaabaaaadaaaaaaegbcbaaaacaaaaaaegacbaaaafaaaaaadiaaaaaipccabaaa
acaaaaaaegaobaaaadaaaaaapgipcaaaadaaaaaabeaaaaaadgaaaaaficaabaaa
adaaaaaabkaabaaaacaaaaaadgaaaaagbcaabaaaafaaaaaabkiacaaaadaaaaaa
amaaaaaadgaaaaagccaabaaaafaaaaaabkiacaaaadaaaaaaanaaaaaadgaaaaag
ecaabaaaafaaaaaabkiacaaaadaaaaaaaoaaaaaabaaaaaahccaabaaaadaaaaaa
egacbaaaaeaaaaaaegacbaaaafaaaaaabaaaaaahbcaabaaaadaaaaaaegbcbaaa
abaaaaaaegacbaaaafaaaaaabaaaaaahecaabaaaadaaaaaaegbcbaaaacaaaaaa
egacbaaaafaaaaaadiaaaaaipccabaaaadaaaaaaegaobaaaadaaaaaapgipcaaa
adaaaaaabeaaaaaadgaaaaagbcaabaaaadaaaaaackiacaaaadaaaaaaamaaaaaa
dgaaaaagccaabaaaadaaaaaackiacaaaadaaaaaaanaaaaaadgaaaaagecaabaaa
adaaaaaackiacaaaadaaaaaaaoaaaaaabaaaaaahccaabaaaacaaaaaaegacbaaa
aeaaaaaaegacbaaaadaaaaaabaaaaaahbcaabaaaacaaaaaaegbcbaaaabaaaaaa
egacbaaaadaaaaaabaaaaaahecaabaaaacaaaaaaegbcbaaaacaaaaaaegacbaaa
adaaaaaadiaaaaaipccabaaaaeaaaaaaegaobaaaacaaaaaapgipcaaaadaaaaaa
beaaaaaadiaaaaajhcaabaaaacaaaaaafgifcaaaacaaaaaaaaaaaaaaegiccaaa
adaaaaaabbaaaaaadcaaaaalhcaabaaaacaaaaaaegiccaaaadaaaaaabaaaaaaa
agiacaaaacaaaaaaaaaaaaaaegacbaaaacaaaaaadcaaaaalhcaabaaaacaaaaaa
egiccaaaadaaaaaabcaaaaaakgikcaaaacaaaaaaaaaaaaaaegacbaaaacaaaaaa
dcaaaaalhcaabaaaacaaaaaaegiccaaaadaaaaaabdaaaaaapgipcaaaacaaaaaa
aaaaaaaaegacbaaaacaaaaaabaaaaaahcccabaaaafaaaaaaegacbaaaaeaaaaaa
egacbaaaacaaaaaabaaaaaahcccabaaaahaaaaaaegacbaaaaeaaaaaaegacbaaa
abaaaaaabaaaaaahbccabaaaafaaaaaaegbcbaaaabaaaaaaegacbaaaacaaaaaa
baaaaaaheccabaaaafaaaaaaegbcbaaaacaaaaaaegacbaaaacaaaaaadgaaaaaf
icaabaaaacaaaaaaabeaaaaaaaaaiadpdiaaaaaihcaabaaaadaaaaaaegbcbaaa
acaaaaaapgipcaaaadaaaaaabeaaaaaadiaaaaaihcaabaaaaeaaaaaafgafbaaa
adaaaaaaegiccaaaadaaaaaaanaaaaaadcaaaaaklcaabaaaadaaaaaaegiicaaa
adaaaaaaamaaaaaaagaabaaaadaaaaaaegaibaaaaeaaaaaadcaaaaakhcaabaaa
acaaaaaaegiccaaaadaaaaaaaoaaaaaakgakbaaaadaaaaaaegadbaaaadaaaaaa
bbaaaaaibcaabaaaadaaaaaaegiocaaaacaaaaaabcaaaaaaegaobaaaacaaaaaa
bbaaaaaiccaabaaaadaaaaaaegiocaaaacaaaaaabdaaaaaaegaobaaaacaaaaaa
bbaaaaaiecaabaaaadaaaaaaegiocaaaacaaaaaabeaaaaaaegaobaaaacaaaaaa
diaaaaahpcaabaaaaeaaaaaajgacbaaaacaaaaaaegakbaaaacaaaaaabbaaaaai
bcaabaaaafaaaaaaegiocaaaacaaaaaabfaaaaaaegaobaaaaeaaaaaabbaaaaai
ccaabaaaafaaaaaaegiocaaaacaaaaaabgaaaaaaegaobaaaaeaaaaaabbaaaaai
ecaabaaaafaaaaaaegiocaaaacaaaaaabhaaaaaaegaobaaaaeaaaaaaaaaaaaah
hcaabaaaadaaaaaaegacbaaaadaaaaaaegacbaaaafaaaaaadiaaaaahicaabaaa
abaaaaaabkaabaaaacaaaaaabkaabaaaacaaaaaadcaaaaakicaabaaaabaaaaaa
akaabaaaacaaaaaaakaabaaaacaaaaaadkaabaiaebaaaaaaabaaaaaadcaaaaak
hcaabaaaadaaaaaaegiccaaaacaaaaaabiaaaaaapgapbaaaabaaaaaaegacbaaa
adaaaaaadiaaaaaihcaabaaaaeaaaaaafgbfbaaaaaaaaaaaegiccaaaadaaaaaa
anaaaaaadcaaaaakhcaabaaaaeaaaaaaegiccaaaadaaaaaaamaaaaaaagbabaaa
aaaaaaaaegacbaaaaeaaaaaadcaaaaakhcaabaaaaeaaaaaaegiccaaaadaaaaaa
aoaaaaaakgbkbaaaaaaaaaaaegacbaaaaeaaaaaadcaaaaakhcaabaaaaeaaaaaa
egiccaaaadaaaaaaapaaaaaapgbpbaaaaaaaaaaaegacbaaaaeaaaaaaaaaaaaaj
pcaabaaaafaaaaaafgafbaiaebaaaaaaaeaaaaaaegiocaaaacaaaaaaadaaaaaa
diaaaaahpcaabaaaagaaaaaafgafbaaaacaaaaaaegaobaaaafaaaaaadiaaaaah
pcaabaaaafaaaaaaegaobaaaafaaaaaaegaobaaaafaaaaaaaaaaaaajpcaabaaa
ahaaaaaaagaabaiaebaaaaaaaeaaaaaaegiocaaaacaaaaaaacaaaaaaaaaaaaaj
pcaabaaaaeaaaaaakgakbaiaebaaaaaaaeaaaaaaegiocaaaacaaaaaaaeaaaaaa
dcaaaaajpcaabaaaagaaaaaaegaobaaaahaaaaaaagaabaaaacaaaaaaegaobaaa
agaaaaaadcaaaaajpcaabaaaacaaaaaaegaobaaaaeaaaaaakgakbaaaacaaaaaa
egaobaaaagaaaaaadcaaaaajpcaabaaaafaaaaaaegaobaaaahaaaaaaegaobaaa
ahaaaaaaegaobaaaafaaaaaadcaaaaajpcaabaaaaeaaaaaaegaobaaaaeaaaaaa
egaobaaaaeaaaaaaegaobaaaafaaaaaaeeaaaaafpcaabaaaafaaaaaaegaobaaa
aeaaaaaadcaaaaanpcaabaaaaeaaaaaaegaobaaaaeaaaaaaegiocaaaacaaaaaa
afaaaaaaaceaaaaaaaaaiadpaaaaiadpaaaaiadpaaaaiadpaoaaaaakpcaabaaa
aeaaaaaaaceaaaaaaaaaiadpaaaaiadpaaaaiadpaaaaiadpegaobaaaaeaaaaaa
diaaaaahpcaabaaaacaaaaaaegaobaaaacaaaaaaegaobaaaafaaaaaadeaaaaak
pcaabaaaacaaaaaaegaobaaaacaaaaaaaceaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaadiaaaaahpcaabaaaacaaaaaaegaobaaaaeaaaaaaegaobaaaacaaaaaa
diaaaaaihcaabaaaaeaaaaaafgafbaaaacaaaaaaegiccaaaacaaaaaaahaaaaaa
dcaaaaakhcaabaaaaeaaaaaaegiccaaaacaaaaaaagaaaaaaagaabaaaacaaaaaa
egacbaaaaeaaaaaadcaaaaakhcaabaaaacaaaaaaegiccaaaacaaaaaaaiaaaaaa
kgakbaaaacaaaaaaegacbaaaaeaaaaaadcaaaaakhcaabaaaacaaaaaaegiccaaa
acaaaaaaajaaaaaapgapbaaaacaaaaaaegacbaaaacaaaaaaaaaaaaahhccabaaa
agaaaaaaegacbaaaacaaaaaaegacbaaaadaaaaaabaaaaaahbccabaaaahaaaaaa
egbcbaaaabaaaaaaegacbaaaabaaaaaabaaaaaaheccabaaaahaaaaaaegbcbaaa
acaaaaaaegacbaaaabaaaaaadiaaaaaiccaabaaaaaaaaaaabkaabaaaaaaaaaaa
akiacaaaabaaaaaaafaaaaaadiaaaaakncaabaaaabaaaaaaagahbaaaaaaaaaaa
aceaaaaaaaaaaadpaaaaaaaaaaaaaadpaaaaaadpdgaaaaafmccabaaaaiaaaaaa
kgaobaaaaaaaaaaaaaaaaaahdccabaaaaiaaaaaakgakbaaaabaaaaaamgaabaaa
abaaaaaadoaaaaab"
}

SubProgram "gles " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" "VERTEXLIGHT_ON" }
"!!GLES
#define SHADER_API_GLES 1
#define tex2D texture2D


#ifdef VERTEX
#define gl_ModelViewProjectionMatrix glstate_matrix_mvp
uniform mat4 glstate_matrix_mvp;

varying highp vec4 xlv_TEXCOORD7;
varying highp vec3 xlv_TEXCOORD6;
varying lowp vec3 xlv_TEXCOORD5;
varying lowp vec3 xlv_TEXCOORD4;
varying lowp vec4 xlv_TEXCOORD3;
varying lowp vec4 xlv_TEXCOORD2;
varying lowp vec4 xlv_TEXCOORD1;
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
  xlv_TEXCOORD1 = tmpvar_4;
  xlv_TEXCOORD2 = tmpvar_5;
  xlv_TEXCOORD3 = tmpvar_6;
  xlv_TEXCOORD4 = tmpvar_7;
  xlv_TEXCOORD5 = tmpvar_8;
  xlv_TEXCOORD6 = (tmpvar_14 * (((_World2Object * tmpvar_27).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD7 = (unity_World2Shadow[0] * (_Object2World * _glesVertex));
}



#endif
#ifdef FRAGMENT

varying highp vec4 xlv_TEXCOORD7;
varying highp vec3 xlv_TEXCOORD6;
varying lowp vec3 xlv_TEXCOORD5;
varying lowp vec3 xlv_TEXCOORD4;
varying lowp vec4 xlv_TEXCOORD3;
varying lowp vec4 xlv_TEXCOORD2;
varying lowp vec4 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform mediump float _Gloss;
uniform mediump float _Shininess;
uniform mediump float _ReflectPower;
uniform lowp vec4 _ReflectColor;
uniform lowp vec4 _Color;
uniform samplerCube _Cube;
uniform sampler2D _SpecMap;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform sampler2D _ShadowMapTexture;
uniform lowp vec4 _LightColor0;
uniform highp vec4 _LightShadowData;
void main ()
{
  lowp vec4 c_1;
  highp vec3 tmpvar_2;
  mediump vec3 tmpvar_3;
  mediump vec3 tmpvar_4;
  mediump vec3 tmpvar_5;
  lowp vec3 tmpvar_6;
  tmpvar_6.x = xlv_TEXCOORD1.w;
  tmpvar_6.y = xlv_TEXCOORD2.w;
  tmpvar_6.z = xlv_TEXCOORD3.w;
  tmpvar_2 = tmpvar_6;
  lowp vec3 tmpvar_7;
  tmpvar_7 = xlv_TEXCOORD1.xyz;
  tmpvar_3 = tmpvar_7;
  lowp vec3 tmpvar_8;
  tmpvar_8 = xlv_TEXCOORD2.xyz;
  tmpvar_4 = tmpvar_8;
  lowp vec3 tmpvar_9;
  tmpvar_9 = xlv_TEXCOORD3.xyz;
  tmpvar_5 = tmpvar_9;
  mediump vec3 tmpvar_10;
  mediump vec3 tmpvar_11;
  mediump float tmpvar_12;
  mediump vec4 spec_13;
  lowp vec4 tmpvar_14;
  tmpvar_14 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_15;
  tmpvar_15 = (tmpvar_14.xyz * _Color.xyz);
  tmpvar_10 = tmpvar_15;
  lowp vec4 tmpvar_16;
  tmpvar_16 = texture2D (_SpecMap, xlv_TEXCOORD0);
  spec_13 = tmpvar_16;
  mediump vec3 tmpvar_17;
  tmpvar_17 = (spec_13.xyz * _Gloss);
  lowp vec3 tmpvar_18;
  tmpvar_18 = ((texture2D (_BumpMap, xlv_TEXCOORD0).xyz * 2.0) - 1.0);
  tmpvar_11 = tmpvar_18;
  mediump vec3 tmpvar_19;
  tmpvar_19.x = dot (tmpvar_3, tmpvar_11);
  tmpvar_19.y = dot (tmpvar_4, tmpvar_11);
  tmpvar_19.z = dot (tmpvar_5, tmpvar_11);
  highp vec3 tmpvar_20;
  tmpvar_20 = (tmpvar_2 - (2.0 * (dot (tmpvar_19, tmpvar_2) * tmpvar_19)));
  lowp vec4 tmpvar_21;
  tmpvar_21 = (textureCube (_Cube, tmpvar_20) * tmpvar_14.w);
  lowp float tmpvar_22;
  tmpvar_22 = (tmpvar_21.w * _ReflectColor.w);
  tmpvar_12 = tmpvar_22;
  lowp float tmpvar_23;
  mediump float lightShadowDataX_24;
  highp float dist_25;
  lowp float tmpvar_26;
  tmpvar_26 = texture2DProj (_ShadowMapTexture, xlv_TEXCOORD7).x;
  dist_25 = tmpvar_26;
  highp float tmpvar_27;
  tmpvar_27 = _LightShadowData.x;
  lightShadowDataX_24 = tmpvar_27;
  highp float tmpvar_28;
  tmpvar_28 = max (float((dist_25 > (xlv_TEXCOORD7.z / xlv_TEXCOORD7.w))), lightShadowDataX_24);
  tmpvar_23 = tmpvar_28;
  highp vec3 tmpvar_29;
  tmpvar_29 = normalize(xlv_TEXCOORD6);
  mediump vec3 lightDir_30;
  lightDir_30 = xlv_TEXCOORD4;
  mediump vec3 viewDir_31;
  viewDir_31 = tmpvar_29;
  mediump float atten_32;
  atten_32 = tmpvar_23;
  mediump vec4 c_33;
  mediump vec3 specCol_34;
  highp float nh_35;
  mediump float tmpvar_36;
  tmpvar_36 = max (0.0, dot (tmpvar_11, normalize((lightDir_30 + viewDir_31))));
  nh_35 = tmpvar_36;
  mediump float arg1_37;
  arg1_37 = (32.0 * _Shininess);
  highp vec3 tmpvar_38;
  tmpvar_38 = (pow (nh_35, arg1_37) * tmpvar_17);
  specCol_34 = tmpvar_38;
  c_33.xyz = ((((tmpvar_10 * _LightColor0.xyz) * max (0.0, dot (tmpvar_11, lightDir_30))) + (_LightColor0.xyz * specCol_34)) * (atten_32 * 2.0));
  c_33.w = tmpvar_12;
  c_1 = c_33;
  mediump vec3 tmpvar_39;
  tmpvar_39 = (c_1.xyz + (tmpvar_10 * xlv_TEXCOORD5));
  c_1.xyz = tmpvar_39;
  mediump vec3 tmpvar_40;
  tmpvar_40 = (c_1.xyz + ((tmpvar_21.xyz * _ReflectColor.xyz) * _ReflectPower));
  c_1.xyz = tmpvar_40;
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

varying highp vec4 xlv_TEXCOORD7;
varying highp vec3 xlv_TEXCOORD6;
varying lowp vec3 xlv_TEXCOORD5;
varying lowp vec3 xlv_TEXCOORD4;
varying lowp vec4 xlv_TEXCOORD3;
varying lowp vec4 xlv_TEXCOORD2;
varying lowp vec4 xlv_TEXCOORD1;
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
  lowp vec4 tmpvar_4;
  lowp vec4 tmpvar_5;
  lowp vec4 tmpvar_6;
  lowp vec3 tmpvar_7;
  lowp vec3 tmpvar_8;
  highp vec4 tmpvar_9;
  tmpvar_9 = (gl_ModelViewProjectionMatrix * _glesVertex);
  highp vec4 tmpvar_10;
  tmpvar_10.w = 1.0;
  tmpvar_10.xyz = _WorldSpaceCameraPos;
  mat3 tmpvar_11;
  tmpvar_11[0] = _Object2World[0].xyz;
  tmpvar_11[1] = _Object2World[1].xyz;
  tmpvar_11[2] = _Object2World[2].xyz;
  highp vec3 tmpvar_12;
  tmpvar_12 = (tmpvar_11 * (_glesVertex.xyz - ((_World2Object * tmpvar_10).xyz * unity_Scale.w)));
  highp vec3 tmpvar_13;
  highp vec3 tmpvar_14;
  tmpvar_13 = tmpvar_1.xyz;
  tmpvar_14 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_15;
  tmpvar_15[0].x = tmpvar_13.x;
  tmpvar_15[0].y = tmpvar_14.x;
  tmpvar_15[0].z = tmpvar_2.x;
  tmpvar_15[1].x = tmpvar_13.y;
  tmpvar_15[1].y = tmpvar_14.y;
  tmpvar_15[1].z = tmpvar_2.y;
  tmpvar_15[2].x = tmpvar_13.z;
  tmpvar_15[2].y = tmpvar_14.z;
  tmpvar_15[2].z = tmpvar_2.z;
  vec4 v_16;
  v_16.x = _Object2World[0].x;
  v_16.y = _Object2World[1].x;
  v_16.z = _Object2World[2].x;
  v_16.w = _Object2World[3].x;
  highp vec4 tmpvar_17;
  tmpvar_17.xyz = (tmpvar_15 * v_16.xyz);
  tmpvar_17.w = tmpvar_12.x;
  highp vec4 tmpvar_18;
  tmpvar_18 = (tmpvar_17 * unity_Scale.w);
  tmpvar_4 = tmpvar_18;
  vec4 v_19;
  v_19.x = _Object2World[0].y;
  v_19.y = _Object2World[1].y;
  v_19.z = _Object2World[2].y;
  v_19.w = _Object2World[3].y;
  highp vec4 tmpvar_20;
  tmpvar_20.xyz = (tmpvar_15 * v_19.xyz);
  tmpvar_20.w = tmpvar_12.y;
  highp vec4 tmpvar_21;
  tmpvar_21 = (tmpvar_20 * unity_Scale.w);
  tmpvar_5 = tmpvar_21;
  vec4 v_22;
  v_22.x = _Object2World[0].z;
  v_22.y = _Object2World[1].z;
  v_22.z = _Object2World[2].z;
  v_22.w = _Object2World[3].z;
  highp vec4 tmpvar_23;
  tmpvar_23.xyz = (tmpvar_15 * v_22.xyz);
  tmpvar_23.w = tmpvar_12.z;
  highp vec4 tmpvar_24;
  tmpvar_24 = (tmpvar_23 * unity_Scale.w);
  tmpvar_6 = tmpvar_24;
  mat3 tmpvar_25;
  tmpvar_25[0] = _Object2World[0].xyz;
  tmpvar_25[1] = _Object2World[1].xyz;
  tmpvar_25[2] = _Object2World[2].xyz;
  highp vec3 tmpvar_26;
  tmpvar_26 = (tmpvar_25 * (tmpvar_2 * unity_Scale.w));
  highp vec3 tmpvar_27;
  tmpvar_27 = (tmpvar_15 * (_World2Object * _WorldSpaceLightPos0).xyz);
  tmpvar_7 = tmpvar_27;
  highp vec4 tmpvar_28;
  tmpvar_28.w = 1.0;
  tmpvar_28.xyz = _WorldSpaceCameraPos;
  highp vec4 tmpvar_29;
  tmpvar_29.w = 1.0;
  tmpvar_29.xyz = tmpvar_26;
  mediump vec3 tmpvar_30;
  mediump vec4 normal_31;
  normal_31 = tmpvar_29;
  highp float vC_32;
  mediump vec3 x3_33;
  mediump vec3 x2_34;
  mediump vec3 x1_35;
  highp float tmpvar_36;
  tmpvar_36 = dot (unity_SHAr, normal_31);
  x1_35.x = tmpvar_36;
  highp float tmpvar_37;
  tmpvar_37 = dot (unity_SHAg, normal_31);
  x1_35.y = tmpvar_37;
  highp float tmpvar_38;
  tmpvar_38 = dot (unity_SHAb, normal_31);
  x1_35.z = tmpvar_38;
  mediump vec4 tmpvar_39;
  tmpvar_39 = (normal_31.xyzz * normal_31.yzzx);
  highp float tmpvar_40;
  tmpvar_40 = dot (unity_SHBr, tmpvar_39);
  x2_34.x = tmpvar_40;
  highp float tmpvar_41;
  tmpvar_41 = dot (unity_SHBg, tmpvar_39);
  x2_34.y = tmpvar_41;
  highp float tmpvar_42;
  tmpvar_42 = dot (unity_SHBb, tmpvar_39);
  x2_34.z = tmpvar_42;
  mediump float tmpvar_43;
  tmpvar_43 = ((normal_31.x * normal_31.x) - (normal_31.y * normal_31.y));
  vC_32 = tmpvar_43;
  highp vec3 tmpvar_44;
  tmpvar_44 = (unity_SHC.xyz * vC_32);
  x3_33 = tmpvar_44;
  tmpvar_30 = ((x1_35 + x2_34) + x3_33);
  shlight_3 = tmpvar_30;
  tmpvar_8 = shlight_3;
  highp vec3 tmpvar_45;
  tmpvar_45 = (_Object2World * _glesVertex).xyz;
  highp vec4 tmpvar_46;
  tmpvar_46 = (unity_4LightPosX0 - tmpvar_45.x);
  highp vec4 tmpvar_47;
  tmpvar_47 = (unity_4LightPosY0 - tmpvar_45.y);
  highp vec4 tmpvar_48;
  tmpvar_48 = (unity_4LightPosZ0 - tmpvar_45.z);
  highp vec4 tmpvar_49;
  tmpvar_49 = (((tmpvar_46 * tmpvar_46) + (tmpvar_47 * tmpvar_47)) + (tmpvar_48 * tmpvar_48));
  highp vec4 tmpvar_50;
  tmpvar_50 = (max (vec4(0.0, 0.0, 0.0, 0.0), ((((tmpvar_46 * tmpvar_26.x) + (tmpvar_47 * tmpvar_26.y)) + (tmpvar_48 * tmpvar_26.z)) * inversesqrt(tmpvar_49))) * (1.0/((1.0 + (tmpvar_49 * unity_4LightAtten0)))));
  highp vec3 tmpvar_51;
  tmpvar_51 = (tmpvar_8 + ((((unity_LightColor[0].xyz * tmpvar_50.x) + (unity_LightColor[1].xyz * tmpvar_50.y)) + (unity_LightColor[2].xyz * tmpvar_50.z)) + (unity_LightColor[3].xyz * tmpvar_50.w)));
  tmpvar_8 = tmpvar_51;
  highp vec4 o_52;
  highp vec4 tmpvar_53;
  tmpvar_53 = (tmpvar_9 * 0.5);
  highp vec2 tmpvar_54;
  tmpvar_54.x = tmpvar_53.x;
  tmpvar_54.y = (tmpvar_53.y * _ProjectionParams.x);
  o_52.xy = (tmpvar_54 + tmpvar_53.w);
  o_52.zw = tmpvar_9.zw;
  gl_Position = tmpvar_9;
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = tmpvar_4;
  xlv_TEXCOORD2 = tmpvar_5;
  xlv_TEXCOORD3 = tmpvar_6;
  xlv_TEXCOORD4 = tmpvar_7;
  xlv_TEXCOORD5 = tmpvar_8;
  xlv_TEXCOORD6 = (tmpvar_15 * (((_World2Object * tmpvar_28).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD7 = o_52;
}



#endif
#ifdef FRAGMENT

varying highp vec4 xlv_TEXCOORD7;
varying highp vec3 xlv_TEXCOORD6;
varying lowp vec3 xlv_TEXCOORD5;
varying lowp vec3 xlv_TEXCOORD4;
varying lowp vec4 xlv_TEXCOORD3;
varying lowp vec4 xlv_TEXCOORD2;
varying lowp vec4 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform mediump float _Gloss;
uniform mediump float _Shininess;
uniform mediump float _ReflectPower;
uniform lowp vec4 _ReflectColor;
uniform lowp vec4 _Color;
uniform samplerCube _Cube;
uniform sampler2D _SpecMap;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform sampler2D _ShadowMapTexture;
uniform lowp vec4 _LightColor0;
void main ()
{
  lowp vec4 c_1;
  highp vec3 tmpvar_2;
  mediump vec3 tmpvar_3;
  mediump vec3 tmpvar_4;
  mediump vec3 tmpvar_5;
  lowp vec3 tmpvar_6;
  tmpvar_6.x = xlv_TEXCOORD1.w;
  tmpvar_6.y = xlv_TEXCOORD2.w;
  tmpvar_6.z = xlv_TEXCOORD3.w;
  tmpvar_2 = tmpvar_6;
  lowp vec3 tmpvar_7;
  tmpvar_7 = xlv_TEXCOORD1.xyz;
  tmpvar_3 = tmpvar_7;
  lowp vec3 tmpvar_8;
  tmpvar_8 = xlv_TEXCOORD2.xyz;
  tmpvar_4 = tmpvar_8;
  lowp vec3 tmpvar_9;
  tmpvar_9 = xlv_TEXCOORD3.xyz;
  tmpvar_5 = tmpvar_9;
  mediump vec3 tmpvar_10;
  mediump vec3 tmpvar_11;
  mediump float tmpvar_12;
  mediump vec4 spec_13;
  lowp vec4 tmpvar_14;
  tmpvar_14 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_15;
  tmpvar_15 = (tmpvar_14.xyz * _Color.xyz);
  tmpvar_10 = tmpvar_15;
  lowp vec4 tmpvar_16;
  tmpvar_16 = texture2D (_SpecMap, xlv_TEXCOORD0);
  spec_13 = tmpvar_16;
  mediump vec3 tmpvar_17;
  tmpvar_17 = (spec_13.xyz * _Gloss);
  lowp vec3 normal_18;
  normal_18.xy = ((texture2D (_BumpMap, xlv_TEXCOORD0).wy * 2.0) - 1.0);
  normal_18.z = sqrt(((1.0 - (normal_18.x * normal_18.x)) - (normal_18.y * normal_18.y)));
  tmpvar_11 = normal_18;
  mediump vec3 tmpvar_19;
  tmpvar_19.x = dot (tmpvar_3, tmpvar_11);
  tmpvar_19.y = dot (tmpvar_4, tmpvar_11);
  tmpvar_19.z = dot (tmpvar_5, tmpvar_11);
  highp vec3 tmpvar_20;
  tmpvar_20 = (tmpvar_2 - (2.0 * (dot (tmpvar_19, tmpvar_2) * tmpvar_19)));
  lowp vec4 tmpvar_21;
  tmpvar_21 = (textureCube (_Cube, tmpvar_20) * tmpvar_14.w);
  lowp float tmpvar_22;
  tmpvar_22 = (tmpvar_21.w * _ReflectColor.w);
  tmpvar_12 = tmpvar_22;
  lowp float tmpvar_23;
  tmpvar_23 = texture2DProj (_ShadowMapTexture, xlv_TEXCOORD7).x;
  highp vec3 tmpvar_24;
  tmpvar_24 = normalize(xlv_TEXCOORD6);
  mediump vec3 lightDir_25;
  lightDir_25 = xlv_TEXCOORD4;
  mediump vec3 viewDir_26;
  viewDir_26 = tmpvar_24;
  mediump float atten_27;
  atten_27 = tmpvar_23;
  mediump vec4 c_28;
  mediump vec3 specCol_29;
  highp float nh_30;
  mediump float tmpvar_31;
  tmpvar_31 = max (0.0, dot (tmpvar_11, normalize((lightDir_25 + viewDir_26))));
  nh_30 = tmpvar_31;
  mediump float arg1_32;
  arg1_32 = (32.0 * _Shininess);
  highp vec3 tmpvar_33;
  tmpvar_33 = (pow (nh_30, arg1_32) * tmpvar_17);
  specCol_29 = tmpvar_33;
  c_28.xyz = ((((tmpvar_10 * _LightColor0.xyz) * max (0.0, dot (tmpvar_11, lightDir_25))) + (_LightColor0.xyz * specCol_29)) * (atten_27 * 2.0));
  c_28.w = tmpvar_12;
  c_1 = c_28;
  mediump vec3 tmpvar_34;
  tmpvar_34 = (c_1.xyz + (tmpvar_10 * xlv_TEXCOORD5));
  c_1.xyz = tmpvar_34;
  mediump vec3 tmpvar_35;
  tmpvar_35 = (c_1.xyz + ((tmpvar_21.xyz * _ReflectColor.xyz) * _ReflectPower));
  c_1.xyz = tmpvar_35;
  gl_FragData[0] = c_1;
}



#endif"
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
varying highp vec4 xlv_TEXCOORD7;
varying highp vec3 xlv_TEXCOORD6;
varying lowp vec3 xlv_TEXCOORD5;
varying lowp vec3 xlv_TEXCOORD4;
varying lowp vec4 xlv_TEXCOORD3;
varying lowp vec4 xlv_TEXCOORD2;
varying lowp vec4 xlv_TEXCOORD1;
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
  xlv_TEXCOORD1 = tmpvar_4;
  xlv_TEXCOORD2 = tmpvar_5;
  xlv_TEXCOORD3 = tmpvar_6;
  xlv_TEXCOORD4 = tmpvar_7;
  xlv_TEXCOORD5 = tmpvar_8;
  xlv_TEXCOORD6 = (tmpvar_14 * (((_World2Object * tmpvar_26).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD7 = (unity_World2Shadow[0] * (_Object2World * _glesVertex));
}



#endif
#ifdef FRAGMENT

#extension GL_EXT_shadow_samplers : enable
varying highp vec4 xlv_TEXCOORD7;
varying highp vec3 xlv_TEXCOORD6;
varying lowp vec3 xlv_TEXCOORD5;
varying lowp vec3 xlv_TEXCOORD4;
varying lowp vec4 xlv_TEXCOORD3;
varying lowp vec4 xlv_TEXCOORD2;
varying lowp vec4 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform mediump float _Gloss;
uniform mediump float _Shininess;
uniform mediump float _ReflectPower;
uniform lowp vec4 _ReflectColor;
uniform lowp vec4 _Color;
uniform samplerCube _Cube;
uniform sampler2D _SpecMap;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform sampler2DShadow _ShadowMapTexture;
uniform lowp vec4 _LightColor0;
uniform highp vec4 _LightShadowData;
void main ()
{
  lowp vec4 c_1;
  highp vec3 tmpvar_2;
  mediump vec3 tmpvar_3;
  mediump vec3 tmpvar_4;
  mediump vec3 tmpvar_5;
  lowp vec3 tmpvar_6;
  tmpvar_6.x = xlv_TEXCOORD1.w;
  tmpvar_6.y = xlv_TEXCOORD2.w;
  tmpvar_6.z = xlv_TEXCOORD3.w;
  tmpvar_2 = tmpvar_6;
  lowp vec3 tmpvar_7;
  tmpvar_7 = xlv_TEXCOORD1.xyz;
  tmpvar_3 = tmpvar_7;
  lowp vec3 tmpvar_8;
  tmpvar_8 = xlv_TEXCOORD2.xyz;
  tmpvar_4 = tmpvar_8;
  lowp vec3 tmpvar_9;
  tmpvar_9 = xlv_TEXCOORD3.xyz;
  tmpvar_5 = tmpvar_9;
  mediump vec3 tmpvar_10;
  mediump vec3 tmpvar_11;
  mediump float tmpvar_12;
  mediump vec4 spec_13;
  lowp vec4 tmpvar_14;
  tmpvar_14 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_15;
  tmpvar_15 = (tmpvar_14.xyz * _Color.xyz);
  tmpvar_10 = tmpvar_15;
  lowp vec4 tmpvar_16;
  tmpvar_16 = texture2D (_SpecMap, xlv_TEXCOORD0);
  spec_13 = tmpvar_16;
  mediump vec3 tmpvar_17;
  tmpvar_17 = (spec_13.xyz * _Gloss);
  lowp vec3 tmpvar_18;
  tmpvar_18 = ((texture2D (_BumpMap, xlv_TEXCOORD0).xyz * 2.0) - 1.0);
  tmpvar_11 = tmpvar_18;
  mediump vec3 tmpvar_19;
  tmpvar_19.x = dot (tmpvar_3, tmpvar_11);
  tmpvar_19.y = dot (tmpvar_4, tmpvar_11);
  tmpvar_19.z = dot (tmpvar_5, tmpvar_11);
  highp vec3 tmpvar_20;
  tmpvar_20 = (tmpvar_2 - (2.0 * (dot (tmpvar_19, tmpvar_2) * tmpvar_19)));
  lowp vec4 tmpvar_21;
  tmpvar_21 = (textureCube (_Cube, tmpvar_20) * tmpvar_14.w);
  lowp float tmpvar_22;
  tmpvar_22 = (tmpvar_21.w * _ReflectColor.w);
  tmpvar_12 = tmpvar_22;
  lowp float shadow_23;
  lowp float tmpvar_24;
  tmpvar_24 = shadow2DEXT (_ShadowMapTexture, xlv_TEXCOORD7.xyz);
  highp float tmpvar_25;
  tmpvar_25 = (_LightShadowData.x + (tmpvar_24 * (1.0 - _LightShadowData.x)));
  shadow_23 = tmpvar_25;
  highp vec3 tmpvar_26;
  tmpvar_26 = normalize(xlv_TEXCOORD6);
  mediump vec3 lightDir_27;
  lightDir_27 = xlv_TEXCOORD4;
  mediump vec3 viewDir_28;
  viewDir_28 = tmpvar_26;
  mediump float atten_29;
  atten_29 = shadow_23;
  mediump vec4 c_30;
  mediump vec3 specCol_31;
  highp float nh_32;
  mediump float tmpvar_33;
  tmpvar_33 = max (0.0, dot (tmpvar_11, normalize((lightDir_27 + viewDir_28))));
  nh_32 = tmpvar_33;
  mediump float arg1_34;
  arg1_34 = (32.0 * _Shininess);
  highp vec3 tmpvar_35;
  tmpvar_35 = (pow (nh_32, arg1_34) * tmpvar_17);
  specCol_31 = tmpvar_35;
  c_30.xyz = ((((tmpvar_10 * _LightColor0.xyz) * max (0.0, dot (tmpvar_11, lightDir_27))) + (_LightColor0.xyz * specCol_31)) * (atten_29 * 2.0));
  c_30.w = tmpvar_12;
  c_1 = c_30;
  mediump vec3 tmpvar_36;
  tmpvar_36 = (c_1.xyz + (tmpvar_10 * xlv_TEXCOORD5));
  c_1.xyz = tmpvar_36;
  mediump vec3 tmpvar_37;
  tmpvar_37 = (c_1.xyz + ((tmpvar_21.xyz * _ReflectColor.xyz) * _ReflectPower));
  c_1.xyz = tmpvar_37;
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
varying highp vec4 xlv_TEXCOORD5;
varying highp vec2 xlv_TEXCOORD4;
varying lowp vec4 xlv_TEXCOORD3;
varying lowp vec4 xlv_TEXCOORD2;
varying lowp vec4 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
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
  gl_Position = (gl_ModelViewProjectionMatrix * _glesVertex);
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = tmpvar_3;
  xlv_TEXCOORD2 = tmpvar_4;
  xlv_TEXCOORD3 = tmpvar_5;
  xlv_TEXCOORD4 = ((_glesMultiTexCoord1.xy * unity_LightmapST.xy) + unity_LightmapST.zw);
  xlv_TEXCOORD5 = (unity_World2Shadow[0] * (_Object2World * _glesVertex));
}



#endif
#ifdef FRAGMENT

#extension GL_EXT_shadow_samplers : enable
varying highp vec4 xlv_TEXCOORD5;
varying highp vec2 xlv_TEXCOORD4;
varying lowp vec4 xlv_TEXCOORD3;
varying lowp vec4 xlv_TEXCOORD2;
varying lowp vec4 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform sampler2D unity_Lightmap;
uniform mediump float _ReflectPower;
uniform lowp vec4 _ReflectColor;
uniform lowp vec4 _Color;
uniform samplerCube _Cube;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform sampler2DShadow _ShadowMapTexture;
uniform highp vec4 _LightShadowData;
void main ()
{
  lowp vec4 c_1;
  highp vec3 tmpvar_2;
  mediump vec3 tmpvar_3;
  mediump vec3 tmpvar_4;
  mediump vec3 tmpvar_5;
  lowp vec3 tmpvar_6;
  tmpvar_6.x = xlv_TEXCOORD1.w;
  tmpvar_6.y = xlv_TEXCOORD2.w;
  tmpvar_6.z = xlv_TEXCOORD3.w;
  tmpvar_2 = tmpvar_6;
  lowp vec3 tmpvar_7;
  tmpvar_7 = xlv_TEXCOORD1.xyz;
  tmpvar_3 = tmpvar_7;
  lowp vec3 tmpvar_8;
  tmpvar_8 = xlv_TEXCOORD2.xyz;
  tmpvar_4 = tmpvar_8;
  lowp vec3 tmpvar_9;
  tmpvar_9 = xlv_TEXCOORD3.xyz;
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
  lowp vec4 tmpvar_18;
  tmpvar_18 = (textureCube (_Cube, tmpvar_17) * tmpvar_13.w);
  lowp float tmpvar_19;
  tmpvar_19 = (tmpvar_18.w * _ReflectColor.w);
  tmpvar_12 = tmpvar_19;
  lowp float shadow_20;
  lowp float tmpvar_21;
  tmpvar_21 = shadow2DEXT (_ShadowMapTexture, xlv_TEXCOORD5.xyz);
  highp float tmpvar_22;
  tmpvar_22 = (_LightShadowData.x + (tmpvar_21 * (1.0 - _LightShadowData.x)));
  shadow_20 = tmpvar_22;
  lowp vec3 tmpvar_23;
  tmpvar_23 = min ((2.0 * texture2D (unity_Lightmap, xlv_TEXCOORD4).xyz), vec3((shadow_20 * 2.0)));
  mediump vec3 tmpvar_24;
  tmpvar_24 = (tmpvar_10 * tmpvar_23);
  c_1.xyz = tmpvar_24;
  c_1.w = tmpvar_12;
  mediump vec3 tmpvar_25;
  tmpvar_25 = (c_1.xyz + ((tmpvar_18.xyz * _ReflectColor.xyz) * _ReflectPower));
  c_1.xyz = tmpvar_25;
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
varying highp vec4 xlv_TEXCOORD7;
varying highp vec3 xlv_TEXCOORD6;
varying lowp vec3 xlv_TEXCOORD5;
varying lowp vec3 xlv_TEXCOORD4;
varying lowp vec4 xlv_TEXCOORD3;
varying lowp vec4 xlv_TEXCOORD2;
varying lowp vec4 xlv_TEXCOORD1;
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
  xlv_TEXCOORD1 = tmpvar_4;
  xlv_TEXCOORD2 = tmpvar_5;
  xlv_TEXCOORD3 = tmpvar_6;
  xlv_TEXCOORD4 = tmpvar_7;
  xlv_TEXCOORD5 = tmpvar_8;
  xlv_TEXCOORD6 = (tmpvar_14 * (((_World2Object * tmpvar_27).xyz * unity_Scale.w) - _glesVertex.xyz));
  xlv_TEXCOORD7 = (unity_World2Shadow[0] * (_Object2World * _glesVertex));
}



#endif
#ifdef FRAGMENT

#extension GL_EXT_shadow_samplers : enable
varying highp vec4 xlv_TEXCOORD7;
varying highp vec3 xlv_TEXCOORD6;
varying lowp vec3 xlv_TEXCOORD5;
varying lowp vec3 xlv_TEXCOORD4;
varying lowp vec4 xlv_TEXCOORD3;
varying lowp vec4 xlv_TEXCOORD2;
varying lowp vec4 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform mediump float _Gloss;
uniform mediump float _Shininess;
uniform mediump float _ReflectPower;
uniform lowp vec4 _ReflectColor;
uniform lowp vec4 _Color;
uniform samplerCube _Cube;
uniform sampler2D _SpecMap;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform sampler2DShadow _ShadowMapTexture;
uniform lowp vec4 _LightColor0;
uniform highp vec4 _LightShadowData;
void main ()
{
  lowp vec4 c_1;
  highp vec3 tmpvar_2;
  mediump vec3 tmpvar_3;
  mediump vec3 tmpvar_4;
  mediump vec3 tmpvar_5;
  lowp vec3 tmpvar_6;
  tmpvar_6.x = xlv_TEXCOORD1.w;
  tmpvar_6.y = xlv_TEXCOORD2.w;
  tmpvar_6.z = xlv_TEXCOORD3.w;
  tmpvar_2 = tmpvar_6;
  lowp vec3 tmpvar_7;
  tmpvar_7 = xlv_TEXCOORD1.xyz;
  tmpvar_3 = tmpvar_7;
  lowp vec3 tmpvar_8;
  tmpvar_8 = xlv_TEXCOORD2.xyz;
  tmpvar_4 = tmpvar_8;
  lowp vec3 tmpvar_9;
  tmpvar_9 = xlv_TEXCOORD3.xyz;
  tmpvar_5 = tmpvar_9;
  mediump vec3 tmpvar_10;
  mediump vec3 tmpvar_11;
  mediump float tmpvar_12;
  mediump vec4 spec_13;
  lowp vec4 tmpvar_14;
  tmpvar_14 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_15;
  tmpvar_15 = (tmpvar_14.xyz * _Color.xyz);
  tmpvar_10 = tmpvar_15;
  lowp vec4 tmpvar_16;
  tmpvar_16 = texture2D (_SpecMap, xlv_TEXCOORD0);
  spec_13 = tmpvar_16;
  mediump vec3 tmpvar_17;
  tmpvar_17 = (spec_13.xyz * _Gloss);
  lowp vec3 tmpvar_18;
  tmpvar_18 = ((texture2D (_BumpMap, xlv_TEXCOORD0).xyz * 2.0) - 1.0);
  tmpvar_11 = tmpvar_18;
  mediump vec3 tmpvar_19;
  tmpvar_19.x = dot (tmpvar_3, tmpvar_11);
  tmpvar_19.y = dot (tmpvar_4, tmpvar_11);
  tmpvar_19.z = dot (tmpvar_5, tmpvar_11);
  highp vec3 tmpvar_20;
  tmpvar_20 = (tmpvar_2 - (2.0 * (dot (tmpvar_19, tmpvar_2) * tmpvar_19)));
  lowp vec4 tmpvar_21;
  tmpvar_21 = (textureCube (_Cube, tmpvar_20) * tmpvar_14.w);
  lowp float tmpvar_22;
  tmpvar_22 = (tmpvar_21.w * _ReflectColor.w);
  tmpvar_12 = tmpvar_22;
  lowp float shadow_23;
  lowp float tmpvar_24;
  tmpvar_24 = shadow2DEXT (_ShadowMapTexture, xlv_TEXCOORD7.xyz);
  highp float tmpvar_25;
  tmpvar_25 = (_LightShadowData.x + (tmpvar_24 * (1.0 - _LightShadowData.x)));
  shadow_23 = tmpvar_25;
  highp vec3 tmpvar_26;
  tmpvar_26 = normalize(xlv_TEXCOORD6);
  mediump vec3 lightDir_27;
  lightDir_27 = xlv_TEXCOORD4;
  mediump vec3 viewDir_28;
  viewDir_28 = tmpvar_26;
  mediump float atten_29;
  atten_29 = shadow_23;
  mediump vec4 c_30;
  mediump vec3 specCol_31;
  highp float nh_32;
  mediump float tmpvar_33;
  tmpvar_33 = max (0.0, dot (tmpvar_11, normalize((lightDir_27 + viewDir_28))));
  nh_32 = tmpvar_33;
  mediump float arg1_34;
  arg1_34 = (32.0 * _Shininess);
  highp vec3 tmpvar_35;
  tmpvar_35 = (pow (nh_32, arg1_34) * tmpvar_17);
  specCol_31 = tmpvar_35;
  c_30.xyz = ((((tmpvar_10 * _LightColor0.xyz) * max (0.0, dot (tmpvar_11, lightDir_27))) + (_LightColor0.xyz * specCol_31)) * (atten_29 * 2.0));
  c_30.w = tmpvar_12;
  c_1 = c_30;
  mediump vec3 tmpvar_36;
  tmpvar_36 = (c_1.xyz + (tmpvar_10 * xlv_TEXCOORD5));
  c_1.xyz = tmpvar_36;
  mediump vec3 tmpvar_37;
  tmpvar_37 = (c_1.xyz + ((tmpvar_21.xyz * _ReflectColor.xyz) * _ReflectPower));
  c_1.xyz = tmpvar_37;
  gl_FragData[0] = c_1;
}



#endif"
}

}
Program "fp" {
// Fragment combos: 4
//   opengl - ALU: 27 to 47, TEX: 4 to 5
//   d3d9 - ALU: 23 to 44, TEX: 4 to 5
//   d3d11 - ALU: 13 to 30, TEX: 4 to 5, FLOW: 1 to 1
SubProgram "opengl " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Vector 2 [_ReflectColor]
Float 3 [_ReflectPower]
Float 4 [_Shininess]
Float 5 [_Gloss]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_Cube] CUBE
"3.0-!!ARBfp1.0
OPTION ARB_precision_hint_fastest;
# 45 ALU, 4 TEX
PARAM c[7] = { program.local[0..5],
		{ 2, 1, 0, 32 } };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
TEX R0.yw, fragment.texcoord[0], texture[2], 2D;
MAD R1.xy, R0.wyzw, c[6].x, -c[6].y;
MUL R0.x, R1.y, R1.y;
MAD R0.x, -R1, R1, -R0;
ADD R0.x, R0, c[6].y;
RSQ R0.x, R0.x;
RCP R1.z, R0.x;
DP3 R0.y, fragment.texcoord[6], fragment.texcoord[6];
MOV R2.xyz, fragment.texcoord[4];
RSQ R0.y, R0.y;
MAD R3.xyz, R0.y, fragment.texcoord[6], R2;
DP3 R0.y, R3, R3;
RSQ R1.w, R0.y;
DP3 R2.x, fragment.texcoord[1], R1;
DP3 R2.y, R1, fragment.texcoord[2];
DP3 R2.z, R1, fragment.texcoord[3];
MOV R0.x, fragment.texcoord[1].w;
MOV R0.z, fragment.texcoord[3].w;
MOV R0.y, fragment.texcoord[2].w;
DP3 R0.w, R2, R0;
MUL R2.xyz, R2, R0.w;
MAD R2.xyz, -R2, c[6].x, R0;
MUL R3.xyz, R1.w, R3;
DP3 R0.w, R1, R3;
MAX R1.w, R0, c[6].z;
MOV R0.w, c[6];
TEX R0.xyz, fragment.texcoord[0], texture[1], 2D;
MUL R0.w, R0, c[4].x;
POW R0.w, R1.w, R0.w;
MUL R0.xyz, R0, c[5].x;
MUL R3.xyz, R0.w, R0;
TEX R0, fragment.texcoord[0], texture[0], 2D;
DP3 R1.w, R1, fragment.texcoord[4];
MUL R0.xyz, R0, c[1];
MUL R1.xyz, R0, c[0];
TEX R2, R2, texture[3], CUBE;
MAX R1.w, R1, c[6].z;
MUL R3.xyz, R3, c[0];
MAD R1.xyz, R1, R1.w, R3;
MUL R0.xyz, R0, fragment.texcoord[5];
MAD R0.xyz, R1, c[6].x, R0;
MUL R2, R2, R0.w;
MUL R1, R2, c[2];
MAD result.color.xyz, R1, c[3].x, R0;
MOV result.color.w, R1;
END
# 45 instructions, 4 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Vector 2 [_ReflectColor]
Float 3 [_ReflectPower]
Float 4 [_Shininess]
Float 5 [_Gloss]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_Cube] CUBE
"ps_3_0
; 43 ALU, 4 TEX
dcl_2d s0
dcl_2d s1
dcl_2d s2
dcl_cube s3
def c6, 2.00000000, -1.00000000, 1.00000000, 0.00000000
def c7, 32.00000000, 0, 0, 0
dcl_texcoord0 v0.xy
dcl_texcoord1 v1
dcl_texcoord2 v2
dcl_texcoord3 v3
dcl_texcoord4 v4.xyz
dcl_texcoord5 v5.xyz
dcl_texcoord6 v6.xyz
texld r0.yw, v0, s2
mad_pp r2.xy, r0.wyzw, c6.x, c6.y
mul_pp r0.x, r2.y, r2.y
mad_pp r0.x, -r2, r2, -r0
add_pp r0.x, r0, c6.z
rsq_pp r0.x, r0.x
rcp_pp r2.z, r0.x
dp3_pp r0.w, v6, v6
dp3_pp r3.x, v1, r2
dp3_pp r3.y, r2, v2
dp3_pp r3.z, r2, v3
rsq_pp r0.w, r0.w
mov_pp r0.xyz, v4
mad_pp r0.xyz, r0.w, v6, r0
dp3_pp r0.w, r0, r0
rsq_pp r0.w, r0.w
mul_pp r0.xyz, r0.w, r0
mov_pp r0.w, c4.x
dp3_pp r0.x, r2, r0
mov r1.x, v1.w
mov r1.z, v3.w
mov r1.y, v2.w
dp3 r1.w, r3, r1
mul r3.xyz, r3, r1.w
mad r1.xyz, -r3, c6.x, r1
texld r3.xyz, v0, s1
max_pp r1.w, r0.x, c6
mul_pp r2.w, c7.x, r0
pow r0, r1.w, r2.w
dp3_pp r1.w, r2, v4
mul_pp r3.xyz, r3, c5.x
mul r3.xyz, r0.x, r3
texld r0, v0, s0
mul_pp r0.xyz, r0, c1
mul_pp r2.xyz, r0, c0
max_pp r1.w, r1, c6
mul_pp r3.xyz, r3, c0
mad_pp r2.xyz, r2, r1.w, r3
texld r1, r1, s3
mul_pp r0.xyz, r0, v5
mul_pp r1, r1, r0.w
mul_pp r1, r1, c2
mad_pp r0.xyz, r2, c6.x, r0
mad_pp oC0.xyz, r1, c3.x, r0
mov_pp oC0.w, r1
"
}

SubProgram "xbox360 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Vector 1 [_Color]
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
// ALU: 44.00 (33 instructions), vertex: 0, texture: 16,
//   sequencer: 16, interpolator: 28;    10 GPRs, 18 threads,
// Performance (if enough threads): ~44 cycles per vector
// * Texture cycle estimates are assuming an 8bit/component texture with no
//     aniso or trilinear filtering.

"ps_360
backbbaaaaaaacdaaaaaacdiaaaaaaaaaaaaaaceaaaaabmmaaaaabpeaaaaaaaa
aaaaaaaaaaaaabkeaaaaaabmaaaaabjfppppadaaaaaaaaakaaaaaabmaaaaaaaa
aaaaabioaaaaaaoeaaadaaabaaabaaaaaaaaaapaaaaaaaaaaaaaabaaaaacaaab
aaabaaaaaaaaabaiaaaaaaaaaaaaabbiaaadaaadaaabaaaaaaaaabcaaaaaaaaa
aaaaabdaaaacaaafaaabaaaaaaaaabdiaaaaaaaaaaaaabeiaaacaaaaaaabaaaa
aaaaabaiaaaaaaaaaaaaabffaaadaaaaaaabaaaaaaaaaapaaaaaaaaaaaaaabfo
aaacaaacaaabaaaaaaaaabaiaaaaaaaaaaaaabgmaaacaaadaaabaaaaaaaaabdi
aaaaaaaaaaaaabhkaaacaaaeaaabaaaaaaaaabdiaaaaaaaaaaaaabifaaadaaac
aaabaaaaaaaaaapaaaaaaaaafpechfgnhaengbhaaaklklklaaaeaaamaaabaaab
aaabaaaaaaaaaaaafpedgpgmgphcaaklaaabaaadaaabaaaeaaabaaaaaaaaaaaa
fpedhfgcgfaaklklaaaeaaaoaaabaaabaaabaaaaaaaaaaaafpehgmgphdhdaakl
aaaaaaadaaabaaabaaabaaaaaaaaaaaafpemgjghgiheedgpgmgphcdaaafpengb
gjgofegfhiaafpfcgfgggmgfgdheedgpgmgphcaafpfcgfgggmgfgdhefagphhgf
hcaafpfdgigjgogjgogfhdhdaafpfdhagfgdengbhaaahahdfpddfpdaaadccoda
codcdadddfddcodaaaklklklaaaaaaaaaaaaaaabaaaaaaaaaaaaaaaaaaaaaabe
abpmaabaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaeaaaaaabpibaaaajaa
aaaaaaaeaaaaaaaaaaaafmohaahpaahpaaaaaaabaaaadafaaaaapbfbaaaapcfc
aaaapdfdaaaahefeaaaahfffaaaahgfgaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaecaaaaaadpiaaaaaaaaaaaaaaaaaaaaa
eaaaaaaaaaaaaaaalpiaaaaadpmaaaaaaajfgaaegaakbcaabcaaaaaaaaaagaba
gabgbcaabcaaaaaaaaaagabmcaccbcaabcaaaaaeaaaaaaaafacemeaaccaaaaaa
bacaiaabbpbppoiiaaaaeaaabaaahaabbpbppgiiaaaaeaaababaaaabbpbppomp
aaaaeaaamiabaaaaaaloloaapaagagaamiadaaajaalcgmmgilaappppfibhaaah
aamamagmkbahabiamiahaaagaagmmamaolaaagaemiabaaaaaegngnlbnbajajpo
kaeiajaaaalologmpaagagiamiabaaaeaaloloaapaajaeaamiabaaaaaaloloaa
paajabaamiaeaaaaaaloloaapaajacaafiicaaaaaaloloblpaajadiamiaoaaae
aapmblaaobagaaaamiacaaaeaamdloaapaaeajaabeaiaaaaaalbblgmmbaaadae
miaiaaaaaamgblblolaaacaamiaiaaaeaagmblblolaaabaaamidagaeaalalbgm
icaepppoeaehaeagaalelelbkbahaaiemiapaaagaadeomaaobagaeaadiihaaae
aamagmblkbaiafagmiahaaaeaaleblaaobaeaaaamiahaaaeaamaleleklaeaaag
miapaaaeaadedeaaoaaeaeaamiahaaaaaablleaaobaeaaaabeacaaaaaflbblmg
oaaaacaaaeebaaaaaegmblbloaaaabadmiapaaaaaakgmnaapcaaaaaaembeabab
aablblmgocaaaaiamiadaaabaagngmblmlaaabppjadiaacbbpbppgiiaaaamaaa
miapaaaaaaaablaaobaaahaamiaiiaaaaablblaakbaaacaamiahaaabaaleleaa
kbaaacaamiahaaaaaaleleleolahafaemiahiaaaaalegmleklabadaaaaaaaaaa
aaaaaaaaaaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Vector 2 [_ReflectColor]
Float 3 [_ReflectPower]
Float 4 [_Shininess]
Float 5 [_Gloss]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_Cube] CUBE
"sce_fp_rsx // 58 instructions using 4 registers
[Configuration]
24
ffffffff001fc020007fff81000000000000840004000000
[Offsets]
6
_LightColor0 2 0
0000026000000210
_Color 1 0
000000e0
_ReflectColor 1 0
00000350
_ReflectPower 1 0
00000380
_Shininess 1 0
00000140
_Gloss 1 0
00000020
[Microcode]
928
8e001702c8011c9dc8000001c8003fe10e8a0240c8001c9d00020000c8000001
0000000000000000000000000000000094021704c8011c9dc8000001c8003fe1
4e803941c8011c9dc8000029c800bfe106820440ce041c9d00020000aa020000
000040000000bf80000000000000000010880240ab041c9cab040000c8000001
0e880141c8011c9dc8000001c8003fe10e060340c9101c9dc9000001c8000001
1088044001041c9e01040000c91000039e021700c8011c9dc8000001c8003fe1
0e803940c80c1c9dc8000029c80000010e840240c8041c9dc8020001c8000001
0000000000000000000000000000000010800340c9101c9dc8020001c8000001
00000000000000000000000000003f8008823b40ff003c9dff000001c8000001
10820540c9041c9dc9000001c80000011088014000021c9cc8000001c8000001
00000000000000000000000000000000be800140c8011c9dc8000001c8003fe1
028c0540c9001c9dc9040001c8000001108c0240c9101c9d00020000c8000001
0000420000000000000000000000000010060900c9041c9d00020000c8000001
0000000000000000000000000000000002880540c9041c9dc9100001c8000001
02880900c9101c9d00020000c800000100000000000000000000000000000000
08061d00fe0c1c9dc8000001c800000110060200540c1c9dc9180001c8000001
0e800240c9081c9dc8020001c800000100000000000000000000000000000000
0e800240c9001c9d01100000c800000104061c00fe0c1c9dc8000001c8000001
0e880200aa0c1c9cc9140001c80000010e800440c9101c9dc8021001c9000001
000000000000000000000000000000002e800441c9081c9dc8010001c9003fe1
fe880140c8011c9dc8000001c8003fe1088a0140ff101c9dc8000001c8000001
028a0140ff001c9dc8000001c8000001de840140c8011c9dc8000001c8003fe1
048a0140ff081c9dc8000001c8000001048c0540c9041c9dc9080001c8000001
088c0540c9041c9dc9100001c80000011e7e7e00c8001c9dc8000001c8000001
1e7e7d00c8001c9dc8000001c800000108000500c9181c9dc9141001c8000001
0e040400c9181c9f54000001c91400011e041706c8081c9dc8000001c8000001
1e820240c8081c9dc8020001c800000100000000000000000000000000000000
1e820240c9041c9dfe040001c80000010e800440c9041c9d00020000c9000001
0000000000000000000000000000000010810140c9041c9dc8000001c8000001
"
}

SubProgram "d3d11 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
ConstBuffer "$Globals" 112 // 92 used size, 9 vars
Vector 16 [_LightColor0] 4
Vector 48 [_Color] 4
Vector 64 [_ReflectColor] 4
Float 80 [_ReflectPower]
Float 84 [_Shininess]
Float 88 [_Gloss]
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 0
SetTexture 1 [_SpecMap] 2D 2
SetTexture 2 [_BumpMap] 2D 1
SetTexture 3 [_Cube] CUBE 3
// 44 instructions, 4 temp regs, 0 temp arrays:
// ALU 28 float, 0 int, 0 uint
// TEX 4 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0
eefiecedjcigelkfgcenbigghedhfhpmfhickkhaabaaaaaafmahaaaaadaaaaaa
cmaaaaaabeabaaaaeiabaaaaejfdeheooaaaaaaaaiaaaaaaaiaaaaaamiaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaneaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaadadaaaaneaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaa
apapaaaaneaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaapapaaaaneaaaaaa
adaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapapaaaaneaaaaaaaeaaaaaaaaaaaaaa
adaaaaaaafaaaaaaahahaaaaneaaaaaaafaaaaaaaaaaaaaaadaaaaaaagaaaaaa
ahahaaaaneaaaaaaagaaaaaaaaaaaaaaadaaaaaaahaaaaaaahahaaaafdfgfpfa
epfdejfeejepeoaafeeffiedepepfceeaaklklklepfdeheocmaaaaaaabaaaaaa
aiaaaaaacaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaafdfgfpfe
gbhcghgfheaaklklfdeieefcamagaaaaeaaaaaaaidabaaaafjaaaaaeegiocaaa
aaaaaaaaagaaaaaafkaaaaadaagabaaaaaaaaaaafkaaaaadaagabaaaabaaaaaa
fkaaaaadaagabaaaacaaaaaafkaaaaadaagabaaaadaaaaaafibiaaaeaahabaaa
aaaaaaaaffffaaaafibiaaaeaahabaaaabaaaaaaffffaaaafibiaaaeaahabaaa
acaaaaaaffffaaaafidaaaaeaahabaaaadaaaaaaffffaaaagcbaaaaddcbabaaa
abaaaaaagcbaaaadpcbabaaaacaaaaaagcbaaaadpcbabaaaadaaaaaagcbaaaad
pcbabaaaaeaaaaaagcbaaaadhcbabaaaafaaaaaagcbaaaadhcbabaaaagaaaaaa
gcbaaaadhcbabaaaahaaaaaagfaaaaadpccabaaaaaaaaaaagiaaaaacaeaaaaaa
efaaaaajpcaabaaaaaaaaaaaegbabaaaabaaaaaaeghobaaaabaaaaaaaagabaaa
acaaaaaadiaaaaaihcaabaaaaaaaaaaaegacbaaaaaaaaaaakgikcaaaaaaaaaaa
afaaaaaadiaaaaaiicaabaaaaaaaaaaabkiacaaaaaaaaaaaafaaaaaaabeaaaaa
aaaaaaecbaaaaaahbcaabaaaabaaaaaaegbcbaaaahaaaaaaegbcbaaaahaaaaaa
eeaaaaafbcaabaaaabaaaaaaakaabaaaabaaaaaadcaaaaajhcaabaaaabaaaaaa
egbcbaaaahaaaaaaagaabaaaabaaaaaaegbcbaaaafaaaaaabaaaaaahicaabaaa
abaaaaaaegacbaaaabaaaaaaegacbaaaabaaaaaaeeaaaaaficaabaaaabaaaaaa
dkaabaaaabaaaaaadiaaaaahhcaabaaaabaaaaaapgapbaaaabaaaaaaegacbaaa
abaaaaaaefaaaaajpcaabaaaacaaaaaaegbabaaaabaaaaaaeghobaaaacaaaaaa
aagabaaaabaaaaaadcaaaaapdcaabaaaacaaaaaahgapbaaaacaaaaaaaceaaaaa
aaaaaaeaaaaaaaeaaaaaaaaaaaaaaaaaaceaaaaaaaaaialpaaaaialpaaaaaaaa
aaaaaaaadcaaaaakicaabaaaabaaaaaaakaabaiaebaaaaaaacaaaaaaakaabaaa
acaaaaaaabeaaaaaaaaaiadpdcaaaaakicaabaaaabaaaaaabkaabaiaebaaaaaa
acaaaaaabkaabaaaacaaaaaadkaabaaaabaaaaaaelaaaaafecaabaaaacaaaaaa
dkaabaaaabaaaaaabaaaaaahbcaabaaaabaaaaaaegacbaaaacaaaaaaegacbaaa
abaaaaaadeaaaaahbcaabaaaabaaaaaaakaabaaaabaaaaaaabeaaaaaaaaaaaaa
cpaaaaafbcaabaaaabaaaaaaakaabaaaabaaaaaadiaaaaahicaabaaaaaaaaaaa
dkaabaaaaaaaaaaaakaabaaaabaaaaaabjaaaaaficaabaaaaaaaaaaadkaabaaa
aaaaaaaadiaaaaahhcaabaaaaaaaaaaaegacbaaaaaaaaaaapgapbaaaaaaaaaaa
diaaaaaihcaabaaaaaaaaaaaegacbaaaaaaaaaaaegiccaaaaaaaaaaaabaaaaaa
efaaaaajpcaabaaaabaaaaaaegbabaaaabaaaaaaeghobaaaaaaaaaaaaagabaaa
aaaaaaaadiaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaaegiccaaaaaaaaaaa
adaaaaaadiaaaaaihcaabaaaadaaaaaaegacbaaaabaaaaaaegiccaaaaaaaaaaa
abaaaaaabaaaaaahicaabaaaaaaaaaaaegacbaaaacaaaaaaegbcbaaaafaaaaaa
deaaaaahicaabaaaaaaaaaaadkaabaaaaaaaaaaaabeaaaaaaaaaaaaadcaaaaaj
hcaabaaaaaaaaaaaegacbaaaadaaaaaapgapbaaaaaaaaaaaegacbaaaaaaaaaaa
aaaaaaahhcaabaaaaaaaaaaaegacbaaaaaaaaaaaegacbaaaaaaaaaaadcaaaaaj
hcaabaaaaaaaaaaaegacbaaaabaaaaaaegbcbaaaagaaaaaaegacbaaaaaaaaaaa
baaaaaahbcaabaaaabaaaaaaegbcbaaaacaaaaaaegacbaaaacaaaaaabaaaaaah
ccaabaaaabaaaaaaegbcbaaaadaaaaaaegacbaaaacaaaaaabaaaaaahecaabaaa
abaaaaaaegbcbaaaaeaaaaaaegacbaaaacaaaaaadgaaaaafbcaabaaaacaaaaaa
dkbabaaaacaaaaaadgaaaaafccaabaaaacaaaaaadkbabaaaadaaaaaadgaaaaaf
ecaabaaaacaaaaaadkbabaaaaeaaaaaabaaaaaahicaabaaaaaaaaaaaegacbaaa
acaaaaaaegacbaaaabaaaaaaaaaaaaahicaabaaaaaaaaaaadkaabaaaaaaaaaaa
dkaabaaaaaaaaaaadcaaaaakhcaabaaaabaaaaaaegacbaaaabaaaaaapgapbaia
ebaaaaaaaaaaaaaaegacbaaaacaaaaaaefaaaaajpcaabaaaacaaaaaaegacbaaa
abaaaaaaeghobaaaadaaaaaaaagabaaaadaaaaaadiaaaaahpcaabaaaabaaaaaa
pgapbaaaabaaaaaaegaobaaaacaaaaaadiaaaaaihcaabaaaabaaaaaaegacbaaa
abaaaaaaegiccaaaaaaaaaaaaeaaaaaadiaaaaaiiccabaaaaaaaaaaadkaabaaa
abaaaaaadkiacaaaaaaaaaaaaeaaaaaadcaaaaakhccabaaaaaaaaaaaegacbaaa
abaaaaaaagiacaaaaaaaaaaaafaaaaaaegacbaaaaaaaaaaadoaaaaab"
}

SubProgram "gles " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
"!!GLES"
}

SubProgram "glesdesktop " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
"!!GLES"
}

SubProgram "opengl " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Vector 0 [_Color]
Vector 1 [_ReflectColor]
Float 2 [_ReflectPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_BumpMap] 2D
SetTexture 2 [_Cube] CUBE
SetTexture 3 [unity_Lightmap] 2D
"3.0-!!ARBfp1.0
OPTION ARB_precision_hint_fastest;
# 27 ALU, 4 TEX
PARAM c[4] = { program.local[0..2],
		{ 8, 2, 1 } };
TEMP R0;
TEMP R1;
TEMP R2;
TEX R0.yw, fragment.texcoord[0], texture[1], 2D;
MAD R1.xy, R0.wyzw, c[3].y, -c[3].z;
MUL R0.x, R1.y, R1.y;
MAD R0.x, -R1, R1, -R0;
ADD R0.x, R0, c[3].z;
RSQ R0.x, R0.x;
RCP R1.z, R0.x;
DP3 R0.x, fragment.texcoord[1], R1;
DP3 R0.y, R1, fragment.texcoord[2];
DP3 R0.z, R1, fragment.texcoord[3];
MOV R1.x, fragment.texcoord[1].w;
MOV R1.z, fragment.texcoord[3].w;
MOV R1.y, fragment.texcoord[2].w;
DP3 R0.w, R0, R1;
MUL R0.xyz, R0, R0.w;
MAD R0.xyz, -R0, c[3].y, R1;
TEX R1, fragment.texcoord[0], texture[0], 2D;
TEX R0, R0, texture[2], CUBE;
MUL R0, R1.w, R0;
MUL R2, R0, c[1];
TEX R0, fragment.texcoord[4], texture[3], 2D;
MUL R2.xyz, R2, c[2].x;
MUL R1.xyz, R1, c[0];
MUL R0.xyz, R0.w, R0;
MUL R0.xyz, R0, R1;
MAD result.color.xyz, R0, c[3].x, R2;
MOV result.color.w, R2;
END
# 27 instructions, 3 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Vector 0 [_Color]
Vector 1 [_ReflectColor]
Float 2 [_ReflectPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_BumpMap] 2D
SetTexture 2 [_Cube] CUBE
SetTexture 3 [unity_Lightmap] 2D
"ps_3_0
; 23 ALU, 4 TEX
dcl_2d s0
dcl_2d s1
dcl_cube s2
dcl_2d s3
def c3, 2.00000000, -1.00000000, 1.00000000, 8.00000000
dcl_texcoord0 v0.xy
dcl_texcoord1 v1
dcl_texcoord2 v2
dcl_texcoord3 v3
dcl_texcoord4 v4.xy
texld r0.yw, v0, s1
mad_pp r1.xy, r0.wyzw, c3.x, c3.y
mul_pp r0.x, r1.y, r1.y
mad_pp r0.x, -r1, r1, -r0
add_pp r0.x, r0, c3.z
rsq_pp r0.x, r0.x
rcp_pp r1.z, r0.x
dp3_pp r0.x, v1, r1
dp3_pp r0.y, r1, v2
dp3_pp r0.z, r1, v3
mov r1.x, v1.w
mov r1.z, v3.w
mov r1.y, v2.w
dp3 r0.w, r0, r1
mul r0.xyz, r0, r0.w
mad r0.xyz, -r0, c3.x, r1
texld r1, v0, s0
texld r0, r0, s2
mul_pp r0, r1.w, r0
mul_pp r2, r0, c1
texld r0, v4, s3
mul_pp r2.xyz, r2, c2.x
mul_pp r1.xyz, r1, c0
mul_pp r0.xyz, r0.w, r0
mul_pp r0.xyz, r0, r1
mad_pp oC0.xyz, r0, c3.w, r2
mov_pp oC0.w, r2
"
}

SubProgram "xbox360 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Vector 0 [_Color]
Vector 1 [_ReflectColor]
Float 2 [_ReflectPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_BumpMap] 2D
SetTexture 2 [_Cube] CUBE
SetTexture 3 [unity_Lightmap] 2D
// Shader Timing Estimate, in Cycles/64 pixel vector:
// ALU: 30.67 (23 instructions), vertex: 0, texture: 16,
//   sequencer: 14, interpolator: 20;    7 GPRs, 27 threads,
// Performance (if enough threads): ~30 cycles per vector
// * Texture cycle estimates are assuming an 8bit/component texture with no
//     aniso or trilinear filtering.

"ps_360
backbbaaaaaaabneaaaaabmaaaaaaaaaaaaaaaceaaaaabhiaaaaabkaaaaaaaaa
aaaaaaaaaaaaabfaaaaaaabmaaaaabecppppadaaaaaaaaahaaaaaabmaaaaaaaa
aaaaabdlaaaaaakiaaadaaabaaabaaaaaaaaaaleaaaaaaaaaaaaaameaaacaaaa
aaabaaaaaaaaaammaaaaaaaaaaaaaanmaaadaaacaaabaaaaaaaaaaoeaaaaaaaa
aaaaaapeaaadaaaaaaabaaaaaaaaaaleaaaaaaaaaaaaaapnaaacaaabaaabaaaa
aaaaaammaaaaaaaaaaaaabalaaacaaacaaabaaaaaaaaabbmaaaaaaaaaaaaabcm
aaadaaadaaabaaaaaaaaaaleaaaaaaaafpechfgnhaengbhaaaklklklaaaeaaam
aaabaaabaaabaaaaaaaaaaaafpedgpgmgphcaaklaaabaaadaaabaaaeaaabaaaa
aaaaaaaafpedhfgcgfaaklklaaaeaaaoaaabaaabaaabaaaaaaaaaaaafpengbgj
gofegfhiaafpfcgfgggmgfgdheedgpgmgphcaafpfcgfgggmgfgdhefagphhgfhc
aaklklklaaaaaaadaaabaaabaaabaaaaaaaaaaaahfgogjhehjfpemgjghgihegn
gbhaaahahdfpddfpdaaadccodacodcdadddfddcodaaaklklaaaaaaaaaaaaaaab
aaaaaaaaaaaaaaaaaaaaaabeabpmaabaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaeaaaaaabiabaaaagaaaaaaaaaeaaaaaaaaaaaaeakfaabpaabpaaaaaaab
aaaadafaaaaapbfbaaaapcfcaaaapdfdaaaadefeaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaebaaaaaaeaaaaaaalpiaaaaa
dpmaaaaadpiaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaajgaaegaakbcaabcaaaaaa
aeaagabacabgbcaabcaaaaafaaaaaaaagabimeaabcaaaaaaaaaababoaaaaccaa
aaaaaaaababaaaabbpbppghpaaaaeaaamiadaaagaamhlbmgilaapopomiaeaaaa
aegngngmnbagagppkaeaagaaaaaaaamgocaaaaiamiabaaafaaloloaapaagabaa
miaeaaafaaloloaapaagacaamiacaaafaaloloaapaagadaamiaeaaaaaalbblaa
obafadaamiaeaaaaaamgblmgolafacaamiaeaaaaaagmblmgolafabaaaaeaaaaa
aaaaaamgocaaaaaamiahaaafaamgmaaaobaaafaabeacaaacafmgbllboaafacaf
aeebacacaegmblbloaafabadmiapaaabaakgmnaapcacacaaemeeaaacaablblmg
ocababibmiadaaacaagnmgblmlabaapojacidaebbpbppgiiaaaamaaabaaiaaab
bpbppgecaaaaeaaabadicaibbpbppgiiaaaaeaaakiiaabaaaaaaaaedocaaaapo
becpaaadaaaabllbobadaaaamiaiiaaaaablblaakbadabaakibhadabaamamaeb
ibadabaakichadacaablmaicmbabacaakiehadabaamagmmaibabacaamiahiaaa
aamamamaoladacabaaaaaaaaaaaaaaaaaaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
Vector 0 [_Color]
Vector 1 [_ReflectColor]
Float 2 [_ReflectPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_BumpMap] 2D
SetTexture 2 [_Cube] CUBE
SetTexture 3 [unity_Lightmap] 2D
"sce_fp_rsx // 32 instructions using 4 registers
[Configuration]
24
ffffffff0007c020001ffff1000000000000840004000000
[Offsets]
3
_Color 1 0
00000020
_ReflectColor 1 0
000001b0
_ReflectPower 1 0
000001e0
[Microcode]
512
9e001700c8011c9dc8000001c8003fe10e800240c8001c9dc8020001c8000001
000000000000000000000000000000001e021707c8011c9dc8000001c8003fe1
0e800240fe041c9dc9000001c8000001de880140c8011c9dc8000001c8003fe1
0e800240c9001c9dc8043001c800000194061702c8011c9dc8000001c8003fe1
18860440ee0c1c9c00020000aa020000000040000000bf800000000000000000
048c0140ff101c9dc8000001c800000110800240c90c1c9dc90c0001c8000001
10800440550c1c9f550c0001c9000003be8a0140c8011c9dc8000001c8003fe1
10800340c9001c9d00020000c800000100003f80000000000000000000000000
02863b40ff003c9dff000001c8000001028e0540c9141c9d1d0c0000c8000001
fe840140c8011c9dc8000001c8003fe1088c0140ff081c9dc8000001c8000001
028c0140ff141c9dc8000001c8000001048e05401d0c1c9cc9100001c8000001
088e05401d0c1c9cc9080001c800000108000500c91c1c9dc9181001c8000001
0e020400c91c1c9f54000001c91800011e021704c8041c9dc8000001c8000001
1e840240c8041c9dc8020001c800000100000000000000000000000000000000
1e820240c9081c9dfe000001c80000010e800440c9041c9d00020000c9000001
0000000000000000000000000000000010810140c9041c9dc8000001c8000001
"
}

SubProgram "d3d11 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
ConstBuffer "$Globals" 128 // 84 used size, 10 vars
Vector 48 [_Color] 4
Vector 64 [_ReflectColor] 4
Float 80 [_ReflectPower]
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 0
SetTexture 1 [_BumpMap] 2D 1
SetTexture 2 [_Cube] CUBE 2
SetTexture 3 [unity_Lightmap] 2D 3
// 26 instructions, 3 temp regs, 0 temp arrays:
// ALU 13 float, 0 int, 0 uint
// TEX 4 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0
eefiecedlomfiaagmnjpnakajobonccbhobdpnbdabaaaaaabeafaaaaadaaaaaa
cmaaaaaaoeaaaaaabiabaaaaejfdeheolaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaakeaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaadadaaaakeaaaaaaaeaaaaaaaaaaaaaaadaaaaaaabaaaaaa
amamaaaakeaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaaapapaaaakeaaaaaa
acaaaaaaaaaaaaaaadaaaaaaadaaaaaaapapaaaakeaaaaaaadaaaaaaaaaaaaaa
adaaaaaaaeaaaaaaapapaaaafdfgfpfaepfdejfeejepeoaafeeffiedepepfcee
aaklklklepfdeheocmaaaaaaabaaaaaaaiaaaaaacaaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaaaaaaaaaapaaaaaafdfgfpfegbhcghgfheaaklklfdeieefcpeadaaaa
eaaaaaaapnaaaaaafjaaaaaeegiocaaaaaaaaaaaagaaaaaafkaaaaadaagabaaa
aaaaaaaafkaaaaadaagabaaaabaaaaaafkaaaaadaagabaaaacaaaaaafkaaaaad
aagabaaaadaaaaaafibiaaaeaahabaaaaaaaaaaaffffaaaafibiaaaeaahabaaa
abaaaaaaffffaaaafidaaaaeaahabaaaacaaaaaaffffaaaafibiaaaeaahabaaa
adaaaaaaffffaaaagcbaaaaddcbabaaaabaaaaaagcbaaaadmcbabaaaabaaaaaa
gcbaaaadpcbabaaaacaaaaaagcbaaaadpcbabaaaadaaaaaagcbaaaadpcbabaaa
aeaaaaaagfaaaaadpccabaaaaaaaaaaagiaaaaacadaaaaaaefaaaaajpcaabaaa
aaaaaaaaegbabaaaabaaaaaaeghobaaaabaaaaaaaagabaaaabaaaaaadcaaaaap
dcaabaaaaaaaaaaahgapbaaaaaaaaaaaaceaaaaaaaaaaaeaaaaaaaeaaaaaaaaa
aaaaaaaaaceaaaaaaaaaialpaaaaialpaaaaaaaaaaaaaaaadcaaaaakicaabaaa
aaaaaaaaakaabaiaebaaaaaaaaaaaaaaakaabaaaaaaaaaaaabeaaaaaaaaaiadp
dcaaaaakicaabaaaaaaaaaaabkaabaiaebaaaaaaaaaaaaaabkaabaaaaaaaaaaa
dkaabaaaaaaaaaaaelaaaaafecaabaaaaaaaaaaadkaabaaaaaaaaaaabaaaaaah
bcaabaaaabaaaaaaegbcbaaaacaaaaaaegacbaaaaaaaaaaabaaaaaahccaabaaa
abaaaaaaegbcbaaaadaaaaaaegacbaaaaaaaaaaabaaaaaahecaabaaaabaaaaaa
egbcbaaaaeaaaaaaegacbaaaaaaaaaaadgaaaaafbcaabaaaaaaaaaaadkbabaaa
acaaaaaadgaaaaafccaabaaaaaaaaaaadkbabaaaadaaaaaadgaaaaafecaabaaa
aaaaaaaadkbabaaaaeaaaaaabaaaaaahicaabaaaaaaaaaaaegacbaaaaaaaaaaa
egacbaaaabaaaaaaaaaaaaahicaabaaaaaaaaaaadkaabaaaaaaaaaaadkaabaaa
aaaaaaaadcaaaaakhcaabaaaaaaaaaaaegacbaaaabaaaaaapgapbaiaebaaaaaa
aaaaaaaaegacbaaaaaaaaaaaefaaaaajpcaabaaaaaaaaaaaegacbaaaaaaaaaaa
eghobaaaacaaaaaaaagabaaaacaaaaaaefaaaaajpcaabaaaabaaaaaaegbabaaa
abaaaaaaeghobaaaaaaaaaaaaagabaaaaaaaaaaadiaaaaahpcaabaaaaaaaaaaa
egaobaaaaaaaaaaapgapbaaaabaaaaaadiaaaaaihcaabaaaabaaaaaaegacbaaa
abaaaaaaegiccaaaaaaaaaaaadaaaaaadiaaaaaihcaabaaaaaaaaaaaegacbaaa
aaaaaaaaegiccaaaaaaaaaaaaeaaaaaadiaaaaaiiccabaaaaaaaaaaadkaabaaa
aaaaaaaadkiacaaaaaaaaaaaaeaaaaaadiaaaaaihcaabaaaaaaaaaaaegacbaaa
aaaaaaaaagiacaaaaaaaaaaaafaaaaaaefaaaaajpcaabaaaacaaaaaaogbkbaaa
abaaaaaaeghobaaaadaaaaaaaagabaaaadaaaaaadiaaaaahicaabaaaaaaaaaaa
dkaabaaaacaaaaaaabeaaaaaaaaaaaebdiaaaaahhcaabaaaacaaaaaaegacbaaa
acaaaaaapgapbaaaaaaaaaaadcaaaaajhccabaaaaaaaaaaaegacbaaaabaaaaaa
egacbaaaacaaaaaaegacbaaaaaaaaaaadoaaaaab"
}

SubProgram "gles " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
"!!GLES"
}

SubProgram "glesdesktop " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_OFF" }
"!!GLES"
}

SubProgram "opengl " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Vector 2 [_ReflectColor]
Float 3 [_ReflectPower]
Float 4 [_Shininess]
Float 5 [_Gloss]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_Cube] CUBE
SetTexture 4 [_ShadowMapTexture] 2D
"3.0-!!ARBfp1.0
OPTION ARB_precision_hint_fastest;
# 47 ALU, 5 TEX
PARAM c[7] = { program.local[0..5],
		{ 2, 1, 0, 32 } };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
TEMP R4;
TEX R0.yw, fragment.texcoord[0], texture[2], 2D;
MAD R2.xy, R0.wyzw, c[6].x, -c[6].y;
MUL R0.x, R2.y, R2.y;
MAD R0.w, -R2.x, R2.x, -R0.x;
DP3 R1.x, fragment.texcoord[6], fragment.texcoord[6];
ADD R0.w, R0, c[6].y;
RSQ R0.w, R0.w;
RCP R2.z, R0.w;
RSQ R1.x, R1.x;
MOV R0.xyz, fragment.texcoord[4];
MAD R0.xyz, R1.x, fragment.texcoord[6], R0;
DP3 R1.x, R0, R0;
RSQ R1.x, R1.x;
MUL R3.xyz, R1.x, R0;
DP3 R1.w, R2, R3;
DP3 R0.x, fragment.texcoord[1], R2;
DP3 R0.y, R2, fragment.texcoord[2];
DP3 R0.z, R2, fragment.texcoord[3];
MOV R1.x, fragment.texcoord[1].w;
MOV R1.z, fragment.texcoord[3].w;
MOV R1.y, fragment.texcoord[2].w;
DP3 R0.w, R0, R1;
MUL R3.xyz, R0, R0.w;
MAD R1.xyz, -R3, c[6].x, R1;
MOV R0.w, c[6];
TEX R0.xyz, fragment.texcoord[0], texture[1], 2D;
MAX R1.w, R1, c[6].z;
MUL R0.w, R0, c[4].x;
POW R0.w, R1.w, R0.w;
MUL R0.xyz, R0, c[5].x;
MUL R4.xyz, R0.w, R0;
TEX R0, fragment.texcoord[0], texture[0], 2D;
DP3 R1.w, R2, fragment.texcoord[4];
MUL R0.xyz, R0, c[1];
MUL R2.xyz, R0, c[0];
MAX R1.w, R1, c[6].z;
MUL R4.xyz, R4, c[0];
MAD R2.xyz, R2, R1.w, R4;
TXP R3.x, fragment.texcoord[7], texture[4], 2D;
TEX R1, R1, texture[3], CUBE;
MUL R1, R1, R0.w;
MUL R1, R1, c[2];
MUL R2.xyz, R3.x, R2;
MUL R0.xyz, R0, fragment.texcoord[5];
MAD R0.xyz, R2, c[6].x, R0;
MAD result.color.xyz, R1, c[3].x, R0;
MOV result.color.w, R1;
END
# 47 instructions, 5 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Vector 2 [_ReflectColor]
Float 3 [_ReflectPower]
Float 4 [_Shininess]
Float 5 [_Gloss]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_Cube] CUBE
SetTexture 4 [_ShadowMapTexture] 2D
"ps_3_0
; 44 ALU, 5 TEX
dcl_2d s0
dcl_2d s1
dcl_2d s2
dcl_cube s3
dcl_2d s4
def c6, 2.00000000, -1.00000000, 1.00000000, 0.00000000
def c7, 32.00000000, 0, 0, 0
dcl_texcoord0 v0.xy
dcl_texcoord1 v1
dcl_texcoord2 v2
dcl_texcoord3 v3
dcl_texcoord4 v4.xyz
dcl_texcoord5 v5.xyz
dcl_texcoord6 v6.xyz
dcl_texcoord7 v7
texld r0.yw, v0, s2
mad_pp r2.xy, r0.wyzw, c6.x, c6.y
mul_pp r0.x, r2.y, r2.y
mad_pp r0.x, -r2, r2, -r0
add_pp r0.x, r0, c6.z
rsq_pp r0.w, r0.x
rcp_pp r2.z, r0.w
dp3_pp r1.x, v6, v6
texld r4.xyz, v0, s1
dp3_pp r3.x, v1, r2
dp3_pp r3.y, r2, v2
dp3_pp r3.z, r2, v3
rsq_pp r1.x, r1.x
mov_pp r0.xyz, v4
mad_pp r0.xyz, r1.x, v6, r0
dp3_pp r1.w, r0, r0
rsq_pp r1.w, r1.w
mul_pp r0.xyz, r1.w, r0
dp3_pp r0.x, r2, r0
max_pp r1.w, r0.x, c6
mov r1.x, v1.w
mov r1.z, v3.w
mov r1.y, v2.w
dp3 r0.w, r3, r1
mul r3.xyz, r3, r0.w
mov_pp r0.w, c4.x
mad r1.xyz, -r3, c6.x, r1
mul_pp r2.w, c7.x, r0
pow r0, r1.w, r2.w
dp3_pp r1.w, r2, v4
mul_pp r4.xyz, r4, c5.x
mul r4.xyz, r0.x, r4
texld r0, v0, s0
mul_pp r0.xyz, r0, c1
mul_pp r2.xyz, r0, c0
max_pp r1.w, r1, c6
mul_pp r4.xyz, r4, c0
mad_pp r2.xyz, r2, r1.w, r4
texldp r3.x, v7, s4
texld r1, r1, s3
mul_pp r1, r1, r0.w
mul_pp r1, r1, c2
mul_pp r2.xyz, r3.x, r2
mul_pp r0.xyz, r0, v5
mad_pp r0.xyz, r2, c6.x, r0
mad_pp oC0.xyz, r1, c3.x, r0
mov_pp oC0.w, r1
"
}

SubProgram "xbox360 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Vector 1 [_Color]
Float 5 [_Gloss]
Vector 0 [_LightColor0]
Vector 2 [_ReflectColor]
Float 3 [_ReflectPower]
Float 4 [_Shininess]
SetTexture 0 [_ShadowMapTexture] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_SpecMap] 2D
SetTexture 4 [_Cube] CUBE
// Shader Timing Estimate, in Cycles/64 pixel vector:
// ALU: 46.67 (35 instructions), vertex: 0, texture: 20,
//   sequencer: 16, interpolator: 32;    9 GPRs, 21 threads,
// Performance (if enough threads): ~46 cycles per vector
// * Texture cycle estimates are assuming an 8bit/component texture with no
//     aniso or trilinear filtering.

"ps_360
backbbaaaaaaacfiaaaaacfmaaaaaaaaaaaaaaceaaaaabpaaaaaacbiaaaaaaaa
aaaaaaaaaaaaabmiaaaaaabmaaaaabllppppadaaaaaaaaalaaaaaabmaaaaaaaa
aaaaableaaaaaapiaaadaaacaaabaaaaaaaaabaeaaaaaaaaaaaaabbeaaacaaab
aaabaaaaaaaaabbmaaaaaaaaaaaaabcmaaadaaaeaaabaaaaaaaaabdeaaaaaaaa
aaaaabeeaaacaaafaaabaaaaaaaaabemaaaaaaaaaaaaabfmaaacaaaaaaabaaaa
aaaaabbmaaaaaaaaaaaaabgjaaadaaabaaabaaaaaaaaabaeaaaaaaaaaaaaabhc
aaacaaacaaabaaaaaaaaabbmaaaaaaaaaaaaabiaaaacaaadaaabaaaaaaaaabem
aaaaaaaaaaaaabioaaadaaaaaaabaaaaaaaaabaeaaaaaaaaaaaaabkaaaacaaae
aaabaaaaaaaaabemaaaaaaaaaaaaabklaaadaaadaaabaaaaaaaaabaeaaaaaaaa
fpechfgnhaengbhaaaklklklaaaeaaamaaabaaabaaabaaaaaaaaaaaafpedgpgm
gphcaaklaaabaaadaaabaaaeaaabaaaaaaaaaaaafpedhfgcgfaaklklaaaeaaao
aaabaaabaaabaaaaaaaaaaaafpehgmgphdhdaaklaaaaaaadaaabaaabaaabaaaa
aaaaaaaafpemgjghgiheedgpgmgphcdaaafpengbgjgofegfhiaafpfcgfgggmgf
gdheedgpgmgphcaafpfcgfgggmgfgdhefagphhgfhcaafpfdgigbgegphhengbha
fegfhihehfhcgfaafpfdgigjgogjgogfhdhdaafpfdhagfgdengbhaaahahdfpdd
fpdaaadccodacodcdadddfddcodaaaklaaaaaaaaaaaaaaabaaaaaaaaaaaaaaaa
aaaaaabeabpmaabaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaeaaaaaacbm
baaaaiaaaaaaaaaeaaaaaaaaaaaagnaiaappaappaaaaaaabaaaadafaaaaapbfb
aaaapcfcaaaapdfdaaaahefeaaaahfffaaaahgfgaaaaphfhaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaecaaaaaadpiaaaaa
aaaaaaaaaaaaaaaaeaaaaaaaaaaaaaaalpiaaaaadpmaaaaaacfagaaegaakbcaa
bcaaaaaaaaaagabagabgbcaabcaaaaaaabfafabmaaaabcaameaaaaaaaaaagacb
fachbcaaccaaaaaaemieaaaaaaloloblpaagagahmiadaaahaabllaaaobaaahaa
baaaaaobbpbppbppaaaaeaaabacahaabbpbpppnjaaaaeaaamiagaaahaagbgmmg
ilahppppfiebaaahaagmgmmgcbaepoiamiahaaaiaamgmamaolaaagaemiaiaaae
aelclclbnbahahpokaieahaaaaloloblpaaiaiiemiabaaagaamdloaapaahabaa
miabaaaeaamdloaapaahaeaamiaeaaagaamdloaapaahacaafiecaaagaamdlomg
paahadiamiaoaaaeaapmmgaaobaiaaaamiacaaaeaamdmdaapaaeahaamiaeaaaa
aalbblaaobagadaamiaeaaaaaamgblmgolagacaamiadaaaeaalalbaakcaeppaa
miaeaaaaaagmblmgolagabaaeaigagahaabgbglboaaaaaiemiapaaagaalappaa
obahagaabeacaaacafblblmgoaagacagaeebacacaelbblbloaagabadmiapaaab
aakgmnaapcacacaaemeeaaacaablblmgocababibmiadaaacaagnmgblmlabaapp
jaeicaebbpbppgiiaaaamaaabadibaabbpbppoiiaaaaeaaababiaaabbpbppgec
aaaaeaaabechaaabaamagmlbkbabafaakibpacadaaaablebmbacaaabmiaiiaaa
aablblaakbadacaakichacadaaleleicibadacabkiehacadaamagmmaibadadab
miahaaaaaamamaleolacafaddiihaaacaamamagmkbacaaagmiahaaabaamablaa
obabaaaamiahaaabaamamaaakbabaaaamiahaaabaalegmleolacaeabmiahiaaa
aalemgmaolabahaaaaaaaaaaaaaaaaaaaaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Vector 2 [_ReflectColor]
Float 3 [_ReflectPower]
Float 4 [_Shininess]
Float 5 [_Gloss]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_Cube] CUBE
SetTexture 4 [_ShadowMapTexture] 2D
"sce_fp_rsx // 59 instructions using 4 registers
[Configuration]
24
ffffffff003fc02000ffff01000000000000840004000000
[Offsets]
6
_LightColor0 2 0
0000034000000200
_Color 1 0
000000d0
_ReflectColor 1 0
00000310
_ReflectPower 1 0
000003a0
_Shininess 1 0
00000190
_Gloss 1 0
00000020
[Microcode]
944
8e001702c8011c9dc8000001c8003fe10e880240c8001c9d00020000c8000001
0000000000000000000000000000000094001704c8011c9dc8000001c8003fe1
4e8c3941c8011c9dc8000029c800bfe1068e0440ce001c9d00020000aa020000
000040000000bf80000000000000000010840240ab1c1c9cab1c0000c8000001
0e8a0141c8011c9dc8000001c8003fe110860440011c1c9e011c0000c9080003
0e020340c9141c9dc9180001c80000019e001700c8011c9dc8000001c8003fe1
0e8c0240c8001c9dc8020001c800000100000000000000000000000000000000
10800340c90c1c9dc8020001c800000100000000000000000000000000003f80
088e3b40ff003c9dff000001c80000010e843940c8041c9dc8000029c8000001
02820540c91c1c9dc9080001c8000001be840140c8011c9dc8000001c8003fe1
02840540c9081c9dc91c0001c8000001de860140c8011c9dc8000001c8003fe1
04840540c91c1c9dc90c0001c800000104860140ff0c1c9dc8000001c8000001
1086014000021c9cc8000001c800000100000000000000000000000000000000
10880540c91c1c9dc9140001c8000001108a0240c90c1c9d00020000c8000001
00004200000000000000000000000000fe800140c8011c9dc8000001c8003fe1
08840540c91c1c9dc9000001c80000010e800240c9181c9dc8020001c8000001
0000000000000000000000000000000002860140ff081c9dc8000001c8000001
08860140ff001c9dc8000001c800000108040500c9081c9dc90c1001c8000001
0e020400c9081c9f54080001c90c00011002090001041c9c00020000c8000001
000000000000000000000000000000002e8a0241c9181c9dc8015001c8003fe1
10021d00fe041c9dc8000001c800000162061809c8011c9dc8000001c8003fe1
10800900c9101c9d00020000c800000100000000000000000000000000000000
08000200fe041c9dff140001c80000011e021706c8041c9dc8000001c8000001
08001c0054001c9dc8000001c80000010e800240c9001c9dff000001c8000001
1e8e0240c8041c9dc8020001c800000100000000000000000000000000000000
0e88020054001c9dc9100001c80000010e800440c9101c9dc8020001c9000001
000000000000000000000000000000001e820240c91c1c9dfe000001c8000001
10800140c9041c9dc8000001c800000106060100c80c1c9dc8000001c8000001
0e800440000c1c9cc9001001c91400010e810440c9041c9d00020000c9000001
00000000000000000000000000000000
"
}

SubProgram "d3d11 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
ConstBuffer "$Globals" 176 // 156 used size, 10 vars
Vector 16 [_LightColor0] 4
Vector 112 [_Color] 4
Vector 128 [_ReflectColor] 4
Float 144 [_ReflectPower]
Float 148 [_Shininess]
Float 152 [_Gloss]
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 1
SetTexture 1 [_SpecMap] 2D 3
SetTexture 2 [_BumpMap] 2D 2
SetTexture 3 [_Cube] CUBE 4
SetTexture 4 [_ShadowMapTexture] 2D 0
// 47 instructions, 4 temp regs, 0 temp arrays:
// ALU 30 float, 0 int, 0 uint
// TEX 5 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0
eefiecedminddafjfmgdpkgdmjkcakjbmfccinfpabaaaaaapiahaaaaadaaaaaa
cmaaaaaacmabaaaagaabaaaaejfdeheopiaaaaaaajaaaaaaaiaaaaaaoaaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaomaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaadadaaaaomaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaa
apapaaaaomaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaapapaaaaomaaaaaa
adaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapapaaaaomaaaaaaaeaaaaaaaaaaaaaa
adaaaaaaafaaaaaaahahaaaaomaaaaaaafaaaaaaaaaaaaaaadaaaaaaagaaaaaa
ahahaaaaomaaaaaaagaaaaaaaaaaaaaaadaaaaaaahaaaaaaahahaaaaomaaaaaa
ahaaaaaaaaaaaaaaadaaaaaaaiaaaaaaapalaaaafdfgfpfaepfdejfeejepeoaa
feeffiedepepfceeaaklklklepfdeheocmaaaaaaabaaaaaaaiaaaaaacaaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaafdfgfpfegbhcghgfheaaklkl
fdeieefcjaagaaaaeaaaaaaakeabaaaafjaaaaaeegiocaaaaaaaaaaaakaaaaaa
fkaaaaadaagabaaaaaaaaaaafkaaaaadaagabaaaabaaaaaafkaaaaadaagabaaa
acaaaaaafkaaaaadaagabaaaadaaaaaafkaaaaadaagabaaaaeaaaaaafibiaaae
aahabaaaaaaaaaaaffffaaaafibiaaaeaahabaaaabaaaaaaffffaaaafibiaaae
aahabaaaacaaaaaaffffaaaafidaaaaeaahabaaaadaaaaaaffffaaaafibiaaae
aahabaaaaeaaaaaaffffaaaagcbaaaaddcbabaaaabaaaaaagcbaaaadpcbabaaa
acaaaaaagcbaaaadpcbabaaaadaaaaaagcbaaaadpcbabaaaaeaaaaaagcbaaaad
hcbabaaaafaaaaaagcbaaaadhcbabaaaagaaaaaagcbaaaadhcbabaaaahaaaaaa
gcbaaaadlcbabaaaaiaaaaaagfaaaaadpccabaaaaaaaaaaagiaaaaacaeaaaaaa
aoaaaaahdcaabaaaaaaaaaaaegbabaaaaiaaaaaapgbpbaaaaiaaaaaaefaaaaaj
pcaabaaaaaaaaaaaegaabaaaaaaaaaaaeghobaaaaeaaaaaaaagabaaaaaaaaaaa
aaaaaaahbcaabaaaaaaaaaaaakaabaaaaaaaaaaaakaabaaaaaaaaaaaefaaaaaj
pcaabaaaabaaaaaaegbabaaaabaaaaaaeghobaaaabaaaaaaaagabaaaadaaaaaa
diaaaaaiocaabaaaaaaaaaaaagajbaaaabaaaaaakgikcaaaaaaaaaaaajaaaaaa
diaaaaaibcaabaaaabaaaaaabkiacaaaaaaaaaaaajaaaaaaabeaaaaaaaaaaaec
baaaaaahccaabaaaabaaaaaaegbcbaaaahaaaaaaegbcbaaaahaaaaaaeeaaaaaf
ccaabaaaabaaaaaabkaabaaaabaaaaaadcaaaaajocaabaaaabaaaaaaagbjbaaa
ahaaaaaafgafbaaaabaaaaaaagbjbaaaafaaaaaabaaaaaahbcaabaaaacaaaaaa
jgahbaaaabaaaaaajgahbaaaabaaaaaaeeaaaaafbcaabaaaacaaaaaaakaabaaa
acaaaaaadiaaaaahocaabaaaabaaaaaafgaobaaaabaaaaaaagaabaaaacaaaaaa
efaaaaajpcaabaaaacaaaaaaegbabaaaabaaaaaaeghobaaaacaaaaaaaagabaaa
acaaaaaadcaaaaapdcaabaaaacaaaaaahgapbaaaacaaaaaaaceaaaaaaaaaaaea
aaaaaaeaaaaaaaaaaaaaaaaaaceaaaaaaaaaialpaaaaialpaaaaaaaaaaaaaaaa
dcaaaaakicaabaaaacaaaaaaakaabaiaebaaaaaaacaaaaaaakaabaaaacaaaaaa
abeaaaaaaaaaiadpdcaaaaakicaabaaaacaaaaaabkaabaiaebaaaaaaacaaaaaa
bkaabaaaacaaaaaadkaabaaaacaaaaaaelaaaaafecaabaaaacaaaaaadkaabaaa
acaaaaaabaaaaaahccaabaaaabaaaaaaegacbaaaacaaaaaajgahbaaaabaaaaaa
deaaaaahccaabaaaabaaaaaabkaabaaaabaaaaaaabeaaaaaaaaaaaaacpaaaaaf
ccaabaaaabaaaaaabkaabaaaabaaaaaadiaaaaahbcaabaaaabaaaaaabkaabaaa
abaaaaaaakaabaaaabaaaaaabjaaaaafbcaabaaaabaaaaaaakaabaaaabaaaaaa
diaaaaahocaabaaaaaaaaaaafgaobaaaaaaaaaaaagaabaaaabaaaaaadiaaaaai
ocaabaaaaaaaaaaafgaobaaaaaaaaaaaagijcaaaaaaaaaaaabaaaaaaefaaaaaj
pcaabaaaabaaaaaaegbabaaaabaaaaaaeghobaaaaaaaaaaaaagabaaaabaaaaaa
diaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaaegiccaaaaaaaaaaaahaaaaaa
diaaaaaihcaabaaaadaaaaaaegacbaaaabaaaaaaegiccaaaaaaaaaaaabaaaaaa
diaaaaahhcaabaaaabaaaaaaegacbaaaabaaaaaaegbcbaaaagaaaaaabaaaaaah
icaabaaaacaaaaaaegacbaaaacaaaaaaegbcbaaaafaaaaaadeaaaaahicaabaaa
acaaaaaadkaabaaaacaaaaaaabeaaaaaaaaaaaaadcaaaaajocaabaaaaaaaaaaa
agajbaaaadaaaaaapgapbaaaacaaaaaafgaobaaaaaaaaaaadcaaaaajhcaabaaa
aaaaaaaajgahbaaaaaaaaaaaagaabaaaaaaaaaaaegacbaaaabaaaaaabaaaaaah
bcaabaaaabaaaaaaegbcbaaaacaaaaaaegacbaaaacaaaaaabaaaaaahccaabaaa
abaaaaaaegbcbaaaadaaaaaaegacbaaaacaaaaaabaaaaaahecaabaaaabaaaaaa
egbcbaaaaeaaaaaaegacbaaaacaaaaaadgaaaaafbcaabaaaacaaaaaadkbabaaa
acaaaaaadgaaaaafccaabaaaacaaaaaadkbabaaaadaaaaaadgaaaaafecaabaaa
acaaaaaadkbabaaaaeaaaaaabaaaaaahicaabaaaaaaaaaaaegacbaaaacaaaaaa
egacbaaaabaaaaaaaaaaaaahicaabaaaaaaaaaaadkaabaaaaaaaaaaadkaabaaa
aaaaaaaadcaaaaakhcaabaaaabaaaaaaegacbaaaabaaaaaapgapbaiaebaaaaaa
aaaaaaaaegacbaaaacaaaaaaefaaaaajpcaabaaaacaaaaaaegacbaaaabaaaaaa
eghobaaaadaaaaaaaagabaaaaeaaaaaadiaaaaahpcaabaaaabaaaaaapgapbaaa
abaaaaaaegaobaaaacaaaaaadiaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaa
egiccaaaaaaaaaaaaiaaaaaadiaaaaaiiccabaaaaaaaaaaadkaabaaaabaaaaaa
dkiacaaaaaaaaaaaaiaaaaaadcaaaaakhccabaaaaaaaaaaaegacbaaaabaaaaaa
agiacaaaaaaaaaaaajaaaaaaegacbaaaaaaaaaaadoaaaaab"
}

SubProgram "gles " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
"!!GLES"
}

SubProgram "glesdesktop " {
Keywords { "DIRECTIONAL" "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
"!!GLES"
}

SubProgram "opengl " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Vector 0 [_Color]
Vector 1 [_ReflectColor]
Float 2 [_ReflectPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_BumpMap] 2D
SetTexture 2 [_Cube] CUBE
SetTexture 3 [_ShadowMapTexture] 2D
SetTexture 4 [unity_Lightmap] 2D
"3.0-!!ARBfp1.0
OPTION ARB_precision_hint_fastest;
# 33 ALU, 5 TEX
PARAM c[4] = { program.local[0..2],
		{ 8, 2, 1 } };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
TEMP R4;
TEX R0.yw, fragment.texcoord[0], texture[1], 2D;
MAD R1.xy, R0.wyzw, c[3].y, -c[3].z;
MUL R0.x, R1.y, R1.y;
MAD R0.x, -R1, R1, -R0;
ADD R0.x, R0, c[3].z;
RSQ R0.x, R0.x;
RCP R1.z, R0.x;
DP3 R0.x, fragment.texcoord[1], R1;
DP3 R0.y, R1, fragment.texcoord[2];
DP3 R0.z, R1, fragment.texcoord[3];
MOV R1.x, fragment.texcoord[1].w;
MOV R1.z, fragment.texcoord[3].w;
MOV R1.y, fragment.texcoord[2].w;
DP3 R0.w, R0, R1;
MUL R0.xyz, R0, R0.w;
MAD R0.xyz, -R0, c[3].y, R1;
TEX R1, R0, texture[2], CUBE;
TEX R0, fragment.texcoord[0], texture[0], 2D;
MUL R1, R0.w, R1;
MUL R2, R1, c[1];
TEX R1, fragment.texcoord[4], texture[4], 2D;
MUL R3.xyz, R1.w, R1;
TXP R4.x, fragment.texcoord[5], texture[3], 2D;
MUL R1.xyz, R1, R4.x;
MUL R3.xyz, R3, c[3].x;
MUL R1.xyz, R1, c[3].y;
MUL R4.xyz, R3, R4.x;
MIN R1.xyz, R3, R1;
MAX R1.xyz, R1, R4;
MUL R2.xyz, R2, c[2].x;
MUL R0.xyz, R0, c[0];
MAD result.color.xyz, R0, R1, R2;
MOV result.color.w, R2;
END
# 33 instructions, 5 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Vector 0 [_Color]
Vector 1 [_ReflectColor]
Float 2 [_ReflectPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_BumpMap] 2D
SetTexture 2 [_Cube] CUBE
SetTexture 3 [_ShadowMapTexture] 2D
SetTexture 4 [unity_Lightmap] 2D
"ps_3_0
; 28 ALU, 5 TEX
dcl_2d s0
dcl_2d s1
dcl_cube s2
dcl_2d s3
dcl_2d s4
def c3, 8.00000000, 2.00000000, -1.00000000, 1.00000000
dcl_texcoord0 v0.xy
dcl_texcoord1 v1
dcl_texcoord2 v2
dcl_texcoord3 v3
dcl_texcoord4 v4.xy
dcl_texcoord5 v5
texld r0.yw, v0, s1
mad_pp r1.xy, r0.wyzw, c3.y, c3.z
mul_pp r0.x, r1.y, r1.y
mad_pp r0.x, -r1, r1, -r0
add_pp r0.x, r0, c3.w
rsq_pp r0.x, r0.x
rcp_pp r1.z, r0.x
dp3_pp r0.x, v1, r1
dp3_pp r0.y, r1, v2
dp3_pp r0.z, r1, v3
mov r1.x, v1.w
mov r1.z, v3.w
mov r1.y, v2.w
dp3 r0.w, r0, r1
mul r0.xyz, r0, r0.w
mad r0.xyz, -r0, c3.y, r1
texld r1, r0, s2
texld r0, v0, s0
mul_pp r1, r0.w, r1
mul_pp r2, r1, c1
texld r1, v4, s4
mul_pp r3.xyz, r1.w, r1
texldp r4.x, v5, s3
mul_pp r1.xyz, r1, r4.x
mul_pp r3.xyz, r3, c3.x
mul_pp r1.xyz, r1, c3.y
mul_pp r4.xyz, r3, r4.x
min_pp r1.xyz, r3, r1
max_pp r1.xyz, r1, r4
mul_pp r2.xyz, r2, c2.x
mul_pp r0.xyz, r0, c0
mad_pp oC0.xyz, r0, r1, r2
mov_pp oC0.w, r2
"
}

SubProgram "xbox360 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Vector 0 [_Color]
Vector 1 [_ReflectColor]
Float 2 [_ReflectPower]
SetTexture 0 [_ShadowMapTexture] 2D
SetTexture 1 [_MainTex] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_Cube] CUBE
SetTexture 4 [unity_Lightmap] 2D
// Shader Timing Estimate, in Cycles/64 pixel vector:
// ALU: 37.33 (28 instructions), vertex: 0, texture: 20,
//   sequencer: 14, interpolator: 24;    7 GPRs, 27 threads,
// Performance (if enough threads): ~37 cycles per vector
// * Texture cycle estimates are assuming an 8bit/component texture with no
//     aniso or trilinear filtering.

"ps_360
backbbaaaaaaabpmaaaaacaiaaaaaaaaaaaaaaceaaaaabjmaaaaabmeaaaaaaaa
aaaaaaaaaaaaabheaaaaaabmaaaaabgippppadaaaaaaaaaiaaaaaabmaaaaaaaa
aaaaabgbaaaaaalmaaadaaacaaabaaaaaaaaaamiaaaaaaaaaaaaaaniaaacaaaa
aaabaaaaaaaaaaoaaaaaaaaaaaaaaapaaaadaaadaaabaaaaaaaaaapiaaaaaaaa
aaaaabaiaaadaaabaaabaaaaaaaaaamiaaaaaaaaaaaaabbbaaacaaabaaabaaaa
aaaaaaoaaaaaaaaaaaaaabbpaaacaaacaaabaaaaaaaaabdaaaaaaaaaaaaaabea
aaadaaaaaaabaaaaaaaaaamiaaaaaaaaaaaaabfcaaadaaaeaaabaaaaaaaaaami
aaaaaaaafpechfgnhaengbhaaaklklklaaaeaaamaaabaaabaaabaaaaaaaaaaaa
fpedgpgmgphcaaklaaabaaadaaabaaaeaaabaaaaaaaaaaaafpedhfgcgfaaklkl
aaaeaaaoaaabaaabaaabaaaaaaaaaaaafpengbgjgofegfhiaafpfcgfgggmgfgd
heedgpgmgphcaafpfcgfgggmgfgdhefagphhgfhcaaklklklaaaaaaadaaabaaab
aaabaaaaaaaaaaaafpfdgigbgegphhengbhafegfhihehfhcgfaahfgogjhehjfp
emgjghgihegngbhaaahahdfpddfpdaaadccodacodcdadddfddcodaaaaaaaaaaa
aaaaaaabaaaaaaaaaaaaaaaaaaaaaabeabpmaabaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaeaaaaaabmibaaaagaaaaaaaaaeaaaaaaaaaaaafamgaadpaadp
aaaaaaabaaaadafaaaaapbfbaaaapcfcaaaapdfdaaaadefeaaaapfffaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaebaaaaaa
eaaaaaaalpiaaaaadpmaaaaadpiaaaaaaaaaaaaaaaaaaaaaaaaaaaaaacfagaae
gaakbcaabcaaaaaaaaaagabafabgbcaabcaaabfaaaaaaaaagablmeaabcaaaaaa
aaaaeacbaaaaccaaaaaaaaaaemeaaaaaaaaaaablocaaaaafmiamaaaaaamgkmaa
obaaafaaliaagaabbpbpppmhaaaaeaaabacaaaabbpbppghpaaaaeaaamiafaaag
aamhlbmgilaapopomiaeaaaaaegogogmnbagagppkaiaagaaaaaaaamgocaaaaia
miabaaafaamploaapaagabaamiacaaafaamploaapaagadaabeaeaaafaamplolb
paagacafameeagaaaamgblblobafacadmiabaaagaagmblmgolafabaamiamaaae
aakmigaaoaagagaaaaeaaaaaaaaaaamgocaaaaaemiahaaafaamgmaaaobaaafaa
beacaaacafmgbllboaafacafaeebacacaegmblbloaafabadmiapaaabaakgmnaa
pcacacaaemeeaaacaablblmgocababibmiadaaacaagnmgblmlabaapojadicaeb
bpbppgiiaaaamaaababiaaabbpbppgecaaaaeaaabaeifaibbpbppgiiaaaaeaaa
kmbaabaaaaaaaaehmcaaaapobecpaaacaaaabllbobacaaaamiaiiaaaaablblaa
kbacabaakiboadabaapmpmebibacabaakichadacaagmmaicmbabafaakiehadae
aamalbmambacagaakmboaaaaaablpmebmbaeafackmchaaacaamabfecmdacaaac
kmehaaacaamamaedmcaeacacmiahiaaaaamamamaoladacaaaaaaaaaaaaaaaaaa
aaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
Vector 0 [_Color]
Vector 1 [_ReflectColor]
Float 2 [_ReflectPower]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_BumpMap] 2D
SetTexture 2 [_Cube] CUBE
SetTexture 3 [_ShadowMapTexture] 2D
SetTexture 4 [unity_Lightmap] 2D
"sce_fp_rsx // 39 instructions using 4 registers
[Configuration]
24
ffffffff000fc020003fffd1000000000000840004000000
[Offsets]
3
_Color 1 0
00000100
_ReflectColor 1 0
00000220
_ReflectPower 1 0
00000250
[Microcode]
624
1e021709c8011c9dc8000001c8003fe10e800240fe041c9dc8043001c8000001
22041807c8011c9dc8000001c8003fe10e820240c8041c9d00081000c8000001
0e820840c9001c9dc9040001c80000010e800240c9001c9d00080000c8000001
94021702c8011c9dc8000001c8003fe11c8a094021041c9d21000001c8000001
9e061700c8011c9dc8000001c8003fe106880440ce041c9d00020000aa020000
000040000000bf800000000000000000be860140c8011c9dc8000001c8003fe1
028a0140ff0c1c9dc8000001c800000110880240ab101c9cab100000c8000001
de820140c8011c9dc8000001c8003fe10e800240c80c1c9dc8020001c8000001
000000000000000000000000000000001080044001101c9e01100000c9100003
fe840140c8011c9dc8000001c8003fe10e800240c9001c9df3140001c8000001
048a0140ff041c9dc8000001c8000001088a0140ff081c9dc8000001c8000001
10840340c9001c9d00020000c800000100003f80000000000000000000000000
08883b40ff083c9dff080001c800000102860540c90c1c9dc9100001c8000001
08860540c9101c9dc9080001c800000104860540c9101c9dc9040001c8000001
1e7e7e00c8001c9dc8000001c80000011e7e7d00c8001c9dc8000001c8000001
08000500c90c1c9dc9141001c80000010e020400c90c1c9f54000001c9140001
1e021704c8041c9dc8000001c80000011e820240c8041c9dc8020001c8000001
000000000000000000000000000000001e820240c9041c9dfe0c0001c8000001
0e800440c9041c9d00020000c900000100000000000000000000000000000000
10810140c9041c9dc8000001c8000001
"
}

SubProgram "d3d11 " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
ConstBuffer "$Globals" 192 // 148 used size, 11 vars
Vector 112 [_Color] 4
Vector 128 [_ReflectColor] 4
Float 144 [_ReflectPower]
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 1
SetTexture 1 [_BumpMap] 2D 2
SetTexture 2 [_Cube] CUBE 3
SetTexture 3 [_ShadowMapTexture] 2D 0
SetTexture 4 [unity_Lightmap] 2D 4
// 33 instructions, 3 temp regs, 0 temp arrays:
// ALU 19 float, 0 int, 0 uint
// TEX 5 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0
eefiecedpanmdlmjloafiobimlfbnbhknbmeppofabaaaaaacaagaaaaadaaaaaa
cmaaaaaapmaaaaaadaabaaaaejfdeheomiaaaaaaahaaaaaaaiaaaaaalaaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaalmaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaadadaaaalmaaaaaaaeaaaaaaaaaaaaaaadaaaaaaabaaaaaa
amamaaaalmaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaaapapaaaalmaaaaaa
acaaaaaaaaaaaaaaadaaaaaaadaaaaaaapapaaaalmaaaaaaadaaaaaaaaaaaaaa
adaaaaaaaeaaaaaaapapaaaalmaaaaaaafaaaaaaaaaaaaaaadaaaaaaafaaaaaa
apalaaaafdfgfpfaepfdejfeejepeoaafeeffiedepepfceeaaklklklepfdeheo
cmaaaaaaabaaaaaaaiaaaaaacaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaaaaaaaaa
apaaaaaafdfgfpfegbhcghgfheaaklklfdeieefcoiaeaaaaeaaaaaaadkabaaaa
fjaaaaaeegiocaaaaaaaaaaaakaaaaaafkaaaaadaagabaaaaaaaaaaafkaaaaad
aagabaaaabaaaaaafkaaaaadaagabaaaacaaaaaafkaaaaadaagabaaaadaaaaaa
fkaaaaadaagabaaaaeaaaaaafibiaaaeaahabaaaaaaaaaaaffffaaaafibiaaae
aahabaaaabaaaaaaffffaaaafidaaaaeaahabaaaacaaaaaaffffaaaafibiaaae
aahabaaaadaaaaaaffffaaaafibiaaaeaahabaaaaeaaaaaaffffaaaagcbaaaad
dcbabaaaabaaaaaagcbaaaadmcbabaaaabaaaaaagcbaaaadpcbabaaaacaaaaaa
gcbaaaadpcbabaaaadaaaaaagcbaaaadpcbabaaaaeaaaaaagcbaaaadlcbabaaa
afaaaaaagfaaaaadpccabaaaaaaaaaaagiaaaaacadaaaaaaaoaaaaahdcaabaaa
aaaaaaaaegbabaaaafaaaaaapgbpbaaaafaaaaaaefaaaaajpcaabaaaaaaaaaaa
egaabaaaaaaaaaaaeghobaaaadaaaaaaaagabaaaaaaaaaaaaaaaaaahccaabaaa
aaaaaaaaakaabaaaaaaaaaaaakaabaaaaaaaaaaaefaaaaajpcaabaaaabaaaaaa
ogbkbaaaabaaaaaaeghobaaaaeaaaaaaaagabaaaaeaaaaaadiaaaaahocaabaaa
aaaaaaaafgafbaaaaaaaaaaaagajbaaaabaaaaaadiaaaaahicaabaaaabaaaaaa
dkaabaaaabaaaaaaabeaaaaaaaaaaaebdiaaaaahhcaabaaaabaaaaaaegacbaaa
abaaaaaapgapbaaaabaaaaaaddaaaaahocaabaaaaaaaaaaafgaobaaaaaaaaaaa
agajbaaaabaaaaaadiaaaaahhcaabaaaabaaaaaaagaabaaaaaaaaaaaegacbaaa
abaaaaaadeaaaaahhcaabaaaaaaaaaaajgahbaaaaaaaaaaaegacbaaaabaaaaaa
efaaaaajpcaabaaaabaaaaaaegbabaaaabaaaaaaeghobaaaabaaaaaaaagabaaa
acaaaaaadcaaaaapdcaabaaaabaaaaaahgapbaaaabaaaaaaaceaaaaaaaaaaaea
aaaaaaeaaaaaaaaaaaaaaaaaaceaaaaaaaaaialpaaaaialpaaaaaaaaaaaaaaaa
dcaaaaakicaabaaaaaaaaaaaakaabaiaebaaaaaaabaaaaaaakaabaaaabaaaaaa
abeaaaaaaaaaiadpdcaaaaakicaabaaaaaaaaaaabkaabaiaebaaaaaaabaaaaaa
bkaabaaaabaaaaaadkaabaaaaaaaaaaaelaaaaafecaabaaaabaaaaaadkaabaaa
aaaaaaaabaaaaaahbcaabaaaacaaaaaaegbcbaaaacaaaaaaegacbaaaabaaaaaa
baaaaaahccaabaaaacaaaaaaegbcbaaaadaaaaaaegacbaaaabaaaaaabaaaaaah
ecaabaaaacaaaaaaegbcbaaaaeaaaaaaegacbaaaabaaaaaadgaaaaafbcaabaaa
abaaaaaadkbabaaaacaaaaaadgaaaaafccaabaaaabaaaaaadkbabaaaadaaaaaa
dgaaaaafecaabaaaabaaaaaadkbabaaaaeaaaaaabaaaaaahicaabaaaaaaaaaaa
egacbaaaabaaaaaaegacbaaaacaaaaaaaaaaaaahicaabaaaaaaaaaaadkaabaaa
aaaaaaaadkaabaaaaaaaaaaadcaaaaakhcaabaaaabaaaaaaegacbaaaacaaaaaa
pgapbaiaebaaaaaaaaaaaaaaegacbaaaabaaaaaaefaaaaajpcaabaaaabaaaaaa
egacbaaaabaaaaaaeghobaaaacaaaaaaaagabaaaadaaaaaaefaaaaajpcaabaaa
acaaaaaaegbabaaaabaaaaaaeghobaaaaaaaaaaaaagabaaaabaaaaaadiaaaaah
pcaabaaaabaaaaaaegaobaaaabaaaaaapgapbaaaacaaaaaadiaaaaaihcaabaaa
acaaaaaaegacbaaaacaaaaaaegiccaaaaaaaaaaaahaaaaaadiaaaaaihcaabaaa
abaaaaaaegacbaaaabaaaaaaegiccaaaaaaaaaaaaiaaaaaadiaaaaaiiccabaaa
aaaaaaaadkaabaaaabaaaaaadkiacaaaaaaaaaaaaiaaaaaadiaaaaaihcaabaaa
abaaaaaaegacbaaaabaaaaaaagiacaaaaaaaaaaaajaaaaaadcaaaaajhccabaaa
aaaaaaaaegacbaaaacaaaaaaegacbaaaaaaaaaaaegacbaaaabaaaaaadoaaaaab
"
}

SubProgram "gles " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
"!!GLES"
}

SubProgram "glesdesktop " {
Keywords { "DIRECTIONAL" "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "SHADOWS_SCREEN" }
"!!GLES"
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
dp3 o2.y, r0, r2
dp3 o2.z, v2, r0
dp3 o2.x, r0, v1
dp4 r0.w, v0, c7
dp4 r0.z, v0, c6
dp4 r0.x, v0, c4
dp4 r0.y, v0, c5
dp3 o3.y, r2, r3
dp3 o3.z, v2, r3
dp3 o3.x, v1, r3
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
ConstBuffer "$Globals" 176 // 176 used size, 10 vars
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
eefiecedmpnffcfnddpfijlfmbeiokghpcckhcpcabaaaaaapiagaaaaadaaaaaa
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
uniform lowp vec4 _ReflectColor;
uniform lowp vec4 _Color;
uniform samplerCube _Cube;
uniform sampler2D _SpecMap;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform sampler2D _LightTexture0;
uniform lowp vec4 _LightColor0;
void main ()
{
  lowp vec4 c_1;
  lowp vec3 lightDir_2;
  highp vec3 tmpvar_3;
  mediump vec3 tmpvar_4;
  mediump vec3 tmpvar_5;
  mediump float tmpvar_6;
  mediump vec4 spec_7;
  lowp vec4 tmpvar_8;
  tmpvar_8 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_9;
  tmpvar_9 = (tmpvar_8.xyz * _Color.xyz);
  tmpvar_4 = tmpvar_9;
  lowp vec4 tmpvar_10;
  tmpvar_10 = texture2D (_SpecMap, xlv_TEXCOORD0);
  spec_7 = tmpvar_10;
  mediump vec3 tmpvar_11;
  tmpvar_11 = (spec_7.xyz * _Gloss);
  lowp vec3 tmpvar_12;
  tmpvar_12 = ((texture2D (_BumpMap, xlv_TEXCOORD0).xyz * 2.0) - 1.0);
  tmpvar_5 = tmpvar_12;
  lowp float tmpvar_13;
  tmpvar_13 = ((textureCube (_Cube, tmpvar_3) * tmpvar_8.w).w * _ReflectColor.w);
  tmpvar_6 = tmpvar_13;
  mediump vec3 tmpvar_14;
  tmpvar_14 = normalize(xlv_TEXCOORD1);
  lightDir_2 = tmpvar_14;
  highp float tmpvar_15;
  tmpvar_15 = dot (xlv_TEXCOORD3, xlv_TEXCOORD3);
  lowp vec4 tmpvar_16;
  tmpvar_16 = texture2D (_LightTexture0, vec2(tmpvar_15));
  mediump vec3 lightDir_17;
  lightDir_17 = lightDir_2;
  mediump float atten_18;
  atten_18 = tmpvar_16.w;
  mediump vec4 c_19;
  mediump vec3 specCol_20;
  highp float nh_21;
  mediump float tmpvar_22;
  tmpvar_22 = max (0.0, dot (tmpvar_5, normalize((lightDir_17 + normalize(xlv_TEXCOORD2)))));
  nh_21 = tmpvar_22;
  mediump float arg1_23;
  arg1_23 = (32.0 * _Shininess);
  highp vec3 tmpvar_24;
  tmpvar_24 = (pow (nh_21, arg1_23) * tmpvar_11);
  specCol_20 = tmpvar_24;
  c_19.xyz = ((((tmpvar_4 * _LightColor0.xyz) * max (0.0, dot (tmpvar_5, lightDir_17))) + (_LightColor0.xyz * specCol_20)) * (atten_18 * 2.0));
  c_19.w = tmpvar_6;
  c_1.xyz = c_19.xyz;
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
uniform lowp vec4 _ReflectColor;
uniform lowp vec4 _Color;
uniform samplerCube _Cube;
uniform sampler2D _SpecMap;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform sampler2D _LightTexture0;
uniform lowp vec4 _LightColor0;
void main ()
{
  lowp vec4 c_1;
  lowp vec3 lightDir_2;
  highp vec3 tmpvar_3;
  mediump vec3 tmpvar_4;
  mediump vec3 tmpvar_5;
  mediump float tmpvar_6;
  mediump vec4 spec_7;
  lowp vec4 tmpvar_8;
  tmpvar_8 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_9;
  tmpvar_9 = (tmpvar_8.xyz * _Color.xyz);
  tmpvar_4 = tmpvar_9;
  lowp vec4 tmpvar_10;
  tmpvar_10 = texture2D (_SpecMap, xlv_TEXCOORD0);
  spec_7 = tmpvar_10;
  mediump vec3 tmpvar_11;
  tmpvar_11 = (spec_7.xyz * _Gloss);
  lowp vec3 normal_12;
  normal_12.xy = ((texture2D (_BumpMap, xlv_TEXCOORD0).wy * 2.0) - 1.0);
  normal_12.z = sqrt(((1.0 - (normal_12.x * normal_12.x)) - (normal_12.y * normal_12.y)));
  tmpvar_5 = normal_12;
  lowp float tmpvar_13;
  tmpvar_13 = ((textureCube (_Cube, tmpvar_3) * tmpvar_8.w).w * _ReflectColor.w);
  tmpvar_6 = tmpvar_13;
  mediump vec3 tmpvar_14;
  tmpvar_14 = normalize(xlv_TEXCOORD1);
  lightDir_2 = tmpvar_14;
  highp float tmpvar_15;
  tmpvar_15 = dot (xlv_TEXCOORD3, xlv_TEXCOORD3);
  lowp vec4 tmpvar_16;
  tmpvar_16 = texture2D (_LightTexture0, vec2(tmpvar_15));
  mediump vec3 lightDir_17;
  lightDir_17 = lightDir_2;
  mediump float atten_18;
  atten_18 = tmpvar_16.w;
  mediump vec4 c_19;
  mediump vec3 specCol_20;
  highp float nh_21;
  mediump float tmpvar_22;
  tmpvar_22 = max (0.0, dot (tmpvar_5, normalize((lightDir_17 + normalize(xlv_TEXCOORD2)))));
  nh_21 = tmpvar_22;
  mediump float arg1_23;
  arg1_23 = (32.0 * _Shininess);
  highp vec3 tmpvar_24;
  tmpvar_24 = (pow (nh_21, arg1_23) * tmpvar_11);
  specCol_20 = tmpvar_24;
  c_19.xyz = ((((tmpvar_4 * _LightColor0.xyz) * max (0.0, dot (tmpvar_5, lightDir_17))) + (_LightColor0.xyz * specCol_20)) * (atten_18 * 2.0));
  c_19.w = tmpvar_6;
  c_1.xyz = c_19.xyz;
  c_1.w = 0.0;
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
dp3 o2.y, r4, r2
dp3 o3.y, r2, r3
dp3 o2.z, v2, r4
dp3 o2.x, r4, v1
dp3 o3.z, v2, r3
dp3 o3.x, v1, r3
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
ConstBuffer "$Globals" 112 // 112 used size, 9 vars
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
eefiecedlhabnpifpjcihdlkepdolfgegicemckaabaaaaaahiafaaaaadaaaaaa
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
  lowp vec3 lightDir_2;
  highp vec3 tmpvar_3;
  mediump vec3 tmpvar_4;
  mediump vec3 tmpvar_5;
  mediump float tmpvar_6;
  mediump vec4 spec_7;
  lowp vec4 tmpvar_8;
  tmpvar_8 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_9;
  tmpvar_9 = (tmpvar_8.xyz * _Color.xyz);
  tmpvar_4 = tmpvar_9;
  lowp vec4 tmpvar_10;
  tmpvar_10 = texture2D (_SpecMap, xlv_TEXCOORD0);
  spec_7 = tmpvar_10;
  mediump vec3 tmpvar_11;
  tmpvar_11 = (spec_7.xyz * _Gloss);
  lowp vec3 tmpvar_12;
  tmpvar_12 = ((texture2D (_BumpMap, xlv_TEXCOORD0).xyz * 2.0) - 1.0);
  tmpvar_5 = tmpvar_12;
  lowp float tmpvar_13;
  tmpvar_13 = ((textureCube (_Cube, tmpvar_3) * tmpvar_8.w).w * _ReflectColor.w);
  tmpvar_6 = tmpvar_13;
  lightDir_2 = xlv_TEXCOORD1;
  mediump vec3 lightDir_14;
  lightDir_14 = lightDir_2;
  mediump vec4 c_15;
  mediump vec3 specCol_16;
  highp float nh_17;
  mediump float tmpvar_18;
  tmpvar_18 = max (0.0, dot (tmpvar_5, normalize((lightDir_14 + normalize(xlv_TEXCOORD2)))));
  nh_17 = tmpvar_18;
  mediump float arg1_19;
  arg1_19 = (32.0 * _Shininess);
  highp vec3 tmpvar_20;
  tmpvar_20 = (pow (nh_17, arg1_19) * tmpvar_11);
  specCol_16 = tmpvar_20;
  c_15.xyz = ((((tmpvar_4 * _LightColor0.xyz) * max (0.0, dot (tmpvar_5, lightDir_14))) + (_LightColor0.xyz * specCol_16)) * 2.0);
  c_15.w = tmpvar_6;
  c_1.xyz = c_15.xyz;
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
  lowp vec3 lightDir_2;
  highp vec3 tmpvar_3;
  mediump vec3 tmpvar_4;
  mediump vec3 tmpvar_5;
  mediump float tmpvar_6;
  mediump vec4 spec_7;
  lowp vec4 tmpvar_8;
  tmpvar_8 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_9;
  tmpvar_9 = (tmpvar_8.xyz * _Color.xyz);
  tmpvar_4 = tmpvar_9;
  lowp vec4 tmpvar_10;
  tmpvar_10 = texture2D (_SpecMap, xlv_TEXCOORD0);
  spec_7 = tmpvar_10;
  mediump vec3 tmpvar_11;
  tmpvar_11 = (spec_7.xyz * _Gloss);
  lowp vec3 normal_12;
  normal_12.xy = ((texture2D (_BumpMap, xlv_TEXCOORD0).wy * 2.0) - 1.0);
  normal_12.z = sqrt(((1.0 - (normal_12.x * normal_12.x)) - (normal_12.y * normal_12.y)));
  tmpvar_5 = normal_12;
  lowp float tmpvar_13;
  tmpvar_13 = ((textureCube (_Cube, tmpvar_3) * tmpvar_8.w).w * _ReflectColor.w);
  tmpvar_6 = tmpvar_13;
  lightDir_2 = xlv_TEXCOORD1;
  mediump vec3 lightDir_14;
  lightDir_14 = lightDir_2;
  mediump vec4 c_15;
  mediump vec3 specCol_16;
  highp float nh_17;
  mediump float tmpvar_18;
  tmpvar_18 = max (0.0, dot (tmpvar_5, normalize((lightDir_14 + normalize(xlv_TEXCOORD2)))));
  nh_17 = tmpvar_18;
  mediump float arg1_19;
  arg1_19 = (32.0 * _Shininess);
  highp vec3 tmpvar_20;
  tmpvar_20 = (pow (nh_17, arg1_19) * tmpvar_11);
  specCol_16 = tmpvar_20;
  c_15.xyz = ((((tmpvar_4 * _LightColor0.xyz) * max (0.0, dot (tmpvar_5, lightDir_14))) + (_LightColor0.xyz * specCol_16)) * 2.0);
  c_15.w = tmpvar_6;
  c_1.xyz = c_15.xyz;
  c_1.w = 0.0;
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
dp3 o2.y, r0, r2
dp3 o2.z, v2, r0
dp3 o2.x, r0, v1
dp4 r0.z, v0, c6
dp4 r0.x, v0, c4
dp4 r0.y, v0, c5
dp3 o3.y, r2, r3
dp3 o3.z, v2, r3
dp3 o3.x, v1, r3
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
ConstBuffer "$Globals" 176 // 176 used size, 10 vars
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
eefiecedllmbfcpcemleoeeccjddlfdplaiccggnabaaaaaapiagaaaaadaaaaaa
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
uniform lowp vec4 _ReflectColor;
uniform lowp vec4 _Color;
uniform samplerCube _Cube;
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
  highp vec3 tmpvar_3;
  mediump vec3 tmpvar_4;
  mediump vec3 tmpvar_5;
  mediump float tmpvar_6;
  mediump vec4 spec_7;
  lowp vec4 tmpvar_8;
  tmpvar_8 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_9;
  tmpvar_9 = (tmpvar_8.xyz * _Color.xyz);
  tmpvar_4 = tmpvar_9;
  lowp vec4 tmpvar_10;
  tmpvar_10 = texture2D (_SpecMap, xlv_TEXCOORD0);
  spec_7 = tmpvar_10;
  mediump vec3 tmpvar_11;
  tmpvar_11 = (spec_7.xyz * _Gloss);
  lowp vec3 tmpvar_12;
  tmpvar_12 = ((texture2D (_BumpMap, xlv_TEXCOORD0).xyz * 2.0) - 1.0);
  tmpvar_5 = tmpvar_12;
  lowp float tmpvar_13;
  tmpvar_13 = ((textureCube (_Cube, tmpvar_3) * tmpvar_8.w).w * _ReflectColor.w);
  tmpvar_6 = tmpvar_13;
  mediump vec3 tmpvar_14;
  tmpvar_14 = normalize(xlv_TEXCOORD1);
  lightDir_2 = tmpvar_14;
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
  mediump float atten_20;
  atten_20 = ((float((xlv_TEXCOORD3.z > 0.0)) * tmpvar_15.w) * tmpvar_18.w);
  mediump vec4 c_21;
  mediump vec3 specCol_22;
  highp float nh_23;
  mediump float tmpvar_24;
  tmpvar_24 = max (0.0, dot (tmpvar_5, normalize((lightDir_19 + normalize(xlv_TEXCOORD2)))));
  nh_23 = tmpvar_24;
  mediump float arg1_25;
  arg1_25 = (32.0 * _Shininess);
  highp vec3 tmpvar_26;
  tmpvar_26 = (pow (nh_23, arg1_25) * tmpvar_11);
  specCol_22 = tmpvar_26;
  c_21.xyz = ((((tmpvar_4 * _LightColor0.xyz) * max (0.0, dot (tmpvar_5, lightDir_19))) + (_LightColor0.xyz * specCol_22)) * (atten_20 * 2.0));
  c_21.w = tmpvar_6;
  c_1.xyz = c_21.xyz;
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
uniform lowp vec4 _ReflectColor;
uniform lowp vec4 _Color;
uniform samplerCube _Cube;
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
  highp vec3 tmpvar_3;
  mediump vec3 tmpvar_4;
  mediump vec3 tmpvar_5;
  mediump float tmpvar_6;
  mediump vec4 spec_7;
  lowp vec4 tmpvar_8;
  tmpvar_8 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_9;
  tmpvar_9 = (tmpvar_8.xyz * _Color.xyz);
  tmpvar_4 = tmpvar_9;
  lowp vec4 tmpvar_10;
  tmpvar_10 = texture2D (_SpecMap, xlv_TEXCOORD0);
  spec_7 = tmpvar_10;
  mediump vec3 tmpvar_11;
  tmpvar_11 = (spec_7.xyz * _Gloss);
  lowp vec3 normal_12;
  normal_12.xy = ((texture2D (_BumpMap, xlv_TEXCOORD0).wy * 2.0) - 1.0);
  normal_12.z = sqrt(((1.0 - (normal_12.x * normal_12.x)) - (normal_12.y * normal_12.y)));
  tmpvar_5 = normal_12;
  lowp float tmpvar_13;
  tmpvar_13 = ((textureCube (_Cube, tmpvar_3) * tmpvar_8.w).w * _ReflectColor.w);
  tmpvar_6 = tmpvar_13;
  mediump vec3 tmpvar_14;
  tmpvar_14 = normalize(xlv_TEXCOORD1);
  lightDir_2 = tmpvar_14;
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
  mediump float atten_20;
  atten_20 = ((float((xlv_TEXCOORD3.z > 0.0)) * tmpvar_15.w) * tmpvar_18.w);
  mediump vec4 c_21;
  mediump vec3 specCol_22;
  highp float nh_23;
  mediump float tmpvar_24;
  tmpvar_24 = max (0.0, dot (tmpvar_5, normalize((lightDir_19 + normalize(xlv_TEXCOORD2)))));
  nh_23 = tmpvar_24;
  mediump float arg1_25;
  arg1_25 = (32.0 * _Shininess);
  highp vec3 tmpvar_26;
  tmpvar_26 = (pow (nh_23, arg1_25) * tmpvar_11);
  specCol_22 = tmpvar_26;
  c_21.xyz = ((((tmpvar_4 * _LightColor0.xyz) * max (0.0, dot (tmpvar_5, lightDir_19))) + (_LightColor0.xyz * specCol_22)) * (atten_20 * 2.0));
  c_21.w = tmpvar_6;
  c_1.xyz = c_21.xyz;
  c_1.w = 0.0;
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
dp3 o2.y, r0, r2
dp3 o2.z, v2, r0
dp3 o2.x, r0, v1
dp4 r0.w, v0, c7
dp4 r0.z, v0, c6
dp4 r0.x, v0, c4
dp4 r0.y, v0, c5
dp3 o3.y, r2, r3
dp3 o3.z, v2, r3
dp3 o3.x, v1, r3
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
ConstBuffer "$Globals" 176 // 176 used size, 10 vars
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
eefiecedmpnffcfnddpfijlfmbeiokghpcckhcpcabaaaaaapiagaaaaadaaaaaa
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
uniform lowp vec4 _ReflectColor;
uniform lowp vec4 _Color;
uniform samplerCube _Cube;
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
  highp vec3 tmpvar_3;
  mediump vec3 tmpvar_4;
  mediump vec3 tmpvar_5;
  mediump float tmpvar_6;
  mediump vec4 spec_7;
  lowp vec4 tmpvar_8;
  tmpvar_8 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_9;
  tmpvar_9 = (tmpvar_8.xyz * _Color.xyz);
  tmpvar_4 = tmpvar_9;
  lowp vec4 tmpvar_10;
  tmpvar_10 = texture2D (_SpecMap, xlv_TEXCOORD0);
  spec_7 = tmpvar_10;
  mediump vec3 tmpvar_11;
  tmpvar_11 = (spec_7.xyz * _Gloss);
  lowp vec3 tmpvar_12;
  tmpvar_12 = ((texture2D (_BumpMap, xlv_TEXCOORD0).xyz * 2.0) - 1.0);
  tmpvar_5 = tmpvar_12;
  lowp float tmpvar_13;
  tmpvar_13 = ((textureCube (_Cube, tmpvar_3) * tmpvar_8.w).w * _ReflectColor.w);
  tmpvar_6 = tmpvar_13;
  mediump vec3 tmpvar_14;
  tmpvar_14 = normalize(xlv_TEXCOORD1);
  lightDir_2 = tmpvar_14;
  highp float tmpvar_15;
  tmpvar_15 = dot (xlv_TEXCOORD3, xlv_TEXCOORD3);
  lowp vec4 tmpvar_16;
  tmpvar_16 = texture2D (_LightTextureB0, vec2(tmpvar_15));
  lowp vec4 tmpvar_17;
  tmpvar_17 = textureCube (_LightTexture0, xlv_TEXCOORD3);
  mediump vec3 lightDir_18;
  lightDir_18 = lightDir_2;
  mediump float atten_19;
  atten_19 = (tmpvar_16.w * tmpvar_17.w);
  mediump vec4 c_20;
  mediump vec3 specCol_21;
  highp float nh_22;
  mediump float tmpvar_23;
  tmpvar_23 = max (0.0, dot (tmpvar_5, normalize((lightDir_18 + normalize(xlv_TEXCOORD2)))));
  nh_22 = tmpvar_23;
  mediump float arg1_24;
  arg1_24 = (32.0 * _Shininess);
  highp vec3 tmpvar_25;
  tmpvar_25 = (pow (nh_22, arg1_24) * tmpvar_11);
  specCol_21 = tmpvar_25;
  c_20.xyz = ((((tmpvar_4 * _LightColor0.xyz) * max (0.0, dot (tmpvar_5, lightDir_18))) + (_LightColor0.xyz * specCol_21)) * (atten_19 * 2.0));
  c_20.w = tmpvar_6;
  c_1.xyz = c_20.xyz;
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
uniform lowp vec4 _ReflectColor;
uniform lowp vec4 _Color;
uniform samplerCube _Cube;
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
  highp vec3 tmpvar_3;
  mediump vec3 tmpvar_4;
  mediump vec3 tmpvar_5;
  mediump float tmpvar_6;
  mediump vec4 spec_7;
  lowp vec4 tmpvar_8;
  tmpvar_8 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_9;
  tmpvar_9 = (tmpvar_8.xyz * _Color.xyz);
  tmpvar_4 = tmpvar_9;
  lowp vec4 tmpvar_10;
  tmpvar_10 = texture2D (_SpecMap, xlv_TEXCOORD0);
  spec_7 = tmpvar_10;
  mediump vec3 tmpvar_11;
  tmpvar_11 = (spec_7.xyz * _Gloss);
  lowp vec3 normal_12;
  normal_12.xy = ((texture2D (_BumpMap, xlv_TEXCOORD0).wy * 2.0) - 1.0);
  normal_12.z = sqrt(((1.0 - (normal_12.x * normal_12.x)) - (normal_12.y * normal_12.y)));
  tmpvar_5 = normal_12;
  lowp float tmpvar_13;
  tmpvar_13 = ((textureCube (_Cube, tmpvar_3) * tmpvar_8.w).w * _ReflectColor.w);
  tmpvar_6 = tmpvar_13;
  mediump vec3 tmpvar_14;
  tmpvar_14 = normalize(xlv_TEXCOORD1);
  lightDir_2 = tmpvar_14;
  highp float tmpvar_15;
  tmpvar_15 = dot (xlv_TEXCOORD3, xlv_TEXCOORD3);
  lowp vec4 tmpvar_16;
  tmpvar_16 = texture2D (_LightTextureB0, vec2(tmpvar_15));
  lowp vec4 tmpvar_17;
  tmpvar_17 = textureCube (_LightTexture0, xlv_TEXCOORD3);
  mediump vec3 lightDir_18;
  lightDir_18 = lightDir_2;
  mediump float atten_19;
  atten_19 = (tmpvar_16.w * tmpvar_17.w);
  mediump vec4 c_20;
  mediump vec3 specCol_21;
  highp float nh_22;
  mediump float tmpvar_23;
  tmpvar_23 = max (0.0, dot (tmpvar_5, normalize((lightDir_18 + normalize(xlv_TEXCOORD2)))));
  nh_22 = tmpvar_23;
  mediump float arg1_24;
  arg1_24 = (32.0 * _Shininess);
  highp vec3 tmpvar_25;
  tmpvar_25 = (pow (nh_22, arg1_24) * tmpvar_11);
  specCol_21 = tmpvar_25;
  c_20.xyz = ((((tmpvar_4 * _LightColor0.xyz) * max (0.0, dot (tmpvar_5, lightDir_18))) + (_LightColor0.xyz * specCol_21)) * (atten_19 * 2.0));
  c_20.w = tmpvar_6;
  c_1.xyz = c_20.xyz;
  c_1.w = 0.0;
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
dp3 o2.y, r4, r2
dp3 o3.y, r2, r3
dp3 o2.z, v2, r4
dp3 o2.x, r4, v1
dp3 o3.z, v2, r3
dp3 o3.x, v1, r3
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
ConstBuffer "$Globals" 176 // 176 used size, 10 vars
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
eefiecedehmeppbkdommeghfffoogjjfffjojmkoabaaaaaammagaaaaadaaaaaa
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
uniform lowp vec4 _ReflectColor;
uniform lowp vec4 _Color;
uniform samplerCube _Cube;
uniform sampler2D _SpecMap;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform sampler2D _LightTexture0;
uniform lowp vec4 _LightColor0;
void main ()
{
  lowp vec4 c_1;
  lowp vec3 lightDir_2;
  highp vec3 tmpvar_3;
  mediump vec3 tmpvar_4;
  mediump vec3 tmpvar_5;
  mediump float tmpvar_6;
  mediump vec4 spec_7;
  lowp vec4 tmpvar_8;
  tmpvar_8 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_9;
  tmpvar_9 = (tmpvar_8.xyz * _Color.xyz);
  tmpvar_4 = tmpvar_9;
  lowp vec4 tmpvar_10;
  tmpvar_10 = texture2D (_SpecMap, xlv_TEXCOORD0);
  spec_7 = tmpvar_10;
  mediump vec3 tmpvar_11;
  tmpvar_11 = (spec_7.xyz * _Gloss);
  lowp vec3 tmpvar_12;
  tmpvar_12 = ((texture2D (_BumpMap, xlv_TEXCOORD0).xyz * 2.0) - 1.0);
  tmpvar_5 = tmpvar_12;
  lowp float tmpvar_13;
  tmpvar_13 = ((textureCube (_Cube, tmpvar_3) * tmpvar_8.w).w * _ReflectColor.w);
  tmpvar_6 = tmpvar_13;
  lightDir_2 = xlv_TEXCOORD1;
  lowp vec4 tmpvar_14;
  tmpvar_14 = texture2D (_LightTexture0, xlv_TEXCOORD3);
  mediump vec3 lightDir_15;
  lightDir_15 = lightDir_2;
  mediump float atten_16;
  atten_16 = tmpvar_14.w;
  mediump vec4 c_17;
  mediump vec3 specCol_18;
  highp float nh_19;
  mediump float tmpvar_20;
  tmpvar_20 = max (0.0, dot (tmpvar_5, normalize((lightDir_15 + normalize(xlv_TEXCOORD2)))));
  nh_19 = tmpvar_20;
  mediump float arg1_21;
  arg1_21 = (32.0 * _Shininess);
  highp vec3 tmpvar_22;
  tmpvar_22 = (pow (nh_19, arg1_21) * tmpvar_11);
  specCol_18 = tmpvar_22;
  c_17.xyz = ((((tmpvar_4 * _LightColor0.xyz) * max (0.0, dot (tmpvar_5, lightDir_15))) + (_LightColor0.xyz * specCol_18)) * (atten_16 * 2.0));
  c_17.w = tmpvar_6;
  c_1.xyz = c_17.xyz;
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
uniform lowp vec4 _ReflectColor;
uniform lowp vec4 _Color;
uniform samplerCube _Cube;
uniform sampler2D _SpecMap;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform sampler2D _LightTexture0;
uniform lowp vec4 _LightColor0;
void main ()
{
  lowp vec4 c_1;
  lowp vec3 lightDir_2;
  highp vec3 tmpvar_3;
  mediump vec3 tmpvar_4;
  mediump vec3 tmpvar_5;
  mediump float tmpvar_6;
  mediump vec4 spec_7;
  lowp vec4 tmpvar_8;
  tmpvar_8 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_9;
  tmpvar_9 = (tmpvar_8.xyz * _Color.xyz);
  tmpvar_4 = tmpvar_9;
  lowp vec4 tmpvar_10;
  tmpvar_10 = texture2D (_SpecMap, xlv_TEXCOORD0);
  spec_7 = tmpvar_10;
  mediump vec3 tmpvar_11;
  tmpvar_11 = (spec_7.xyz * _Gloss);
  lowp vec3 normal_12;
  normal_12.xy = ((texture2D (_BumpMap, xlv_TEXCOORD0).wy * 2.0) - 1.0);
  normal_12.z = sqrt(((1.0 - (normal_12.x * normal_12.x)) - (normal_12.y * normal_12.y)));
  tmpvar_5 = normal_12;
  lowp float tmpvar_13;
  tmpvar_13 = ((textureCube (_Cube, tmpvar_3) * tmpvar_8.w).w * _ReflectColor.w);
  tmpvar_6 = tmpvar_13;
  lightDir_2 = xlv_TEXCOORD1;
  lowp vec4 tmpvar_14;
  tmpvar_14 = texture2D (_LightTexture0, xlv_TEXCOORD3);
  mediump vec3 lightDir_15;
  lightDir_15 = lightDir_2;
  mediump float atten_16;
  atten_16 = tmpvar_14.w;
  mediump vec4 c_17;
  mediump vec3 specCol_18;
  highp float nh_19;
  mediump float tmpvar_20;
  tmpvar_20 = max (0.0, dot (tmpvar_5, normalize((lightDir_15 + normalize(xlv_TEXCOORD2)))));
  nh_19 = tmpvar_20;
  mediump float arg1_21;
  arg1_21 = (32.0 * _Shininess);
  highp vec3 tmpvar_22;
  tmpvar_22 = (pow (nh_19, arg1_21) * tmpvar_11);
  specCol_18 = tmpvar_22;
  c_17.xyz = ((((tmpvar_4 * _LightColor0.xyz) * max (0.0, dot (tmpvar_5, lightDir_15))) + (_LightColor0.xyz * specCol_18)) * (atten_16 * 2.0));
  c_17.w = tmpvar_6;
  c_1.xyz = c_17.xyz;
  c_1.w = 0.0;
  gl_FragData[0] = c_1;
}



#endif"
}

}
Program "fp" {
// Fragment combos: 5
//   opengl - ALU: 31 to 42, TEX: 3 to 5
//   d3d9 - ALU: 30 to 40, TEX: 3 to 5
//   d3d11 - ALU: 19 to 29, TEX: 3 to 5, FLOW: 1 to 1
SubProgram "opengl " {
Keywords { "POINT" }
Vector 0 [_LightColor0]
Vector 1 [_Color]
Float 3 [_Shininess]
Float 4 [_Gloss]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 4 [_LightTexture0] 2D
"3.0-!!ARBfp1.0
OPTION ARB_precision_hint_fastest;
# 36 ALU, 4 TEX
PARAM c[6] = { program.local[0..4],
		{ 0, 2, 1, 32 } };
TEMP R0;
TEMP R1;
TEMP R2;
TEX R0.yw, fragment.texcoord[0], texture[2], 2D;
MAD R0.xy, R0.wyzw, c[5].y, -c[5].z;
MUL R0.z, R0.y, R0.y;
MAD R0.z, -R0.x, R0.x, -R0;
DP3 R0.w, fragment.texcoord[1], fragment.texcoord[1];
RSQ R0.w, R0.w;
ADD R0.z, R0, c[5];
RSQ R0.z, R0.z;
MUL R1.xyz, R0.w, fragment.texcoord[1];
DP3 R1.w, fragment.texcoord[2], fragment.texcoord[2];
RSQ R0.w, R1.w;
MAD R2.xyz, R0.w, fragment.texcoord[2], R1;
DP3 R0.w, R2, R2;
RCP R0.z, R0.z;
RSQ R1.w, R0.w;
DP3 R0.w, R0, R1;
MUL R1.xyz, R1.w, R2;
DP3 R0.y, R0, R1;
MOV R0.x, c[5].w;
MAX R1.x, R0.y, c[5];
MUL R1.y, R0.x, c[3].x;
POW R1.w, R1.x, R1.y;
TEX R0.xyz, fragment.texcoord[0], texture[1], 2D;
MUL R1.xyz, R0, c[4].x;
MUL R1.xyz, R1.w, R1;
TEX R0.xyz, fragment.texcoord[0], texture[0], 2D;
MUL R0.xyz, R0, c[1];
MAX R0.w, R0, c[5].x;
MUL R1.xyz, R1, c[0];
MUL R0.xyz, R0, c[0];
MAD R0.xyz, R0, R0.w, R1;
DP3 R1.w, fragment.texcoord[3], fragment.texcoord[3];
TEX R0.w, R1.w, texture[4], 2D;
MUL R0.xyz, R0.w, R0;
MUL result.color.xyz, R0, c[5].y;
MOV result.color.w, c[5].x;
END
# 36 instructions, 3 R-regs
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
SetTexture 4 [_LightTexture0] 2D
"ps_3_0
; 35 ALU, 4 TEX
dcl_2d s0
dcl_2d s1
dcl_2d s2
dcl_2d s4
def c4, 2.00000000, -1.00000000, 1.00000000, 0.00000000
def c5, 32.00000000, 0, 0, 0
dcl_texcoord0 v0.xy
dcl_texcoord1 v1.xyz
dcl_texcoord2 v2.xyz
dcl_texcoord3 v3.xyz
texld r0.yw, v0, s2
mad_pp r0.xy, r0.wyzw, c4.x, c4.y
mul_pp r0.z, r0.y, r0.y
mad_pp r0.z, -r0.x, r0.x, -r0
dp3_pp r0.w, v1, v1
rsq_pp r0.w, r0.w
add_pp r0.z, r0, c4
rsq_pp r0.z, r0.z
mul_pp r1.xyz, r0.w, v1
dp3_pp r1.w, v2, v2
rsq_pp r0.w, r1.w
mad_pp r2.xyz, r0.w, v2, r1
dp3_pp r0.w, r2, r2
rcp_pp r0.z, r0.z
rsq_pp r1.w, r0.w
dp3_pp r0.w, r0, r1
mul_pp r1.xyz, r1.w, r2
dp3_pp r0.x, r0, r1
mov_pp r1.w, c2.x
mul_pp r0.y, c5.x, r1.w
max_pp r0.x, r0, c4.w
pow r1, r0.x, r0.y
texld r0.xyz, v0, s1
mov r1.w, r1.x
mul_pp r1.xyz, r0, c3.x
texld r0.xyz, v0, s0
mul_pp r0.xyz, r0, c1
mul_pp r2.xyz, r0, c0
mul r1.xyz, r1.w, r1
dp3 r0.x, v3, v3
max_pp r0.w, r0, c4
mul_pp r1.xyz, r1, c0
mad_pp r1.xyz, r2, r0.w, r1
texld r0.x, r0.x, s4
mul_pp r0.xyz, r0.x, r1
mul_pp oC0.xyz, r0, c4.x
mov_pp oC0.w, c4
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
// ALU: 26.67 (20 instructions), vertex: 0, texture: 16,
//   sequencer: 12, interpolator: 16;    7 GPRs, 27 threads,
// Performance (if enough threads): ~26 cycles per vector
// * Texture cycle estimates are assuming an 8bit/component texture with no
//     aniso or trilinear filtering.

"ps_360
backbbaaaaaaabneaaaaabjaaaaaaaaaaaaaaaceaaaaabhmaaaaabkeaaaaaaaa
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
aaaaaaaaaaaaaaeaaaaaabfabaaaagaaaaaaaaaeaaaaaaaaaaaacmieaaapaaap
aaaaaaabaaaadafaaaaahbfbaaaahcfcaaaahdfdaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaalpiaaaaaaaaaaaaadpiaaaaaecaaaaaaabfefaadaaaabcaameaaaaaa
aaaagaaigaaobcaabcaaaaaaaaaagabebabkbcaaccaaaaaamiaeaaaaaaloloaa
paadadaabadidaabbpbppoiiaaaaeaaababigaabbpbppoiiaaaaeaaakiaieaab
bpbppppiaaaaeaaabacieaabbpbppompaaaaeaaamiaeaaaaaaloloaapaacacaa
miaiaaabaaloloaapaababaafiilabaaaagcgcbloaaeaeibfiehaaafaablmamg
obababiamiahaaaeaamgmamaolaaacafmiagaaabaalmgmaakaaappaamiaeaaaa
aelclcmgnbababppmiabaaabaaloloaapaaeaeaafibhabacaamamagmkbagabib
kaihabaeaamagmmgobaeabiamiabaaabaalomdaapaafabaabeacaaabaalomdgm
naaeabacamidacabaalalbblicabppppeaehabacaamamalbkbacaaibmiapaaab
aaaaomaaobacabaadiehaaacaamagmblkbadadabmiahaaacaamamgaaobacaaaa
miahaaaaaamamamaklacaaabmiahmaaaaablmaaaobaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaa"
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
SetTexture 4 [_LightTexture0] 2D
"sce_fp_rsx // 44 instructions using 4 registers
[Configuration]
24
ffffffff0003c020000ffff1000000000000840004000000
[Offsets]
4
_LightColor0 2 0
000002a000000210
_Color 1 0
000001f0
_Shininess 1 0
00000050
_Gloss 1 0
000000c0
[Microcode]
704
94001704c8011c9dc8000001c8003fe1ae843940c8011c9dc8000029c800bfe1
06800440ce001c9daa0200005402000100000000000040000000bf8000000000
1082014000021c9cc8000001c800000100000000000000000000000000000000
10840240ab001c9cab000000c8000001ee040100c8011c9dc8000001c8003fe1
08020500c8081c9dc8080001c80000011084044001001c9e01000000c9080003
8e041702c8011c9dc8000001c8003fe10e820240c8081c9d00020000c8000001
00000000000000000000000000000000108a0340c9081c9daa020000c8000001
0000000000003f80000000000000000008803b40ff143c9dff140001c8000001
10800540c9001c9dc9080001c80000010206170854041c9dc8000001c8000001
ce883940c8011c9dc8000029c800bfe10e040340c9081c9dc9100001c8000001
8e021700c8011c9dc8000001c8003fe10e883940c8081c9dc8000029c8000001
08860540c9001c9dc9100001c800000104040900550c1c9d00020000c8000001
0000000000000000000000000000000010000100c8001c9dc8000001c8000001
04041d00aa081c9cc8000001c800000102880240ff041c9d00020000c8000001
0000420000000000000000000000000010020200aa081c9c01100000c8000001
0e800240c8041c9dc8020001c800000100000000000000000000000000000000
0e800240c9001c9dc8020001c800000100000000000000000000000000000000
02021c00fe041c9dc8000001c80000010e82020000041c9cc9040001c8000001
10800900c9001c9d00020000c800000100000000000000000000000000000000
0e800240c9001c9dff000001c80000011080014000021c9cc8000001c8000001
000000000000000000000000000000000e800440c9041c9dc8020001c9000001
000000000000000000000000000000000e810240000c1c9cc9001001c8000001
"
}

SubProgram "d3d11 " {
Keywords { "POINT" }
ConstBuffer "$Globals" 176 // 156 used size, 10 vars
Vector 16 [_LightColor0] 4
Vector 112 [_Color] 4
Float 148 [_Shininess]
Float 152 [_Gloss]
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 1
SetTexture 1 [_SpecMap] 2D 3
SetTexture 2 [_BumpMap] 2D 2
SetTexture 3 [_LightTexture0] 2D 0
// 35 instructions, 3 temp regs, 0 temp arrays:
// ALU 24 float, 0 int, 0 uint
// TEX 4 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0
eefiecedikamcniafhgofknncaemfhfnfhoikpjmabaaaaaaoaafaaaaadaaaaaa
cmaaaaaammaaaaaaaaabaaaaejfdeheojiaaaaaaafaaaaaaaiaaaaaaiaaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaadadaaaaimaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaahahaaaaimaaaaaa
adaaaaaaaaaaaaaaadaaaaaaaeaaaaaaahahaaaafdfgfpfaepfdejfeejepeoaa
feeffiedepepfceeaaklklklepfdeheocmaaaaaaabaaaaaaaiaaaaaacaaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaafdfgfpfegbhcghgfheaaklkl
fdeieefcniaeaaaaeaaaaaaadgabaaaafjaaaaaeegiocaaaaaaaaaaaakaaaaaa
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
aaaaaaaabkiacaaaaaaaaaaaajaaaaaaabeaaaaaaaaaaaecdiaaaaahbcaabaaa
aaaaaaaaakaabaaaaaaaaaaackaabaaaaaaaaaaabjaaaaafbcaabaaaaaaaaaaa
akaabaaaaaaaaaaaefaaaaajpcaabaaaabaaaaaaegbabaaaabaaaaaaeghobaaa
abaaaaaaaagabaaaadaaaaaadiaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaa
kgikcaaaaaaaaaaaajaaaaaadiaaaaahncaabaaaaaaaaaaaagaabaaaaaaaaaaa
agajbaaaabaaaaaadiaaaaaincaabaaaaaaaaaaaagaobaaaaaaaaaaaagijcaaa
aaaaaaaaabaaaaaaefaaaaajpcaabaaaabaaaaaaegbabaaaabaaaaaaeghobaaa
aaaaaaaaaagabaaaabaaaaaadiaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaa
egiccaaaaaaaaaaaahaaaaaadiaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaa
egiccaaaaaaaaaaaabaaaaaadcaaaaajhcaabaaaaaaaaaaaegacbaaaabaaaaaa
fgafbaaaaaaaaaaaigadbaaaaaaaaaaabaaaaaahicaabaaaaaaaaaaaegbcbaaa
aeaaaaaaegbcbaaaaeaaaaaaefaaaaajpcaabaaaabaaaaaapgapbaaaaaaaaaaa
eghobaaaadaaaaaaaagabaaaaaaaaaaaaaaaaaahicaabaaaaaaaaaaaakaabaaa
abaaaaaaakaabaaaabaaaaaadiaaaaahhccabaaaaaaaaaaapgapbaaaaaaaaaaa
egacbaaaaaaaaaaadgaaaaaficcabaaaaaaaaaaaabeaaaaaaaaaaaaadoaaaaab
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
Float 3 [_Shininess]
Float 4 [_Gloss]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
"3.0-!!ARBfp1.0
OPTION ARB_precision_hint_fastest;
# 31 ALU, 3 TEX
PARAM c[6] = { program.local[0..4],
		{ 0, 2, 1, 32 } };
TEMP R0;
TEMP R1;
TEMP R2;
TEX R1.yw, fragment.texcoord[0], texture[2], 2D;
MAD R1.xy, R1.wyzw, c[5].y, -c[5].z;
DP3 R1.z, fragment.texcoord[2], fragment.texcoord[2];
MUL R0.w, R1.y, R1.y;
MAD R0.w, -R1.x, R1.x, -R0;
ADD R0.w, R0, c[5].z;
RSQ R1.z, R1.z;
MOV R0.xyz, fragment.texcoord[1];
MAD R0.xyz, R1.z, fragment.texcoord[2], R0;
DP3 R1.z, R0, R0;
RSQ R1.z, R1.z;
MUL R0.xyz, R1.z, R0;
RSQ R0.w, R0.w;
RCP R1.z, R0.w;
DP3 R0.x, R1, R0;
MAX R1.w, R0.x, c[5].x;
MOV R0.w, c[5];
TEX R0.xyz, fragment.texcoord[0], texture[1], 2D;
MUL R0.w, R0, c[3].x;
MUL R0.xyz, R0, c[4].x;
POW R0.w, R1.w, R0.w;
MUL R2.xyz, R0.w, R0;
TEX R0.xyz, fragment.texcoord[0], texture[0], 2D;
MUL R0.xyz, R0, c[1];
DP3 R0.w, R1, fragment.texcoord[1];
MUL R2.xyz, R2, c[0];
MUL R0.xyz, R0, c[0];
MAX R0.w, R0, c[5].x;
MAD R0.xyz, R0, R0.w, R2;
MUL result.color.xyz, R0, c[5].y;
MOV result.color.w, c[5].x;
END
# 31 instructions, 3 R-regs
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
"ps_3_0
; 30 ALU, 3 TEX
dcl_2d s0
dcl_2d s1
dcl_2d s2
def c4, 2.00000000, -1.00000000, 1.00000000, 0.00000000
def c5, 32.00000000, 0, 0, 0
dcl_texcoord0 v0.xy
dcl_texcoord1 v1.xyz
dcl_texcoord2 v2.xyz
texld r1.yw, v0, s2
mad_pp r1.xy, r1.wyzw, c4.x, c4.y
dp3_pp r1.z, v2, v2
mul_pp r0.w, r1.y, r1.y
mad_pp r0.w, -r1.x, r1.x, -r0
add_pp r0.w, r0, c4.z
rsq_pp r1.z, r1.z
mov_pp r0.xyz, v1
mad_pp r0.xyz, r1.z, v2, r0
dp3_pp r1.z, r0, r0
rsq_pp r1.z, r1.z
mul_pp r0.xyz, r1.z, r0
rsq_pp r0.w, r0.w
rcp_pp r1.z, r0.w
mov_pp r0.w, c2.x
dp3_pp r0.x, r1, r0
mul_pp r2.x, c5, r0.w
max_pp r1.w, r0.x, c4
pow r0, r1.w, r2.x
texld r2.xyz, v0, s1
mul_pp r2.xyz, r2, c3.x
mul r2.xyz, r0.x, r2
texld r0.xyz, v0, s0
mul_pp r0.xyz, r0, c1
dp3_pp r0.w, r1, v1
mul_pp r2.xyz, r2, c0
mul_pp r0.xyz, r0, c0
max_pp r0.w, r0, c4
mad_pp r0.xyz, r0, r0.w, r2
mul_pp oC0.xyz, r0, c4.x
mov_pp oC0.w, c4
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
// ALU: 21.33 (16 instructions), vertex: 0, texture: 12,
//   sequencer: 10, interpolator: 12;    5 GPRs, 36 threads,
// Performance (if enough threads): ~21 cycles per vector
// * Texture cycle estimates are assuming an 8bit/component texture with no
//     aniso or trilinear filtering.

"ps_360
backbbaaaaaaablaaaaaabfeaaaaaaaaaaaaaaceaaaaabfmaaaaabieaaaaaaaa
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
aaaaaaaaaaaaaaeaaaaaabbebaaaaeaaaaaaaaaeaaaaaaaaaaaacagdaaahaaah
aaaaaaabaaaadafaaaaahbfbaaaahcfcaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaeaaaaaaaaaaaaaaalpiaaaaadpiaaaaa
ecaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaabfdaadaaaabcaameaaaaaaaaaagaag
gaambcaabcaaaaaaaaaaeabcaaaaccaaaaaaaaaabacidaabbpbppoiiaaaaeaaa
baaieaabbpbppoiiaaaaeaaababiaaabbpbppompaaaaeaaamiabaaaaaaloloaa
paacacaamiagaaaaaalggmmgilaapopofibhaaaeaamamagmkbaeabiamiahaaac
aagmmamaolaaacabmiabaaaaaelclcblnbaaaapokaiiaaabaalologmpaacacia
fibbabaaaamdloblpaaaabibmiahaaabaamagmaaobacabaabeacaaaaaalomdgm
naabaaacamidabaaaalalbgmicaapoppeaehaaabaamamalbkbaeaaiamiapaaaa
aaaaomaaobabaaaadiboababaapmgmblkbadadaamiahaaabaabfgmaaobababaa
miahaaaaaamamamaklabaaaamiahmaaaaamamaaaoaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaa"
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
"sce_fp_rsx // 39 instructions using 4 registers
[Configuration]
24
ffffffff0001c0200007fff9000000000000840004000000
[Offsets]
4
_LightColor0 2 0
00000260000001b0
_Color 1 0
000000e0
_Shininess 1 0
00000130
_Gloss 1 0
00000020
[Microcode]
624
8e001702c8011c9dc8000001c8003fe10e860240c8001c9d00020000c8000001
0000000000000000000000000000000094041704c8011c9dc8000001c8003fe1
ce803940c8011c9dc8000029c800bfe106820440ce081c9d00020000aa020000
000040000000bf80000000000000000010840240ab041c9cab040000c8000001
ae840140c8011c9dc8000001c8003fe10e060340c9081c9dc9000001c8000001
1084044001041c9e01040000c90800038e041700c8011c9dc8000001c8003fe1
0e803940c80c1c9dc8000029c80000010e880240c8081c9dc8020001c8000001
0000000000000000000000000000000010800340c9081c9dc8020001c8000001
00000000000000000000000000003f8008823b40ff003c9dff000001c8000001
028a014000021c9cc8000001c800000100000000000000000000000000000000
1088024001141c9c00020000c800000100004200000000000000000000000000
02800540c9041c9dc9000001c80000011004090001001c9c00020000c8000001
0000000000000000000000000000000010840540c9041c9dc9080001c8000001
0e840240c9101c9dc8020001c800000100000000000000000000000000000000
08041d00fe081c9dc8000001c80000010204020054081c9dff100001c8000001
10800900c9081c9d00020000c800000100000000000000000000000000000000
0e800240c9081c9dff000001c800000108001c00c8081c9dc8000001c8000001
0e82020054001c9dc90c0001c80000011080014000021c9cc8000001c8000001
000000000000000000000000000000000e810440c9041c9dc8021001c9000001
00000000000000000000000000000000
"
}

SubProgram "d3d11 " {
Keywords { "DIRECTIONAL" }
ConstBuffer "$Globals" 112 // 92 used size, 9 vars
Vector 16 [_LightColor0] 4
Vector 48 [_Color] 4
Float 84 [_Shininess]
Float 88 [_Gloss]
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 0
SetTexture 1 [_SpecMap] 2D 2
SetTexture 2 [_BumpMap] 2D 1
// 29 instructions, 2 temp regs, 0 temp arrays:
// ALU 19 float, 0 int, 0 uint
// TEX 3 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0
eefiecedpdfbongomjgmegefmbjmpjckaieiocjaabaaaaaapiaeaaaaadaaaaaa
cmaaaaaaleaaaaaaoiaaaaaaejfdeheoiaaaaaaaaeaaaaaaaiaaaaaagiaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaheaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaadadaaaaheaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaaheaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaahahaaaafdfgfpfa
epfdejfeejepeoaafeeffiedepepfceeaaklklklepfdeheocmaaaaaaabaaaaaa
aiaaaaaacaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaafdfgfpfe
gbhcghgfheaaklklfdeieefcaiaeaaaaeaaaaaaaacabaaaafjaaaaaeegiocaaa
aaaaaaaaagaaaaaafkaaaaadaagabaaaaaaaaaaafkaaaaadaagabaaaabaaaaaa
fkaaaaadaagabaaaacaaaaaafibiaaaeaahabaaaaaaaaaaaffffaaaafibiaaae
aahabaaaabaaaaaaffffaaaafibiaaaeaahabaaaacaaaaaaffffaaaagcbaaaad
dcbabaaaabaaaaaagcbaaaadhcbabaaaacaaaaaagcbaaaadhcbabaaaadaaaaaa
gfaaaaadpccabaaaaaaaaaaagiaaaaacacaaaaaabaaaaaahbcaabaaaaaaaaaaa
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
aaaaaaaaakaabaaaaaaaaaaadiaaaaaiecaabaaaaaaaaaaabkiacaaaaaaaaaaa
afaaaaaaabeaaaaaaaaaaaecdiaaaaahbcaabaaaaaaaaaaaakaabaaaaaaaaaaa
ckaabaaaaaaaaaaabjaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaaefaaaaaj
pcaabaaaabaaaaaaegbabaaaabaaaaaaeghobaaaabaaaaaaaagabaaaacaaaaaa
diaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaakgikcaaaaaaaaaaaafaaaaaa
diaaaaahncaabaaaaaaaaaaaagaabaaaaaaaaaaaagajbaaaabaaaaaadiaaaaai
ncaabaaaaaaaaaaaagaobaaaaaaaaaaaagijcaaaaaaaaaaaabaaaaaaefaaaaaj
pcaabaaaabaaaaaaegbabaaaabaaaaaaeghobaaaaaaaaaaaaagabaaaaaaaaaaa
diaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaaegiccaaaaaaaaaaaadaaaaaa
diaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaaegiccaaaaaaaaaaaabaaaaaa
dcaaaaajhcaabaaaaaaaaaaaegacbaaaabaaaaaafgafbaaaaaaaaaaaigadbaaa
aaaaaaaaaaaaaaahhccabaaaaaaaaaaaegacbaaaaaaaaaaaegacbaaaaaaaaaaa
dgaaaaaficcabaaaaaaaaaaaabeaaaaaaaaaaaaadoaaaaab"
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
Float 3 [_Shininess]
Float 4 [_Gloss]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 4 [_LightTexture0] 2D
SetTexture 5 [_LightTextureB0] 2D
"3.0-!!ARBfp1.0
OPTION ARB_precision_hint_fastest;
# 42 ALU, 5 TEX
PARAM c[7] = { program.local[0..4],
		{ 0, 0.5, 2, 1 },
		{ 32 } };
TEMP R0;
TEMP R1;
TEMP R2;
TEX R0.yw, fragment.texcoord[0], texture[2], 2D;
MAD R0.xy, R0.wyzw, c[5].z, -c[5].w;
MUL R0.z, R0.y, R0.y;
MAD R0.z, -R0.x, R0.x, -R0;
DP3 R0.w, fragment.texcoord[1], fragment.texcoord[1];
RSQ R0.w, R0.w;
ADD R0.z, R0, c[5].w;
RSQ R0.z, R0.z;
MUL R1.xyz, R0.w, fragment.texcoord[1];
DP3 R1.w, fragment.texcoord[2], fragment.texcoord[2];
RSQ R0.w, R1.w;
MAD R2.xyz, R0.w, fragment.texcoord[2], R1;
DP3 R0.w, R2, R2;
RCP R0.z, R0.z;
RSQ R1.w, R0.w;
DP3 R0.w, R0, R1;
MUL R1.xyz, R1.w, R2;
DP3 R0.y, R0, R1;
MOV R0.x, c[6];
MAX R1.x, R0.y, c[5];
MUL R1.y, R0.x, c[3].x;
POW R1.w, R1.x, R1.y;
TEX R0.xyz, fragment.texcoord[0], texture[1], 2D;
MUL R1.xyz, R0, c[4].x;
MUL R1.xyz, R1.w, R1;
TEX R0.xyz, fragment.texcoord[0], texture[0], 2D;
MUL R0.xyz, R0, c[1];
MUL R1.xyz, R1, c[0];
MAX R0.w, R0, c[5].x;
MUL R0.xyz, R0, c[0];
MAD R0.xyz, R0, R0.w, R1;
RCP R1.w, fragment.texcoord[3].w;
MAD R1.xy, fragment.texcoord[3], R1.w, c[5].y;
TEX R0.w, R1, texture[4], 2D;
DP3 R1.z, fragment.texcoord[3], fragment.texcoord[3];
SLT R1.x, c[5], fragment.texcoord[3].z;
TEX R1.w, R1.z, texture[5], 2D;
MUL R0.w, R1.x, R0;
MUL R0.w, R0, R1;
MUL R0.xyz, R0.w, R0;
MUL result.color.xyz, R0, c[5].z;
MOV result.color.w, c[5].x;
END
# 42 instructions, 3 R-regs
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
SetTexture 4 [_LightTexture0] 2D
SetTexture 5 [_LightTextureB0] 2D
"ps_3_0
; 40 ALU, 5 TEX
dcl_2d s0
dcl_2d s1
dcl_2d s2
dcl_2d s4
dcl_2d s5
def c4, 0.00000000, 1.00000000, 0.50000000, 32.00000000
def c5, 2.00000000, -1.00000000, 0, 0
dcl_texcoord0 v0.xy
dcl_texcoord1 v1.xyz
dcl_texcoord2 v2.xyz
dcl_texcoord3 v3
texld r0.yw, v0, s2
mad_pp r0.xy, r0.wyzw, c5.x, c5.y
mul_pp r0.z, r0.y, r0.y
mad_pp r0.z, -r0.x, r0.x, -r0
dp3_pp r0.w, v1, v1
rsq_pp r0.w, r0.w
add_pp r0.z, r0, c4.y
rsq_pp r0.z, r0.z
mul_pp r1.xyz, r0.w, v1
dp3_pp r1.w, v2, v2
rsq_pp r0.w, r1.w
mad_pp r2.xyz, r0.w, v2, r1
dp3_pp r0.w, r2, r2
rcp_pp r0.z, r0.z
rsq_pp r1.w, r0.w
dp3_pp r0.w, r0, r1
mul_pp r1.xyz, r1.w, r2
dp3_pp r0.x, r0, r1
mov_pp r1.w, c2.x
mul_pp r0.y, c4.w, r1.w
max_pp r0.x, r0, c4
pow r1, r0.x, r0.y
mov r1.w, r1.x
texld r0.xyz, v0, s1
mul_pp r1.xyz, r0, c3.x
mul r1.xyz, r1.w, r1
texld r0.xyz, v0, s0
mul_pp r0.xyz, r0, c1
rcp r1.w, v3.w
mul_pp r0.xyz, r0, c0
max_pp r0.w, r0, c4.x
mul_pp r1.xyz, r1, c0
mad_pp r1.xyz, r0, r0.w, r1
mad r2.xy, v3, r1.w, c4.z
dp3 r0.x, v3, v3
texld r0.w, r2, s4
cmp r0.y, -v3.z, c4.x, c4
mul_pp r0.y, r0, r0.w
texld r0.x, r0.x, s5
mul_pp r0.x, r0.y, r0
mul_pp r0.xyz, r0.x, r1
mul_pp oC0.xyz, r0, c5.x
mov_pp oC0.w, c4.x
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
//   sequencer: 14, interpolator: 16;    8 GPRs, 24 threads,
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
aaaaaaaaaaaaaaaaaaaaaaeaaaaaabimbaaaahaaaaaaaaaeaaaaaaaaaaaadaie
aaapaaapaaaaaaabaaaadafaaaaahbfbaaaahcfcaaaapdfdaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaadpaaaaaaaaaaaaaa
dpiaaaaaecaaaaaalpiaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaffagaaebaakbcaa
bcaaaaabaaaaaaaagaalmeaabcaaaaaaaaaagabbgabhbcaabcaaaaaaaaaadabn
aaaaccaaaaaaaaaaemeiaaaaaaloloblpaadadadmiadaaafaamglagmmlaaadpo
baeieaabbpbppoiiaaaaeaaabacigaabbpbppoiiaaaaeaaabadiaaabbpbppomp
aaaaeaaapmbiaaabbpbppppiaaaaeaaabaaihakbbpbpppplaaaaeaaamiaiaaab
aaloloaapaababaafiiiabaaaaloloblpaacacibfiihaaafaablmablobababia
miaoaaahaablpmpmolaaacafcabiabaaaamdmdmgpaahahadficbabacaagmblbl
cbacpoiamiapaaadaaaalaaaobahabaamiabaaaaaagmgmaaobadaaaamialaaaa
aagcgcaaoaaaaaaamiagaaabaalmgmaakaaappaamiaeaaaaaelclcmgnbababpo
kaioabacaapmpmmgkbagabiamiabaaabaalomdaapaafabaamiacaaabaamdmdaa
paadabaamiagaaabaalmlbaakcabpoaaeaboabacaaabpmmgkbacaaibmiapaaab
aaaalaaaobacabaadiehaaacaamagmgmkbaeadabmiahaaacaamamgaaobacaaaa
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
SetTexture 4 [_LightTexture0] 2D
SetTexture 5 [_LightTextureB0] 2D
"sce_fp_rsx // 52 instructions using 4 registers
[Configuration]
24
ffffffff0003c020000ffff1000000000000840004000000
[Offsets]
4
_LightColor0 2 0
0000030000000230
_Color 1 0
000001a0
_Shininess 1 0
00000120
_Gloss 1 0
00000200
[Microcode]
832
94001704c8011c9dc8000001c8003fe1ae8a3940c8011c9dc8000029c800bfe1
06880440ce001c9d00020000aa020000000040000000bf800000000000000000
fe000100c8011c9dc8000001c8003fe102860240ab101c9cab100000c8000001
06023a00c8001c9dfe000001c800000106020300c8041c9d00020000c8000001
00003f000000000000000000000000001086044001101c9e01100000010c0002
10001708c8041c9dc8000001c8000001ce8c3940c8011c9dc8000029c800bfe1
02000500c8001c9dc8000001c800000110800340c90c1c9d00020000c8000001
00003f8000000000000000000000000008883b40ff003c9dff000001c8000001
108a0540c9101c9dc9140001c80000011086014000021c9cc8000001c8000001
000000000000000000000000000000000200170a00001c9cc8000001c8000001
0e060340c9141c9dc9180001c80000018e021702c8011c9dc8000001c8003fe1
0e8a3940c80c1c9dc8000029c800000108860540c9101c9dc9140001c8000001
8e041700c8011c9dc8000001c8003fe10e880240c8081c9dc8020001c8000001
0000000000000000000000000000000010800240c90c1c9dc8020001c8000001
0000000000000000000000000000420010020900550c1c9daa020000c8000001
000000000000000000000000000000000e840240c8041c9d00020000c8000001
0000000000000000000000000000000008041d00fe041c9dc8000001c8000001
0e880240c9101c9dc8020001c800000100000000000000000000000000000000
0206020054081c9dff000001c800000110880900c9141c9d00020000c8000001
000000000000000000000000000000000e880240c9101c9dff100001c8000001
08041c00c80c1c9dc8000001c800000110800d0054001c9d00020000c8000001
0000000000000000000000000000000008040100c8081c9dc8000001c8000001
10800240c9001c9dc8000001c80000010e82020054081c9dc9080001c8000001
10800240c9001c9d00000000c80000010e800440c9041c9dc8020001c9100001
000000000000000000000000000000000e800240ff001c9dc9001001c8000001
1081014000021c9cc8000001c800000100000000000000000000000000000000
"
}

SubProgram "d3d11 " {
Keywords { "SPOT" }
ConstBuffer "$Globals" 176 // 156 used size, 10 vars
Vector 16 [_LightColor0] 4
Vector 112 [_Color] 4
Float 148 [_Shininess]
Float 152 [_Gloss]
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 2
SetTexture 1 [_SpecMap] 2D 4
SetTexture 2 [_BumpMap] 2D 3
SetTexture 3 [_LightTexture0] 2D 0
SetTexture 4 [_LightTextureB0] 2D 1
// 41 instructions, 3 temp regs, 0 temp arrays:
// ALU 28 float, 0 int, 1 uint
// TEX 5 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0
eefiecedggfekldjegdggdcpnmpekapignjnndlpabaaaaaaliagaaaaadaaaaaa
cmaaaaaammaaaaaaaaabaaaaejfdeheojiaaaaaaafaaaaaaaiaaaaaaiaaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaadadaaaaimaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaahahaaaaimaaaaaa
adaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapapaaaafdfgfpfaepfdejfeejepeoaa
feeffiedepepfceeaaklklklepfdeheocmaaaaaaabaaaaaaaiaaaaaacaaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaafdfgfpfegbhcghgfheaaklkl
fdeieefclaafaaaaeaaaaaaagmabaaaafjaaaaaeegiocaaaaaaaaaaaakaaaaaa
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
bkiacaaaaaaaaaaaajaaaaaaabeaaaaaaaaaaaecdiaaaaahbcaabaaaaaaaaaaa
akaabaaaaaaaaaaackaabaaaaaaaaaaabjaaaaafbcaabaaaaaaaaaaaakaabaaa
aaaaaaaaefaaaaajpcaabaaaabaaaaaaegbabaaaabaaaaaaeghobaaaabaaaaaa
aagabaaaaeaaaaaadiaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaakgikcaaa
aaaaaaaaajaaaaaadiaaaaahncaabaaaaaaaaaaaagaabaaaaaaaaaaaagajbaaa
abaaaaaadiaaaaaincaabaaaaaaaaaaaagaobaaaaaaaaaaaagijcaaaaaaaaaaa
abaaaaaaefaaaaajpcaabaaaabaaaaaaegbabaaaabaaaaaaeghobaaaaaaaaaaa
aagabaaaacaaaaaadiaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaaegiccaaa
aaaaaaaaahaaaaaadiaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaaegiccaaa
aaaaaaaaabaaaaaadcaaaaajhcaabaaaaaaaaaaaegacbaaaabaaaaaafgafbaaa
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
dgaaaaaficcabaaaaaaaaaaaabeaaaaaaaaaaaaadoaaaaab"
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
Float 3 [_Shininess]
Float 4 [_Gloss]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 4 [_LightTextureB0] 2D
SetTexture 5 [_LightTexture0] CUBE
"3.0-!!ARBfp1.0
OPTION ARB_precision_hint_fastest;
# 38 ALU, 5 TEX
PARAM c[6] = { program.local[0..4],
		{ 0, 2, 1, 32 } };
TEMP R0;
TEMP R1;
TEMP R2;
TEX R0.yw, fragment.texcoord[0], texture[2], 2D;
MAD R0.xy, R0.wyzw, c[5].y, -c[5].z;
MUL R0.z, R0.y, R0.y;
MAD R0.z, -R0.x, R0.x, -R0;
DP3 R0.w, fragment.texcoord[1], fragment.texcoord[1];
RSQ R0.w, R0.w;
ADD R0.z, R0, c[5];
RSQ R0.z, R0.z;
MUL R1.xyz, R0.w, fragment.texcoord[1];
DP3 R1.w, fragment.texcoord[2], fragment.texcoord[2];
RSQ R0.w, R1.w;
MAD R2.xyz, R0.w, fragment.texcoord[2], R1;
DP3 R0.w, R2, R2;
RCP R0.z, R0.z;
RSQ R1.w, R0.w;
DP3 R0.w, R0, R1;
MUL R1.xyz, R1.w, R2;
DP3 R0.y, R0, R1;
MOV R0.x, c[5].w;
MAX R1.x, R0.y, c[5];
MUL R1.y, R0.x, c[3].x;
POW R1.w, R1.x, R1.y;
TEX R0.xyz, fragment.texcoord[0], texture[1], 2D;
MUL R1.xyz, R0, c[4].x;
MUL R1.xyz, R1.w, R1;
TEX R0.xyz, fragment.texcoord[0], texture[0], 2D;
MUL R0.xyz, R0, c[1];
MAX R0.w, R0, c[5].x;
MUL R1.xyz, R1, c[0];
MUL R0.xyz, R0, c[0];
MAD R0.xyz, R0, R0.w, R1;
DP3 R1.x, fragment.texcoord[3], fragment.texcoord[3];
TEX R0.w, fragment.texcoord[3], texture[5], CUBE;
TEX R1.w, R1.x, texture[4], 2D;
MUL R0.w, R1, R0;
MUL R0.xyz, R0.w, R0;
MUL result.color.xyz, R0, c[5].y;
MOV result.color.w, c[5].x;
END
# 38 instructions, 3 R-regs
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
SetTexture 4 [_LightTextureB0] 2D
SetTexture 5 [_LightTexture0] CUBE
"ps_3_0
; 36 ALU, 5 TEX
dcl_2d s0
dcl_2d s1
dcl_2d s2
dcl_2d s4
dcl_cube s5
def c4, 2.00000000, -1.00000000, 1.00000000, 0.00000000
def c5, 32.00000000, 0, 0, 0
dcl_texcoord0 v0.xy
dcl_texcoord1 v1.xyz
dcl_texcoord2 v2.xyz
dcl_texcoord3 v3.xyz
texld r0.yw, v0, s2
mad_pp r0.xy, r0.wyzw, c4.x, c4.y
mul_pp r0.z, r0.y, r0.y
mad_pp r0.z, -r0.x, r0.x, -r0
dp3_pp r0.w, v1, v1
rsq_pp r0.w, r0.w
add_pp r0.z, r0, c4
rsq_pp r0.z, r0.z
mul_pp r1.xyz, r0.w, v1
dp3_pp r1.w, v2, v2
rsq_pp r0.w, r1.w
mad_pp r2.xyz, r0.w, v2, r1
dp3_pp r0.w, r2, r2
rcp_pp r0.z, r0.z
rsq_pp r1.w, r0.w
dp3_pp r0.w, r0, r1
mul_pp r1.xyz, r1.w, r2
dp3_pp r0.x, r0, r1
mov_pp r1.w, c2.x
mul_pp r0.y, c5.x, r1.w
max_pp r0.x, r0, c4.w
pow r1, r0.x, r0.y
max_pp r0.w, r0, c4
texld r0.xyz, v0, s1
mov r1.w, r1.x
mul_pp r1.xyz, r0, c3.x
texld r0.xyz, v0, s0
mul r1.xyz, r1.w, r1
mul_pp r0.xyz, r0, c1
mul_pp r0.xyz, r0, c0
mul_pp r1.xyz, r1, c0
mad_pp r1.xyz, r0, r0.w, r1
dp3 r0.x, v3, v3
texld r0.w, v3, s5
texld r0.x, r0.x, s4
mul r0.x, r0, r0.w
mul_pp r0.xyz, r0.x, r1
mul_pp oC0.xyz, r0, c4.x
mov_pp oC0.w, c4
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
// ALU: 32.00 (24 instructions), vertex: 0, texture: 20,
//   sequencer: 14, interpolator: 16;    7 GPRs, 27 threads,
// Performance (if enough threads): ~32 cycles per vector
// * Texture cycle estimates are assuming an 8bit/component texture with no
//     aniso or trilinear filtering.

"ps_360
backbbaaaaaaacaiaaaaabniaaaaaaaaaaaaaaceaaaaablaaaaaabniaaaaaaaa
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
aaaaaabeabpmaabaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaeaaaaaabji
baaaagaaaaaaaaaeaaaaaaaaaaaacmieaaapaaapaaaaaaabaaaadafaaaaahbfb
aaaahcfcaaaahdfdaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaalpiaaaaaaaaaaaaadpmaaaaadpiaaaaaecaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaafaagaaedaakbcaabcaaaabfaaaaaaaagaanmeaabcaaaaaa
aaaagabdgabjbcaabcaaaaaaaaaacabpaaaaccaaaaaaaaaamiaeaaaaaaloloaa
paadadaamiapaaadaakgmnaapcadadaaemieaaafaablblmgocadadidmiadaaaf
aagnblmgmladaapobaeidaabbpbppoiiaaaaeaaabadigaabbpbppompaaaaeaaa
bacieaabbpbppoiiaaaaeaaajaaiaakbbpbpppnpaaaamaaakibiaaabbpbppppi
aaaaeaaamiabaaagaalbgmaaobaaaaaamiaeaaaaaaloloaapaacacaamiabaaaa
aaloloaapaababaafibhaaaeaamamagmkbaeabiamiahaaafaagmmaaaobaaabaa
fielaaaaaagcgcmgoaagagiamiahaaacaamgmamaolaaacafmiaeaaaaaaloloaa
paacacaafibgababaalmgmmgkaaapoiamiaeaaaaaelclcblnbababpokaihabac
aamagmmgobacabiamiabaaabaalomdaapaafabaabeacaaabaalomdgmnaacabac
ambgacabaalmlbgmicabpoppeaboabacaapmpmmgkbaeaaibmiapaaabaaaalaaa
obacabaadiehaaacaamagmgmkbadadabmiahaaacaamamgaaobacaaaamiahaaaa
aamamabfklacaaabmiahmaaaaablmaaaobaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
"
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
SetTexture 4 [_LightTextureB0] 2D
SetTexture 5 [_LightTexture0] CUBE
"sce_fp_rsx // 45 instructions using 3 registers
[Configuration]
24
ffffffff0003c020000ffff1000000000000840003000000
[Offsets]
4
_LightColor0 2 0
0000029000000220
_Color 1 0
00000190
_Shininess 1 0
00000050
_Gloss 1 0
000001e0
[Microcode]
720
94021704c8011c9dc8000001c8003fe1ae803940c8011c9dc8000029c800bfe1
06820440ce041c9daa0200005402000100000000000040000000bf8000000000
1082014000021c9cc8000001c800000100000000000000000000000000000000
10800240ab041c9cab040000c8000001ee020100c8011c9dc8000001c8003fe1
10020500c8041c9dc8040001c80000011080044001041c9e01040000c9000003
8e041700c8011c9dc8000001c8003fe1ce843940c8011c9dc8000029c800bfe1
0e020340c9001c9dc9080001c8000001108a0340c9001c9d00020000c8000001
00003f8000000000000000000000000008823b40ff143c9dff140001c8000001
08800540c9041c9dc9000001c800000102001708fe041c9dc8000001c8000001
0e843940c8041c9dc8000029c800000110800540c9041c9dc9080001c8000001
10020900c9001c9d00020000c800000100000000000000000000000000000000
088a0240ff041c9daa020000c800000100000000000042000000000000000000
0e880240c8081c9dc8020001c800000100000000000000000000000000000000
1088090055001c9dc8020001c800000100000000000000000000000000000000
8e021702c8011c9dc8000001c8003fe10e840240c8041c9d00020000c8000001
0000000000000000000000000000000008021d00fe041c9dc8000001c8000001
1004020054041c9d55140001c80000010e880240c9101c9dc8020001c8000001
0000000000000000000000000000000008001c00fe081c9dc8000001c8000001
0e880240c9101c9dff100001c8000001f000170ac8011c9dc8000001c8003fe1
0e8a020054001c9dc9080001c80000011080020000001c9cc8000001c8000001
0e800440c9141c9dc8020001c910000100000000000000000000000000000000
0e800240ff001c9dc9001001c80000011081014000021c9cc8000001c8000001
00000000000000000000000000000000
"
}

SubProgram "d3d11 " {
Keywords { "POINT_COOKIE" }
ConstBuffer "$Globals" 176 // 156 used size, 10 vars
Vector 16 [_LightColor0] 4
Vector 112 [_Color] 4
Float 148 [_Shininess]
Float 152 [_Gloss]
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 2
SetTexture 1 [_SpecMap] 2D 4
SetTexture 2 [_BumpMap] 2D 3
SetTexture 3 [_LightTextureB0] 2D 1
SetTexture 4 [_LightTexture0] CUBE 0
// 36 instructions, 3 temp regs, 0 temp arrays:
// ALU 24 float, 0 int, 0 uint
// TEX 5 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0
eefiecedgbldbnkffmhbpfdgggkimlafkaceajhnabaaaaaacaagaaaaadaaaaaa
cmaaaaaammaaaaaaaaabaaaaejfdeheojiaaaaaaafaaaaaaaiaaaaaaiaaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaadadaaaaimaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaahahaaaaimaaaaaa
adaaaaaaaaaaaaaaadaaaaaaaeaaaaaaahahaaaafdfgfpfaepfdejfeejepeoaa
feeffiedepepfceeaaklklklepfdeheocmaaaaaaabaaaaaaaiaaaaaacaaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaafdfgfpfegbhcghgfheaaklkl
fdeieefcbiafaaaaeaaaaaaaegabaaaafjaaaaaeegiocaaaaaaaaaaaakaaaaaa
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
bkiacaaaaaaaaaaaajaaaaaaabeaaaaaaaaaaaecdiaaaaahbcaabaaaaaaaaaaa
akaabaaaaaaaaaaackaabaaaaaaaaaaabjaaaaafbcaabaaaaaaaaaaaakaabaaa
aaaaaaaaefaaaaajpcaabaaaabaaaaaaegbabaaaabaaaaaaeghobaaaabaaaaaa
aagabaaaaeaaaaaadiaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaakgikcaaa
aaaaaaaaajaaaaaadiaaaaahncaabaaaaaaaaaaaagaabaaaaaaaaaaaagajbaaa
abaaaaaadiaaaaaincaabaaaaaaaaaaaagaobaaaaaaaaaaaagijcaaaaaaaaaaa
abaaaaaaefaaaaajpcaabaaaabaaaaaaegbabaaaabaaaaaaeghobaaaaaaaaaaa
aagabaaaacaaaaaadiaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaaegiccaaa
aaaaaaaaahaaaaaadiaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaaegiccaaa
aaaaaaaaabaaaaaadcaaaaajhcaabaaaaaaaaaaaegacbaaaabaaaaaafgafbaaa
aaaaaaaaigadbaaaaaaaaaaabaaaaaahicaabaaaaaaaaaaaegbcbaaaaeaaaaaa
egbcbaaaaeaaaaaaefaaaaajpcaabaaaabaaaaaapgapbaaaaaaaaaaaeghobaaa
adaaaaaaaagabaaaabaaaaaaefaaaaajpcaabaaaacaaaaaaegbcbaaaaeaaaaaa
eghobaaaaeaaaaaaaagabaaaaaaaaaaaapaaaaahicaabaaaaaaaaaaaagaabaaa
abaaaaaapgapbaaaacaaaaaadiaaaaahhccabaaaaaaaaaaapgapbaaaaaaaaaaa
egacbaaaaaaaaaaadgaaaaaficcabaaaaaaaaaaaabeaaaaaaaaaaaaadoaaaaab
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
Float 3 [_Shininess]
Float 4 [_Gloss]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 4 [_LightTexture0] 2D
"3.0-!!ARBfp1.0
OPTION ARB_precision_hint_fastest;
# 33 ALU, 4 TEX
PARAM c[6] = { program.local[0..4],
		{ 0, 2, 1, 32 } };
TEMP R0;
TEMP R1;
TEMP R2;
TEX R1.yw, fragment.texcoord[0], texture[2], 2D;
MAD R1.xy, R1.wyzw, c[5].y, -c[5].z;
DP3 R1.z, fragment.texcoord[2], fragment.texcoord[2];
MUL R0.w, R1.y, R1.y;
MAD R0.w, -R1.x, R1.x, -R0;
ADD R0.w, R0, c[5].z;
RSQ R1.z, R1.z;
MOV R0.xyz, fragment.texcoord[1];
MAD R0.xyz, R1.z, fragment.texcoord[2], R0;
DP3 R1.z, R0, R0;
RSQ R1.z, R1.z;
MUL R0.xyz, R1.z, R0;
RSQ R0.w, R0.w;
RCP R1.z, R0.w;
DP3 R0.x, R1, R0;
MAX R1.w, R0.x, c[5].x;
MOV R0.w, c[5];
TEX R0.xyz, fragment.texcoord[0], texture[1], 2D;
MUL R0.w, R0, c[3].x;
MUL R0.xyz, R0, c[4].x;
POW R0.w, R1.w, R0.w;
MUL R2.xyz, R0.w, R0;
TEX R0.xyz, fragment.texcoord[0], texture[0], 2D;
MUL R0.xyz, R0, c[1];
DP3 R0.w, R1, fragment.texcoord[1];
MAX R0.w, R0, c[5].x;
MUL R2.xyz, R2, c[0];
MUL R0.xyz, R0, c[0];
MAD R0.xyz, R0, R0.w, R2;
TEX R0.w, fragment.texcoord[3], texture[4], 2D;
MUL R0.xyz, R0.w, R0;
MUL result.color.xyz, R0, c[5].y;
MOV result.color.w, c[5].x;
END
# 33 instructions, 3 R-regs
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
SetTexture 4 [_LightTexture0] 2D
"ps_3_0
; 31 ALU, 4 TEX
dcl_2d s0
dcl_2d s1
dcl_2d s2
dcl_2d s4
def c4, 2.00000000, -1.00000000, 1.00000000, 0.00000000
def c5, 32.00000000, 0, 0, 0
dcl_texcoord0 v0.xy
dcl_texcoord1 v1.xyz
dcl_texcoord2 v2.xyz
dcl_texcoord3 v3.xy
texld r1.yw, v0, s2
mad_pp r1.xy, r1.wyzw, c4.x, c4.y
dp3_pp r1.z, v2, v2
mul_pp r0.w, r1.y, r1.y
mad_pp r0.w, -r1.x, r1.x, -r0
add_pp r0.w, r0, c4.z
rsq_pp r1.z, r1.z
mov_pp r0.xyz, v1
mad_pp r0.xyz, r1.z, v2, r0
dp3_pp r1.z, r0, r0
rsq_pp r1.z, r1.z
mul_pp r0.xyz, r1.z, r0
rsq_pp r0.w, r0.w
rcp_pp r1.z, r0.w
mov_pp r0.w, c2.x
dp3_pp r0.x, r1, r0
mul_pp r2.x, c5, r0.w
max_pp r1.w, r0.x, c4
pow r0, r1.w, r2.x
texld r2.xyz, v0, s1
mul_pp r2.xyz, r2, c3.x
mul r2.xyz, r0.x, r2
texld r0.xyz, v0, s0
mul_pp r0.xyz, r0, c1
dp3_pp r0.w, r1, v1
max_pp r0.w, r0, c4
mul_pp r2.xyz, r2, c0
mul_pp r0.xyz, r0, c0
mad_pp r0.xyz, r0, r0.w, r2
texld r0.w, v3, s4
mul_pp r0.xyz, r0.w, r0
mul_pp oC0.xyz, r0, c4.x
mov_pp oC0.w, c4
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
aaaagaahgaanbcaabcaaaaaaaaaafabdaaaaccaaaaaaaaaabadieaabbpbppoii
aaaaeaaabaaidagbbpbpppplaaaaeaaabacidaabbpbppompaaaaeaaababiaaab
bpbppoecaaaaeaaamiaiaaafaagmblaacbacppaamiaiaaaaaaloloaapaacacaa
fiihaaadaalelebloaadadiamiahaaafaablmamaolaaacabmiaiaaaaaaloloaa
paafafaafiigabacaambgmblkaadppiamiaiaaaaaelclcmgnbacacppkaihacaf
aamablblobafabiakibbacabaamdloebnaacababkiccacabaalomdicnaafacab
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
SetTexture 4 [_LightTexture0] 2D
"sce_fp_rsx // 41 instructions using 4 registers
[Configuration]
24
ffffffff0003c020000ffff9000000000000840004000000
[Offsets]
4
_LightColor0 2 0
00000260000001b0
_Color 1 0
000000e0
_Shininess 1 0
00000130
_Gloss 1 0
00000020
[Microcode]
656
8e001702c8011c9dc8000001c8003fe10e860240c8001c9d00020000c8000001
0000000000000000000000000000000094041704c8011c9dc8000001c8003fe1
ce803940c8011c9dc8000029c800bfe106820440ce081c9d00020000aa020000
000040000000bf80000000000000000010840240ab041c9cab040000c8000001
ae840140c8011c9dc8000001c8003fe10e060340c9081c9dc9000001c8000001
1084044001041c9e01040000c90800038e041700c8011c9dc8000001c8003fe1
0e803940c80c1c9dc8000029c80000010e880240c8081c9dc8020001c8000001
0000000000000000000000000000000010800340c9081c9dc8020001c8000001
00000000000000000000000000003f8008823b40ff003c9dff000001c8000001
028a014000021c9cc8000001c800000100000000000000000000000000000000
1088024001141c9c00020000c800000100004200000000000000000000000000
02800540c9041c9dc9000001c80000011004090001001c9c00020000c8000001
0000000000000000000000000000000010840540c9041c9dc9080001c8000001
0e840240c9101c9dc8020001c800000100000000000000000000000000000000
08041d00fe081c9dc8000001c80000010204020054081c9dff100001c8000001
10800900c9081c9d00020000c800000100000000000000000000000000000000
0e800240c9081c9dff000001c800000108001c00c8081c9dc8000001c8000001
0e82020054001c9dc90c0001c80000011080014000021c9cc8000001c8000001
000000000000000000000000000000000e800440c9041c9dc8020001c9000001
00000000000000000000000000000000f0001708c8011c9dc8000001c8003fe1
0e810240fe001c9dc9001001c8000001
"
}

SubProgram "d3d11 " {
Keywords { "DIRECTIONAL_COOKIE" }
ConstBuffer "$Globals" 176 // 156 used size, 10 vars
Vector 16 [_LightColor0] 4
Vector 112 [_Color] 4
Float 148 [_Shininess]
Float 152 [_Gloss]
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 1
SetTexture 1 [_SpecMap] 2D 3
SetTexture 2 [_BumpMap] 2D 2
SetTexture 3 [_LightTexture0] 2D 0
// 31 instructions, 2 temp regs, 0 temp arrays:
// ALU 20 float, 0 int, 0 uint
// TEX 4 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0
eefiecedcikllphaigiicllldddfcjgfcckabdclabaaaaaahiafaaaaadaaaaaa
cmaaaaaammaaaaaaaaabaaaaejfdeheojiaaaaaaafaaaaaaaiaaaaaaiaaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaadadaaaaimaaaaaaadaaaaaaaaaaaaaaadaaaaaaabaaaaaa
amamaaaaimaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaaahahaaaaimaaaaaa
acaaaaaaaaaaaaaaadaaaaaaadaaaaaaahahaaaafdfgfpfaepfdejfeejepeoaa
feeffiedepepfceeaaklklklepfdeheocmaaaaaaabaaaaaaaiaaaaaacaaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaafdfgfpfegbhcghgfheaaklkl
fdeieefchaaeaaaaeaaaaaaabmabaaaafjaaaaaeegiocaaaaaaaaaaaakaaaaaa
fkaaaaadaagabaaaaaaaaaaafkaaaaadaagabaaaabaaaaaafkaaaaadaagabaaa
acaaaaaafkaaaaadaagabaaaadaaaaaafibiaaaeaahabaaaaaaaaaaaffffaaaa
fibiaaaeaahabaaaabaaaaaaffffaaaafibiaaaeaahabaaaacaaaaaaffffaaaa
fibiaaaeaahabaaaadaaaaaaffffaaaagcbaaaaddcbabaaaabaaaaaagcbaaaad
mcbabaaaabaaaaaagcbaaaadhcbabaaaacaaaaaagcbaaaadhcbabaaaadaaaaaa
gfaaaaadpccabaaaaaaaaaaagiaaaaacacaaaaaabaaaaaahbcaabaaaaaaaaaaa
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
aaaaaaaaakaabaaaaaaaaaaadiaaaaaiecaabaaaaaaaaaaabkiacaaaaaaaaaaa
ajaaaaaaabeaaaaaaaaaaaecdiaaaaahbcaabaaaaaaaaaaaakaabaaaaaaaaaaa
ckaabaaaaaaaaaaabjaaaaafbcaabaaaaaaaaaaaakaabaaaaaaaaaaaefaaaaaj
pcaabaaaabaaaaaaegbabaaaabaaaaaaeghobaaaabaaaaaaaagabaaaadaaaaaa
diaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaakgikcaaaaaaaaaaaajaaaaaa
diaaaaahncaabaaaaaaaaaaaagaabaaaaaaaaaaaagajbaaaabaaaaaadiaaaaai
ncaabaaaaaaaaaaaagaobaaaaaaaaaaaagijcaaaaaaaaaaaabaaaaaaefaaaaaj
pcaabaaaabaaaaaaegbabaaaabaaaaaaeghobaaaaaaaaaaaaagabaaaabaaaaaa
diaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaaegiccaaaaaaaaaaaahaaaaaa
diaaaaaihcaabaaaabaaaaaaegacbaaaabaaaaaaegiccaaaaaaaaaaaabaaaaaa
dcaaaaajhcaabaaaaaaaaaaaegacbaaaabaaaaaafgafbaaaaaaaaaaaigadbaaa
aaaaaaaaefaaaaajpcaabaaaabaaaaaaogbkbaaaabaaaaaaeghobaaaadaaaaaa
aagabaaaaaaaaaaaaaaaaaahicaabaaaaaaaaaaadkaabaaaabaaaaaadkaabaaa
abaaaaaadiaaaaahhccabaaaaaaaaaaapgapbaaaaaaaaaaaegacbaaaaaaaaaaa
dgaaaaaficcabaaaaaaaaaaaabeaaaaaaaaaaaaadoaaaaab"
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
	Pass {
		Name "PREPASS"
		Tags { "LightMode" = "PrePassBase" }
		Fog {Mode Off}
Program "vp" {
// Vertex combos: 1
//   opengl - ALU: 21 to 21
//   d3d9 - ALU: 22 to 22
//   d3d11 - ALU: 15 to 15, TEX: 0 to 0, FLOW: 1 to 1
SubProgram "opengl " {
Keywords { }
Bind "vertex" Vertex
Bind "tangent" ATTR14
Bind "normal" Normal
Bind "texcoord" TexCoord0
Matrix 5 [_Object2World]
Vector 9 [unity_Scale]
Vector 10 [_MainTex_ST]
"3.0-!!ARBvp1.0
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
"vs_3_0
; 22 ALU
dcl_position o0
dcl_texcoord0 o1
dcl_texcoord1 o2
dcl_texcoord2 o3
dcl_texcoord3 o4
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
mul o2.xyz, r0, c8.w
dp3 r0.y, r1, c5
dp3 r0.x, v1, c5
dp3 r0.z, v2, c5
mul o3.xyz, r0, c8.w
dp3 r0.y, r1, c6
dp3 r0.x, v1, c6
dp3 r0.z, v2, c6
mul o4.xyz, r0, c8.w
mad o1.xy, v3, c9, c9.zwzw
dp4 o0.w, v0, c3
dp4 o0.z, v0, c2
dp4 o0.y, v0, c1
dp4 o0.x, v0, c0
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
ConstBuffer "$Globals" 112 // 112 used size, 9 vars
Vector 96 [_MainTex_ST] 4
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
eefiecedpaekonjhagdjlhoopgkbnldomcfoneihabaaaaaajiafaaaaadaaaaaa
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
ppaaaaaafjaaaaaeegiocaaaaaaaaaaaahaaaaaafjaaaaaeegiocaaaabaaaaaa
bfaaaaaafpaaaaadpcbabaaaaaaaaaaafpaaaaadpcbabaaaabaaaaaafpaaaaad
hcbabaaaacaaaaaafpaaaaaddcbabaaaadaaaaaaghaaaaaepccabaaaaaaaaaaa
abaaaaaagfaaaaaddccabaaaabaaaaaagfaaaaadhccabaaaacaaaaaagfaaaaad
hccabaaaadaaaaaagfaaaaadhccabaaaaeaaaaaagiaaaaacadaaaaaadiaaaaai
pcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaabaaaaaaabaaaaaadcaaaaak
pcaabaaaaaaaaaaaegiocaaaabaaaaaaaaaaaaaaagbabaaaaaaaaaaaegaobaaa
aaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaabaaaaaaacaaaaaakgbkbaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaakpccabaaaaaaaaaaaegiocaaaabaaaaaa
adaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaaldccabaaaabaaaaaa
egbabaaaadaaaaaaegiacaaaaaaaaaaaagaaaaaaogikcaaaaaaaaaaaagaaaaaa
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

}
Program "fp" {
// Fragment combos: 1
//   opengl - ALU: 12 to 12, TEX: 1 to 1
//   d3d9 - ALU: 11 to 11, TEX: 1 to 1
//   d3d11 - ALU: 4 to 4, TEX: 1 to 1, FLOW: 1 to 1
SubProgram "opengl " {
Keywords { }
Float 0 [_Shininess]
SetTexture 0 [_BumpMap] 2D
"3.0-!!ARBfp1.0
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
"ps_3_0
; 11 ALU, 1 TEX
dcl_2d s0
def c1, 2.00000000, -1.00000000, 1.00000000, 0.50000000
dcl_texcoord0 v0.xy
dcl_texcoord1 v1.xyz
dcl_texcoord2 v2.xyz
dcl_texcoord3 v3.xyz
texld r0.yw, v0, s0
mad_pp r0.xy, r0.wyzw, c1.x, c1.y
mul_pp r0.z, r0.y, r0.y
mad_pp r0.z, -r0.x, r0.x, -r0
add_pp r0.z, r0, c1
rsq_pp r0.z, r0.z
rcp_pp r0.z, r0.z
dp3 r1.z, v3, r0
dp3 r1.x, r0, v1
dp3 r1.y, r0, v2
mad_pp oC0.xyz, r1, c1.w, c1.w
mov_pp oC0.w, c0.x
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
ConstBuffer "$Globals" 112 // 88 used size, 9 vars
Float 84 [_Shininess]
BindCB "$Globals" 0
SetTexture 0 [_BumpMap] 2D 0
// 11 instructions, 2 temp regs, 0 temp arrays:
// ALU 4 float, 0 int, 0 uint
// TEX 1 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0
eefiecedmihjfnhaeendabggcnbpadjimeofdmjcabaaaaaapaacaaaaadaaaaaa
cmaaaaaammaaaaaaaaabaaaaejfdeheojiaaaaaaafaaaaaaaiaaaaaaiaaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaimaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaadadaaaaimaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaaimaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaahahaaaaimaaaaaa
adaaaaaaaaaaaaaaadaaaaaaaeaaaaaaahahaaaafdfgfpfaepfdejfeejepeoaa
feeffiedepepfceeaaklklklepfdeheocmaaaaaaabaaaaaaaiaaaaaacaaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaafdfgfpfegbhcghgfheaaklkl
fdeieefcoiabaaaaeaaaaaaahkaaaaaafjaaaaaeegiocaaaaaaaaaaaagaaaaaa
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
bkiacaaaaaaaaaaaafaaaaaadoaaaaab"
}

SubProgram "gles " {
Keywords { }
"!!GLES"
}

SubProgram "glesdesktop " {
Keywords { }
"!!GLES"
}

}
	}
	Pass {
		Name "PREPASS"
		Tags { "LightMode" = "PrePassFinal" }
		ZWrite Off
Program "vp" {
// Vertex combos: 4
//   opengl - ALU: 46 to 54
//   d3d9 - ALU: 47 to 55
//   d3d11 - ALU: 27 to 32, TEX: 0 to 0, FLOW: 1 to 1
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
"3.0-!!ARBvp1.0
# 54 ALU
PARAM c[24] = { { 0.5, 1 },
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
MOV R0.w, c[0].y;
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
MUL R1.xyz, vertex.attrib[14].w, R0;
MOV R0.xyz, c[13];
MOV R0.w, c[0].y;
ADD result.texcoord[5].xyz, R2, R3;
DP4 R2.z, R0, c[11];
DP4 R2.x, R0, c[9];
DP4 R2.y, R0, c[10];
MAD R2.xyz, R2, c[22].w, -vertex.position;
DP3 R0.y, R1, c[5];
DP3 R0.w, -R2, c[5];
DP4 R1.w, vertex.position, c[4];
DP3 R0.x, vertex.attrib[14], c[5];
DP3 R0.z, vertex.normal, c[5];
MUL result.texcoord[2], R0, c[22].w;
DP3 R0.y, R1, c[6];
DP3 R0.w, -R2, c[6];
DP3 R0.x, vertex.attrib[14], c[6];
DP3 R0.z, vertex.normal, c[6];
MUL result.texcoord[3], R0, c[22].w;
DP3 R0.y, R1, c[7];
DP4 R1.z, vertex.position, c[3];
DP3 R0.w, -R2, c[7];
DP4 R1.x, vertex.position, c[1];
DP4 R1.y, vertex.position, c[2];
MUL R2.xyz, R1.xyww, c[0].x;
DP3 R0.x, vertex.attrib[14], c[7];
DP3 R0.z, vertex.normal, c[7];
MUL result.texcoord[4], R0, c[22].w;
MOV R0.x, R2;
MUL R0.y, R2, c[14].x;
ADD result.texcoord[1].xy, R0, R2.z;
MOV result.position, R1;
MOV result.texcoord[1].zw, R1;
MAD result.texcoord[0].xy, vertex.texcoord[0], c[23], c[23].zwzw;
END
# 54 instructions, 4 R-regs
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
"vs_3_0
; 55 ALU
dcl_position o0
dcl_texcoord0 o1
dcl_texcoord1 o2
dcl_texcoord2 o3
dcl_texcoord3 o4
dcl_texcoord4 o5
dcl_texcoord5 o6
def c24, 0.50000000, 1.00000000, 0, 0
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
mov r0.w, c24.y
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
mul r1.xyz, v1.w, r0
mov r0.xyz, c12
mov r0.w, c24.y
add o6.xyz, r2, r3
dp4 r2.z, r0, c10
dp4 r2.x, r0, c8
dp4 r2.y, r0, c9
mad r2.xyz, r2, c22.w, -v0
dp3 r0.y, r1, c4
dp3 r0.w, -r2, c4
dp4 r1.w, v0, c3
dp3 r0.x, v1, c4
dp3 r0.z, v2, c4
mul o3, r0, c22.w
dp3 r0.y, r1, c5
dp3 r0.w, -r2, c5
dp3 r0.x, v1, c5
dp3 r0.z, v2, c5
mul o4, r0, c22.w
dp3 r0.y, r1, c6
dp4 r1.z, v0, c2
dp3 r0.w, -r2, c6
dp4 r1.x, v0, c0
dp4 r1.y, v0, c1
mul r2.xyz, r1.xyww, c24.x
dp3 r0.x, v1, c6
dp3 r0.z, v2, c6
mul o5, r0, c22.w
mov r0.x, r2
mul r0.y, r2, c13.x
mad o2.xy, r2.z, c14.zwzw, r0
mov o0, r1
mov o2.zw, r1
mad o1.xy, v3, c23, c23.zwzw
"
}

SubProgram "xbox360 " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
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
// ALU: 66.67 (50 instructions), vertex: 32, texture: 0,
//   sequencer: 24,  12 GPRs, 15 threads,
// Performance (if enough threads): ~66 cycles per vector
// * Vertex cycle estimates are assuming 3 vfetch_minis for every vfetch_full,
//     with <= 32 bytes per vfetch_full group.

"vs_360
backbbabaaaaadbeaaaaadbmaaaaaaaaaaaaaaceaaaaaciaaaaaackiaaaaaaaa
aaaaaaaaaaaaacfiaaaaaabmaaaaacekpppoadaaaaaaaaapaaaaaabmaaaaaaaa
aaaaacedaaaaabeiaaacaabgaaabaaaaaaaaabfeaaaaaaaaaaaaabgeaaacaaao
aaadaaaaaaaaabheaaaaaaaaaaaaabieaaacaaabaaabaaaaaaaaabfeaaaaaaaa
aaaaabjgaaacaaacaaabaaaaaaaaabfeaaaaaaaaaaaaabkeaaacaabbaaaeaaaa
aaaaabheaaaaaaaaaaaaablcaaacaaaaaaabaaaaaaaaabmiaaaaaaaaaaaaabni
aaacaaakaaaeaaaaaaaaabheaaaaaaaaaaaaabolaaacaaafaaabaaaaaaaaabfe
aaaaaaaaaaaaabpgaaacaaaeaaabaaaaaaaaabfeaaaaaaaaaaaaacabaaacaaad
aaabaaaaaaaaabfeaaaaaaaaaaaaacamaaacaaaiaaabaaaaaaaaabfeaaaaaaaa
aaaaacbhaaacaaahaaabaaaaaaaaabfeaaaaaaaaaaaaacccaaacaaagaaabaaaa
aaaaabfeaaaaaaaaaaaaaccnaaacaaajaaabaaaaaaaaabfeaaaaaaaaaaaaacdh
aaacaabfaaabaaaaaaaaabfeaaaaaaaafpengbgjgofegfhifpfdfeaaaaabaaad
aaabaaaeaaabaaaaaaaaaaaafpepgcgkgfgdhedcfhgphcgmgeaaklklaaadaaad
aaaeaaaeaaabaaaaaaaaaaaafpfahcgpgkgfgdhegjgpgofagbhcgbgnhdaafpfd
gdhcgfgfgofagbhcgbgnhdaafpfhgphcgmgedcepgcgkgfgdheaafpfhgphcgmge
fdhagbgdgfedgbgngfhcgbfagphdaaklaaabaaadaaabaaadaaabaaaaaaaaaaaa
ghgmhdhegbhegffpgngbhehcgjhifpgnhghaaahfgogjhehjfpfdeiebgcaahfgo
gjhehjfpfdeiebghaahfgogjhehjfpfdeiebhcaahfgogjhehjfpfdeiecgcaahf
gogjhehjfpfdeiecghaahfgogjhehjfpfdeiechcaahfgogjhehjfpfdeiedaahf
gogjhehjfpfdgdgbgmgfaahghdfpddfpdaaadccodacodcdadddfddcodaaaklkl
aaaaaaaaaaaaaaabaaaaaaaaaaaaaaaaaaaaaabeaapmaabaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaeaaaaaacnmaafbaaalaaaaaaaaaaaaaaaaaaaafemg
aaaaaaabaaaaaaaeaaaaaaahaaaaacjaaabaaaagaaaagaahaaaadaaiaacafaaj
aaaadafaaaabpbfbaaadpcfcaaaepdfdaaafpefeaaaghfffaaaabacmaaaaaacl
aaaabadgaaaabadhaaaabadiaaaabadjaaaabadlaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaadpaaaaaaaaaaaaaaaaaaaaaaaaaaaaaapaffeaagaaaabcaamcaaaaaa
aaaafaakaaaabcaameaaaaaaaaaagaapgabfbcaabcaaaaaaaaaagablgacbbcaa
bcaaaaaaaaaagachgacnbcaabcaaaaaaaaaagadddadjbcaaccaaaaaaafpijaaa
aaaaagiiaaaaaaaaafpibaaaaaaaagiiaaaaaaaaafpigaaaaaaaaoiiaaaaaaaa
afpidaaaaaaaapmiaaaaaaaamiapaaaaaabliiaakbajanaamiapaaaaaamgnapi
klajamaamiapaaaaaalbdepiklajalaamiapaaaiaagmiipiklajakaamiapiado
aaiiiiaaocaiaiaamiahaaahaamamgmaalbdaabeceibajakaablgmgmkbabaoia
beceaaakaablgmblkbabbaabkiclakacaamggcebibagbaapbebhaaaeaalbgcmg
kbagapabkiehadafaalbgcmaibabapbamialaaaaaalelbleclbcaaahkmehaaah
aalogficmbagabbamiahaaahabgflomaolagabahmiahaaalaamagmliclbbaaaa
miahaaafaagmmagcklabaoafmiahaaaeaagmmngfklagaoaekmeeacaeaamggmec
maaeacbabealaaaaaagcblmgkbagbfafaebeaeafaagmlbmgoaaeacadbeaoaaag
aagmimlbkbaabaafaebbafagaagmmgmgoaafacaamiahaaacabmablmaklalbfaj
miahaaajaalblebfklaaapagbeahaaaaaamnbllbobahabaeaeemagabaamgigbl
kbaabaacmiahaaajaabllemaklaaaoajkibcabagaalolomanaakahapkichabah
aelbgciaibacapapmiakaaafaalbmbgbklaaaoabmiahaaahaegmloleklacaoah
aiboabacaemghggmkbacbaajaibhaiaaaaligmggkbaippajmiamiaabaaigigaa
ocaiaiaamiadiaaaaalalabkiladbgbgaicbaiadaadoanmbgpadajajaiecaiad
aadoanlbgpaeajajaiieaiadaadoanlmgpafajajaicbabacaakhkhmgkpaiagaj
kiiiaaaeaagmlbebmaahacabbeacaaacaakhkhblkpaiahafaeciaeafaamgmgmg
oaahacabbeaeaaacaakhkhlbkpaiaiafaeciafagaalbblbloaahacabmiadiaab
aamgbkbiklaaacaamiapiaacaaaablaakbagbfaamiapiaadaaaablaakbafbfaa
miapiaaeaaaablaakbaebfaageihaaaaaalologboaadacabmiahiaafaablmagf
klaaajaaaaaaaaaaaaaaaaaaaaaaaaaa"
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
"sce_vp_rsx // 51 instructions using 8 registers
[Configuration]
8
0000003341050800
[Defaults]
1
456 1
3f000000
[Microcode]
816
00021c6c00400e0c0106c0836041dffc00039c6c005d300c0186c0836041dffc
00031c6c009ca20c013fc0c36041dffc401f9c6c011c9808010400d740619f9c
00001c6c01506e0c010600c360411ffc00001c6c0150620c010600c360405ffc
00009c6c01505e0c010600c360411ffc00009c6c0150520c010600c360405ffc
00011c6c01504e0c010600c360411ffc00011c6c0150420c010600c360405ffc
00029c6c01d0300d8106c0c360403ffc00029c6c01d0200d8106c0c360405ffc
00029c6c01d0100d8106c0c360409ffc00029c6c01d0000d8106c0c360411ffc
00019c6c0150400c0c8600c360411ffc00019c6c0150600c0c8600c360405ffc
00001c6c0150500c0c8600c360409ffc00031c6c0190a00c0e86c0c360405ffc
00031c6c0190900c0e86c0c360409ffc00031c6c0190800c0e86c0c360411ffc
00039c6c00800243011844436041dffc00021c6c010002308121846303a1dffc
401f9c6c0040000d8a86c0836041ff80401f9c6c004000558a86c08360407fa0
00031c6c011ca00c0cbfc0e30041dffc00029c6c009c800e0a8000c36041dffc
00029c6c009d202a8a8000c360409ffc00009c6c0080002a8095404360409ffc
00019c6c0040002a8086c08360409ffc401f9c6c00c000080a86c09542a19fa0
00001c6c0150608c0c8600c360403ffc00009c6c0150508c0c8600c360403ffc
00011c6c0150408c0c8600c360403ffc00029c6c00800e7f810604436041dffc
00021c6c019cf00c0686c0c360405ffc00021c6c019d000c0686c0c360409ffc
00021c6c019d100c0686c0c360411ffc00021c6c010000000680036aa0a03ffc
00019c6c0080000d069a03436041fffc00001c6c0150600c0a8600c360409ffc
00009c6c0150500c0a8600c360409ffc00011c6c0150400c0a8600c360409ffc
00029c6c01dcc00d8686c0c360405ffc00029c6c01dcd00d8686c0c360409ffc
00029c6c01dce00d8686c0c360411ffc00019c6c00c0000c0886c08302a1dffc
00021c6c009cb07f888600c36041dffc401f9c6c00c0000c0886c08301a1dfb0
401f9c6c009ca00d84bfc0c36041ffa4401f9c6c009ca00d82bfc0c36041ffa8
401f9c6c009ca00d80bfc0c36041ffad
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
Vector 96 [_MainTex_ST] 4
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
// 61 instructions, 4 temp regs, 0 temp arrays:
// ALU 32 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0
eefiecedmdkkeokklbapfnbcpbdjmjkccmbghjmdabaaaaaaaaakaaaaadaaaaaa
cmaaaaaapeaaaaaameabaaaaejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaa
abaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaljaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfc
enebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheomiaaaaaaahaaaaaa
aiaaaaaalaaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaalmaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaabaaaaaaadamaaaalmaaaaaaabaaaaaaaaaaaaaa
adaaaaaaacaaaaaaapaaaaaalmaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaa
apaaaaaalmaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaalmaaaaaa
aeaaaaaaaaaaaaaaadaaaaaaafaaaaaaapaaaaaalmaaaaaaafaaaaaaaaaaaaaa
adaaaaaaagaaaaaaahaiaaaafdfgfpfaepfdejfeejepeoaafeeffiedepepfcee
aaklklklfdeieefcdeaiaaaaeaaaabaaanacaaaafjaaaaaeegiocaaaaaaaaaaa
ahaaaaaafjaaaaaeegiocaaaabaaaaaaagaaaaaafjaaaaaeegiocaaaacaaaaaa
bjaaaaaafjaaaaaeegiocaaaadaaaaaabfaaaaaafpaaaaadpcbabaaaaaaaaaaa
fpaaaaadpcbabaaaabaaaaaafpaaaaadhcbabaaaacaaaaaafpaaaaaddcbabaaa
adaaaaaaghaaaaaepccabaaaaaaaaaaaabaaaaaagfaaaaaddccabaaaabaaaaaa
gfaaaaadpccabaaaacaaaaaagfaaaaadpccabaaaadaaaaaagfaaaaadpccabaaa
aeaaaaaagfaaaaadpccabaaaafaaaaaagfaaaaadhccabaaaagaaaaaagiaaaaac
aeaaaaaadiaaaaaipcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaadaaaaaa
abaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaaaaaaaaaagbabaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaa
acaaaaaakgbkbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaa
egiocaaaadaaaaaaadaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadgaaaaaf
pccabaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaaldccabaaaabaaaaaaegbabaaa
adaaaaaaegiacaaaaaaaaaaaagaaaaaaogikcaaaaaaaaaaaagaaaaaadiaaaaai
ccaabaaaaaaaaaaabkaabaaaaaaaaaaaakiacaaaabaaaaaaafaaaaaadiaaaaak
ncaabaaaabaaaaaaagahbaaaaaaaaaaaaceaaaaaaaaaaadpaaaaaaaaaaaaaadp
aaaaaadpdgaaaaafmccabaaaacaaaaaakgaobaaaaaaaaaaaaaaaaaahdccabaaa
acaaaaaakgakbaaaabaaaaaamgaabaaaabaaaaaadiaaaaajhcaabaaaaaaaaaaa
fgifcaaaabaaaaaaaeaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaa
aaaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaaabaaaaaaaeaaaaaaegacbaaa
aaaaaaaadcaaaaalhcaabaaaaaaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaa
abaaaaaaaeaaaaaaegacbaaaaaaaaaaaaaaaaaaihcaabaaaaaaaaaaaegacbaaa
aaaaaaaaegiccaaaadaaaaaabdaaaaaadcaaaaalhcaabaaaaaaaaaaaegacbaaa
aaaaaaaapgipcaaaadaaaaaabeaaaaaaegbcbaiaebaaaaaaaaaaaaaadiaaaaaj
hcaabaaaabaaaaaafgafbaiaebaaaaaaaaaaaaaaegiccaaaadaaaaaaanaaaaaa
dcaaaaallcaabaaaaaaaaaaaegiicaaaadaaaaaaamaaaaaaagaabaiaebaaaaaa
aaaaaaaaegaibaaaabaaaaaadcaaaaallcaabaaaaaaaaaaaegiicaaaadaaaaaa
aoaaaaaakgakbaiaebaaaaaaaaaaaaaaegambaaaaaaaaaaadgaaaaaficaabaaa
abaaaaaaakaabaaaaaaaaaaadiaaaaahhcaabaaaacaaaaaajgbebaaaabaaaaaa
cgbjbaaaacaaaaaadcaaaaakhcaabaaaacaaaaaajgbebaaaacaaaaaacgbjbaaa
abaaaaaaegacbaiaebaaaaaaacaaaaaadiaaaaahhcaabaaaacaaaaaaegacbaaa
acaaaaaapgbpbaaaabaaaaaadgaaaaagbcaabaaaadaaaaaaakiacaaaadaaaaaa
amaaaaaadgaaaaagccaabaaaadaaaaaaakiacaaaadaaaaaaanaaaaaadgaaaaag
ecaabaaaadaaaaaaakiacaaaadaaaaaaaoaaaaaabaaaaaahccaabaaaabaaaaaa
egacbaaaacaaaaaaegacbaaaadaaaaaabaaaaaahbcaabaaaabaaaaaaegbcbaaa
abaaaaaaegacbaaaadaaaaaabaaaaaahecaabaaaabaaaaaaegbcbaaaacaaaaaa
egacbaaaadaaaaaadiaaaaaipccabaaaadaaaaaaegaobaaaabaaaaaapgipcaaa
adaaaaaabeaaaaaadgaaaaaficaabaaaabaaaaaabkaabaaaaaaaaaaadgaaaaag
bcaabaaaadaaaaaabkiacaaaadaaaaaaamaaaaaadgaaaaagccaabaaaadaaaaaa
bkiacaaaadaaaaaaanaaaaaadgaaaaagecaabaaaadaaaaaabkiacaaaadaaaaaa
aoaaaaaabaaaaaahccaabaaaabaaaaaaegacbaaaacaaaaaaegacbaaaadaaaaaa
baaaaaahbcaabaaaabaaaaaaegbcbaaaabaaaaaaegacbaaaadaaaaaabaaaaaah
ecaabaaaabaaaaaaegbcbaaaacaaaaaaegacbaaaadaaaaaadiaaaaaipccabaaa
aeaaaaaaegaobaaaabaaaaaapgipcaaaadaaaaaabeaaaaaadgaaaaagbcaabaaa
abaaaaaackiacaaaadaaaaaaamaaaaaadgaaaaagccaabaaaabaaaaaackiacaaa
adaaaaaaanaaaaaadgaaaaagecaabaaaabaaaaaackiacaaaadaaaaaaaoaaaaaa
baaaaaahccaabaaaaaaaaaaaegacbaaaacaaaaaaegacbaaaabaaaaaabaaaaaah
bcaabaaaaaaaaaaaegbcbaaaabaaaaaaegacbaaaabaaaaaabaaaaaahecaabaaa
aaaaaaaaegbcbaaaacaaaaaaegacbaaaabaaaaaadiaaaaaipccabaaaafaaaaaa
egaobaaaaaaaaaaapgipcaaaadaaaaaabeaaaaaadiaaaaaihcaabaaaaaaaaaaa
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
akaabaaaaaaaaaaabkaabaiaebaaaaaaaaaaaaaadcaaaaakhccabaaaagaaaaaa
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

varying highp vec3 xlv_TEXCOORD5;
varying lowp vec4 xlv_TEXCOORD4;
varying lowp vec4 xlv_TEXCOORD3;
varying lowp vec4 xlv_TEXCOORD2;
varying highp vec4 xlv_TEXCOORD1;
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
  lowp vec4 tmpvar_3;
  lowp vec4 tmpvar_4;
  lowp vec4 tmpvar_5;
  highp vec3 tmpvar_6;
  highp vec4 tmpvar_7;
  tmpvar_7 = (gl_ModelViewProjectionMatrix * _glesVertex);
  highp vec4 tmpvar_8;
  tmpvar_8.w = 1.0;
  tmpvar_8.xyz = _WorldSpaceCameraPos;
  mat3 tmpvar_9;
  tmpvar_9[0] = _Object2World[0].xyz;
  tmpvar_9[1] = _Object2World[1].xyz;
  tmpvar_9[2] = _Object2World[2].xyz;
  highp vec3 tmpvar_10;
  tmpvar_10 = (tmpvar_9 * (_glesVertex.xyz - ((_World2Object * tmpvar_8).xyz * unity_Scale.w)));
  highp vec3 tmpvar_11;
  highp vec3 tmpvar_12;
  tmpvar_11 = tmpvar_1.xyz;
  tmpvar_12 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_13;
  tmpvar_13[0].x = tmpvar_11.x;
  tmpvar_13[0].y = tmpvar_12.x;
  tmpvar_13[0].z = tmpvar_2.x;
  tmpvar_13[1].x = tmpvar_11.y;
  tmpvar_13[1].y = tmpvar_12.y;
  tmpvar_13[1].z = tmpvar_2.y;
  tmpvar_13[2].x = tmpvar_11.z;
  tmpvar_13[2].y = tmpvar_12.z;
  tmpvar_13[2].z = tmpvar_2.z;
  vec4 v_14;
  v_14.x = _Object2World[0].x;
  v_14.y = _Object2World[1].x;
  v_14.z = _Object2World[2].x;
  v_14.w = _Object2World[3].x;
  highp vec4 tmpvar_15;
  tmpvar_15.xyz = (tmpvar_13 * v_14.xyz);
  tmpvar_15.w = tmpvar_10.x;
  highp vec4 tmpvar_16;
  tmpvar_16 = (tmpvar_15 * unity_Scale.w);
  tmpvar_3 = tmpvar_16;
  vec4 v_17;
  v_17.x = _Object2World[0].y;
  v_17.y = _Object2World[1].y;
  v_17.z = _Object2World[2].y;
  v_17.w = _Object2World[3].y;
  highp vec4 tmpvar_18;
  tmpvar_18.xyz = (tmpvar_13 * v_17.xyz);
  tmpvar_18.w = tmpvar_10.y;
  highp vec4 tmpvar_19;
  tmpvar_19 = (tmpvar_18 * unity_Scale.w);
  tmpvar_4 = tmpvar_19;
  vec4 v_20;
  v_20.x = _Object2World[0].z;
  v_20.y = _Object2World[1].z;
  v_20.z = _Object2World[2].z;
  v_20.w = _Object2World[3].z;
  highp vec4 tmpvar_21;
  tmpvar_21.xyz = (tmpvar_13 * v_20.xyz);
  tmpvar_21.w = tmpvar_10.z;
  highp vec4 tmpvar_22;
  tmpvar_22 = (tmpvar_21 * unity_Scale.w);
  tmpvar_5 = tmpvar_22;
  highp vec4 o_23;
  highp vec4 tmpvar_24;
  tmpvar_24 = (tmpvar_7 * 0.5);
  highp vec2 tmpvar_25;
  tmpvar_25.x = tmpvar_24.x;
  tmpvar_25.y = (tmpvar_24.y * _ProjectionParams.x);
  o_23.xy = (tmpvar_25 + tmpvar_24.w);
  o_23.zw = tmpvar_7.zw;
  mat3 tmpvar_26;
  tmpvar_26[0] = _Object2World[0].xyz;
  tmpvar_26[1] = _Object2World[1].xyz;
  tmpvar_26[2] = _Object2World[2].xyz;
  highp vec4 tmpvar_27;
  tmpvar_27.w = 1.0;
  tmpvar_27.xyz = (tmpvar_26 * (tmpvar_2 * unity_Scale.w));
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
  tmpvar_6 = tmpvar_28;
  gl_Position = tmpvar_7;
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = o_23;
  xlv_TEXCOORD2 = tmpvar_3;
  xlv_TEXCOORD3 = tmpvar_4;
  xlv_TEXCOORD4 = tmpvar_5;
  xlv_TEXCOORD5 = tmpvar_6;
}



#endif
#ifdef FRAGMENT

varying highp vec3 xlv_TEXCOORD5;
varying lowp vec4 xlv_TEXCOORD4;
varying lowp vec4 xlv_TEXCOORD3;
varying lowp vec4 xlv_TEXCOORD2;
varying highp vec4 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform sampler2D _LightBuffer;
uniform mediump float _Gloss;
uniform mediump float _ReflectPower;
uniform lowp vec4 _ReflectColor;
uniform lowp vec4 _Color;
uniform samplerCube _Cube;
uniform sampler2D _SpecMap;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform lowp vec4 _SpecColor;
void main ()
{
  lowp vec4 tmpvar_1;
  mediump vec4 c_2;
  mediump vec4 light_3;
  highp vec3 tmpvar_4;
  mediump vec3 tmpvar_5;
  mediump vec3 tmpvar_6;
  mediump vec3 tmpvar_7;
  lowp vec3 tmpvar_8;
  tmpvar_8.x = xlv_TEXCOORD2.w;
  tmpvar_8.y = xlv_TEXCOORD3.w;
  tmpvar_8.z = xlv_TEXCOORD4.w;
  tmpvar_4 = tmpvar_8;
  lowp vec3 tmpvar_9;
  tmpvar_9 = xlv_TEXCOORD2.xyz;
  tmpvar_5 = tmpvar_9;
  lowp vec3 tmpvar_10;
  tmpvar_10 = xlv_TEXCOORD3.xyz;
  tmpvar_6 = tmpvar_10;
  lowp vec3 tmpvar_11;
  tmpvar_11 = xlv_TEXCOORD4.xyz;
  tmpvar_7 = tmpvar_11;
  mediump vec3 tmpvar_12;
  mediump vec3 tmpvar_13;
  mediump float tmpvar_14;
  mediump vec4 spec_15;
  lowp vec4 tmpvar_16;
  tmpvar_16 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_17;
  tmpvar_17 = (tmpvar_16.xyz * _Color.xyz);
  tmpvar_12 = tmpvar_17;
  lowp vec4 tmpvar_18;
  tmpvar_18 = texture2D (_SpecMap, xlv_TEXCOORD0);
  spec_15 = tmpvar_18;
  lowp vec3 tmpvar_19;
  tmpvar_19 = ((texture2D (_BumpMap, xlv_TEXCOORD0).xyz * 2.0) - 1.0);
  tmpvar_13 = tmpvar_19;
  mediump vec3 tmpvar_20;
  tmpvar_20.x = dot (tmpvar_5, tmpvar_13);
  tmpvar_20.y = dot (tmpvar_6, tmpvar_13);
  tmpvar_20.z = dot (tmpvar_7, tmpvar_13);
  highp vec3 tmpvar_21;
  tmpvar_21 = (tmpvar_4 - (2.0 * (dot (tmpvar_20, tmpvar_4) * tmpvar_20)));
  lowp vec4 tmpvar_22;
  tmpvar_22 = (textureCube (_Cube, tmpvar_21) * tmpvar_16.w);
  lowp float tmpvar_23;
  tmpvar_23 = (tmpvar_22.w * _ReflectColor.w);
  tmpvar_14 = tmpvar_23;
  lowp vec4 tmpvar_24;
  tmpvar_24 = texture2DProj (_LightBuffer, xlv_TEXCOORD1);
  light_3 = tmpvar_24;
  mediump vec4 tmpvar_25;
  tmpvar_25 = -(log2(max (light_3, vec4(0.001, 0.001, 0.001, 0.001))));
  light_3.w = tmpvar_25.w;
  highp vec3 tmpvar_26;
  tmpvar_26 = (tmpvar_25.xyz + xlv_TEXCOORD5);
  light_3.xyz = tmpvar_26;
  mediump vec4 c_27;
  mediump vec3 tmpvar_28;
  tmpvar_28 = (tmpvar_25.w * (spec_15.xyz * _Gloss));
  c_27.xyz = ((tmpvar_12 * light_3.xyz) + (light_3.xyz * tmpvar_28));
  c_27.w = (tmpvar_14 + (tmpvar_28 * _SpecColor.w)).x;
  c_2.w = c_27.w;
  c_2.xyz = (c_27.xyz + ((tmpvar_22.xyz * _ReflectColor.xyz) * _ReflectPower));
  tmpvar_1 = c_2;
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

varying highp vec3 xlv_TEXCOORD5;
varying lowp vec4 xlv_TEXCOORD4;
varying lowp vec4 xlv_TEXCOORD3;
varying lowp vec4 xlv_TEXCOORD2;
varying highp vec4 xlv_TEXCOORD1;
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
  lowp vec4 tmpvar_3;
  lowp vec4 tmpvar_4;
  lowp vec4 tmpvar_5;
  highp vec3 tmpvar_6;
  highp vec4 tmpvar_7;
  tmpvar_7 = (gl_ModelViewProjectionMatrix * _glesVertex);
  highp vec4 tmpvar_8;
  tmpvar_8.w = 1.0;
  tmpvar_8.xyz = _WorldSpaceCameraPos;
  mat3 tmpvar_9;
  tmpvar_9[0] = _Object2World[0].xyz;
  tmpvar_9[1] = _Object2World[1].xyz;
  tmpvar_9[2] = _Object2World[2].xyz;
  highp vec3 tmpvar_10;
  tmpvar_10 = (tmpvar_9 * (_glesVertex.xyz - ((_World2Object * tmpvar_8).xyz * unity_Scale.w)));
  highp vec3 tmpvar_11;
  highp vec3 tmpvar_12;
  tmpvar_11 = tmpvar_1.xyz;
  tmpvar_12 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_13;
  tmpvar_13[0].x = tmpvar_11.x;
  tmpvar_13[0].y = tmpvar_12.x;
  tmpvar_13[0].z = tmpvar_2.x;
  tmpvar_13[1].x = tmpvar_11.y;
  tmpvar_13[1].y = tmpvar_12.y;
  tmpvar_13[1].z = tmpvar_2.y;
  tmpvar_13[2].x = tmpvar_11.z;
  tmpvar_13[2].y = tmpvar_12.z;
  tmpvar_13[2].z = tmpvar_2.z;
  vec4 v_14;
  v_14.x = _Object2World[0].x;
  v_14.y = _Object2World[1].x;
  v_14.z = _Object2World[2].x;
  v_14.w = _Object2World[3].x;
  highp vec4 tmpvar_15;
  tmpvar_15.xyz = (tmpvar_13 * v_14.xyz);
  tmpvar_15.w = tmpvar_10.x;
  highp vec4 tmpvar_16;
  tmpvar_16 = (tmpvar_15 * unity_Scale.w);
  tmpvar_3 = tmpvar_16;
  vec4 v_17;
  v_17.x = _Object2World[0].y;
  v_17.y = _Object2World[1].y;
  v_17.z = _Object2World[2].y;
  v_17.w = _Object2World[3].y;
  highp vec4 tmpvar_18;
  tmpvar_18.xyz = (tmpvar_13 * v_17.xyz);
  tmpvar_18.w = tmpvar_10.y;
  highp vec4 tmpvar_19;
  tmpvar_19 = (tmpvar_18 * unity_Scale.w);
  tmpvar_4 = tmpvar_19;
  vec4 v_20;
  v_20.x = _Object2World[0].z;
  v_20.y = _Object2World[1].z;
  v_20.z = _Object2World[2].z;
  v_20.w = _Object2World[3].z;
  highp vec4 tmpvar_21;
  tmpvar_21.xyz = (tmpvar_13 * v_20.xyz);
  tmpvar_21.w = tmpvar_10.z;
  highp vec4 tmpvar_22;
  tmpvar_22 = (tmpvar_21 * unity_Scale.w);
  tmpvar_5 = tmpvar_22;
  highp vec4 o_23;
  highp vec4 tmpvar_24;
  tmpvar_24 = (tmpvar_7 * 0.5);
  highp vec2 tmpvar_25;
  tmpvar_25.x = tmpvar_24.x;
  tmpvar_25.y = (tmpvar_24.y * _ProjectionParams.x);
  o_23.xy = (tmpvar_25 + tmpvar_24.w);
  o_23.zw = tmpvar_7.zw;
  mat3 tmpvar_26;
  tmpvar_26[0] = _Object2World[0].xyz;
  tmpvar_26[1] = _Object2World[1].xyz;
  tmpvar_26[2] = _Object2World[2].xyz;
  highp vec4 tmpvar_27;
  tmpvar_27.w = 1.0;
  tmpvar_27.xyz = (tmpvar_26 * (tmpvar_2 * unity_Scale.w));
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
  tmpvar_6 = tmpvar_28;
  gl_Position = tmpvar_7;
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = o_23;
  xlv_TEXCOORD2 = tmpvar_3;
  xlv_TEXCOORD3 = tmpvar_4;
  xlv_TEXCOORD4 = tmpvar_5;
  xlv_TEXCOORD5 = tmpvar_6;
}



#endif
#ifdef FRAGMENT

varying highp vec3 xlv_TEXCOORD5;
varying lowp vec4 xlv_TEXCOORD4;
varying lowp vec4 xlv_TEXCOORD3;
varying lowp vec4 xlv_TEXCOORD2;
varying highp vec4 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform sampler2D _LightBuffer;
uniform mediump float _Gloss;
uniform mediump float _ReflectPower;
uniform lowp vec4 _ReflectColor;
uniform lowp vec4 _Color;
uniform samplerCube _Cube;
uniform sampler2D _SpecMap;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform lowp vec4 _SpecColor;
void main ()
{
  lowp vec4 tmpvar_1;
  mediump vec4 c_2;
  mediump vec4 light_3;
  highp vec3 tmpvar_4;
  mediump vec3 tmpvar_5;
  mediump vec3 tmpvar_6;
  mediump vec3 tmpvar_7;
  lowp vec3 tmpvar_8;
  tmpvar_8.x = xlv_TEXCOORD2.w;
  tmpvar_8.y = xlv_TEXCOORD3.w;
  tmpvar_8.z = xlv_TEXCOORD4.w;
  tmpvar_4 = tmpvar_8;
  lowp vec3 tmpvar_9;
  tmpvar_9 = xlv_TEXCOORD2.xyz;
  tmpvar_5 = tmpvar_9;
  lowp vec3 tmpvar_10;
  tmpvar_10 = xlv_TEXCOORD3.xyz;
  tmpvar_6 = tmpvar_10;
  lowp vec3 tmpvar_11;
  tmpvar_11 = xlv_TEXCOORD4.xyz;
  tmpvar_7 = tmpvar_11;
  mediump vec3 tmpvar_12;
  mediump vec3 tmpvar_13;
  mediump float tmpvar_14;
  mediump vec4 spec_15;
  lowp vec4 tmpvar_16;
  tmpvar_16 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_17;
  tmpvar_17 = (tmpvar_16.xyz * _Color.xyz);
  tmpvar_12 = tmpvar_17;
  lowp vec4 tmpvar_18;
  tmpvar_18 = texture2D (_SpecMap, xlv_TEXCOORD0);
  spec_15 = tmpvar_18;
  lowp vec3 normal_19;
  normal_19.xy = ((texture2D (_BumpMap, xlv_TEXCOORD0).wy * 2.0) - 1.0);
  normal_19.z = sqrt(((1.0 - (normal_19.x * normal_19.x)) - (normal_19.y * normal_19.y)));
  tmpvar_13 = normal_19;
  mediump vec3 tmpvar_20;
  tmpvar_20.x = dot (tmpvar_5, tmpvar_13);
  tmpvar_20.y = dot (tmpvar_6, tmpvar_13);
  tmpvar_20.z = dot (tmpvar_7, tmpvar_13);
  highp vec3 tmpvar_21;
  tmpvar_21 = (tmpvar_4 - (2.0 * (dot (tmpvar_20, tmpvar_4) * tmpvar_20)));
  lowp vec4 tmpvar_22;
  tmpvar_22 = (textureCube (_Cube, tmpvar_21) * tmpvar_16.w);
  lowp float tmpvar_23;
  tmpvar_23 = (tmpvar_22.w * _ReflectColor.w);
  tmpvar_14 = tmpvar_23;
  lowp vec4 tmpvar_24;
  tmpvar_24 = texture2DProj (_LightBuffer, xlv_TEXCOORD1);
  light_3 = tmpvar_24;
  mediump vec4 tmpvar_25;
  tmpvar_25 = -(log2(max (light_3, vec4(0.001, 0.001, 0.001, 0.001))));
  light_3.w = tmpvar_25.w;
  highp vec3 tmpvar_26;
  tmpvar_26 = (tmpvar_25.xyz + xlv_TEXCOORD5);
  light_3.xyz = tmpvar_26;
  mediump vec4 c_27;
  mediump vec3 tmpvar_28;
  tmpvar_28 = (tmpvar_25.w * (spec_15.xyz * _Gloss));
  c_27.xyz = ((tmpvar_12 * light_3.xyz) + (light_3.xyz * tmpvar_28));
  c_27.w = (tmpvar_14 + (tmpvar_28 * _SpecColor.w)).x;
  c_2.w = c_27.w;
  c_2.xyz = (c_27.xyz + ((tmpvar_22.xyz * _ReflectColor.xyz) * _ReflectPower));
  tmpvar_1 = c_2;
  gl_FragData[0] = tmpvar_1;
}



#endif"
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
"3.0-!!ARBvp1.0
# 46 ALU
PARAM c[23] = { { 0.5, 1 },
		state.matrix.modelview[0],
		state.matrix.mvp,
		program.local[9..22] };
TEMP R0;
TEMP R1;
TEMP R2;
MOV R0.xyz, vertex.attrib[14];
MUL R1.xyz, vertex.normal.zxyw, R0.yzxw;
MAD R0.xyz, vertex.normal.yzxw, R0.zxyw, -R1;
MUL R1.xyz, vertex.attrib[14].w, R0;
MOV R0.xyz, c[17];
MOV R0.w, c[0].y;
DP4 R2.z, R0, c[15];
DP4 R2.x, R0, c[13];
DP4 R2.y, R0, c[14];
MAD R2.xyz, R2, c[20].w, -vertex.position;
DP3 R0.y, R1, c[9];
DP3 R0.w, -R2, c[9];
DP4 R1.w, vertex.position, c[8];
DP3 R0.x, vertex.attrib[14], c[9];
DP3 R0.z, vertex.normal, c[9];
MUL result.texcoord[2], R0, c[20].w;
DP3 R0.y, R1, c[10];
DP3 R0.w, -R2, c[10];
DP3 R0.x, vertex.attrib[14], c[10];
DP3 R0.z, vertex.normal, c[10];
MUL result.texcoord[3], R0, c[20].w;
DP3 R0.y, R1, c[11];
DP4 R1.z, vertex.position, c[7];
DP3 R0.w, -R2, c[11];
DP4 R1.x, vertex.position, c[5];
DP4 R1.y, vertex.position, c[6];
MUL R2.xyz, R1.xyww, c[0].x;
DP3 R0.x, vertex.attrib[14], c[11];
DP3 R0.z, vertex.normal, c[11];
MUL result.texcoord[4], R0, c[20].w;
MOV R0.x, R2;
MUL R0.y, R2, c[18].x;
ADD result.texcoord[1].xy, R0, R2.z;
DP4 R0.z, vertex.position, c[11];
DP4 R0.x, vertex.position, c[9];
DP4 R0.y, vertex.position, c[10];
ADD R0.xyz, R0, -c[19];
MUL result.texcoord[6].xyz, R0, c[19].w;
MOV R0.x, c[0].y;
ADD R0.y, R0.x, -c[19].w;
DP4 R0.x, vertex.position, c[3];
MOV result.position, R1;
MOV result.texcoord[1].zw, R1;
MAD result.texcoord[0].xy, vertex.texcoord[0], c[22], c[22].zwzw;
MAD result.texcoord[5].xy, vertex.texcoord[1], c[21], c[21].zwzw;
MUL result.texcoord[6].w, -R0.x, R0.y;
END
# 46 instructions, 3 R-regs
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
"vs_3_0
; 47 ALU
dcl_position o0
dcl_texcoord0 o1
dcl_texcoord1 o2
dcl_texcoord2 o3
dcl_texcoord3 o4
dcl_texcoord4 o5
dcl_texcoord5 o6
dcl_texcoord6 o7
def c23, 0.50000000, 1.00000000, 0, 0
dcl_position0 v0
dcl_tangent0 v1
dcl_normal0 v2
dcl_texcoord0 v3
dcl_texcoord1 v4
mov r0.xyz, v1
mul r1.xyz, v2.zxyw, r0.yzxw
mov r0.xyz, v1
mad r0.xyz, v2.yzxw, r0.zxyw, -r1
mul r1.xyz, v1.w, r0
mov r0.xyz, c16
mov r0.w, c23.y
dp4 r2.z, r0, c14
dp4 r2.x, r0, c12
dp4 r2.y, r0, c13
mad r2.xyz, r2, c20.w, -v0
dp3 r0.y, r1, c8
dp3 r0.w, -r2, c8
dp4 r1.w, v0, c7
dp3 r0.x, v1, c8
dp3 r0.z, v2, c8
mul o3, r0, c20.w
dp3 r0.y, r1, c9
dp3 r0.w, -r2, c9
dp3 r0.x, v1, c9
dp3 r0.z, v2, c9
mul o4, r0, c20.w
dp3 r0.y, r1, c10
dp4 r1.z, v0, c6
dp3 r0.w, -r2, c10
dp4 r1.x, v0, c4
dp4 r1.y, v0, c5
mul r2.xyz, r1.xyww, c23.x
dp3 r0.x, v1, c10
dp3 r0.z, v2, c10
mul o5, r0, c20.w
mov r0.x, r2
mul r0.y, r2, c17.x
mad o2.xy, r2.z, c18.zwzw, r0
dp4 r0.z, v0, c10
dp4 r0.x, v0, c8
dp4 r0.y, v0, c9
add r0.xyz, r0, -c19
mul o7.xyz, r0, c19.w
mov r0.x, c19.w
add r0.y, c23, -r0.x
dp4 r0.x, v0, c2
mov o0, r1
mov o2.zw, r1
mad o1.xy, v3, c22, c22.zwzw
mad o6.xy, v4, c21, c21.zwzw
mul o7.w, -r0.x, r0.y
"
}

SubProgram "xbox360 " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
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
// ALU: 66.67 (50 instructions), vertex: 64, texture: 0,
//   sequencer: 24,  14 GPRs, 12 threads,
// Performance (if enough threads): ~66 cycles per vector
// * Vertex cycle estimates are assuming 3 vfetch_minis for every vfetch_full,
//     with <= 32 bytes per vfetch_full group.

"vs_360
backbbabaaaaacmmaaaaadciaaaaaaaaaaaaaaceaaaaaccmaaaaacfeaaaaaaaa
aaaaaaaaaaaaacaeaaaaaabmaaaaabphpppoadaaaaaaaaalaaaaaabmaaaaaaaa
aaaaabpaaaaaaapiaaacaabgaaabaaaaaaaaabaeaaaaaaaaaaaaabbeaaacaaam
aaaeaaaaaaaaabceaaaaaaaaaaaaabdeaaacaaabaaabaaaaaaaaabaeaaaaaaaa
aaaaabegaaacaaacaaabaaaaaaaaabaeaaaaaaaaaaaaabfeaaacaabaaaaeaaaa
aaaaabceaaaaaaaaaaaaabgcaaacaaaaaaabaaaaaaaaabhiaaaaaaaaaaaaabii
aaacaaaiaaaeaaaaaaaaabceaaaaaaaaaaaaabkcaaacaaaeaaaeaaaaaaaaabce
aaaaaaaaaaaaablfaaacaabfaaabaaaaaaaaabaeaaaaaaaaaaaaabmgaaacaabe
aaabaaaaaaaaabaeaaaaaaaaaaaaabncaaacaaadaaabaaaaaaaaabaeaaaaaaaa
fpengbgjgofegfhifpfdfeaaaaabaaadaaabaaaeaaabaaaaaaaaaaaafpepgcgk
gfgdhedcfhgphcgmgeaaklklaaadaaadaaaeaaaeaaabaaaaaaaaaaaafpfahcgp
gkgfgdhegjgpgofagbhcgbgnhdaafpfdgdhcgfgfgofagbhcgbgnhdaafpfhgphc
gmgedcepgcgkgfgdheaafpfhgphcgmgefdhagbgdgfedgbgngfhcgbfagphdaakl
aaabaaadaaabaaadaaabaaaaaaaaaaaaghgmhdhegbhegffpgngbhehcgjhifpgn
gpgegfgmhggjgfhhdaaaghgmhdhegbhegffpgngbhehcgjhifpgnhghaaahfgogj
hehjfpemgjghgihegngbhafdfeaahfgogjhehjfpfdgdgbgmgfaahfgogjhehjfp
fdgigbgegphheggbgegfedgfgohegfhcebgogefehjhagfaahghdfpddfpdaaadc
codacodcdadddfddcodaaaklaaaaaaaaaaaaaaabaaaaaaaaaaaaaaaaaaaaaabe
aapmaabaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaeaaaaaacoiaagbaaan
aaaaaaaaaaaaaaaaaaaagaohaaaaaaabaaaaaaafaaaaaaaiaaaaacjaaabaaaag
aaaagaahaaaadaaiaaaafaajaacbfaakaaaadafaaaabpbfbaaadpcfcaaaepdfd
aaafpefeaaagdfffaaahpgfgaaaabacpaaaaaacoaaaabadjaaaabadkaaaabadl
aaaabadmaaaabadaaaaabadiaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaadpiaaaaa
dpaaaaaaaaaaaaaaaaaaaaaapbfffaagaaaabcabmcaaaaaaaaaafaalaaaabcaa
meaaaaaaaaaagabagabgbcaabcaaaaaaaaaagabmgaccbcaabcaaaaaaaaaagaci
gacobcaabcaaaaaaaaaagadedadkbcaaccaaaaaaafpigaaaaaaaagiiaaaaaaaa
afpibaaaaaaaagiiaaaaaaaaafpiiaaaaaaaaoiiaaaaaaaaafpilaaaaaaaapmi
aaaaaaaaafpihaaaaaaaacdpaaaaaaaamiapaaaaaabliiaakbagahaamiapaaaa
aamgnapiklagagaamiapaaaaaalbdepiklagafaamiapaaamaagmnajeklagaeaa
miapiadoaananaaaocamamaamiabaaafaeblgmaacaadppaamiaoaaafaapmmgpm
albcaabdmiaiaaaiaamgmgaakbagakaabebiaaajaalbmggmkbagajagkiibakan
aablgmmaibabamaibeceaaanaablgmblkbabaoabkicoanadaamgimebibaiaoan
bebhaaaeaalblomgkbaianabkiihaeacaalbgcmaibabanaomialaaaaaalelbmj
clbbaaafkmbhadajaalbleicibaganaokmeoaaafaakgebecmbaiabaomiahaaak
aagmmaleklagamajmiaoaaafabebkgabolaiabafmialaaaaaamagmliclbaaaaa
miahaaaeaagmgcleklaiamaemialaaacaagmloleklabamacbeaeaaacaagmmggm
oaaeadacaebeacadaalbblbloaaeadaebeahaaaiaablleblkbagapacaebeadae
aamglbgmoaaeadadbeahaaajaamglelbkbagaoacmiahaaagabbablmaklaabeag
aeblaeaaaabcblmgobafabaakiccacaeaalomdmdnaanafaokicoadafaemghgma
ibagaoankiihadabaelbgciaibagananmiadaaahaalblcbjklaaamadmiahaaag
aegmloleklagamabkichadaaaamalbidibamppaomiamiaabaanlnlaaocamamaa
miadiaaaaalalabkilalbgbgmiadiaafaabklabkilahbfbfkiiiaaacaagmlbeb
maagafabbeapaaabaadeaagmoaakajahaeciacadaamgmglboaagafacbeapaaab
aadedelboaabaiahaeciadaeaalbbllboaagafadmiaiaaabaablmgblklagalab
beahaaabadmamablkaabadabamihiaagaamablgmkbabadafmiadiaabaamgbkbi
klaaacaamiapiaacaaaablaakbaebeaamiapiaadaaaablaakbadbeaamiapiaae
aaaablaakbacbeaaaaaaaaaaaaaaaaaaaaaaaaaa"
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
"sce_vp_rsx // 44 instructions using 8 registers
[Configuration]
8
0000002c43050800
[Defaults]
1
461 2
3f0000003f800000
[Microcode]
704
00029c6c00400e0c0106c0836041dffc00039c6c005d300c0186c0836041dffc
401f9c6c011ce808010400d740619f9c401f9c6c011cf908010400d740619fb0
00001c6c0150ae0c010600c360411ffc00001c6c0150a20c010600c360405ffc
00009c6c01509e0c010600c360411ffc00009c6c0150920c010600c360405ffc
00011c6c01508e0c010600c360411ffc00011c6c0150820c010600c360405ffc
00001c6c01d0200d8106c0c360409ffc00019c6c01d0700d8106c0c360403ffc
00019c6c01d0600d8106c0c360405ffc00019c6c01d0500d8106c0c360409ffc
00019c6c01d0400d8106c0c360411ffc00001c6c005d107f8186c08360403ffc
00031c6c01d0a00d8106c0c360405ffc00031c6c01d0900d8106c0c360409ffc
00031c6c01d0800d8106c0c360411ffc00021c6c0190e00c0e86c0c360405ffc
00021c6c0190d00c0e86c0c360409ffc00021c6c0190c00c0e86c0c360411ffc
00039c6c00dd108c0186c0830321dffc00031c6c00800243011845436041dffc
00001c6c00dcd02a8186c0bfe0203ffc00029c6c01000230812185630321dffc
401f9c6c0040000d8686c0836041ff80401f9c6c004000558686c08360407fa0
00021c6c011d000c08bfc0e30041dffc00019c6c009cd00e068000c36041dffc
401f9c6c009d100c0ebfc0c36041dfb4401f9c6c008000aa80bfc04360403fb4
00019c6c009d202a868000c360409ffc401f9c6c00c000080686c09541a19fa0
00001c6c0150a08c088600c360403ffc00009c6c0150908c088600c360403ffc
00011c6c0150808c088600c360403ffc00019c6c00800e7f810605436041dffc
00001c6c0150a00c068600c360409ffc00011c6c0150800c068600c360409ffc
00009c6c0150900c068600c360409ffc401f9c6c009d000d84bfc0c36041ffa4
401f9c6c009d000d82bfc0c36041ffa8401f9c6c009d000d80bfc0c36041ffad
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
Vector 96 [unity_LightmapST] 4
Vector 112 [_MainTex_ST] 4
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
// 58 instructions, 4 temp regs, 0 temp arrays:
// ALU 27 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0
eefiecedplhibnpohhdgcioijpemcdpoolgcapcdabaaaaaaamakaaaaadaaaaaa
cmaaaaaapeaaaaaanmabaaaaejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaa
abaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapadaaaaljaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfc
enebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheooaaaaaaaaiaaaaaa
aiaaaaaamiaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaneaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaabaaaaaaadamaaaaneaaaaaaafaaaaaaaaaaaaaa
adaaaaaaabaaaaaaamadaaaaneaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaa
apaaaaaaneaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaapaaaaaaneaaaaaa
adaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaneaaaaaaaeaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaaneaaaaaaagaaaaaaaaaaaaaaadaaaaaaagaaaaaa
apaaaaaafdfgfpfaepfdejfeejepeoaafeeffiedepepfceeaaklklklfdeieefc
ciaiaaaaeaaaabaaakacaaaafjaaaaaeegiocaaaaaaaaaaaaiaaaaaafjaaaaae
egiocaaaabaaaaaaagaaaaaafjaaaaaeegiocaaaacaaaaaabkaaaaaafjaaaaae
egiocaaaadaaaaaabfaaaaaafpaaaaadpcbabaaaaaaaaaaafpaaaaadpcbabaaa
abaaaaaafpaaaaadhcbabaaaacaaaaaafpaaaaaddcbabaaaadaaaaaafpaaaaad
dcbabaaaaeaaaaaaghaaaaaepccabaaaaaaaaaaaabaaaaaagfaaaaaddccabaaa
abaaaaaagfaaaaadmccabaaaabaaaaaagfaaaaadpccabaaaacaaaaaagfaaaaad
pccabaaaadaaaaaagfaaaaadpccabaaaaeaaaaaagfaaaaadpccabaaaafaaaaaa
gfaaaaadpccabaaaagaaaaaagiaaaaacaeaaaaaadiaaaaaipcaabaaaaaaaaaaa
fgbfbaaaaaaaaaaaegiocaaaadaaaaaaabaaaaaadcaaaaakpcaabaaaaaaaaaaa
egiocaaaadaaaaaaaaaaaaaaagbabaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaak
pcaabaaaaaaaaaaaegiocaaaadaaaaaaacaaaaaakgbkbaaaaaaaaaaaegaobaaa
aaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaadaaaaaapgbpbaaa
aaaaaaaaegaobaaaaaaaaaaadgaaaaafpccabaaaaaaaaaaaegaobaaaaaaaaaaa
dcaaaaaldccabaaaabaaaaaaegbabaaaadaaaaaaegiacaaaaaaaaaaaahaaaaaa
ogikcaaaaaaaaaaaahaaaaaadcaaaaalmccabaaaabaaaaaaagbebaaaaeaaaaaa
agiecaaaaaaaaaaaagaaaaaakgiocaaaaaaaaaaaagaaaaaadiaaaaaiccaabaaa
aaaaaaaabkaabaaaaaaaaaaaakiacaaaabaaaaaaafaaaaaadiaaaaakncaabaaa
abaaaaaaagahbaaaaaaaaaaaaceaaaaaaaaaaadpaaaaaaaaaaaaaadpaaaaaadp
dgaaaaafmccabaaaacaaaaaakgaobaaaaaaaaaaaaaaaaaahdccabaaaacaaaaaa
kgakbaaaabaaaaaamgaabaaaabaaaaaadiaaaaajhcaabaaaaaaaaaaafgifcaaa
abaaaaaaaeaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaaaaaaaaaa
egiccaaaadaaaaaabaaaaaaaagiacaaaabaaaaaaaeaaaaaaegacbaaaaaaaaaaa
dcaaaaalhcaabaaaaaaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaaabaaaaaa
aeaaaaaaegacbaaaaaaaaaaaaaaaaaaihcaabaaaaaaaaaaaegacbaaaaaaaaaaa
egiccaaaadaaaaaabdaaaaaadcaaaaalhcaabaaaaaaaaaaaegacbaaaaaaaaaaa
pgipcaaaadaaaaaabeaaaaaaegbcbaiaebaaaaaaaaaaaaaadiaaaaajhcaabaaa
abaaaaaafgafbaiaebaaaaaaaaaaaaaaegiccaaaadaaaaaaanaaaaaadcaaaaal
lcaabaaaaaaaaaaaegiicaaaadaaaaaaamaaaaaaagaabaiaebaaaaaaaaaaaaaa
egaibaaaabaaaaaadcaaaaallcaabaaaaaaaaaaaegiicaaaadaaaaaaaoaaaaaa
kgakbaiaebaaaaaaaaaaaaaaegambaaaaaaaaaaadgaaaaaficaabaaaabaaaaaa
akaabaaaaaaaaaaadiaaaaahhcaabaaaacaaaaaajgbebaaaabaaaaaacgbjbaaa
acaaaaaadcaaaaakhcaabaaaacaaaaaajgbebaaaacaaaaaacgbjbaaaabaaaaaa
egacbaiaebaaaaaaacaaaaaadiaaaaahhcaabaaaacaaaaaaegacbaaaacaaaaaa
pgbpbaaaabaaaaaadgaaaaagbcaabaaaadaaaaaaakiacaaaadaaaaaaamaaaaaa
dgaaaaagccaabaaaadaaaaaaakiacaaaadaaaaaaanaaaaaadgaaaaagecaabaaa
adaaaaaaakiacaaaadaaaaaaaoaaaaaabaaaaaahccaabaaaabaaaaaaegacbaaa
acaaaaaaegacbaaaadaaaaaabaaaaaahbcaabaaaabaaaaaaegbcbaaaabaaaaaa
egacbaaaadaaaaaabaaaaaahecaabaaaabaaaaaaegbcbaaaacaaaaaaegacbaaa
adaaaaaadiaaaaaipccabaaaadaaaaaaegaobaaaabaaaaaapgipcaaaadaaaaaa
beaaaaaadgaaaaaficaabaaaabaaaaaabkaabaaaaaaaaaaadgaaaaagbcaabaaa
adaaaaaabkiacaaaadaaaaaaamaaaaaadgaaaaagccaabaaaadaaaaaabkiacaaa
adaaaaaaanaaaaaadgaaaaagecaabaaaadaaaaaabkiacaaaadaaaaaaaoaaaaaa
baaaaaahccaabaaaabaaaaaaegacbaaaacaaaaaaegacbaaaadaaaaaabaaaaaah
bcaabaaaabaaaaaaegbcbaaaabaaaaaaegacbaaaadaaaaaabaaaaaahecaabaaa
abaaaaaaegbcbaaaacaaaaaaegacbaaaadaaaaaadiaaaaaipccabaaaaeaaaaaa
egaobaaaabaaaaaapgipcaaaadaaaaaabeaaaaaadgaaaaagbcaabaaaabaaaaaa
ckiacaaaadaaaaaaamaaaaaadgaaaaagccaabaaaabaaaaaackiacaaaadaaaaaa
anaaaaaadgaaaaagecaabaaaabaaaaaackiacaaaadaaaaaaaoaaaaaabaaaaaah
ccaabaaaaaaaaaaaegacbaaaacaaaaaaegacbaaaabaaaaaabaaaaaahbcaabaaa
aaaaaaaaegbcbaaaabaaaaaaegacbaaaabaaaaaabaaaaaahecaabaaaaaaaaaaa
egbcbaaaacaaaaaaegacbaaaabaaaaaadiaaaaaipccabaaaafaaaaaaegaobaaa
aaaaaaaapgipcaaaadaaaaaabeaaaaaadiaaaaaihcaabaaaaaaaaaaafgbfbaaa
aaaaaaaaegiccaaaadaaaaaaanaaaaaadcaaaaakhcaabaaaaaaaaaaaegiccaaa
adaaaaaaamaaaaaaagbabaaaaaaaaaaaegacbaaaaaaaaaaadcaaaaakhcaabaaa
aaaaaaaaegiccaaaadaaaaaaaoaaaaaakgbkbaaaaaaaaaaaegacbaaaaaaaaaaa
dcaaaaakhcaabaaaaaaaaaaaegiccaaaadaaaaaaapaaaaaapgbpbaaaaaaaaaaa
egacbaaaaaaaaaaaaaaaaaajhcaabaaaaaaaaaaaegacbaaaaaaaaaaaegiccaia
ebaaaaaaacaaaaaabjaaaaaadiaaaaaihccabaaaagaaaaaaegacbaaaaaaaaaaa
pgipcaaaacaaaaaabjaaaaaadiaaaaaibcaabaaaaaaaaaaabkbabaaaaaaaaaaa
ckiacaaaadaaaaaaafaaaaaadcaaaaakbcaabaaaaaaaaaaackiacaaaadaaaaaa
aeaaaaaaakbabaaaaaaaaaaaakaabaaaaaaaaaaadcaaaaakbcaabaaaaaaaaaaa
ckiacaaaadaaaaaaagaaaaaackbabaaaaaaaaaaaakaabaaaaaaaaaaadcaaaaak
bcaabaaaaaaaaaaackiacaaaadaaaaaaahaaaaaadkbabaaaaaaaaaaaakaabaaa
aaaaaaaaaaaaaaajccaabaaaaaaaaaaadkiacaiaebaaaaaaacaaaaaabjaaaaaa
abeaaaaaaaaaiadpdiaaaaaiiccabaaaagaaaaaabkaabaaaaaaaaaaaakaabaia
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

varying highp vec4 xlv_TEXCOORD6;
varying highp vec2 xlv_TEXCOORD5;
varying lowp vec4 xlv_TEXCOORD4;
varying lowp vec4 xlv_TEXCOORD3;
varying lowp vec4 xlv_TEXCOORD2;
varying highp vec4 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
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
  lowp vec4 tmpvar_3;
  lowp vec4 tmpvar_4;
  lowp vec4 tmpvar_5;
  highp vec4 tmpvar_6;
  highp vec4 tmpvar_7;
  tmpvar_7 = (gl_ModelViewProjectionMatrix * _glesVertex);
  highp vec4 tmpvar_8;
  tmpvar_8.w = 1.0;
  tmpvar_8.xyz = _WorldSpaceCameraPos;
  mat3 tmpvar_9;
  tmpvar_9[0] = _Object2World[0].xyz;
  tmpvar_9[1] = _Object2World[1].xyz;
  tmpvar_9[2] = _Object2World[2].xyz;
  highp vec3 tmpvar_10;
  tmpvar_10 = (tmpvar_9 * (_glesVertex.xyz - ((_World2Object * tmpvar_8).xyz * unity_Scale.w)));
  highp vec3 tmpvar_11;
  highp vec3 tmpvar_12;
  tmpvar_11 = tmpvar_1.xyz;
  tmpvar_12 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_13;
  tmpvar_13[0].x = tmpvar_11.x;
  tmpvar_13[0].y = tmpvar_12.x;
  tmpvar_13[0].z = tmpvar_2.x;
  tmpvar_13[1].x = tmpvar_11.y;
  tmpvar_13[1].y = tmpvar_12.y;
  tmpvar_13[1].z = tmpvar_2.y;
  tmpvar_13[2].x = tmpvar_11.z;
  tmpvar_13[2].y = tmpvar_12.z;
  tmpvar_13[2].z = tmpvar_2.z;
  vec4 v_14;
  v_14.x = _Object2World[0].x;
  v_14.y = _Object2World[1].x;
  v_14.z = _Object2World[2].x;
  v_14.w = _Object2World[3].x;
  highp vec4 tmpvar_15;
  tmpvar_15.xyz = (tmpvar_13 * v_14.xyz);
  tmpvar_15.w = tmpvar_10.x;
  highp vec4 tmpvar_16;
  tmpvar_16 = (tmpvar_15 * unity_Scale.w);
  tmpvar_3 = tmpvar_16;
  vec4 v_17;
  v_17.x = _Object2World[0].y;
  v_17.y = _Object2World[1].y;
  v_17.z = _Object2World[2].y;
  v_17.w = _Object2World[3].y;
  highp vec4 tmpvar_18;
  tmpvar_18.xyz = (tmpvar_13 * v_17.xyz);
  tmpvar_18.w = tmpvar_10.y;
  highp vec4 tmpvar_19;
  tmpvar_19 = (tmpvar_18 * unity_Scale.w);
  tmpvar_4 = tmpvar_19;
  vec4 v_20;
  v_20.x = _Object2World[0].z;
  v_20.y = _Object2World[1].z;
  v_20.z = _Object2World[2].z;
  v_20.w = _Object2World[3].z;
  highp vec4 tmpvar_21;
  tmpvar_21.xyz = (tmpvar_13 * v_20.xyz);
  tmpvar_21.w = tmpvar_10.z;
  highp vec4 tmpvar_22;
  tmpvar_22 = (tmpvar_21 * unity_Scale.w);
  tmpvar_5 = tmpvar_22;
  highp vec4 o_23;
  highp vec4 tmpvar_24;
  tmpvar_24 = (tmpvar_7 * 0.5);
  highp vec2 tmpvar_25;
  tmpvar_25.x = tmpvar_24.x;
  tmpvar_25.y = (tmpvar_24.y * _ProjectionParams.x);
  o_23.xy = (tmpvar_25 + tmpvar_24.w);
  o_23.zw = tmpvar_7.zw;
  tmpvar_6.xyz = (((_Object2World * _glesVertex).xyz - unity_ShadowFadeCenterAndType.xyz) * unity_ShadowFadeCenterAndType.w);
  tmpvar_6.w = (-((gl_ModelViewMatrix * _glesVertex).z) * (1.0 - unity_ShadowFadeCenterAndType.w));
  gl_Position = tmpvar_7;
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = o_23;
  xlv_TEXCOORD2 = tmpvar_3;
  xlv_TEXCOORD3 = tmpvar_4;
  xlv_TEXCOORD4 = tmpvar_5;
  xlv_TEXCOORD5 = ((_glesMultiTexCoord1.xy * unity_LightmapST.xy) + unity_LightmapST.zw);
  xlv_TEXCOORD6 = tmpvar_6;
}



#endif
#ifdef FRAGMENT

varying highp vec4 xlv_TEXCOORD6;
varying highp vec2 xlv_TEXCOORD5;
varying lowp vec4 xlv_TEXCOORD4;
varying lowp vec4 xlv_TEXCOORD3;
varying lowp vec4 xlv_TEXCOORD2;
varying highp vec4 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp vec4 unity_LightmapFade;
uniform sampler2D unity_LightmapInd;
uniform sampler2D unity_Lightmap;
uniform sampler2D _LightBuffer;
uniform mediump float _Gloss;
uniform mediump float _ReflectPower;
uniform lowp vec4 _ReflectColor;
uniform lowp vec4 _Color;
uniform samplerCube _Cube;
uniform sampler2D _SpecMap;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform lowp vec4 _SpecColor;
void main ()
{
  lowp vec4 tmpvar_1;
  mediump vec4 c_2;
  mediump vec3 lmIndirect_3;
  mediump vec3 lmFull_4;
  mediump vec4 light_5;
  highp vec3 tmpvar_6;
  mediump vec3 tmpvar_7;
  mediump vec3 tmpvar_8;
  mediump vec3 tmpvar_9;
  lowp vec3 tmpvar_10;
  tmpvar_10.x = xlv_TEXCOORD2.w;
  tmpvar_10.y = xlv_TEXCOORD3.w;
  tmpvar_10.z = xlv_TEXCOORD4.w;
  tmpvar_6 = tmpvar_10;
  lowp vec3 tmpvar_11;
  tmpvar_11 = xlv_TEXCOORD2.xyz;
  tmpvar_7 = tmpvar_11;
  lowp vec3 tmpvar_12;
  tmpvar_12 = xlv_TEXCOORD3.xyz;
  tmpvar_8 = tmpvar_12;
  lowp vec3 tmpvar_13;
  tmpvar_13 = xlv_TEXCOORD4.xyz;
  tmpvar_9 = tmpvar_13;
  mediump vec3 tmpvar_14;
  mediump vec3 tmpvar_15;
  mediump float tmpvar_16;
  mediump vec4 spec_17;
  lowp vec4 tmpvar_18;
  tmpvar_18 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_19;
  tmpvar_19 = (tmpvar_18.xyz * _Color.xyz);
  tmpvar_14 = tmpvar_19;
  lowp vec4 tmpvar_20;
  tmpvar_20 = texture2D (_SpecMap, xlv_TEXCOORD0);
  spec_17 = tmpvar_20;
  lowp vec3 tmpvar_21;
  tmpvar_21 = ((texture2D (_BumpMap, xlv_TEXCOORD0).xyz * 2.0) - 1.0);
  tmpvar_15 = tmpvar_21;
  mediump vec3 tmpvar_22;
  tmpvar_22.x = dot (tmpvar_7, tmpvar_15);
  tmpvar_22.y = dot (tmpvar_8, tmpvar_15);
  tmpvar_22.z = dot (tmpvar_9, tmpvar_15);
  highp vec3 tmpvar_23;
  tmpvar_23 = (tmpvar_6 - (2.0 * (dot (tmpvar_22, tmpvar_6) * tmpvar_22)));
  lowp vec4 tmpvar_24;
  tmpvar_24 = (textureCube (_Cube, tmpvar_23) * tmpvar_18.w);
  lowp float tmpvar_25;
  tmpvar_25 = (tmpvar_24.w * _ReflectColor.w);
  tmpvar_16 = tmpvar_25;
  lowp vec4 tmpvar_26;
  tmpvar_26 = texture2DProj (_LightBuffer, xlv_TEXCOORD1);
  light_5 = tmpvar_26;
  mediump vec4 tmpvar_27;
  tmpvar_27 = -(log2(max (light_5, vec4(0.001, 0.001, 0.001, 0.001))));
  light_5.w = tmpvar_27.w;
  lowp vec3 tmpvar_28;
  tmpvar_28 = (2.0 * texture2D (unity_Lightmap, xlv_TEXCOORD5).xyz);
  lmFull_4 = tmpvar_28;
  lowp vec3 tmpvar_29;
  tmpvar_29 = (2.0 * texture2D (unity_LightmapInd, xlv_TEXCOORD5).xyz);
  lmIndirect_3 = tmpvar_29;
  highp float tmpvar_30;
  tmpvar_30 = clamp (((sqrt(dot (xlv_TEXCOORD6, xlv_TEXCOORD6)) * unity_LightmapFade.z) + unity_LightmapFade.w), 0.0, 1.0);
  light_5.xyz = (tmpvar_27.xyz + mix (lmIndirect_3, lmFull_4, vec3(tmpvar_30)));
  mediump vec4 c_31;
  mediump vec3 tmpvar_32;
  tmpvar_32 = (tmpvar_27.w * (spec_17.xyz * _Gloss));
  c_31.xyz = ((tmpvar_14 * light_5.xyz) + (light_5.xyz * tmpvar_32));
  c_31.w = (tmpvar_16 + (tmpvar_32 * _SpecColor.w)).x;
  c_2.w = c_31.w;
  c_2.xyz = (c_31.xyz + ((tmpvar_24.xyz * _ReflectColor.xyz) * _ReflectPower));
  tmpvar_1 = c_2;
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

varying highp vec4 xlv_TEXCOORD6;
varying highp vec2 xlv_TEXCOORD5;
varying lowp vec4 xlv_TEXCOORD4;
varying lowp vec4 xlv_TEXCOORD3;
varying lowp vec4 xlv_TEXCOORD2;
varying highp vec4 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
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
  lowp vec4 tmpvar_3;
  lowp vec4 tmpvar_4;
  lowp vec4 tmpvar_5;
  highp vec4 tmpvar_6;
  highp vec4 tmpvar_7;
  tmpvar_7 = (gl_ModelViewProjectionMatrix * _glesVertex);
  highp vec4 tmpvar_8;
  tmpvar_8.w = 1.0;
  tmpvar_8.xyz = _WorldSpaceCameraPos;
  mat3 tmpvar_9;
  tmpvar_9[0] = _Object2World[0].xyz;
  tmpvar_9[1] = _Object2World[1].xyz;
  tmpvar_9[2] = _Object2World[2].xyz;
  highp vec3 tmpvar_10;
  tmpvar_10 = (tmpvar_9 * (_glesVertex.xyz - ((_World2Object * tmpvar_8).xyz * unity_Scale.w)));
  highp vec3 tmpvar_11;
  highp vec3 tmpvar_12;
  tmpvar_11 = tmpvar_1.xyz;
  tmpvar_12 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_13;
  tmpvar_13[0].x = tmpvar_11.x;
  tmpvar_13[0].y = tmpvar_12.x;
  tmpvar_13[0].z = tmpvar_2.x;
  tmpvar_13[1].x = tmpvar_11.y;
  tmpvar_13[1].y = tmpvar_12.y;
  tmpvar_13[1].z = tmpvar_2.y;
  tmpvar_13[2].x = tmpvar_11.z;
  tmpvar_13[2].y = tmpvar_12.z;
  tmpvar_13[2].z = tmpvar_2.z;
  vec4 v_14;
  v_14.x = _Object2World[0].x;
  v_14.y = _Object2World[1].x;
  v_14.z = _Object2World[2].x;
  v_14.w = _Object2World[3].x;
  highp vec4 tmpvar_15;
  tmpvar_15.xyz = (tmpvar_13 * v_14.xyz);
  tmpvar_15.w = tmpvar_10.x;
  highp vec4 tmpvar_16;
  tmpvar_16 = (tmpvar_15 * unity_Scale.w);
  tmpvar_3 = tmpvar_16;
  vec4 v_17;
  v_17.x = _Object2World[0].y;
  v_17.y = _Object2World[1].y;
  v_17.z = _Object2World[2].y;
  v_17.w = _Object2World[3].y;
  highp vec4 tmpvar_18;
  tmpvar_18.xyz = (tmpvar_13 * v_17.xyz);
  tmpvar_18.w = tmpvar_10.y;
  highp vec4 tmpvar_19;
  tmpvar_19 = (tmpvar_18 * unity_Scale.w);
  tmpvar_4 = tmpvar_19;
  vec4 v_20;
  v_20.x = _Object2World[0].z;
  v_20.y = _Object2World[1].z;
  v_20.z = _Object2World[2].z;
  v_20.w = _Object2World[3].z;
  highp vec4 tmpvar_21;
  tmpvar_21.xyz = (tmpvar_13 * v_20.xyz);
  tmpvar_21.w = tmpvar_10.z;
  highp vec4 tmpvar_22;
  tmpvar_22 = (tmpvar_21 * unity_Scale.w);
  tmpvar_5 = tmpvar_22;
  highp vec4 o_23;
  highp vec4 tmpvar_24;
  tmpvar_24 = (tmpvar_7 * 0.5);
  highp vec2 tmpvar_25;
  tmpvar_25.x = tmpvar_24.x;
  tmpvar_25.y = (tmpvar_24.y * _ProjectionParams.x);
  o_23.xy = (tmpvar_25 + tmpvar_24.w);
  o_23.zw = tmpvar_7.zw;
  tmpvar_6.xyz = (((_Object2World * _glesVertex).xyz - unity_ShadowFadeCenterAndType.xyz) * unity_ShadowFadeCenterAndType.w);
  tmpvar_6.w = (-((gl_ModelViewMatrix * _glesVertex).z) * (1.0 - unity_ShadowFadeCenterAndType.w));
  gl_Position = tmpvar_7;
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = o_23;
  xlv_TEXCOORD2 = tmpvar_3;
  xlv_TEXCOORD3 = tmpvar_4;
  xlv_TEXCOORD4 = tmpvar_5;
  xlv_TEXCOORD5 = ((_glesMultiTexCoord1.xy * unity_LightmapST.xy) + unity_LightmapST.zw);
  xlv_TEXCOORD6 = tmpvar_6;
}



#endif
#ifdef FRAGMENT

varying highp vec4 xlv_TEXCOORD6;
varying highp vec2 xlv_TEXCOORD5;
varying lowp vec4 xlv_TEXCOORD4;
varying lowp vec4 xlv_TEXCOORD3;
varying lowp vec4 xlv_TEXCOORD2;
varying highp vec4 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp vec4 unity_LightmapFade;
uniform sampler2D unity_LightmapInd;
uniform sampler2D unity_Lightmap;
uniform sampler2D _LightBuffer;
uniform mediump float _Gloss;
uniform mediump float _ReflectPower;
uniform lowp vec4 _ReflectColor;
uniform lowp vec4 _Color;
uniform samplerCube _Cube;
uniform sampler2D _SpecMap;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform lowp vec4 _SpecColor;
void main ()
{
  lowp vec4 tmpvar_1;
  mediump vec4 c_2;
  mediump vec3 lmIndirect_3;
  mediump vec3 lmFull_4;
  mediump vec4 light_5;
  highp vec3 tmpvar_6;
  mediump vec3 tmpvar_7;
  mediump vec3 tmpvar_8;
  mediump vec3 tmpvar_9;
  lowp vec3 tmpvar_10;
  tmpvar_10.x = xlv_TEXCOORD2.w;
  tmpvar_10.y = xlv_TEXCOORD3.w;
  tmpvar_10.z = xlv_TEXCOORD4.w;
  tmpvar_6 = tmpvar_10;
  lowp vec3 tmpvar_11;
  tmpvar_11 = xlv_TEXCOORD2.xyz;
  tmpvar_7 = tmpvar_11;
  lowp vec3 tmpvar_12;
  tmpvar_12 = xlv_TEXCOORD3.xyz;
  tmpvar_8 = tmpvar_12;
  lowp vec3 tmpvar_13;
  tmpvar_13 = xlv_TEXCOORD4.xyz;
  tmpvar_9 = tmpvar_13;
  mediump vec3 tmpvar_14;
  mediump vec3 tmpvar_15;
  mediump float tmpvar_16;
  mediump vec4 spec_17;
  lowp vec4 tmpvar_18;
  tmpvar_18 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_19;
  tmpvar_19 = (tmpvar_18.xyz * _Color.xyz);
  tmpvar_14 = tmpvar_19;
  lowp vec4 tmpvar_20;
  tmpvar_20 = texture2D (_SpecMap, xlv_TEXCOORD0);
  spec_17 = tmpvar_20;
  lowp vec3 normal_21;
  normal_21.xy = ((texture2D (_BumpMap, xlv_TEXCOORD0).wy * 2.0) - 1.0);
  normal_21.z = sqrt(((1.0 - (normal_21.x * normal_21.x)) - (normal_21.y * normal_21.y)));
  tmpvar_15 = normal_21;
  mediump vec3 tmpvar_22;
  tmpvar_22.x = dot (tmpvar_7, tmpvar_15);
  tmpvar_22.y = dot (tmpvar_8, tmpvar_15);
  tmpvar_22.z = dot (tmpvar_9, tmpvar_15);
  highp vec3 tmpvar_23;
  tmpvar_23 = (tmpvar_6 - (2.0 * (dot (tmpvar_22, tmpvar_6) * tmpvar_22)));
  lowp vec4 tmpvar_24;
  tmpvar_24 = (textureCube (_Cube, tmpvar_23) * tmpvar_18.w);
  lowp float tmpvar_25;
  tmpvar_25 = (tmpvar_24.w * _ReflectColor.w);
  tmpvar_16 = tmpvar_25;
  lowp vec4 tmpvar_26;
  tmpvar_26 = texture2DProj (_LightBuffer, xlv_TEXCOORD1);
  light_5 = tmpvar_26;
  mediump vec4 tmpvar_27;
  tmpvar_27 = -(log2(max (light_5, vec4(0.001, 0.001, 0.001, 0.001))));
  light_5.w = tmpvar_27.w;
  lowp vec4 tmpvar_28;
  tmpvar_28 = texture2D (unity_Lightmap, xlv_TEXCOORD5);
  lowp vec3 tmpvar_29;
  tmpvar_29 = ((8.0 * tmpvar_28.w) * tmpvar_28.xyz);
  lmFull_4 = tmpvar_29;
  lowp vec4 tmpvar_30;
  tmpvar_30 = texture2D (unity_LightmapInd, xlv_TEXCOORD5);
  lowp vec3 tmpvar_31;
  tmpvar_31 = ((8.0 * tmpvar_30.w) * tmpvar_30.xyz);
  lmIndirect_3 = tmpvar_31;
  highp float tmpvar_32;
  tmpvar_32 = clamp (((sqrt(dot (xlv_TEXCOORD6, xlv_TEXCOORD6)) * unity_LightmapFade.z) + unity_LightmapFade.w), 0.0, 1.0);
  light_5.xyz = (tmpvar_27.xyz + mix (lmIndirect_3, lmFull_4, vec3(tmpvar_32)));
  mediump vec4 c_33;
  mediump vec3 tmpvar_34;
  tmpvar_34 = (tmpvar_27.w * (spec_17.xyz * _Gloss));
  c_33.xyz = ((tmpvar_14 * light_5.xyz) + (light_5.xyz * tmpvar_34));
  c_33.w = (tmpvar_16 + (tmpvar_34 * _SpecColor.w)).x;
  c_2.w = c_33.w;
  c_2.xyz = (c_33.xyz + ((tmpvar_24.xyz * _ReflectColor.xyz) * _ReflectPower));
  tmpvar_1 = c_2;
  gl_FragData[0] = tmpvar_1;
}



#endif"
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
"3.0-!!ARBvp1.0
# 54 ALU
PARAM c[24] = { { 0.5, 1 },
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
MOV R0.w, c[0].y;
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
MUL R1.xyz, vertex.attrib[14].w, R0;
MOV R0.xyz, c[13];
MOV R0.w, c[0].y;
ADD result.texcoord[5].xyz, R2, R3;
DP4 R2.z, R0, c[11];
DP4 R2.x, R0, c[9];
DP4 R2.y, R0, c[10];
MAD R2.xyz, R2, c[22].w, -vertex.position;
DP3 R0.y, R1, c[5];
DP3 R0.w, -R2, c[5];
DP4 R1.w, vertex.position, c[4];
DP3 R0.x, vertex.attrib[14], c[5];
DP3 R0.z, vertex.normal, c[5];
MUL result.texcoord[2], R0, c[22].w;
DP3 R0.y, R1, c[6];
DP3 R0.w, -R2, c[6];
DP3 R0.x, vertex.attrib[14], c[6];
DP3 R0.z, vertex.normal, c[6];
MUL result.texcoord[3], R0, c[22].w;
DP3 R0.y, R1, c[7];
DP4 R1.z, vertex.position, c[3];
DP3 R0.w, -R2, c[7];
DP4 R1.x, vertex.position, c[1];
DP4 R1.y, vertex.position, c[2];
MUL R2.xyz, R1.xyww, c[0].x;
DP3 R0.x, vertex.attrib[14], c[7];
DP3 R0.z, vertex.normal, c[7];
MUL result.texcoord[4], R0, c[22].w;
MOV R0.x, R2;
MUL R0.y, R2, c[14].x;
ADD result.texcoord[1].xy, R0, R2.z;
MOV result.position, R1;
MOV result.texcoord[1].zw, R1;
MAD result.texcoord[0].xy, vertex.texcoord[0], c[23], c[23].zwzw;
END
# 54 instructions, 4 R-regs
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
"vs_3_0
; 55 ALU
dcl_position o0
dcl_texcoord0 o1
dcl_texcoord1 o2
dcl_texcoord2 o3
dcl_texcoord3 o4
dcl_texcoord4 o5
dcl_texcoord5 o6
def c24, 0.50000000, 1.00000000, 0, 0
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
mov r0.w, c24.y
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
mul r1.xyz, v1.w, r0
mov r0.xyz, c12
mov r0.w, c24.y
add o6.xyz, r2, r3
dp4 r2.z, r0, c10
dp4 r2.x, r0, c8
dp4 r2.y, r0, c9
mad r2.xyz, r2, c22.w, -v0
dp3 r0.y, r1, c4
dp3 r0.w, -r2, c4
dp4 r1.w, v0, c3
dp3 r0.x, v1, c4
dp3 r0.z, v2, c4
mul o3, r0, c22.w
dp3 r0.y, r1, c5
dp3 r0.w, -r2, c5
dp3 r0.x, v1, c5
dp3 r0.z, v2, c5
mul o4, r0, c22.w
dp3 r0.y, r1, c6
dp4 r1.z, v0, c2
dp3 r0.w, -r2, c6
dp4 r1.x, v0, c0
dp4 r1.y, v0, c1
mul r2.xyz, r1.xyww, c24.x
dp3 r0.x, v1, c6
dp3 r0.z, v2, c6
mul o5, r0, c22.w
mov r0.x, r2
mul r0.y, r2, c13.x
mad o2.xy, r2.z, c14.zwzw, r0
mov o0, r1
mov o2.zw, r1
mad o1.xy, v3, c23, c23.zwzw
"
}

SubProgram "xbox360 " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
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
// ALU: 66.67 (50 instructions), vertex: 32, texture: 0,
//   sequencer: 24,  12 GPRs, 15 threads,
// Performance (if enough threads): ~66 cycles per vector
// * Vertex cycle estimates are assuming 3 vfetch_minis for every vfetch_full,
//     with <= 32 bytes per vfetch_full group.

"vs_360
backbbabaaaaadbeaaaaadbmaaaaaaaaaaaaaaceaaaaaciaaaaaackiaaaaaaaa
aaaaaaaaaaaaacfiaaaaaabmaaaaacekpppoadaaaaaaaaapaaaaaabmaaaaaaaa
aaaaacedaaaaabeiaaacaabgaaabaaaaaaaaabfeaaaaaaaaaaaaabgeaaacaaao
aaadaaaaaaaaabheaaaaaaaaaaaaabieaaacaaabaaabaaaaaaaaabfeaaaaaaaa
aaaaabjgaaacaaacaaabaaaaaaaaabfeaaaaaaaaaaaaabkeaaacaabbaaaeaaaa
aaaaabheaaaaaaaaaaaaablcaaacaaaaaaabaaaaaaaaabmiaaaaaaaaaaaaabni
aaacaaakaaaeaaaaaaaaabheaaaaaaaaaaaaabolaaacaaafaaabaaaaaaaaabfe
aaaaaaaaaaaaabpgaaacaaaeaaabaaaaaaaaabfeaaaaaaaaaaaaacabaaacaaad
aaabaaaaaaaaabfeaaaaaaaaaaaaacamaaacaaaiaaabaaaaaaaaabfeaaaaaaaa
aaaaacbhaaacaaahaaabaaaaaaaaabfeaaaaaaaaaaaaacccaaacaaagaaabaaaa
aaaaabfeaaaaaaaaaaaaaccnaaacaaajaaabaaaaaaaaabfeaaaaaaaaaaaaacdh
aaacaabfaaabaaaaaaaaabfeaaaaaaaafpengbgjgofegfhifpfdfeaaaaabaaad
aaabaaaeaaabaaaaaaaaaaaafpepgcgkgfgdhedcfhgphcgmgeaaklklaaadaaad
aaaeaaaeaaabaaaaaaaaaaaafpfahcgpgkgfgdhegjgpgofagbhcgbgnhdaafpfd
gdhcgfgfgofagbhcgbgnhdaafpfhgphcgmgedcepgcgkgfgdheaafpfhgphcgmge
fdhagbgdgfedgbgngfhcgbfagphdaaklaaabaaadaaabaaadaaabaaaaaaaaaaaa
ghgmhdhegbhegffpgngbhehcgjhifpgnhghaaahfgogjhehjfpfdeiebgcaahfgo
gjhehjfpfdeiebghaahfgogjhehjfpfdeiebhcaahfgogjhehjfpfdeiecgcaahf
gogjhehjfpfdeiecghaahfgogjhehjfpfdeiechcaahfgogjhehjfpfdeiedaahf
gogjhehjfpfdgdgbgmgfaahghdfpddfpdaaadccodacodcdadddfddcodaaaklkl
aaaaaaaaaaaaaaabaaaaaaaaaaaaaaaaaaaaaabeaapmaabaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaeaaaaaacnmaafbaaalaaaaaaaaaaaaaaaaaaaafemg
aaaaaaabaaaaaaaeaaaaaaahaaaaacjaaabaaaagaaaagaahaaaadaaiaacafaaj
aaaadafaaaabpbfbaaadpcfcaaaepdfdaaafpefeaaaghfffaaaabacmaaaaaacl
aaaabadgaaaabadhaaaabadiaaaabadjaaaabadlaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaadpaaaaaaaaaaaaaaaaaaaaaaaaaaaaaapaffeaagaaaabcaamcaaaaaa
aaaafaakaaaabcaameaaaaaaaaaagaapgabfbcaabcaaaaaaaaaagablgacbbcaa
bcaaaaaaaaaagachgacnbcaabcaaaaaaaaaagadddadjbcaaccaaaaaaafpijaaa
aaaaagiiaaaaaaaaafpibaaaaaaaagiiaaaaaaaaafpigaaaaaaaaoiiaaaaaaaa
afpidaaaaaaaapmiaaaaaaaamiapaaaaaabliiaakbajanaamiapaaaaaamgnapi
klajamaamiapaaaaaalbdepiklajalaamiapaaaiaagmiipiklajakaamiapiado
aaiiiiaaocaiaiaamiahaaahaamamgmaalbdaabeceibajakaablgmgmkbabaoia
beceaaakaablgmblkbabbaabkiclakacaamggcebibagbaapbebhaaaeaalbgcmg
kbagapabkiehadafaalbgcmaibabapbamialaaaaaalelbleclbcaaahkmehaaah
aalogficmbagabbamiahaaahabgflomaolagabahmiahaaalaamagmliclbbaaaa
miahaaafaagmmagcklabaoafmiahaaaeaagmmngfklagaoaekmeeacaeaamggmec
maaeacbabealaaaaaagcblmgkbagbfafaebeaeafaagmlbmgoaaeacadbeaoaaag
aagmimlbkbaabaafaebbafagaagmmgmgoaafacaamiahaaacabmablmaklalbfaj
miahaaajaalblebfklaaapagbeahaaaaaamnbllbobahabaeaeemagabaamgigbl
kbaabaacmiahaaajaabllemaklaaaoajkibcabagaalolomanaakahapkichabah
aelbgciaibacapapmiakaaafaalbmbgbklaaaoabmiahaaahaegmloleklacaoah
aiboabacaemghggmkbacbaajaibhaiaaaaligmggkbaippajmiamiaabaaigigaa
ocaiaiaamiadiaaaaalalabkiladbgbgaicbaiadaadoanmbgpadajajaiecaiad
aadoanlbgpaeajajaiieaiadaadoanlmgpafajajaicbabacaakhkhmgkpaiagaj
kiiiaaaeaagmlbebmaahacabbeacaaacaakhkhblkpaiahafaeciaeafaamgmgmg
oaahacabbeaeaaacaakhkhlbkpaiaiafaeciafagaalbblbloaahacabmiadiaab
aamgbkbiklaaacaamiapiaacaaaablaakbagbfaamiapiaadaaaablaakbafbfaa
miapiaaeaaaablaakbaebfaageihaaaaaalologboaadacabmiahiaafaablmagf
klaaajaaaaaaaaaaaaaaaaaaaaaaaaaa"
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
"sce_vp_rsx // 51 instructions using 8 registers
[Configuration]
8
0000003341050800
[Defaults]
1
456 1
3f000000
[Microcode]
816
00021c6c00400e0c0106c0836041dffc00039c6c005d300c0186c0836041dffc
00031c6c009ca20c013fc0c36041dffc401f9c6c011c9808010400d740619f9c
00001c6c01506e0c010600c360411ffc00001c6c0150620c010600c360405ffc
00009c6c01505e0c010600c360411ffc00009c6c0150520c010600c360405ffc
00011c6c01504e0c010600c360411ffc00011c6c0150420c010600c360405ffc
00029c6c01d0300d8106c0c360403ffc00029c6c01d0200d8106c0c360405ffc
00029c6c01d0100d8106c0c360409ffc00029c6c01d0000d8106c0c360411ffc
00019c6c0150400c0c8600c360411ffc00019c6c0150600c0c8600c360405ffc
00001c6c0150500c0c8600c360409ffc00031c6c0190a00c0e86c0c360405ffc
00031c6c0190900c0e86c0c360409ffc00031c6c0190800c0e86c0c360411ffc
00039c6c00800243011844436041dffc00021c6c010002308121846303a1dffc
401f9c6c0040000d8a86c0836041ff80401f9c6c004000558a86c08360407fa0
00031c6c011ca00c0cbfc0e30041dffc00029c6c009c800e0a8000c36041dffc
00029c6c009d202a8a8000c360409ffc00009c6c0080002a8095404360409ffc
00019c6c0040002a8086c08360409ffc401f9c6c00c000080a86c09542a19fa0
00001c6c0150608c0c8600c360403ffc00009c6c0150508c0c8600c360403ffc
00011c6c0150408c0c8600c360403ffc00029c6c00800e7f810604436041dffc
00021c6c019cf00c0686c0c360405ffc00021c6c019d000c0686c0c360409ffc
00021c6c019d100c0686c0c360411ffc00021c6c010000000680036aa0a03ffc
00019c6c0080000d069a03436041fffc00001c6c0150600c0a8600c360409ffc
00009c6c0150500c0a8600c360409ffc00011c6c0150400c0a8600c360409ffc
00029c6c01dcc00d8686c0c360405ffc00029c6c01dcd00d8686c0c360409ffc
00029c6c01dce00d8686c0c360411ffc00019c6c00c0000c0886c08302a1dffc
00021c6c009cb07f888600c36041dffc401f9c6c00c0000c0886c08301a1dfb0
401f9c6c009ca00d84bfc0c36041ffa4401f9c6c009ca00d82bfc0c36041ffa8
401f9c6c009ca00d80bfc0c36041ffad
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
Vector 96 [_MainTex_ST] 4
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
// 61 instructions, 4 temp regs, 0 temp arrays:
// ALU 32 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0
eefiecedmdkkeokklbapfnbcpbdjmjkccmbghjmdabaaaaaaaaakaaaaadaaaaaa
cmaaaaaapeaaaaaameabaaaaejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaa
abaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaljaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfc
enebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheomiaaaaaaahaaaaaa
aiaaaaaalaaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaalmaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaabaaaaaaadamaaaalmaaaaaaabaaaaaaaaaaaaaa
adaaaaaaacaaaaaaapaaaaaalmaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaa
apaaaaaalmaaaaaaadaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaalmaaaaaa
aeaaaaaaaaaaaaaaadaaaaaaafaaaaaaapaaaaaalmaaaaaaafaaaaaaaaaaaaaa
adaaaaaaagaaaaaaahaiaaaafdfgfpfaepfdejfeejepeoaafeeffiedepepfcee
aaklklklfdeieefcdeaiaaaaeaaaabaaanacaaaafjaaaaaeegiocaaaaaaaaaaa
ahaaaaaafjaaaaaeegiocaaaabaaaaaaagaaaaaafjaaaaaeegiocaaaacaaaaaa
bjaaaaaafjaaaaaeegiocaaaadaaaaaabfaaaaaafpaaaaadpcbabaaaaaaaaaaa
fpaaaaadpcbabaaaabaaaaaafpaaaaadhcbabaaaacaaaaaafpaaaaaddcbabaaa
adaaaaaaghaaaaaepccabaaaaaaaaaaaabaaaaaagfaaaaaddccabaaaabaaaaaa
gfaaaaadpccabaaaacaaaaaagfaaaaadpccabaaaadaaaaaagfaaaaadpccabaaa
aeaaaaaagfaaaaadpccabaaaafaaaaaagfaaaaadhccabaaaagaaaaaagiaaaaac
aeaaaaaadiaaaaaipcaabaaaaaaaaaaafgbfbaaaaaaaaaaaegiocaaaadaaaaaa
abaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaaaaaaaaaagbabaaa
aaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaa
acaaaaaakgbkbaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaakpcaabaaaaaaaaaaa
egiocaaaadaaaaaaadaaaaaapgbpbaaaaaaaaaaaegaobaaaaaaaaaaadgaaaaaf
pccabaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaaldccabaaaabaaaaaaegbabaaa
adaaaaaaegiacaaaaaaaaaaaagaaaaaaogikcaaaaaaaaaaaagaaaaaadiaaaaai
ccaabaaaaaaaaaaabkaabaaaaaaaaaaaakiacaaaabaaaaaaafaaaaaadiaaaaak
ncaabaaaabaaaaaaagahbaaaaaaaaaaaaceaaaaaaaaaaadpaaaaaaaaaaaaaadp
aaaaaadpdgaaaaafmccabaaaacaaaaaakgaobaaaaaaaaaaaaaaaaaahdccabaaa
acaaaaaakgakbaaaabaaaaaamgaabaaaabaaaaaadiaaaaajhcaabaaaaaaaaaaa
fgifcaaaabaaaaaaaeaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaa
aaaaaaaaegiccaaaadaaaaaabaaaaaaaagiacaaaabaaaaaaaeaaaaaaegacbaaa
aaaaaaaadcaaaaalhcaabaaaaaaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaa
abaaaaaaaeaaaaaaegacbaaaaaaaaaaaaaaaaaaihcaabaaaaaaaaaaaegacbaaa
aaaaaaaaegiccaaaadaaaaaabdaaaaaadcaaaaalhcaabaaaaaaaaaaaegacbaaa
aaaaaaaapgipcaaaadaaaaaabeaaaaaaegbcbaiaebaaaaaaaaaaaaaadiaaaaaj
hcaabaaaabaaaaaafgafbaiaebaaaaaaaaaaaaaaegiccaaaadaaaaaaanaaaaaa
dcaaaaallcaabaaaaaaaaaaaegiicaaaadaaaaaaamaaaaaaagaabaiaebaaaaaa
aaaaaaaaegaibaaaabaaaaaadcaaaaallcaabaaaaaaaaaaaegiicaaaadaaaaaa
aoaaaaaakgakbaiaebaaaaaaaaaaaaaaegambaaaaaaaaaaadgaaaaaficaabaaa
abaaaaaaakaabaaaaaaaaaaadiaaaaahhcaabaaaacaaaaaajgbebaaaabaaaaaa
cgbjbaaaacaaaaaadcaaaaakhcaabaaaacaaaaaajgbebaaaacaaaaaacgbjbaaa
abaaaaaaegacbaiaebaaaaaaacaaaaaadiaaaaahhcaabaaaacaaaaaaegacbaaa
acaaaaaapgbpbaaaabaaaaaadgaaaaagbcaabaaaadaaaaaaakiacaaaadaaaaaa
amaaaaaadgaaaaagccaabaaaadaaaaaaakiacaaaadaaaaaaanaaaaaadgaaaaag
ecaabaaaadaaaaaaakiacaaaadaaaaaaaoaaaaaabaaaaaahccaabaaaabaaaaaa
egacbaaaacaaaaaaegacbaaaadaaaaaabaaaaaahbcaabaaaabaaaaaaegbcbaaa
abaaaaaaegacbaaaadaaaaaabaaaaaahecaabaaaabaaaaaaegbcbaaaacaaaaaa
egacbaaaadaaaaaadiaaaaaipccabaaaadaaaaaaegaobaaaabaaaaaapgipcaaa
adaaaaaabeaaaaaadgaaaaaficaabaaaabaaaaaabkaabaaaaaaaaaaadgaaaaag
bcaabaaaadaaaaaabkiacaaaadaaaaaaamaaaaaadgaaaaagccaabaaaadaaaaaa
bkiacaaaadaaaaaaanaaaaaadgaaaaagecaabaaaadaaaaaabkiacaaaadaaaaaa
aoaaaaaabaaaaaahccaabaaaabaaaaaaegacbaaaacaaaaaaegacbaaaadaaaaaa
baaaaaahbcaabaaaabaaaaaaegbcbaaaabaaaaaaegacbaaaadaaaaaabaaaaaah
ecaabaaaabaaaaaaegbcbaaaacaaaaaaegacbaaaadaaaaaadiaaaaaipccabaaa
aeaaaaaaegaobaaaabaaaaaapgipcaaaadaaaaaabeaaaaaadgaaaaagbcaabaaa
abaaaaaackiacaaaadaaaaaaamaaaaaadgaaaaagccaabaaaabaaaaaackiacaaa
adaaaaaaanaaaaaadgaaaaagecaabaaaabaaaaaackiacaaaadaaaaaaaoaaaaaa
baaaaaahccaabaaaaaaaaaaaegacbaaaacaaaaaaegacbaaaabaaaaaabaaaaaah
bcaabaaaaaaaaaaaegbcbaaaabaaaaaaegacbaaaabaaaaaabaaaaaahecaabaaa
aaaaaaaaegbcbaaaacaaaaaaegacbaaaabaaaaaadiaaaaaipccabaaaafaaaaaa
egaobaaaaaaaaaaapgipcaaaadaaaaaabeaaaaaadiaaaaaihcaabaaaaaaaaaaa
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
akaabaaaaaaaaaaabkaabaiaebaaaaaaaaaaaaaadcaaaaakhccabaaaagaaaaaa
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

varying highp vec3 xlv_TEXCOORD5;
varying lowp vec4 xlv_TEXCOORD4;
varying lowp vec4 xlv_TEXCOORD3;
varying lowp vec4 xlv_TEXCOORD2;
varying highp vec4 xlv_TEXCOORD1;
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
  lowp vec4 tmpvar_3;
  lowp vec4 tmpvar_4;
  lowp vec4 tmpvar_5;
  highp vec3 tmpvar_6;
  highp vec4 tmpvar_7;
  tmpvar_7 = (gl_ModelViewProjectionMatrix * _glesVertex);
  highp vec4 tmpvar_8;
  tmpvar_8.w = 1.0;
  tmpvar_8.xyz = _WorldSpaceCameraPos;
  mat3 tmpvar_9;
  tmpvar_9[0] = _Object2World[0].xyz;
  tmpvar_9[1] = _Object2World[1].xyz;
  tmpvar_9[2] = _Object2World[2].xyz;
  highp vec3 tmpvar_10;
  tmpvar_10 = (tmpvar_9 * (_glesVertex.xyz - ((_World2Object * tmpvar_8).xyz * unity_Scale.w)));
  highp vec3 tmpvar_11;
  highp vec3 tmpvar_12;
  tmpvar_11 = tmpvar_1.xyz;
  tmpvar_12 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_13;
  tmpvar_13[0].x = tmpvar_11.x;
  tmpvar_13[0].y = tmpvar_12.x;
  tmpvar_13[0].z = tmpvar_2.x;
  tmpvar_13[1].x = tmpvar_11.y;
  tmpvar_13[1].y = tmpvar_12.y;
  tmpvar_13[1].z = tmpvar_2.y;
  tmpvar_13[2].x = tmpvar_11.z;
  tmpvar_13[2].y = tmpvar_12.z;
  tmpvar_13[2].z = tmpvar_2.z;
  vec4 v_14;
  v_14.x = _Object2World[0].x;
  v_14.y = _Object2World[1].x;
  v_14.z = _Object2World[2].x;
  v_14.w = _Object2World[3].x;
  highp vec4 tmpvar_15;
  tmpvar_15.xyz = (tmpvar_13 * v_14.xyz);
  tmpvar_15.w = tmpvar_10.x;
  highp vec4 tmpvar_16;
  tmpvar_16 = (tmpvar_15 * unity_Scale.w);
  tmpvar_3 = tmpvar_16;
  vec4 v_17;
  v_17.x = _Object2World[0].y;
  v_17.y = _Object2World[1].y;
  v_17.z = _Object2World[2].y;
  v_17.w = _Object2World[3].y;
  highp vec4 tmpvar_18;
  tmpvar_18.xyz = (tmpvar_13 * v_17.xyz);
  tmpvar_18.w = tmpvar_10.y;
  highp vec4 tmpvar_19;
  tmpvar_19 = (tmpvar_18 * unity_Scale.w);
  tmpvar_4 = tmpvar_19;
  vec4 v_20;
  v_20.x = _Object2World[0].z;
  v_20.y = _Object2World[1].z;
  v_20.z = _Object2World[2].z;
  v_20.w = _Object2World[3].z;
  highp vec4 tmpvar_21;
  tmpvar_21.xyz = (tmpvar_13 * v_20.xyz);
  tmpvar_21.w = tmpvar_10.z;
  highp vec4 tmpvar_22;
  tmpvar_22 = (tmpvar_21 * unity_Scale.w);
  tmpvar_5 = tmpvar_22;
  highp vec4 o_23;
  highp vec4 tmpvar_24;
  tmpvar_24 = (tmpvar_7 * 0.5);
  highp vec2 tmpvar_25;
  tmpvar_25.x = tmpvar_24.x;
  tmpvar_25.y = (tmpvar_24.y * _ProjectionParams.x);
  o_23.xy = (tmpvar_25 + tmpvar_24.w);
  o_23.zw = tmpvar_7.zw;
  mat3 tmpvar_26;
  tmpvar_26[0] = _Object2World[0].xyz;
  tmpvar_26[1] = _Object2World[1].xyz;
  tmpvar_26[2] = _Object2World[2].xyz;
  highp vec4 tmpvar_27;
  tmpvar_27.w = 1.0;
  tmpvar_27.xyz = (tmpvar_26 * (tmpvar_2 * unity_Scale.w));
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
  tmpvar_6 = tmpvar_28;
  gl_Position = tmpvar_7;
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = o_23;
  xlv_TEXCOORD2 = tmpvar_3;
  xlv_TEXCOORD3 = tmpvar_4;
  xlv_TEXCOORD4 = tmpvar_5;
  xlv_TEXCOORD5 = tmpvar_6;
}



#endif
#ifdef FRAGMENT

varying highp vec3 xlv_TEXCOORD5;
varying lowp vec4 xlv_TEXCOORD4;
varying lowp vec4 xlv_TEXCOORD3;
varying lowp vec4 xlv_TEXCOORD2;
varying highp vec4 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform sampler2D _LightBuffer;
uniform mediump float _Gloss;
uniform mediump float _ReflectPower;
uniform lowp vec4 _ReflectColor;
uniform lowp vec4 _Color;
uniform samplerCube _Cube;
uniform sampler2D _SpecMap;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform lowp vec4 _SpecColor;
void main ()
{
  lowp vec4 tmpvar_1;
  mediump vec4 c_2;
  mediump vec4 light_3;
  highp vec3 tmpvar_4;
  mediump vec3 tmpvar_5;
  mediump vec3 tmpvar_6;
  mediump vec3 tmpvar_7;
  lowp vec3 tmpvar_8;
  tmpvar_8.x = xlv_TEXCOORD2.w;
  tmpvar_8.y = xlv_TEXCOORD3.w;
  tmpvar_8.z = xlv_TEXCOORD4.w;
  tmpvar_4 = tmpvar_8;
  lowp vec3 tmpvar_9;
  tmpvar_9 = xlv_TEXCOORD2.xyz;
  tmpvar_5 = tmpvar_9;
  lowp vec3 tmpvar_10;
  tmpvar_10 = xlv_TEXCOORD3.xyz;
  tmpvar_6 = tmpvar_10;
  lowp vec3 tmpvar_11;
  tmpvar_11 = xlv_TEXCOORD4.xyz;
  tmpvar_7 = tmpvar_11;
  mediump vec3 tmpvar_12;
  mediump vec3 tmpvar_13;
  mediump float tmpvar_14;
  mediump vec4 spec_15;
  lowp vec4 tmpvar_16;
  tmpvar_16 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_17;
  tmpvar_17 = (tmpvar_16.xyz * _Color.xyz);
  tmpvar_12 = tmpvar_17;
  lowp vec4 tmpvar_18;
  tmpvar_18 = texture2D (_SpecMap, xlv_TEXCOORD0);
  spec_15 = tmpvar_18;
  lowp vec3 tmpvar_19;
  tmpvar_19 = ((texture2D (_BumpMap, xlv_TEXCOORD0).xyz * 2.0) - 1.0);
  tmpvar_13 = tmpvar_19;
  mediump vec3 tmpvar_20;
  tmpvar_20.x = dot (tmpvar_5, tmpvar_13);
  tmpvar_20.y = dot (tmpvar_6, tmpvar_13);
  tmpvar_20.z = dot (tmpvar_7, tmpvar_13);
  highp vec3 tmpvar_21;
  tmpvar_21 = (tmpvar_4 - (2.0 * (dot (tmpvar_20, tmpvar_4) * tmpvar_20)));
  lowp vec4 tmpvar_22;
  tmpvar_22 = (textureCube (_Cube, tmpvar_21) * tmpvar_16.w);
  lowp float tmpvar_23;
  tmpvar_23 = (tmpvar_22.w * _ReflectColor.w);
  tmpvar_14 = tmpvar_23;
  lowp vec4 tmpvar_24;
  tmpvar_24 = texture2DProj (_LightBuffer, xlv_TEXCOORD1);
  light_3 = tmpvar_24;
  mediump vec4 tmpvar_25;
  tmpvar_25 = max (light_3, vec4(0.001, 0.001, 0.001, 0.001));
  light_3.w = tmpvar_25.w;
  highp vec3 tmpvar_26;
  tmpvar_26 = (tmpvar_25.xyz + xlv_TEXCOORD5);
  light_3.xyz = tmpvar_26;
  mediump vec4 c_27;
  mediump vec3 tmpvar_28;
  tmpvar_28 = (tmpvar_25.w * (spec_15.xyz * _Gloss));
  c_27.xyz = ((tmpvar_12 * light_3.xyz) + (light_3.xyz * tmpvar_28));
  c_27.w = (tmpvar_14 + (tmpvar_28 * _SpecColor.w)).x;
  c_2.w = c_27.w;
  c_2.xyz = (c_27.xyz + ((tmpvar_22.xyz * _ReflectColor.xyz) * _ReflectPower));
  tmpvar_1 = c_2;
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

varying highp vec3 xlv_TEXCOORD5;
varying lowp vec4 xlv_TEXCOORD4;
varying lowp vec4 xlv_TEXCOORD3;
varying lowp vec4 xlv_TEXCOORD2;
varying highp vec4 xlv_TEXCOORD1;
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
  lowp vec4 tmpvar_3;
  lowp vec4 tmpvar_4;
  lowp vec4 tmpvar_5;
  highp vec3 tmpvar_6;
  highp vec4 tmpvar_7;
  tmpvar_7 = (gl_ModelViewProjectionMatrix * _glesVertex);
  highp vec4 tmpvar_8;
  tmpvar_8.w = 1.0;
  tmpvar_8.xyz = _WorldSpaceCameraPos;
  mat3 tmpvar_9;
  tmpvar_9[0] = _Object2World[0].xyz;
  tmpvar_9[1] = _Object2World[1].xyz;
  tmpvar_9[2] = _Object2World[2].xyz;
  highp vec3 tmpvar_10;
  tmpvar_10 = (tmpvar_9 * (_glesVertex.xyz - ((_World2Object * tmpvar_8).xyz * unity_Scale.w)));
  highp vec3 tmpvar_11;
  highp vec3 tmpvar_12;
  tmpvar_11 = tmpvar_1.xyz;
  tmpvar_12 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_13;
  tmpvar_13[0].x = tmpvar_11.x;
  tmpvar_13[0].y = tmpvar_12.x;
  tmpvar_13[0].z = tmpvar_2.x;
  tmpvar_13[1].x = tmpvar_11.y;
  tmpvar_13[1].y = tmpvar_12.y;
  tmpvar_13[1].z = tmpvar_2.y;
  tmpvar_13[2].x = tmpvar_11.z;
  tmpvar_13[2].y = tmpvar_12.z;
  tmpvar_13[2].z = tmpvar_2.z;
  vec4 v_14;
  v_14.x = _Object2World[0].x;
  v_14.y = _Object2World[1].x;
  v_14.z = _Object2World[2].x;
  v_14.w = _Object2World[3].x;
  highp vec4 tmpvar_15;
  tmpvar_15.xyz = (tmpvar_13 * v_14.xyz);
  tmpvar_15.w = tmpvar_10.x;
  highp vec4 tmpvar_16;
  tmpvar_16 = (tmpvar_15 * unity_Scale.w);
  tmpvar_3 = tmpvar_16;
  vec4 v_17;
  v_17.x = _Object2World[0].y;
  v_17.y = _Object2World[1].y;
  v_17.z = _Object2World[2].y;
  v_17.w = _Object2World[3].y;
  highp vec4 tmpvar_18;
  tmpvar_18.xyz = (tmpvar_13 * v_17.xyz);
  tmpvar_18.w = tmpvar_10.y;
  highp vec4 tmpvar_19;
  tmpvar_19 = (tmpvar_18 * unity_Scale.w);
  tmpvar_4 = tmpvar_19;
  vec4 v_20;
  v_20.x = _Object2World[0].z;
  v_20.y = _Object2World[1].z;
  v_20.z = _Object2World[2].z;
  v_20.w = _Object2World[3].z;
  highp vec4 tmpvar_21;
  tmpvar_21.xyz = (tmpvar_13 * v_20.xyz);
  tmpvar_21.w = tmpvar_10.z;
  highp vec4 tmpvar_22;
  tmpvar_22 = (tmpvar_21 * unity_Scale.w);
  tmpvar_5 = tmpvar_22;
  highp vec4 o_23;
  highp vec4 tmpvar_24;
  tmpvar_24 = (tmpvar_7 * 0.5);
  highp vec2 tmpvar_25;
  tmpvar_25.x = tmpvar_24.x;
  tmpvar_25.y = (tmpvar_24.y * _ProjectionParams.x);
  o_23.xy = (tmpvar_25 + tmpvar_24.w);
  o_23.zw = tmpvar_7.zw;
  mat3 tmpvar_26;
  tmpvar_26[0] = _Object2World[0].xyz;
  tmpvar_26[1] = _Object2World[1].xyz;
  tmpvar_26[2] = _Object2World[2].xyz;
  highp vec4 tmpvar_27;
  tmpvar_27.w = 1.0;
  tmpvar_27.xyz = (tmpvar_26 * (tmpvar_2 * unity_Scale.w));
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
  tmpvar_6 = tmpvar_28;
  gl_Position = tmpvar_7;
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = o_23;
  xlv_TEXCOORD2 = tmpvar_3;
  xlv_TEXCOORD3 = tmpvar_4;
  xlv_TEXCOORD4 = tmpvar_5;
  xlv_TEXCOORD5 = tmpvar_6;
}



#endif
#ifdef FRAGMENT

varying highp vec3 xlv_TEXCOORD5;
varying lowp vec4 xlv_TEXCOORD4;
varying lowp vec4 xlv_TEXCOORD3;
varying lowp vec4 xlv_TEXCOORD2;
varying highp vec4 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform sampler2D _LightBuffer;
uniform mediump float _Gloss;
uniform mediump float _ReflectPower;
uniform lowp vec4 _ReflectColor;
uniform lowp vec4 _Color;
uniform samplerCube _Cube;
uniform sampler2D _SpecMap;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform lowp vec4 _SpecColor;
void main ()
{
  lowp vec4 tmpvar_1;
  mediump vec4 c_2;
  mediump vec4 light_3;
  highp vec3 tmpvar_4;
  mediump vec3 tmpvar_5;
  mediump vec3 tmpvar_6;
  mediump vec3 tmpvar_7;
  lowp vec3 tmpvar_8;
  tmpvar_8.x = xlv_TEXCOORD2.w;
  tmpvar_8.y = xlv_TEXCOORD3.w;
  tmpvar_8.z = xlv_TEXCOORD4.w;
  tmpvar_4 = tmpvar_8;
  lowp vec3 tmpvar_9;
  tmpvar_9 = xlv_TEXCOORD2.xyz;
  tmpvar_5 = tmpvar_9;
  lowp vec3 tmpvar_10;
  tmpvar_10 = xlv_TEXCOORD3.xyz;
  tmpvar_6 = tmpvar_10;
  lowp vec3 tmpvar_11;
  tmpvar_11 = xlv_TEXCOORD4.xyz;
  tmpvar_7 = tmpvar_11;
  mediump vec3 tmpvar_12;
  mediump vec3 tmpvar_13;
  mediump float tmpvar_14;
  mediump vec4 spec_15;
  lowp vec4 tmpvar_16;
  tmpvar_16 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_17;
  tmpvar_17 = (tmpvar_16.xyz * _Color.xyz);
  tmpvar_12 = tmpvar_17;
  lowp vec4 tmpvar_18;
  tmpvar_18 = texture2D (_SpecMap, xlv_TEXCOORD0);
  spec_15 = tmpvar_18;
  lowp vec3 normal_19;
  normal_19.xy = ((texture2D (_BumpMap, xlv_TEXCOORD0).wy * 2.0) - 1.0);
  normal_19.z = sqrt(((1.0 - (normal_19.x * normal_19.x)) - (normal_19.y * normal_19.y)));
  tmpvar_13 = normal_19;
  mediump vec3 tmpvar_20;
  tmpvar_20.x = dot (tmpvar_5, tmpvar_13);
  tmpvar_20.y = dot (tmpvar_6, tmpvar_13);
  tmpvar_20.z = dot (tmpvar_7, tmpvar_13);
  highp vec3 tmpvar_21;
  tmpvar_21 = (tmpvar_4 - (2.0 * (dot (tmpvar_20, tmpvar_4) * tmpvar_20)));
  lowp vec4 tmpvar_22;
  tmpvar_22 = (textureCube (_Cube, tmpvar_21) * tmpvar_16.w);
  lowp float tmpvar_23;
  tmpvar_23 = (tmpvar_22.w * _ReflectColor.w);
  tmpvar_14 = tmpvar_23;
  lowp vec4 tmpvar_24;
  tmpvar_24 = texture2DProj (_LightBuffer, xlv_TEXCOORD1);
  light_3 = tmpvar_24;
  mediump vec4 tmpvar_25;
  tmpvar_25 = max (light_3, vec4(0.001, 0.001, 0.001, 0.001));
  light_3.w = tmpvar_25.w;
  highp vec3 tmpvar_26;
  tmpvar_26 = (tmpvar_25.xyz + xlv_TEXCOORD5);
  light_3.xyz = tmpvar_26;
  mediump vec4 c_27;
  mediump vec3 tmpvar_28;
  tmpvar_28 = (tmpvar_25.w * (spec_15.xyz * _Gloss));
  c_27.xyz = ((tmpvar_12 * light_3.xyz) + (light_3.xyz * tmpvar_28));
  c_27.w = (tmpvar_14 + (tmpvar_28 * _SpecColor.w)).x;
  c_2.w = c_27.w;
  c_2.xyz = (c_27.xyz + ((tmpvar_22.xyz * _ReflectColor.xyz) * _ReflectPower));
  tmpvar_1 = c_2;
  gl_FragData[0] = tmpvar_1;
}



#endif"
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
"3.0-!!ARBvp1.0
# 46 ALU
PARAM c[23] = { { 0.5, 1 },
		state.matrix.modelview[0],
		state.matrix.mvp,
		program.local[9..22] };
TEMP R0;
TEMP R1;
TEMP R2;
MOV R0.xyz, vertex.attrib[14];
MUL R1.xyz, vertex.normal.zxyw, R0.yzxw;
MAD R0.xyz, vertex.normal.yzxw, R0.zxyw, -R1;
MUL R1.xyz, vertex.attrib[14].w, R0;
MOV R0.xyz, c[17];
MOV R0.w, c[0].y;
DP4 R2.z, R0, c[15];
DP4 R2.x, R0, c[13];
DP4 R2.y, R0, c[14];
MAD R2.xyz, R2, c[20].w, -vertex.position;
DP3 R0.y, R1, c[9];
DP3 R0.w, -R2, c[9];
DP4 R1.w, vertex.position, c[8];
DP3 R0.x, vertex.attrib[14], c[9];
DP3 R0.z, vertex.normal, c[9];
MUL result.texcoord[2], R0, c[20].w;
DP3 R0.y, R1, c[10];
DP3 R0.w, -R2, c[10];
DP3 R0.x, vertex.attrib[14], c[10];
DP3 R0.z, vertex.normal, c[10];
MUL result.texcoord[3], R0, c[20].w;
DP3 R0.y, R1, c[11];
DP4 R1.z, vertex.position, c[7];
DP3 R0.w, -R2, c[11];
DP4 R1.x, vertex.position, c[5];
DP4 R1.y, vertex.position, c[6];
MUL R2.xyz, R1.xyww, c[0].x;
DP3 R0.x, vertex.attrib[14], c[11];
DP3 R0.z, vertex.normal, c[11];
MUL result.texcoord[4], R0, c[20].w;
MOV R0.x, R2;
MUL R0.y, R2, c[18].x;
ADD result.texcoord[1].xy, R0, R2.z;
DP4 R0.z, vertex.position, c[11];
DP4 R0.x, vertex.position, c[9];
DP4 R0.y, vertex.position, c[10];
ADD R0.xyz, R0, -c[19];
MUL result.texcoord[6].xyz, R0, c[19].w;
MOV R0.x, c[0].y;
ADD R0.y, R0.x, -c[19].w;
DP4 R0.x, vertex.position, c[3];
MOV result.position, R1;
MOV result.texcoord[1].zw, R1;
MAD result.texcoord[0].xy, vertex.texcoord[0], c[22], c[22].zwzw;
MAD result.texcoord[5].xy, vertex.texcoord[1], c[21], c[21].zwzw;
MUL result.texcoord[6].w, -R0.x, R0.y;
END
# 46 instructions, 3 R-regs
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
"vs_3_0
; 47 ALU
dcl_position o0
dcl_texcoord0 o1
dcl_texcoord1 o2
dcl_texcoord2 o3
dcl_texcoord3 o4
dcl_texcoord4 o5
dcl_texcoord5 o6
dcl_texcoord6 o7
def c23, 0.50000000, 1.00000000, 0, 0
dcl_position0 v0
dcl_tangent0 v1
dcl_normal0 v2
dcl_texcoord0 v3
dcl_texcoord1 v4
mov r0.xyz, v1
mul r1.xyz, v2.zxyw, r0.yzxw
mov r0.xyz, v1
mad r0.xyz, v2.yzxw, r0.zxyw, -r1
mul r1.xyz, v1.w, r0
mov r0.xyz, c16
mov r0.w, c23.y
dp4 r2.z, r0, c14
dp4 r2.x, r0, c12
dp4 r2.y, r0, c13
mad r2.xyz, r2, c20.w, -v0
dp3 r0.y, r1, c8
dp3 r0.w, -r2, c8
dp4 r1.w, v0, c7
dp3 r0.x, v1, c8
dp3 r0.z, v2, c8
mul o3, r0, c20.w
dp3 r0.y, r1, c9
dp3 r0.w, -r2, c9
dp3 r0.x, v1, c9
dp3 r0.z, v2, c9
mul o4, r0, c20.w
dp3 r0.y, r1, c10
dp4 r1.z, v0, c6
dp3 r0.w, -r2, c10
dp4 r1.x, v0, c4
dp4 r1.y, v0, c5
mul r2.xyz, r1.xyww, c23.x
dp3 r0.x, v1, c10
dp3 r0.z, v2, c10
mul o5, r0, c20.w
mov r0.x, r2
mul r0.y, r2, c17.x
mad o2.xy, r2.z, c18.zwzw, r0
dp4 r0.z, v0, c10
dp4 r0.x, v0, c8
dp4 r0.y, v0, c9
add r0.xyz, r0, -c19
mul o7.xyz, r0, c19.w
mov r0.x, c19.w
add r0.y, c23, -r0.x
dp4 r0.x, v0, c2
mov o0, r1
mov o2.zw, r1
mad o1.xy, v3, c22, c22.zwzw
mad o6.xy, v4, c21, c21.zwzw
mul o7.w, -r0.x, r0.y
"
}

SubProgram "xbox360 " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
Bind "vertex" Vertex
Bind "tangent" TexCoord2
Bind "normal" Normal
Bind "texcoord" TexCoord0
Bind "texcoord1" TexCoord1
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
// ALU: 66.67 (50 instructions), vertex: 64, texture: 0,
//   sequencer: 24,  14 GPRs, 12 threads,
// Performance (if enough threads): ~66 cycles per vector
// * Vertex cycle estimates are assuming 3 vfetch_minis for every vfetch_full,
//     with <= 32 bytes per vfetch_full group.

"vs_360
backbbabaaaaacmmaaaaadciaaaaaaaaaaaaaaceaaaaaccmaaaaacfeaaaaaaaa
aaaaaaaaaaaaacaeaaaaaabmaaaaabphpppoadaaaaaaaaalaaaaaabmaaaaaaaa
aaaaabpaaaaaaapiaaacaabgaaabaaaaaaaaabaeaaaaaaaaaaaaabbeaaacaaam
aaaeaaaaaaaaabceaaaaaaaaaaaaabdeaaacaaabaaabaaaaaaaaabaeaaaaaaaa
aaaaabegaaacaaacaaabaaaaaaaaabaeaaaaaaaaaaaaabfeaaacaabaaaaeaaaa
aaaaabceaaaaaaaaaaaaabgcaaacaaaaaaabaaaaaaaaabhiaaaaaaaaaaaaabii
aaacaaaiaaaeaaaaaaaaabceaaaaaaaaaaaaabkcaaacaaaeaaaeaaaaaaaaabce
aaaaaaaaaaaaablfaaacaabfaaabaaaaaaaaabaeaaaaaaaaaaaaabmgaaacaabe
aaabaaaaaaaaabaeaaaaaaaaaaaaabncaaacaaadaaabaaaaaaaaabaeaaaaaaaa
fpengbgjgofegfhifpfdfeaaaaabaaadaaabaaaeaaabaaaaaaaaaaaafpepgcgk
gfgdhedcfhgphcgmgeaaklklaaadaaadaaaeaaaeaaabaaaaaaaaaaaafpfahcgp
gkgfgdhegjgpgofagbhcgbgnhdaafpfdgdhcgfgfgofagbhcgbgnhdaafpfhgphc
gmgedcepgcgkgfgdheaafpfhgphcgmgefdhagbgdgfedgbgngfhcgbfagphdaakl
aaabaaadaaabaaadaaabaaaaaaaaaaaaghgmhdhegbhegffpgngbhehcgjhifpgn
gpgegfgmhggjgfhhdaaaghgmhdhegbhegffpgngbhehcgjhifpgnhghaaahfgogj
hehjfpemgjghgihegngbhafdfeaahfgogjhehjfpfdgdgbgmgfaahfgogjhehjfp
fdgigbgegphheggbgegfedgfgohegfhcebgogefehjhagfaahghdfpddfpdaaadc
codacodcdadddfddcodaaaklaaaaaaaaaaaaaaabaaaaaaaaaaaaaaaaaaaaaabe
aapmaabaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaeaaaaaacoiaagbaaan
aaaaaaaaaaaaaaaaaaaagaohaaaaaaabaaaaaaafaaaaaaaiaaaaacjaaabaaaag
aaaagaahaaaadaaiaaaafaajaacbfaakaaaadafaaaabpbfbaaadpcfcaaaepdfd
aaafpefeaaagdfffaaahpgfgaaaabacpaaaaaacoaaaabadjaaaabadkaaaabadl
aaaabadmaaaabadaaaaabadiaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaadpiaaaaa
dpaaaaaaaaaaaaaaaaaaaaaapbfffaagaaaabcabmcaaaaaaaaaafaalaaaabcaa
meaaaaaaaaaagabagabgbcaabcaaaaaaaaaagabmgaccbcaabcaaaaaaaaaagaci
gacobcaabcaaaaaaaaaagadedadkbcaaccaaaaaaafpigaaaaaaaagiiaaaaaaaa
afpibaaaaaaaagiiaaaaaaaaafpiiaaaaaaaaoiiaaaaaaaaafpilaaaaaaaapmi
aaaaaaaaafpihaaaaaaaacdpaaaaaaaamiapaaaaaabliiaakbagahaamiapaaaa
aamgnapiklagagaamiapaaaaaalbdepiklagafaamiapaaamaagmnajeklagaeaa
miapiadoaananaaaocamamaamiabaaafaeblgmaacaadppaamiaoaaafaapmmgpm
albcaabdmiaiaaaiaamgmgaakbagakaabebiaaajaalbmggmkbagajagkiibakan
aablgmmaibabamaibeceaaanaablgmblkbabaoabkicoanadaamgimebibaiaoan
bebhaaaeaalblomgkbaianabkiihaeacaalbgcmaibabanaomialaaaaaalelbmj
clbbaaafkmbhadajaalbleicibaganaokmeoaaafaakgebecmbaiabaomiahaaak
aagmmaleklagamajmiaoaaafabebkgabolaiabafmialaaaaaamagmliclbaaaaa
miahaaaeaagmgcleklaiamaemialaaacaagmloleklabamacbeaeaaacaagmmggm
oaaeadacaebeacadaalbblbloaaeadaebeahaaaiaablleblkbagapacaebeadae
aamglbgmoaaeadadbeahaaajaamglelbkbagaoacmiahaaagabbablmaklaabeag
aeblaeaaaabcblmgobafabaakiccacaeaalomdmdnaanafaokicoadafaemghgma
ibagaoankiihadabaelbgciaibagananmiadaaahaalblcbjklaaamadmiahaaag
aegmloleklagamabkichadaaaamalbidibamppaomiamiaabaanlnlaaocamamaa
miadiaaaaalalabkilalbgbgmiadiaafaabklabkilahbfbfkiiiaaacaagmlbeb
maagafabbeapaaabaadeaagmoaakajahaeciacadaamgmglboaagafacbeapaaab
aadedelboaabaiahaeciadaeaalbbllboaagafadmiaiaaabaablmgblklagalab
beahaaabadmamablkaabadabamihiaagaamablgmkbabadafmiadiaabaamgbkbi
klaaacaamiapiaacaaaablaakbaebeaamiapiaadaaaablaakbadbeaamiapiaae
aaaablaakbacbeaaaaaaaaaaaaaaaaaaaaaaaaaa"
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
"sce_vp_rsx // 44 instructions using 8 registers
[Configuration]
8
0000002c43050800
[Defaults]
1
461 2
3f0000003f800000
[Microcode]
704
00029c6c00400e0c0106c0836041dffc00039c6c005d300c0186c0836041dffc
401f9c6c011ce808010400d740619f9c401f9c6c011cf908010400d740619fb0
00001c6c0150ae0c010600c360411ffc00001c6c0150a20c010600c360405ffc
00009c6c01509e0c010600c360411ffc00009c6c0150920c010600c360405ffc
00011c6c01508e0c010600c360411ffc00011c6c0150820c010600c360405ffc
00001c6c01d0200d8106c0c360409ffc00019c6c01d0700d8106c0c360403ffc
00019c6c01d0600d8106c0c360405ffc00019c6c01d0500d8106c0c360409ffc
00019c6c01d0400d8106c0c360411ffc00001c6c005d107f8186c08360403ffc
00031c6c01d0a00d8106c0c360405ffc00031c6c01d0900d8106c0c360409ffc
00031c6c01d0800d8106c0c360411ffc00021c6c0190e00c0e86c0c360405ffc
00021c6c0190d00c0e86c0c360409ffc00021c6c0190c00c0e86c0c360411ffc
00039c6c00dd108c0186c0830321dffc00031c6c00800243011845436041dffc
00001c6c00dcd02a8186c0bfe0203ffc00029c6c01000230812185630321dffc
401f9c6c0040000d8686c0836041ff80401f9c6c004000558686c08360407fa0
00021c6c011d000c08bfc0e30041dffc00019c6c009cd00e068000c36041dffc
401f9c6c009d100c0ebfc0c36041dfb4401f9c6c008000aa80bfc04360403fb4
00019c6c009d202a868000c360409ffc401f9c6c00c000080686c09541a19fa0
00001c6c0150a08c088600c360403ffc00009c6c0150908c088600c360403ffc
00011c6c0150808c088600c360403ffc00019c6c00800e7f810605436041dffc
00001c6c0150a00c068600c360409ffc00011c6c0150800c068600c360409ffc
00009c6c0150900c068600c360409ffc401f9c6c009d000d84bfc0c36041ffa4
401f9c6c009d000d82bfc0c36041ffa8401f9c6c009d000d80bfc0c36041ffad
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
Vector 96 [unity_LightmapST] 4
Vector 112 [_MainTex_ST] 4
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
// 58 instructions, 4 temp regs, 0 temp arrays:
// ALU 27 float, 0 int, 0 uint
// TEX 0 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"vs_4_0
eefiecedplhibnpohhdgcioijpemcdpoolgcapcdabaaaaaaamakaaaaadaaaaaa
cmaaaaaapeaaaaaanmabaaaaejfdeheomaaaaaaaagaaaaaaaiaaaaaajiaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapapaaaakbaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaapapaaaakjaaaaaaaaaaaaaaaaaaaaaaadaaaaaaacaaaaaa
ahahaaaalaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaadaaaaaaapadaaaalaaaaaaa
abaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapadaaaaljaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaafaepfdejfeejepeoaafeebeoehefeofeaaeoepfc
enebemaafeeffiedepepfceeaaedepemepfcaaklepfdeheooaaaaaaaaiaaaaaa
aiaaaaaamiaaaaaaaaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaneaaaaaa
aaaaaaaaaaaaaaaaadaaaaaaabaaaaaaadamaaaaneaaaaaaafaaaaaaaaaaaaaa
adaaaaaaabaaaaaaamadaaaaneaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaa
apaaaaaaneaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaapaaaaaaneaaaaaa
adaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapaaaaaaneaaaaaaaeaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapaaaaaaneaaaaaaagaaaaaaaaaaaaaaadaaaaaaagaaaaaa
apaaaaaafdfgfpfaepfdejfeejepeoaafeeffiedepepfceeaaklklklfdeieefc
ciaiaaaaeaaaabaaakacaaaafjaaaaaeegiocaaaaaaaaaaaaiaaaaaafjaaaaae
egiocaaaabaaaaaaagaaaaaafjaaaaaeegiocaaaacaaaaaabkaaaaaafjaaaaae
egiocaaaadaaaaaabfaaaaaafpaaaaadpcbabaaaaaaaaaaafpaaaaadpcbabaaa
abaaaaaafpaaaaadhcbabaaaacaaaaaafpaaaaaddcbabaaaadaaaaaafpaaaaad
dcbabaaaaeaaaaaaghaaaaaepccabaaaaaaaaaaaabaaaaaagfaaaaaddccabaaa
abaaaaaagfaaaaadmccabaaaabaaaaaagfaaaaadpccabaaaacaaaaaagfaaaaad
pccabaaaadaaaaaagfaaaaadpccabaaaaeaaaaaagfaaaaadpccabaaaafaaaaaa
gfaaaaadpccabaaaagaaaaaagiaaaaacaeaaaaaadiaaaaaipcaabaaaaaaaaaaa
fgbfbaaaaaaaaaaaegiocaaaadaaaaaaabaaaaaadcaaaaakpcaabaaaaaaaaaaa
egiocaaaadaaaaaaaaaaaaaaagbabaaaaaaaaaaaegaobaaaaaaaaaaadcaaaaak
pcaabaaaaaaaaaaaegiocaaaadaaaaaaacaaaaaakgbkbaaaaaaaaaaaegaobaaa
aaaaaaaadcaaaaakpcaabaaaaaaaaaaaegiocaaaadaaaaaaadaaaaaapgbpbaaa
aaaaaaaaegaobaaaaaaaaaaadgaaaaafpccabaaaaaaaaaaaegaobaaaaaaaaaaa
dcaaaaaldccabaaaabaaaaaaegbabaaaadaaaaaaegiacaaaaaaaaaaaahaaaaaa
ogikcaaaaaaaaaaaahaaaaaadcaaaaalmccabaaaabaaaaaaagbebaaaaeaaaaaa
agiecaaaaaaaaaaaagaaaaaakgiocaaaaaaaaaaaagaaaaaadiaaaaaiccaabaaa
aaaaaaaabkaabaaaaaaaaaaaakiacaaaabaaaaaaafaaaaaadiaaaaakncaabaaa
abaaaaaaagahbaaaaaaaaaaaaceaaaaaaaaaaadpaaaaaaaaaaaaaadpaaaaaadp
dgaaaaafmccabaaaacaaaaaakgaobaaaaaaaaaaaaaaaaaahdccabaaaacaaaaaa
kgakbaaaabaaaaaamgaabaaaabaaaaaadiaaaaajhcaabaaaaaaaaaaafgifcaaa
abaaaaaaaeaaaaaaegiccaaaadaaaaaabbaaaaaadcaaaaalhcaabaaaaaaaaaaa
egiccaaaadaaaaaabaaaaaaaagiacaaaabaaaaaaaeaaaaaaegacbaaaaaaaaaaa
dcaaaaalhcaabaaaaaaaaaaaegiccaaaadaaaaaabcaaaaaakgikcaaaabaaaaaa
aeaaaaaaegacbaaaaaaaaaaaaaaaaaaihcaabaaaaaaaaaaaegacbaaaaaaaaaaa
egiccaaaadaaaaaabdaaaaaadcaaaaalhcaabaaaaaaaaaaaegacbaaaaaaaaaaa
pgipcaaaadaaaaaabeaaaaaaegbcbaiaebaaaaaaaaaaaaaadiaaaaajhcaabaaa
abaaaaaafgafbaiaebaaaaaaaaaaaaaaegiccaaaadaaaaaaanaaaaaadcaaaaal
lcaabaaaaaaaaaaaegiicaaaadaaaaaaamaaaaaaagaabaiaebaaaaaaaaaaaaaa
egaibaaaabaaaaaadcaaaaallcaabaaaaaaaaaaaegiicaaaadaaaaaaaoaaaaaa
kgakbaiaebaaaaaaaaaaaaaaegambaaaaaaaaaaadgaaaaaficaabaaaabaaaaaa
akaabaaaaaaaaaaadiaaaaahhcaabaaaacaaaaaajgbebaaaabaaaaaacgbjbaaa
acaaaaaadcaaaaakhcaabaaaacaaaaaajgbebaaaacaaaaaacgbjbaaaabaaaaaa
egacbaiaebaaaaaaacaaaaaadiaaaaahhcaabaaaacaaaaaaegacbaaaacaaaaaa
pgbpbaaaabaaaaaadgaaaaagbcaabaaaadaaaaaaakiacaaaadaaaaaaamaaaaaa
dgaaaaagccaabaaaadaaaaaaakiacaaaadaaaaaaanaaaaaadgaaaaagecaabaaa
adaaaaaaakiacaaaadaaaaaaaoaaaaaabaaaaaahccaabaaaabaaaaaaegacbaaa
acaaaaaaegacbaaaadaaaaaabaaaaaahbcaabaaaabaaaaaaegbcbaaaabaaaaaa
egacbaaaadaaaaaabaaaaaahecaabaaaabaaaaaaegbcbaaaacaaaaaaegacbaaa
adaaaaaadiaaaaaipccabaaaadaaaaaaegaobaaaabaaaaaapgipcaaaadaaaaaa
beaaaaaadgaaaaaficaabaaaabaaaaaabkaabaaaaaaaaaaadgaaaaagbcaabaaa
adaaaaaabkiacaaaadaaaaaaamaaaaaadgaaaaagccaabaaaadaaaaaabkiacaaa
adaaaaaaanaaaaaadgaaaaagecaabaaaadaaaaaabkiacaaaadaaaaaaaoaaaaaa
baaaaaahccaabaaaabaaaaaaegacbaaaacaaaaaaegacbaaaadaaaaaabaaaaaah
bcaabaaaabaaaaaaegbcbaaaabaaaaaaegacbaaaadaaaaaabaaaaaahecaabaaa
abaaaaaaegbcbaaaacaaaaaaegacbaaaadaaaaaadiaaaaaipccabaaaaeaaaaaa
egaobaaaabaaaaaapgipcaaaadaaaaaabeaaaaaadgaaaaagbcaabaaaabaaaaaa
ckiacaaaadaaaaaaamaaaaaadgaaaaagccaabaaaabaaaaaackiacaaaadaaaaaa
anaaaaaadgaaaaagecaabaaaabaaaaaackiacaaaadaaaaaaaoaaaaaabaaaaaah
ccaabaaaaaaaaaaaegacbaaaacaaaaaaegacbaaaabaaaaaabaaaaaahbcaabaaa
aaaaaaaaegbcbaaaabaaaaaaegacbaaaabaaaaaabaaaaaahecaabaaaaaaaaaaa
egbcbaaaacaaaaaaegacbaaaabaaaaaadiaaaaaipccabaaaafaaaaaaegaobaaa
aaaaaaaapgipcaaaadaaaaaabeaaaaaadiaaaaaihcaabaaaaaaaaaaafgbfbaaa
aaaaaaaaegiccaaaadaaaaaaanaaaaaadcaaaaakhcaabaaaaaaaaaaaegiccaaa
adaaaaaaamaaaaaaagbabaaaaaaaaaaaegacbaaaaaaaaaaadcaaaaakhcaabaaa
aaaaaaaaegiccaaaadaaaaaaaoaaaaaakgbkbaaaaaaaaaaaegacbaaaaaaaaaaa
dcaaaaakhcaabaaaaaaaaaaaegiccaaaadaaaaaaapaaaaaapgbpbaaaaaaaaaaa
egacbaaaaaaaaaaaaaaaaaajhcaabaaaaaaaaaaaegacbaaaaaaaaaaaegiccaia
ebaaaaaaacaaaaaabjaaaaaadiaaaaaihccabaaaagaaaaaaegacbaaaaaaaaaaa
pgipcaaaacaaaaaabjaaaaaadiaaaaaibcaabaaaaaaaaaaabkbabaaaaaaaaaaa
ckiacaaaadaaaaaaafaaaaaadcaaaaakbcaabaaaaaaaaaaackiacaaaadaaaaaa
aeaaaaaaakbabaaaaaaaaaaaakaabaaaaaaaaaaadcaaaaakbcaabaaaaaaaaaaa
ckiacaaaadaaaaaaagaaaaaackbabaaaaaaaaaaaakaabaaaaaaaaaaadcaaaaak
bcaabaaaaaaaaaaackiacaaaadaaaaaaahaaaaaadkbabaaaaaaaaaaaakaabaaa
aaaaaaaaaaaaaaajccaabaaaaaaaaaaadkiacaiaebaaaaaaacaaaaaabjaaaaaa
abeaaaaaaaaaiadpdiaaaaaiiccabaaaagaaaaaabkaabaaaaaaaaaaaakaabaia
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

varying highp vec4 xlv_TEXCOORD6;
varying highp vec2 xlv_TEXCOORD5;
varying lowp vec4 xlv_TEXCOORD4;
varying lowp vec4 xlv_TEXCOORD3;
varying lowp vec4 xlv_TEXCOORD2;
varying highp vec4 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
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
  lowp vec4 tmpvar_3;
  lowp vec4 tmpvar_4;
  lowp vec4 tmpvar_5;
  highp vec4 tmpvar_6;
  highp vec4 tmpvar_7;
  tmpvar_7 = (gl_ModelViewProjectionMatrix * _glesVertex);
  highp vec4 tmpvar_8;
  tmpvar_8.w = 1.0;
  tmpvar_8.xyz = _WorldSpaceCameraPos;
  mat3 tmpvar_9;
  tmpvar_9[0] = _Object2World[0].xyz;
  tmpvar_9[1] = _Object2World[1].xyz;
  tmpvar_9[2] = _Object2World[2].xyz;
  highp vec3 tmpvar_10;
  tmpvar_10 = (tmpvar_9 * (_glesVertex.xyz - ((_World2Object * tmpvar_8).xyz * unity_Scale.w)));
  highp vec3 tmpvar_11;
  highp vec3 tmpvar_12;
  tmpvar_11 = tmpvar_1.xyz;
  tmpvar_12 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_13;
  tmpvar_13[0].x = tmpvar_11.x;
  tmpvar_13[0].y = tmpvar_12.x;
  tmpvar_13[0].z = tmpvar_2.x;
  tmpvar_13[1].x = tmpvar_11.y;
  tmpvar_13[1].y = tmpvar_12.y;
  tmpvar_13[1].z = tmpvar_2.y;
  tmpvar_13[2].x = tmpvar_11.z;
  tmpvar_13[2].y = tmpvar_12.z;
  tmpvar_13[2].z = tmpvar_2.z;
  vec4 v_14;
  v_14.x = _Object2World[0].x;
  v_14.y = _Object2World[1].x;
  v_14.z = _Object2World[2].x;
  v_14.w = _Object2World[3].x;
  highp vec4 tmpvar_15;
  tmpvar_15.xyz = (tmpvar_13 * v_14.xyz);
  tmpvar_15.w = tmpvar_10.x;
  highp vec4 tmpvar_16;
  tmpvar_16 = (tmpvar_15 * unity_Scale.w);
  tmpvar_3 = tmpvar_16;
  vec4 v_17;
  v_17.x = _Object2World[0].y;
  v_17.y = _Object2World[1].y;
  v_17.z = _Object2World[2].y;
  v_17.w = _Object2World[3].y;
  highp vec4 tmpvar_18;
  tmpvar_18.xyz = (tmpvar_13 * v_17.xyz);
  tmpvar_18.w = tmpvar_10.y;
  highp vec4 tmpvar_19;
  tmpvar_19 = (tmpvar_18 * unity_Scale.w);
  tmpvar_4 = tmpvar_19;
  vec4 v_20;
  v_20.x = _Object2World[0].z;
  v_20.y = _Object2World[1].z;
  v_20.z = _Object2World[2].z;
  v_20.w = _Object2World[3].z;
  highp vec4 tmpvar_21;
  tmpvar_21.xyz = (tmpvar_13 * v_20.xyz);
  tmpvar_21.w = tmpvar_10.z;
  highp vec4 tmpvar_22;
  tmpvar_22 = (tmpvar_21 * unity_Scale.w);
  tmpvar_5 = tmpvar_22;
  highp vec4 o_23;
  highp vec4 tmpvar_24;
  tmpvar_24 = (tmpvar_7 * 0.5);
  highp vec2 tmpvar_25;
  tmpvar_25.x = tmpvar_24.x;
  tmpvar_25.y = (tmpvar_24.y * _ProjectionParams.x);
  o_23.xy = (tmpvar_25 + tmpvar_24.w);
  o_23.zw = tmpvar_7.zw;
  tmpvar_6.xyz = (((_Object2World * _glesVertex).xyz - unity_ShadowFadeCenterAndType.xyz) * unity_ShadowFadeCenterAndType.w);
  tmpvar_6.w = (-((gl_ModelViewMatrix * _glesVertex).z) * (1.0 - unity_ShadowFadeCenterAndType.w));
  gl_Position = tmpvar_7;
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = o_23;
  xlv_TEXCOORD2 = tmpvar_3;
  xlv_TEXCOORD3 = tmpvar_4;
  xlv_TEXCOORD4 = tmpvar_5;
  xlv_TEXCOORD5 = ((_glesMultiTexCoord1.xy * unity_LightmapST.xy) + unity_LightmapST.zw);
  xlv_TEXCOORD6 = tmpvar_6;
}



#endif
#ifdef FRAGMENT

varying highp vec4 xlv_TEXCOORD6;
varying highp vec2 xlv_TEXCOORD5;
varying lowp vec4 xlv_TEXCOORD4;
varying lowp vec4 xlv_TEXCOORD3;
varying lowp vec4 xlv_TEXCOORD2;
varying highp vec4 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp vec4 unity_LightmapFade;
uniform sampler2D unity_LightmapInd;
uniform sampler2D unity_Lightmap;
uniform sampler2D _LightBuffer;
uniform mediump float _Gloss;
uniform mediump float _ReflectPower;
uniform lowp vec4 _ReflectColor;
uniform lowp vec4 _Color;
uniform samplerCube _Cube;
uniform sampler2D _SpecMap;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform lowp vec4 _SpecColor;
void main ()
{
  lowp vec4 tmpvar_1;
  mediump vec4 c_2;
  mediump vec3 lmIndirect_3;
  mediump vec3 lmFull_4;
  mediump vec4 light_5;
  highp vec3 tmpvar_6;
  mediump vec3 tmpvar_7;
  mediump vec3 tmpvar_8;
  mediump vec3 tmpvar_9;
  lowp vec3 tmpvar_10;
  tmpvar_10.x = xlv_TEXCOORD2.w;
  tmpvar_10.y = xlv_TEXCOORD3.w;
  tmpvar_10.z = xlv_TEXCOORD4.w;
  tmpvar_6 = tmpvar_10;
  lowp vec3 tmpvar_11;
  tmpvar_11 = xlv_TEXCOORD2.xyz;
  tmpvar_7 = tmpvar_11;
  lowp vec3 tmpvar_12;
  tmpvar_12 = xlv_TEXCOORD3.xyz;
  tmpvar_8 = tmpvar_12;
  lowp vec3 tmpvar_13;
  tmpvar_13 = xlv_TEXCOORD4.xyz;
  tmpvar_9 = tmpvar_13;
  mediump vec3 tmpvar_14;
  mediump vec3 tmpvar_15;
  mediump float tmpvar_16;
  mediump vec4 spec_17;
  lowp vec4 tmpvar_18;
  tmpvar_18 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_19;
  tmpvar_19 = (tmpvar_18.xyz * _Color.xyz);
  tmpvar_14 = tmpvar_19;
  lowp vec4 tmpvar_20;
  tmpvar_20 = texture2D (_SpecMap, xlv_TEXCOORD0);
  spec_17 = tmpvar_20;
  lowp vec3 tmpvar_21;
  tmpvar_21 = ((texture2D (_BumpMap, xlv_TEXCOORD0).xyz * 2.0) - 1.0);
  tmpvar_15 = tmpvar_21;
  mediump vec3 tmpvar_22;
  tmpvar_22.x = dot (tmpvar_7, tmpvar_15);
  tmpvar_22.y = dot (tmpvar_8, tmpvar_15);
  tmpvar_22.z = dot (tmpvar_9, tmpvar_15);
  highp vec3 tmpvar_23;
  tmpvar_23 = (tmpvar_6 - (2.0 * (dot (tmpvar_22, tmpvar_6) * tmpvar_22)));
  lowp vec4 tmpvar_24;
  tmpvar_24 = (textureCube (_Cube, tmpvar_23) * tmpvar_18.w);
  lowp float tmpvar_25;
  tmpvar_25 = (tmpvar_24.w * _ReflectColor.w);
  tmpvar_16 = tmpvar_25;
  lowp vec4 tmpvar_26;
  tmpvar_26 = texture2DProj (_LightBuffer, xlv_TEXCOORD1);
  light_5 = tmpvar_26;
  mediump vec4 tmpvar_27;
  tmpvar_27 = max (light_5, vec4(0.001, 0.001, 0.001, 0.001));
  light_5.w = tmpvar_27.w;
  lowp vec3 tmpvar_28;
  tmpvar_28 = (2.0 * texture2D (unity_Lightmap, xlv_TEXCOORD5).xyz);
  lmFull_4 = tmpvar_28;
  lowp vec3 tmpvar_29;
  tmpvar_29 = (2.0 * texture2D (unity_LightmapInd, xlv_TEXCOORD5).xyz);
  lmIndirect_3 = tmpvar_29;
  highp float tmpvar_30;
  tmpvar_30 = clamp (((sqrt(dot (xlv_TEXCOORD6, xlv_TEXCOORD6)) * unity_LightmapFade.z) + unity_LightmapFade.w), 0.0, 1.0);
  light_5.xyz = (tmpvar_27.xyz + mix (lmIndirect_3, lmFull_4, vec3(tmpvar_30)));
  mediump vec4 c_31;
  mediump vec3 tmpvar_32;
  tmpvar_32 = (tmpvar_27.w * (spec_17.xyz * _Gloss));
  c_31.xyz = ((tmpvar_14 * light_5.xyz) + (light_5.xyz * tmpvar_32));
  c_31.w = (tmpvar_16 + (tmpvar_32 * _SpecColor.w)).x;
  c_2.w = c_31.w;
  c_2.xyz = (c_31.xyz + ((tmpvar_24.xyz * _ReflectColor.xyz) * _ReflectPower));
  tmpvar_1 = c_2;
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

varying highp vec4 xlv_TEXCOORD6;
varying highp vec2 xlv_TEXCOORD5;
varying lowp vec4 xlv_TEXCOORD4;
varying lowp vec4 xlv_TEXCOORD3;
varying lowp vec4 xlv_TEXCOORD2;
varying highp vec4 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
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
  lowp vec4 tmpvar_3;
  lowp vec4 tmpvar_4;
  lowp vec4 tmpvar_5;
  highp vec4 tmpvar_6;
  highp vec4 tmpvar_7;
  tmpvar_7 = (gl_ModelViewProjectionMatrix * _glesVertex);
  highp vec4 tmpvar_8;
  tmpvar_8.w = 1.0;
  tmpvar_8.xyz = _WorldSpaceCameraPos;
  mat3 tmpvar_9;
  tmpvar_9[0] = _Object2World[0].xyz;
  tmpvar_9[1] = _Object2World[1].xyz;
  tmpvar_9[2] = _Object2World[2].xyz;
  highp vec3 tmpvar_10;
  tmpvar_10 = (tmpvar_9 * (_glesVertex.xyz - ((_World2Object * tmpvar_8).xyz * unity_Scale.w)));
  highp vec3 tmpvar_11;
  highp vec3 tmpvar_12;
  tmpvar_11 = tmpvar_1.xyz;
  tmpvar_12 = (((tmpvar_2.yzx * tmpvar_1.zxy) - (tmpvar_2.zxy * tmpvar_1.yzx)) * _glesTANGENT.w);
  highp mat3 tmpvar_13;
  tmpvar_13[0].x = tmpvar_11.x;
  tmpvar_13[0].y = tmpvar_12.x;
  tmpvar_13[0].z = tmpvar_2.x;
  tmpvar_13[1].x = tmpvar_11.y;
  tmpvar_13[1].y = tmpvar_12.y;
  tmpvar_13[1].z = tmpvar_2.y;
  tmpvar_13[2].x = tmpvar_11.z;
  tmpvar_13[2].y = tmpvar_12.z;
  tmpvar_13[2].z = tmpvar_2.z;
  vec4 v_14;
  v_14.x = _Object2World[0].x;
  v_14.y = _Object2World[1].x;
  v_14.z = _Object2World[2].x;
  v_14.w = _Object2World[3].x;
  highp vec4 tmpvar_15;
  tmpvar_15.xyz = (tmpvar_13 * v_14.xyz);
  tmpvar_15.w = tmpvar_10.x;
  highp vec4 tmpvar_16;
  tmpvar_16 = (tmpvar_15 * unity_Scale.w);
  tmpvar_3 = tmpvar_16;
  vec4 v_17;
  v_17.x = _Object2World[0].y;
  v_17.y = _Object2World[1].y;
  v_17.z = _Object2World[2].y;
  v_17.w = _Object2World[3].y;
  highp vec4 tmpvar_18;
  tmpvar_18.xyz = (tmpvar_13 * v_17.xyz);
  tmpvar_18.w = tmpvar_10.y;
  highp vec4 tmpvar_19;
  tmpvar_19 = (tmpvar_18 * unity_Scale.w);
  tmpvar_4 = tmpvar_19;
  vec4 v_20;
  v_20.x = _Object2World[0].z;
  v_20.y = _Object2World[1].z;
  v_20.z = _Object2World[2].z;
  v_20.w = _Object2World[3].z;
  highp vec4 tmpvar_21;
  tmpvar_21.xyz = (tmpvar_13 * v_20.xyz);
  tmpvar_21.w = tmpvar_10.z;
  highp vec4 tmpvar_22;
  tmpvar_22 = (tmpvar_21 * unity_Scale.w);
  tmpvar_5 = tmpvar_22;
  highp vec4 o_23;
  highp vec4 tmpvar_24;
  tmpvar_24 = (tmpvar_7 * 0.5);
  highp vec2 tmpvar_25;
  tmpvar_25.x = tmpvar_24.x;
  tmpvar_25.y = (tmpvar_24.y * _ProjectionParams.x);
  o_23.xy = (tmpvar_25 + tmpvar_24.w);
  o_23.zw = tmpvar_7.zw;
  tmpvar_6.xyz = (((_Object2World * _glesVertex).xyz - unity_ShadowFadeCenterAndType.xyz) * unity_ShadowFadeCenterAndType.w);
  tmpvar_6.w = (-((gl_ModelViewMatrix * _glesVertex).z) * (1.0 - unity_ShadowFadeCenterAndType.w));
  gl_Position = tmpvar_7;
  xlv_TEXCOORD0 = ((_glesMultiTexCoord0.xy * _MainTex_ST.xy) + _MainTex_ST.zw);
  xlv_TEXCOORD1 = o_23;
  xlv_TEXCOORD2 = tmpvar_3;
  xlv_TEXCOORD3 = tmpvar_4;
  xlv_TEXCOORD4 = tmpvar_5;
  xlv_TEXCOORD5 = ((_glesMultiTexCoord1.xy * unity_LightmapST.xy) + unity_LightmapST.zw);
  xlv_TEXCOORD6 = tmpvar_6;
}



#endif
#ifdef FRAGMENT

varying highp vec4 xlv_TEXCOORD6;
varying highp vec2 xlv_TEXCOORD5;
varying lowp vec4 xlv_TEXCOORD4;
varying lowp vec4 xlv_TEXCOORD3;
varying lowp vec4 xlv_TEXCOORD2;
varying highp vec4 xlv_TEXCOORD1;
varying highp vec2 xlv_TEXCOORD0;
uniform highp vec4 unity_LightmapFade;
uniform sampler2D unity_LightmapInd;
uniform sampler2D unity_Lightmap;
uniform sampler2D _LightBuffer;
uniform mediump float _Gloss;
uniform mediump float _ReflectPower;
uniform lowp vec4 _ReflectColor;
uniform lowp vec4 _Color;
uniform samplerCube _Cube;
uniform sampler2D _SpecMap;
uniform sampler2D _BumpMap;
uniform sampler2D _MainTex;
uniform lowp vec4 _SpecColor;
void main ()
{
  lowp vec4 tmpvar_1;
  mediump vec4 c_2;
  mediump vec3 lmIndirect_3;
  mediump vec3 lmFull_4;
  mediump vec4 light_5;
  highp vec3 tmpvar_6;
  mediump vec3 tmpvar_7;
  mediump vec3 tmpvar_8;
  mediump vec3 tmpvar_9;
  lowp vec3 tmpvar_10;
  tmpvar_10.x = xlv_TEXCOORD2.w;
  tmpvar_10.y = xlv_TEXCOORD3.w;
  tmpvar_10.z = xlv_TEXCOORD4.w;
  tmpvar_6 = tmpvar_10;
  lowp vec3 tmpvar_11;
  tmpvar_11 = xlv_TEXCOORD2.xyz;
  tmpvar_7 = tmpvar_11;
  lowp vec3 tmpvar_12;
  tmpvar_12 = xlv_TEXCOORD3.xyz;
  tmpvar_8 = tmpvar_12;
  lowp vec3 tmpvar_13;
  tmpvar_13 = xlv_TEXCOORD4.xyz;
  tmpvar_9 = tmpvar_13;
  mediump vec3 tmpvar_14;
  mediump vec3 tmpvar_15;
  mediump float tmpvar_16;
  mediump vec4 spec_17;
  lowp vec4 tmpvar_18;
  tmpvar_18 = texture2D (_MainTex, xlv_TEXCOORD0);
  lowp vec3 tmpvar_19;
  tmpvar_19 = (tmpvar_18.xyz * _Color.xyz);
  tmpvar_14 = tmpvar_19;
  lowp vec4 tmpvar_20;
  tmpvar_20 = texture2D (_SpecMap, xlv_TEXCOORD0);
  spec_17 = tmpvar_20;
  lowp vec3 normal_21;
  normal_21.xy = ((texture2D (_BumpMap, xlv_TEXCOORD0).wy * 2.0) - 1.0);
  normal_21.z = sqrt(((1.0 - (normal_21.x * normal_21.x)) - (normal_21.y * normal_21.y)));
  tmpvar_15 = normal_21;
  mediump vec3 tmpvar_22;
  tmpvar_22.x = dot (tmpvar_7, tmpvar_15);
  tmpvar_22.y = dot (tmpvar_8, tmpvar_15);
  tmpvar_22.z = dot (tmpvar_9, tmpvar_15);
  highp vec3 tmpvar_23;
  tmpvar_23 = (tmpvar_6 - (2.0 * (dot (tmpvar_22, tmpvar_6) * tmpvar_22)));
  lowp vec4 tmpvar_24;
  tmpvar_24 = (textureCube (_Cube, tmpvar_23) * tmpvar_18.w);
  lowp float tmpvar_25;
  tmpvar_25 = (tmpvar_24.w * _ReflectColor.w);
  tmpvar_16 = tmpvar_25;
  lowp vec4 tmpvar_26;
  tmpvar_26 = texture2DProj (_LightBuffer, xlv_TEXCOORD1);
  light_5 = tmpvar_26;
  mediump vec4 tmpvar_27;
  tmpvar_27 = max (light_5, vec4(0.001, 0.001, 0.001, 0.001));
  light_5.w = tmpvar_27.w;
  lowp vec4 tmpvar_28;
  tmpvar_28 = texture2D (unity_Lightmap, xlv_TEXCOORD5);
  lowp vec3 tmpvar_29;
  tmpvar_29 = ((8.0 * tmpvar_28.w) * tmpvar_28.xyz);
  lmFull_4 = tmpvar_29;
  lowp vec4 tmpvar_30;
  tmpvar_30 = texture2D (unity_LightmapInd, xlv_TEXCOORD5);
  lowp vec3 tmpvar_31;
  tmpvar_31 = ((8.0 * tmpvar_30.w) * tmpvar_30.xyz);
  lmIndirect_3 = tmpvar_31;
  highp float tmpvar_32;
  tmpvar_32 = clamp (((sqrt(dot (xlv_TEXCOORD6, xlv_TEXCOORD6)) * unity_LightmapFade.z) + unity_LightmapFade.w), 0.0, 1.0);
  light_5.xyz = (tmpvar_27.xyz + mix (lmIndirect_3, lmFull_4, vec3(tmpvar_32)));
  mediump vec4 c_33;
  mediump vec3 tmpvar_34;
  tmpvar_34 = (tmpvar_27.w * (spec_17.xyz * _Gloss));
  c_33.xyz = ((tmpvar_14 * light_5.xyz) + (light_5.xyz * tmpvar_34));
  c_33.w = (tmpvar_16 + (tmpvar_34 * _SpecColor.w)).x;
  c_2.w = c_33.w;
  c_2.xyz = (c_33.xyz + ((tmpvar_24.xyz * _ReflectColor.xyz) * _ReflectPower));
  tmpvar_1 = c_2;
  gl_FragData[0] = tmpvar_1;
}



#endif"
}

}
Program "fp" {
// Fragment combos: 4
//   opengl - ALU: 31 to 46, TEX: 5 to 7
//   d3d9 - ALU: 26 to 39, TEX: 5 to 7
//   d3d11 - ALU: 14 to 20, TEX: 5 to 7, FLOW: 1 to 1
SubProgram "opengl " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
Vector 0 [_SpecColor]
Vector 1 [_Color]
Vector 2 [_ReflectColor]
Float 3 [_ReflectPower]
Float 4 [_Gloss]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_Cube] CUBE
SetTexture 4 [_LightBuffer] 2D
"3.0-!!ARBfp1.0
OPTION ARB_precision_hint_fastest;
# 35 ALU, 5 TEX
PARAM c[6] = { program.local[0..4],
		{ 2, 1 } };
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
TEX R3.xyz, fragment.texcoord[0], texture[1], 2D;
DP3 R0.x, fragment.texcoord[2], R1;
DP3 R0.y, R1, fragment.texcoord[3];
DP3 R0.z, R1, fragment.texcoord[4];
MOV R1.x, fragment.texcoord[2].w;
MOV R1.z, fragment.texcoord[4].w;
MOV R1.y, fragment.texcoord[3].w;
DP3 R0.w, R0, R1;
MUL R0.xyz, R0, R0.w;
MAD R1.xyz, -R0, c[5].x, R1;
TEX R0, fragment.texcoord[0], texture[0], 2D;
TEX R1, R1, texture[3], CUBE;
MUL R2, R1, R0.w;
TXP R1, fragment.texcoord[1], texture[4], 2D;
LG2 R0.w, R1.w;
MUL R3.xyz, R3, c[4].x;
MUL R3.xyz, -R0.w, R3;
MUL R0.w, R3.x, c[0];
LG2 R1.x, R1.x;
LG2 R1.z, R1.z;
LG2 R1.y, R1.y;
ADD R1.xyz, -R1, fragment.texcoord[5];
MUL R3.yzw, R1.xxyz, R3.xxyz;
MUL R0.xyz, R0, c[1];
MAD R0.xyz, R0, R1, R3.yzww;
MUL R1.xyz, R2, c[2];
MAD result.color.xyz, R1, c[3].x, R0;
MAD result.color.w, R2, c[2], R0;
END
# 35 instructions, 4 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
Vector 0 [_SpecColor]
Vector 1 [_Color]
Vector 2 [_ReflectColor]
Float 3 [_ReflectPower]
Float 4 [_Gloss]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_Cube] CUBE
SetTexture 4 [_LightBuffer] 2D
"ps_3_0
; 30 ALU, 5 TEX
dcl_2d s0
dcl_2d s1
dcl_2d s2
dcl_cube s3
dcl_2d s4
def c5, 2.00000000, -1.00000000, 1.00000000, 0
dcl_texcoord0 v0.xy
dcl_texcoord1 v1
dcl_texcoord2 v2
dcl_texcoord3 v3
dcl_texcoord4 v4
dcl_texcoord5 v5.xyz
texld r0.yw, v0, s2
mad_pp r1.xy, r0.wyzw, c5.x, c5.y
mul_pp r0.x, r1.y, r1.y
mad_pp r0.x, -r1, r1, -r0
add_pp r0.x, r0, c5.z
rsq_pp r0.x, r0.x
rcp_pp r1.z, r0.x
texld r3.xyz, v0, s1
dp3_pp r0.x, v2, r1
dp3_pp r0.y, r1, v3
dp3_pp r0.z, r1, v4
mov r1.x, v2.w
mov r1.z, v4.w
mov r1.y, v3.w
dp3 r0.w, r0, r1
mul r0.xyz, r0, r0.w
mad r1.xyz, -r0, c5.x, r1
texld r0, v0, s0
texld r1, r1, s3
mul_pp r2, r1, r0.w
texldp r1, v1, s4
log_pp r0.w, r1.w
mul_pp r3.xyz, r3, c4.x
mul_pp r3.xyz, -r0.w, r3
mul_pp r0.w, r3.x, c0
log_pp r1.x, r1.x
log_pp r1.z, r1.z
log_pp r1.y, r1.y
add_pp r1.xyz, -r1, v5
mul_pp r3.yzw, r1.xxyz, r3.xxyz
mul_pp r0.xyz, r0, c1
mad_pp r0.xyz, r0, r1, r3.yzww
mul_pp r1.xyz, r2, c2
mad_pp oC0.xyz, r1, c3.x, r0
mad_pp oC0.w, r2, c2, r0
"
}

SubProgram "xbox360 " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
Vector 1 [_Color]
Float 4 [_Gloss]
Vector 2 [_ReflectColor]
Float 3 [_ReflectPower]
Vector 0 [_SpecColor]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_BumpMap] 2D
SetTexture 2 [_SpecMap] 2D
SetTexture 3 [_Cube] CUBE
SetTexture 4 [_LightBuffer] 2D
// Shader Timing Estimate, in Cycles/64 pixel vector:
// ALU: 36.00 (27 instructions), vertex: 0, texture: 20,
//   sequencer: 14, interpolator: 24;    8 GPRs, 24 threads,
// Performance (if enough threads): ~36 cycles per vector
// * Texture cycle estimates are assuming an 8bit/component texture with no
//     aniso or trilinear filtering.

"ps_360
backbbaaaaaaaccmaaaaabpmaaaaaaaaaaaaaaceaaaaabmmaaaaabpeaaaaaaaa
aaaaaaaaaaaaabkeaaaaaabmaaaaabjfppppadaaaaaaaaakaaaaaabmaaaaaaaa
aaaaabioaaaaaaoeaaadaaabaaabaaaaaaaaaapaaaaaaaaaaaaaabaaaaacaaab
aaabaaaaaaaaabaiaaaaaaaaaaaaabbiaaadaaadaaabaaaaaaaaabcaaaaaaaaa
aaaaabdaaaacaaaeaaabaaaaaaaaabdiaaaaaaaaaaaaabeiaaadaaaeaaabaaaa
aaaaaapaaaaaaaaaaaaaabffaaadaaaaaaabaaaaaaaaaapaaaaaaaaaaaaaabfo
aaacaaacaaabaaaaaaaaabaiaaaaaaaaaaaaabgmaaacaaadaaabaaaaaaaaabdi
aaaaaaaaaaaaabhkaaacaaaaaaabaaaaaaaaabaiaaaaaaaaaaaaabifaaadaaac
aaabaaaaaaaaaapaaaaaaaaafpechfgnhaengbhaaaklklklaaaeaaamaaabaaab
aaabaaaaaaaaaaaafpedgpgmgphcaaklaaabaaadaaabaaaeaaabaaaaaaaaaaaa
fpedhfgcgfaaklklaaaeaaaoaaabaaabaaabaaaaaaaaaaaafpehgmgphdhdaakl
aaaaaaadaaabaaabaaabaaaaaaaaaaaafpemgjghgiheechfgggggfhcaafpengb
gjgofegfhiaafpfcgfgggmgfgdheedgpgmgphcaafpfcgfgggmgfgdhefagphhgf
hcaafpfdhagfgdedgpgmgphcaafpfdhagfgdengbhaaahahdfpddfpdaaadccoda
codcdadddfddcodaaaklklklaaaaaaaaaaaaaaabaaaaaaaaaaaaaaaaaaaaaabe
abpmaabaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaeaaaaaablmbaaaahaa
aaaaaaaeaaaaaaaaaaaafemgaadpaadpaaaaaaabaaaadafaaaaapbfbaaaapcfc
aaaapdfdaaaapefeaaaahfffaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaadpiaaaaa
eaaaaaaalpiaaaaadpmaaaaaaaajgaaegaakbcaabcaaaaaaaaaagabaeabgbcaa
bcaaaaffaaaaaaaagabkmeaabcaaaaaaaaaaeacaaaaaccaaaaaaaaaababaaaab
bpbppghpaaaaeaaamiadaaahaamhlbmgilaappppmiaeaaaaaegngngmnbahahpp
kaeaahaaaaaaaamgocaaaaiamiabaaagaaloloaapaahacaamiaeaaagaaloloaa
paahadaamiacaaagaaloloaapaahaeaamiaeaaaaaalbblaaobagaeaamiaeaaaa
aamgblmgolagadaamiaeaaaaaagmblmgolagacaaaaeaaaaaaaaaaamgocaaaaaa
miahaaagaamgmaaaobaaagaaemecaaadaemgblbloaagadabbeamaaaaabmgkmlb
obaaabagaeebadadaegmblbloaagacaemiapaaabaakgmnaapcadadaaembeacac
aablblmgocababibmiadaaacaagngmblmlabacppjadigaebbpbppgiiaaaamaaa
baaicaabbpbppgiiaaaaeaaalieibaabbpbppemiaaaaeaaabaciaaabbpbppoii
aaaaeaaaeabhadaeaamagmgmkbaaaeibeacpadaaaaaabllbobagacibeaeiadad
aablblmgkbaaacibeaehadabaclemgblobaeadibkmioacaaaapmpmaaibaaacaa
kibhacaeaaleleebibacabadkichacadaemamaecmaadafadkiehacabaamaleed
mbabadadmiahaaadaalemaleolaeadabmiapiaaaaaaaaaaaoaadacaaaaaaaaaa
aaaaaaaaaaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
Vector 0 [_SpecColor]
Vector 1 [_Color]
Vector 2 [_ReflectColor]
Float 3 [_ReflectPower]
Float 4 [_Gloss]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_Cube] CUBE
SetTexture 4 [_LightBuffer] 2D
"sce_fp_rsx // 51 instructions using 4 registers
[Configuration]
24
ffffffff000fc020003fffc1000000000000840004000000
[Offsets]
5
_SpecColor 1 0
00000140
_Color 1 0
00000090
_ReflectColor 2 0
000002e0000000c0
_ReflectPower 1 0
00000320
_Gloss 1 0
000001a0
[Microcode]
816
94001704c8011c9dc8000001c8003fe1068c0440ce001c9d00020000aa020000
000040000000bf800000000000000000be001808c8011c9dc8000001c8003fe1
08821d40fe001c9dc8000001c800000102840240ab181c9cab180000c8000001
9e041700c8011c9dc8000001c8003fe11082044001181c9e0118000001080002
0e880240c8081c9dc8020001c800000100000000000000000000000000000000
de860140c8011c9dc8000001c8003fe1028a0140fe021c9dc8000001c8000001
00000000000000000000000000000000fe840140c8011c9dc8000001c8003fe1
10820340c9041c9d00020000c800000100003f80000000000000000000000000
088c3b40ff043c9dff040001c800000102860540c90c1c9dc9180001c8000001
04860540c9181c9dc9080001c8000001048a0140fe021c9dc8000001c8000001
000000000000000000000000000000001e8e0141c8011c9dc8000001c8003fe1
08860540c9181c9dc91c0001c8000001088e1d40c8001c9dc8000001c8000001
8e061702c8011c9dc8000001c8003fe10e8c0240c80c1c9d00020000c8000001
00000000000000000000000000000000028e1d40aa001c9cc8000001c8000001
0e84024055041c9fc9180001c800000110060100c80c1c9dc8000001c8000001
088c0140ff1c1c9dc8000001c8000001028c0140ff0c1c9dc8000001c8000001
048c0140ff081c9dc8000001c800000108000100c8001c9dc8000001c8000001
048e1d4054001c9dc8000001c800000108000500c90c1c9dc9181001c8000001
2e800301c8011c9da51c0003c8003fe11c84024021001c9d21080001c8000001
08000100c8001c9dc8000001c80000010e060400c90c1c9f54000001c9180001
1e061706c80c1c9dc8000001c80000011e860240c80c1c9dfe080001c8000001
1088014001081c9cc8000001c80000010e880100c9101c9dc8000001c8000001
0e800440c9101c9dc9000001f30800010e820240c90c1c9dc8020001c8000001
0000000000000000000000000000000008880140ff0c1c9dc8000001c8000001
108038405d101c9dc9140001c80000010e810440c9041c9d00020000c9000001
00000000000000000000000000000000
"
}

SubProgram "d3d11 " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
ConstBuffer "$Globals" 128 // 92 used size, 10 vars
Vector 32 [_SpecColor] 4
Vector 48 [_Color] 4
Vector 64 [_ReflectColor] 4
Float 80 [_ReflectPower]
Float 88 [_Gloss]
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 0
SetTexture 1 [_SpecMap] 2D 2
SetTexture 2 [_BumpMap] 2D 1
SetTexture 3 [_Cube] CUBE 3
SetTexture 4 [_LightBuffer] 2D 4
// 31 instructions, 4 temp regs, 0 temp arrays:
// ALU 15 float, 0 int, 0 uint
// TEX 5 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0
eefiecedhffmoeiaodaofehjbejofdnlgpjkjbfjabaaaaaapmafaaaaadaaaaaa
cmaaaaaapmaaaaaadaabaaaaejfdeheomiaaaaaaahaaaaaaaiaaaaaalaaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaalmaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaadadaaaalmaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaa
apalaaaalmaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaapapaaaalmaaaaaa
adaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapapaaaalmaaaaaaaeaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapapaaaalmaaaaaaafaaaaaaaaaaaaaaadaaaaaaagaaaaaa
ahahaaaafdfgfpfaepfdejfeejepeoaafeeffiedepepfceeaaklklklepfdeheo
cmaaaaaaabaaaaaaaiaaaaaacaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaaaaaaaaa
apaaaaaafdfgfpfegbhcghgfheaaklklfdeieefcmeaeaaaaeaaaaaaadbabaaaa
fjaaaaaeegiocaaaaaaaaaaaagaaaaaafkaaaaadaagabaaaaaaaaaaafkaaaaad
aagabaaaabaaaaaafkaaaaadaagabaaaacaaaaaafkaaaaadaagabaaaadaaaaaa
fkaaaaadaagabaaaaeaaaaaafibiaaaeaahabaaaaaaaaaaaffffaaaafibiaaae
aahabaaaabaaaaaaffffaaaafibiaaaeaahabaaaacaaaaaaffffaaaafidaaaae
aahabaaaadaaaaaaffffaaaafibiaaaeaahabaaaaeaaaaaaffffaaaagcbaaaad
dcbabaaaabaaaaaagcbaaaadlcbabaaaacaaaaaagcbaaaadpcbabaaaadaaaaaa
gcbaaaadpcbabaaaaeaaaaaagcbaaaadpcbabaaaafaaaaaagcbaaaadhcbabaaa
agaaaaaagfaaaaadpccabaaaaaaaaaaagiaaaaacaeaaaaaaefaaaaajpcaabaaa
aaaaaaaaegbabaaaabaaaaaaeghobaaaacaaaaaaaagabaaaabaaaaaadcaaaaap
dcaabaaaaaaaaaaahgapbaaaaaaaaaaaaceaaaaaaaaaaaeaaaaaaaeaaaaaaaaa
aaaaaaaaaceaaaaaaaaaialpaaaaialpaaaaaaaaaaaaaaaadcaaaaakicaabaaa
aaaaaaaaakaabaiaebaaaaaaaaaaaaaaakaabaaaaaaaaaaaabeaaaaaaaaaiadp
dcaaaaakicaabaaaaaaaaaaabkaabaiaebaaaaaaaaaaaaaabkaabaaaaaaaaaaa
dkaabaaaaaaaaaaaelaaaaafecaabaaaaaaaaaaadkaabaaaaaaaaaaabaaaaaah
bcaabaaaabaaaaaaegbcbaaaadaaaaaaegacbaaaaaaaaaaabaaaaaahccaabaaa
abaaaaaaegbcbaaaaeaaaaaaegacbaaaaaaaaaaabaaaaaahecaabaaaabaaaaaa
egbcbaaaafaaaaaaegacbaaaaaaaaaaadgaaaaafbcaabaaaaaaaaaaadkbabaaa
adaaaaaadgaaaaafccaabaaaaaaaaaaadkbabaaaaeaaaaaadgaaaaafecaabaaa
aaaaaaaadkbabaaaafaaaaaabaaaaaahicaabaaaaaaaaaaaegacbaaaaaaaaaaa
egacbaaaabaaaaaaaaaaaaahicaabaaaaaaaaaaadkaabaaaaaaaaaaadkaabaaa
aaaaaaaadcaaaaakhcaabaaaaaaaaaaaegacbaaaabaaaaaapgapbaiaebaaaaaa
aaaaaaaaegacbaaaaaaaaaaaefaaaaajpcaabaaaaaaaaaaaegacbaaaaaaaaaaa
eghobaaaadaaaaaaaagabaaaadaaaaaaefaaaaajpcaabaaaabaaaaaaegbabaaa
abaaaaaaeghobaaaaaaaaaaaaagabaaaaaaaaaaadiaaaaahpcaabaaaaaaaaaaa
egaobaaaaaaaaaaapgapbaaaabaaaaaadiaaaaaihcaabaaaabaaaaaaegacbaaa
abaaaaaaegiccaaaaaaaaaaaadaaaaaadiaaaaaipcaabaaaaaaaaaaaegaobaaa
aaaaaaaaegiocaaaaaaaaaaaaeaaaaaaefaaaaajpcaabaaaacaaaaaaegbabaaa
abaaaaaaeghobaaaabaaaaaaaagabaaaacaaaaaadiaaaaaihcaabaaaacaaaaaa
egacbaaaacaaaaaakgikcaaaaaaaaaaaafaaaaaaaoaaaaahdcaabaaaadaaaaaa
egbabaaaacaaaaaapgbpbaaaacaaaaaaefaaaaajpcaabaaaadaaaaaaegaabaaa
adaaaaaaeghobaaaaeaaaaaaaagabaaaaeaaaaaacpaaaaafpcaabaaaadaaaaaa
egaobaaaadaaaaaadiaaaaaihcaabaaaacaaaaaaegacbaaaacaaaaaapgapbaia
ebaaaaaaadaaaaaaaaaaaaaihcaabaaaadaaaaaaegacbaiaebaaaaaaadaaaaaa
egbcbaaaagaaaaaadiaaaaahocaabaaaacaaaaaaagajbaaaacaaaaaaagajbaaa
adaaaaaadcaaaaakiccabaaaaaaaaaaaakaabaaaacaaaaaadkiacaaaaaaaaaaa
acaaaaaadkaabaaaaaaaaaaadcaaaaajhcaabaaaabaaaaaaegacbaaaabaaaaaa
egacbaaaadaaaaaajgahbaaaacaaaaaadcaaaaakhccabaaaaaaaaaaaegacbaaa
aaaaaaaaagiacaaaaaaaaaaaafaaaaaaegacbaaaabaaaaaadoaaaaab"
}

SubProgram "gles " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
"!!GLES"
}

SubProgram "glesdesktop " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
"!!GLES"
}

SubProgram "opengl " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
Vector 0 [_SpecColor]
Vector 1 [_Color]
Vector 2 [_ReflectColor]
Float 3 [_ReflectPower]
Float 4 [_Gloss]
Vector 5 [unity_LightmapFade]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_Cube] CUBE
SetTexture 4 [_LightBuffer] 2D
SetTexture 5 [unity_Lightmap] 2D
SetTexture 6 [unity_LightmapInd] 2D
"3.0-!!ARBfp1.0
OPTION ARB_precision_hint_fastest;
# 46 ALU, 7 TEX
PARAM c[7] = { program.local[0..5],
		{ 2, 1, 8 } };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
TEX R0, fragment.texcoord[5], texture[5], 2D;
MUL R0.xyz, R0.w, R0;
TEX R1, fragment.texcoord[5], texture[6], 2D;
MUL R1.xyz, R1.w, R1;
MUL R1.xyz, R1, c[6].z;
DP4 R0.w, fragment.texcoord[6], fragment.texcoord[6];
TEX R3.xyz, fragment.texcoord[0], texture[1], 2D;
MAD R2.xyz, R0, c[6].z, -R1;
RSQ R0.w, R0.w;
RCP R0.x, R0.w;
MAD_SAT R0.x, R0, c[5].z, c[5].w;
MAD R2.xyz, R0.x, R2, R1;
TXP R1, fragment.texcoord[1], texture[4], 2D;
TEX R0.yw, fragment.texcoord[0], texture[2], 2D;
MAD R0.xy, R0.wyzw, c[6].x, -c[6].y;
MUL R0.z, R0.y, R0.y;
MAD R0.z, -R0.x, R0.x, -R0;
ADD R0.z, R0, c[6].y;
RSQ R0.z, R0.z;
RCP R0.z, R0.z;
MUL R3.xyz, R3, c[4].x;
LG2 R1.w, R1.w;
MUL R3.xyw, -R1.w, R3.yzzx;
LG2 R1.x, R1.x;
LG2 R1.y, R1.y;
LG2 R1.z, R1.z;
ADD R2.xyz, -R1, R2;
DP3 R1.x, fragment.texcoord[2], R0;
DP3 R1.y, R0, fragment.texcoord[3];
DP3 R1.z, R0, fragment.texcoord[4];
MOV R0.x, fragment.texcoord[2].w;
MOV R0.z, fragment.texcoord[4].w;
MOV R0.y, fragment.texcoord[3].w;
DP3 R0.w, R1, R0;
MUL R1.xyz, R1, R0.w;
MAD R0.xyz, -R1, c[6].x, R0;
TEX R1, fragment.texcoord[0], texture[0], 2D;
TEX R0, R0, texture[3], CUBE;
MUL R0, R0, R1.w;
MUL R1.w, R3, c[0];
MUL R3.xyz, R2, R3.wxyw;
MUL R1.xyz, R1, c[1];
MAD R1.xyz, R1, R2, R3;
MUL R0.xyz, R0, c[2];
MAD result.color.xyz, R0, c[3].x, R1;
MAD result.color.w, R0, c[2], R1;
END
# 46 instructions, 4 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
Vector 0 [_SpecColor]
Vector 1 [_Color]
Vector 2 [_ReflectColor]
Float 3 [_ReflectPower]
Float 4 [_Gloss]
Vector 5 [unity_LightmapFade]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_Cube] CUBE
SetTexture 4 [_LightBuffer] 2D
SetTexture 5 [unity_Lightmap] 2D
SetTexture 6 [unity_LightmapInd] 2D
"ps_3_0
; 39 ALU, 7 TEX
dcl_2d s0
dcl_2d s1
dcl_2d s2
dcl_cube s3
dcl_2d s4
dcl_2d s5
dcl_2d s6
def c6, 2.00000000, -1.00000000, 1.00000000, 8.00000000
dcl_texcoord0 v0.xy
dcl_texcoord1 v1
dcl_texcoord2 v2
dcl_texcoord3 v3
dcl_texcoord4 v4
dcl_texcoord5 v5.xy
dcl_texcoord6 v6
texld r0, v5, s5
mul_pp r0.xyz, r0.w, r0
texld r1, v5, s6
mul_pp r1.xyz, r1.w, r1
mul_pp r1.xyz, r1, c6.w
dp4 r0.w, v6, v6
texld r3.xyz, v0, s1
mad_pp r2.xyz, r0, c6.w, -r1
rsq r0.w, r0.w
rcp r0.x, r0.w
mad_sat r0.x, r0, c5.z, c5.w
mad_pp r2.xyz, r0.x, r2, r1
texldp r1, v1, s4
texld r0.yw, v0, s2
mad_pp r0.xy, r0.wyzw, c6.x, c6.y
mul_pp r0.z, r0.y, r0.y
mad_pp r0.z, -r0.x, r0.x, -r0
add_pp r0.z, r0, c6
rsq_pp r0.z, r0.z
rcp_pp r0.z, r0.z
mul_pp r3.xyz, r3, c4.x
log_pp r1.w, r1.w
mul_pp r3.xyw, -r1.w, r3.yzzx
log_pp r1.x, r1.x
log_pp r1.y, r1.y
log_pp r1.z, r1.z
add_pp r2.xyz, -r1, r2
dp3_pp r1.x, v2, r0
dp3_pp r1.y, r0, v3
dp3_pp r1.z, r0, v4
mov r0.x, v2.w
mov r0.z, v4.w
mov r0.y, v3.w
dp3 r0.w, r1, r0
mul r1.xyz, r1, r0.w
mad r0.xyz, -r1, c6.x, r0
texld r1, v0, s0
texld r0, r0, s3
mul_pp r0, r0, r1.w
mul_pp r1.w, r3, c0
mul_pp r3.xyz, r2, r3.wxyw
mul_pp r1.xyz, r1, c1
mad_pp r1.xyz, r1, r2, r3
mul_pp r0.xyz, r0, c2
mad_pp oC0.xyz, r0, c3.x, r1
mad_pp oC0.w, r0, c2, r1
"
}

SubProgram "xbox360 " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
Vector 1 [_Color]
Float 4 [_Gloss]
Vector 2 [_ReflectColor]
Float 3 [_ReflectPower]
Vector 0 [_SpecColor]
Vector 5 [unity_LightmapFade]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_BumpMap] 2D
SetTexture 2 [_SpecMap] 2D
SetTexture 3 [_Cube] CUBE
SetTexture 4 [_LightBuffer] 2D
SetTexture 5 [unity_Lightmap] 2D
SetTexture 6 [unity_LightmapInd] 2D
// Shader Timing Estimate, in Cycles/64 pixel vector:
// ALU: 42.67 (32 instructions), vertex: 0, texture: 28,
//   sequencer: 16, interpolator: 28;    9 GPRs, 21 threads,
// Performance (if enough threads): ~42 cycles per vector
// * Texture cycle estimates are assuming an 8bit/component texture with no
//     aniso or trilinear filtering.

"ps_360
backbbaaaaaaackaaaaaacfaaaaaaaaaaaaaaaceaaaaacdmaaaaacgeaaaaaaaa
aaaaaaaaaaaaacbeaaaaaabmaaaaacafppppadaaaaaaaaanaaaaaabmaaaaaaaa
aaaaabpoaaaaabcaaaadaaabaaabaaaaaaaaabcmaaaaaaaaaaaaabdmaaacaaab
aaabaaaaaaaaabeeaaaaaaaaaaaaabfeaaadaaadaaabaaaaaaaaabfmaaaaaaaa
aaaaabgmaaacaaaeaaabaaaaaaaaabheaaaaaaaaaaaaabieaaadaaaeaaabaaaa
aaaaabcmaaaaaaaaaaaaabjbaaadaaaaaaabaaaaaaaaabcmaaaaaaaaaaaaabjk
aaacaaacaaabaaaaaaaaabeeaaaaaaaaaaaaabkiaaacaaadaaabaaaaaaaaabhe
aaaaaaaaaaaaablgaaacaaaaaaabaaaaaaaaabeeaaaaaaaaaaaaabmbaaadaaac
aaabaaaaaaaaabcmaaaaaaaaaaaaabmkaaadaaafaaabaaaaaaaaabcmaaaaaaaa
aaaaabnjaaacaaafaaabaaaaaaaaabeeaaaaaaaaaaaaabomaaadaaagaaabaaaa
aaaaabcmaaaaaaaafpechfgnhaengbhaaaklklklaaaeaaamaaabaaabaaabaaaa
aaaaaaaafpedgpgmgphcaaklaaabaaadaaabaaaeaaabaaaaaaaaaaaafpedhfgc
gfaaklklaaaeaaaoaaabaaabaaabaaaaaaaaaaaafpehgmgphdhdaaklaaaaaaad
aaabaaabaaabaaaaaaaaaaaafpemgjghgiheechfgggggfhcaafpengbgjgofegf
hiaafpfcgfgggmgfgdheedgpgmgphcaafpfcgfgggmgfgdhefagphhgfhcaafpfd
hagfgdedgpgmgphcaafpfdhagfgdengbhaaahfgogjhehjfpemgjghgihegngbha
aahfgogjhehjfpemgjghgihegngbhaeggbgegfaahfgogjhehjfpemgjghgihegn
gbhaejgogeaahahdfpddfpdaaadccodacodcdadddfddcodaaaklklklaaaaaaaa
aaaaaaabaaaaaaaaaaaaaaaaaaaaaabeabpmaabaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaeaaaaaacbabaaaaiaaaaaaaaaeaaaaaaaaaaaagaohaahpaahp
aaaaaaabaaaadafaaaaapbfbaaaapcfcaaaapdfdaaaapefeaaaadfffaaaapgfg
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
ebaaaaaaeaaaaaaalpiaaaaadpmaaaaadpiaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aajfgaaegaakbcaabcaaaaaaaaaagabagabgbcaabcaaaaaaabfefabmaaaabcaa
meaaaaaaaaaagacbeachbcaaccaaaaaababaiaabbpbpppnjaaaaeaaabagahakb
bpbppgiiaaaaeaaabafafakbbpbppeedaaaaeaaabeeiaaaaaakhkhgmopagagaf
kacbagagaablgmblkbahpoiamiadaaaiaagnlbmgilaipopomiaiaaaaaegngngm
nbaiaippmjabaaafaalbmgblilagafafkieoaaahaagmpmecmbagahpomiaoaaaf
abmgababolaaafahkaeoaiafaaabgmblobafafiamiacaaagaaloloaapaaiaeaa
miabaaagaaloloaapaaiacaabeaeaaagaalologmpaaiadagambbahafaamgblbl
obagadacmiapaaafaaaaaaaaoaahafaamiaeaaaaaalbblgmolagaeafaaeaaaaa
aaaaaamgocaaaaaamiahaaagaamgmaaaobaaagaaemecaaadaemgblbloaagadab
beamaaaaabmgkmlbobaaabagaeebadadaegmblbloaagacaemiapaaabaakgmnaa
pcadadaaembeacacaablblmgocababibmiadaaacaagngmblmlabacpojadigaeb
bpbppgiiaaaamaaabaaicaabbpbppgiiaaaaeaaalieibaabbpbppcnaaaaaeaaa
baciaaabbpbppoiiaaaaeaaaeabhadaeaamagmgmkbaaaeibeacpadaaaaaabllb
obagacibeaeiadadaablblmgkbaaacibeaehadabaclemgblobaeadibkmioacaa
aapmpmaaibaaacaakibhacaeaaleleebibacabadkichacadacmjmaecmaafadad
kiehacabaaleleedmbabadadmiahaaadaalelemaolaeadabmiapiaaaaaaaaaaa
oaadacaaaaaaaaaaaaaaaaaaaaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
Vector 0 [_SpecColor]
Vector 1 [_Color]
Vector 2 [_ReflectColor]
Float 3 [_ReflectPower]
Float 4 [_Gloss]
Vector 5 [unity_LightmapFade]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_Cube] CUBE
SetTexture 4 [_LightBuffer] 2D
SetTexture 5 [unity_Lightmap] 2D
SetTexture 6 [unity_LightmapInd] 2D
"sce_fp_rsx // 62 instructions using 5 registers
[Configuration]
24
ffffffff001fc020007fffa1000000000000840005000000
[Offsets]
6
_SpecColor 1 0
00000270
_Color 1 0
00000330
_ReflectColor 2 0
000003b0000002d0
_ReflectPower 1 0
000003d0
_Gloss 1 0
00000180
unity_LightmapFade 2 0
0000024000000210
[Microcode]
992
94001704c8011c9dc8000001c8003fe10c920440be001c9c00020000aa020000
000040000000bf8000000000000000005e000101c8011c9dc8000001c8003fe1
08060600c8001c9dc8000001c80000013e04170dc8011c9dc8000001c8003fe1
0e8a0240fe081c9dc8083001c80000011080024055241c9d55240001c8000001
de880140c8011c9dc8000001c8003fe102800440ab241c9eab240000ff000003
be021808c8011c9dc8000001c8003fe102800340c9001c9d00020000c8000001
00003f8000000000000000000000000010923b4001003c9cc9000001c8000001
06040100c8081c9dc8000001c800000102880540c9101c9df3240001c8000001
02921d40c8041c9dc8000001c8000001fe8c0140c8011c9dc8000001c8003fe1
04880540f3241c9dc9180001c80000011e900141c8011c9dc8000001c8003fe1
08880540f3241c9dc9200001c800000104921d40aa041c9cc8000001c8000001
8e001702c8011c9dc8000001c8003fe10e840240c8001c9d00020000c8000001
0000000000000000000000000000000010841d40fe041c9dc8000001c8000001
3e00170bc8011c9dc8000001c8003fe1108a0140c8001c9dc8003001c8000001
10001b00540c1c9dc8000001c800000108921d4054041c9dc8000001c8000001
1e7e7d00c8001c9dc8000001c80000010e840240ff081c9fc9080001c8000001
10003a0054021c9dfe000001c800000100000000000000000000000000000000
0e000400ff141c9dc8000001c914000310008300c8001c9dc8020001c8000001
000000000000000000000000000000000e800400fe001c9dc8000001c9140001
10820140c8021c9dc8000001c800000100000000000000000000000000000000
0e860340c9241c9fc9000001c8000001088a0140ff201c9dc8000001c8000001
028a0140ff101c9dc8000001c8000001048a0140ff181c9dc8000001c8000001
08820140fe021c9dc8000001c800000100000000000000000000000000000000
02000500c9101c9dc9141001c80000010e000400c9101c9f00000000c9140001
9e061700c8011c9dc8000001c8003fe11c840240210c1c9d21080001c8000001
0e880240c80c1c9dc8020001c800000100000000000000000000000000000000
0e860440c9101c9dc90c0001f30800011084014001081c9cc8000001c8000001
1e041706c8001c9dc8000001c80000011e800240c8081c9dfe0c0001c8000001
08840140ff001c9dc8000001c8000001108038405d081c9d5d040001c8000001
0e800240c9001c9dc8020001c800000100000000000000000000000000000000
0e810440c9001c9d00020000c90c000100000000000000000000000000000000
"
}

SubProgram "d3d11 " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
ConstBuffer "$Globals" 160 // 144 used size, 12 vars
Vector 32 [_SpecColor] 4
Vector 48 [_Color] 4
Vector 64 [_ReflectColor] 4
Float 80 [_ReflectPower]
Float 88 [_Gloss]
Vector 128 [unity_LightmapFade] 4
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 0
SetTexture 1 [_SpecMap] 2D 2
SetTexture 2 [_BumpMap] 2D 1
SetTexture 3 [_Cube] CUBE 3
SetTexture 4 [_LightBuffer] 2D 4
SetTexture 5 [unity_Lightmap] 2D 5
SetTexture 6 [unity_LightmapInd] 2D 6
// 41 instructions, 4 temp regs, 0 temp arrays:
// ALU 20 float, 0 int, 0 uint
// TEX 7 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0
eefieceddendjnknlcljggjecaeopagoanpljobcabaaaaaajmahaaaaadaaaaaa
cmaaaaaabeabaaaaeiabaaaaejfdeheooaaaaaaaaiaaaaaaaiaaaaaamiaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaneaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaadadaaaaneaaaaaaafaaaaaaaaaaaaaaadaaaaaaabaaaaaa
amamaaaaneaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaaapalaaaaneaaaaaa
acaaaaaaaaaaaaaaadaaaaaaadaaaaaaapapaaaaneaaaaaaadaaaaaaaaaaaaaa
adaaaaaaaeaaaaaaapapaaaaneaaaaaaaeaaaaaaaaaaaaaaadaaaaaaafaaaaaa
apapaaaaneaaaaaaagaaaaaaaaaaaaaaadaaaaaaagaaaaaaapapaaaafdfgfpfa
epfdejfeejepeoaafeeffiedepepfceeaaklklklepfdeheocmaaaaaaabaaaaaa
aiaaaaaacaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaafdfgfpfe
gbhcghgfheaaklklfdeieefcemagaaaaeaaaaaaajdabaaaafjaaaaaeegiocaaa
aaaaaaaaajaaaaaafkaaaaadaagabaaaaaaaaaaafkaaaaadaagabaaaabaaaaaa
fkaaaaadaagabaaaacaaaaaafkaaaaadaagabaaaadaaaaaafkaaaaadaagabaaa
aeaaaaaafkaaaaadaagabaaaafaaaaaafkaaaaadaagabaaaagaaaaaafibiaaae
aahabaaaaaaaaaaaffffaaaafibiaaaeaahabaaaabaaaaaaffffaaaafibiaaae
aahabaaaacaaaaaaffffaaaafidaaaaeaahabaaaadaaaaaaffffaaaafibiaaae
aahabaaaaeaaaaaaffffaaaafibiaaaeaahabaaaafaaaaaaffffaaaafibiaaae
aahabaaaagaaaaaaffffaaaagcbaaaaddcbabaaaabaaaaaagcbaaaadmcbabaaa
abaaaaaagcbaaaadlcbabaaaacaaaaaagcbaaaadpcbabaaaadaaaaaagcbaaaad
pcbabaaaaeaaaaaagcbaaaadpcbabaaaafaaaaaagcbaaaadpcbabaaaagaaaaaa
gfaaaaadpccabaaaaaaaaaaagiaaaaacaeaaaaaabbaaaaahbcaabaaaaaaaaaaa
egbobaaaagaaaaaaegbobaaaagaaaaaaelaaaaafbcaabaaaaaaaaaaaakaabaaa
aaaaaaaadccaaaalbcaabaaaaaaaaaaaakaabaaaaaaaaaaackiacaaaaaaaaaaa
aiaaaaaadkiacaaaaaaaaaaaaiaaaaaaefaaaaajpcaabaaaabaaaaaaogbkbaaa
abaaaaaaeghobaaaagaaaaaaaagabaaaagaaaaaadiaaaaahccaabaaaaaaaaaaa
dkaabaaaabaaaaaaabeaaaaaaaaaaaebdiaaaaahocaabaaaaaaaaaaaagajbaaa
abaaaaaafgafbaaaaaaaaaaaefaaaaajpcaabaaaabaaaaaaogbkbaaaabaaaaaa
eghobaaaafaaaaaaaagabaaaafaaaaaadiaaaaahicaabaaaabaaaaaadkaabaaa
abaaaaaaabeaaaaaaaaaaaebdcaaaaakhcaabaaaabaaaaaapgapbaaaabaaaaaa
egacbaaaabaaaaaajgahbaiaebaaaaaaaaaaaaaadcaaaaajhcaabaaaaaaaaaaa
agaabaaaaaaaaaaaegacbaaaabaaaaaajgahbaaaaaaaaaaaaoaaaaahdcaabaaa
abaaaaaaegbabaaaacaaaaaapgbpbaaaacaaaaaaefaaaaajpcaabaaaabaaaaaa
egaabaaaabaaaaaaeghobaaaaeaaaaaaaagabaaaaeaaaaaacpaaaaafpcaabaaa
abaaaaaaegaobaaaabaaaaaaaaaaaaaihcaabaaaaaaaaaaaegacbaaaaaaaaaaa
egacbaiaebaaaaaaabaaaaaaefaaaaajpcaabaaaacaaaaaaegbabaaaabaaaaaa
eghobaaaabaaaaaaaagabaaaacaaaaaadiaaaaaihcaabaaaabaaaaaaegacbaaa
acaaaaaakgikcaaaaaaaaaaaafaaaaaadiaaaaaihcaabaaaabaaaaaaegacbaaa
abaaaaaapgapbaiaebaaaaaaabaaaaaadiaaaaahocaabaaaabaaaaaaagajbaaa
aaaaaaaaagajbaaaabaaaaaaefaaaaajpcaabaaaacaaaaaaegbabaaaabaaaaaa
eghobaaaaaaaaaaaaagabaaaaaaaaaaadiaaaaaihcaabaaaacaaaaaaegacbaaa
acaaaaaaegiccaaaaaaaaaaaadaaaaaadcaaaaajhcaabaaaaaaaaaaaegacbaaa
acaaaaaaegacbaaaaaaaaaaajgahbaaaabaaaaaaefaaaaajpcaabaaaadaaaaaa
egbabaaaabaaaaaaeghobaaaacaaaaaaaagabaaaabaaaaaadcaaaaapdcaabaaa
acaaaaaahgapbaaaadaaaaaaaceaaaaaaaaaaaeaaaaaaaeaaaaaaaaaaaaaaaaa
aceaaaaaaaaaialpaaaaialpaaaaaaaaaaaaaaaadcaaaaakicaabaaaaaaaaaaa
akaabaiaebaaaaaaacaaaaaaakaabaaaacaaaaaaabeaaaaaaaaaiadpdcaaaaak
icaabaaaaaaaaaaabkaabaiaebaaaaaaacaaaaaabkaabaaaacaaaaaadkaabaaa
aaaaaaaaelaaaaafecaabaaaacaaaaaadkaabaaaaaaaaaaabaaaaaahbcaabaaa
adaaaaaaegbcbaaaadaaaaaaegacbaaaacaaaaaabaaaaaahccaabaaaadaaaaaa
egbcbaaaaeaaaaaaegacbaaaacaaaaaabaaaaaahecaabaaaadaaaaaaegbcbaaa
afaaaaaaegacbaaaacaaaaaadgaaaaafbcaabaaaacaaaaaadkbabaaaadaaaaaa
dgaaaaafccaabaaaacaaaaaadkbabaaaaeaaaaaadgaaaaafecaabaaaacaaaaaa
dkbabaaaafaaaaaabaaaaaahicaabaaaaaaaaaaaegacbaaaacaaaaaaegacbaaa
adaaaaaaaaaaaaahicaabaaaaaaaaaaadkaabaaaaaaaaaaadkaabaaaaaaaaaaa
dcaaaaakocaabaaaabaaaaaaagajbaaaadaaaaaapgapbaiaebaaaaaaaaaaaaaa
agajbaaaacaaaaaaefaaaaajpcaabaaaadaaaaaajgahbaaaabaaaaaaeghobaaa
adaaaaaaaagabaaaadaaaaaadiaaaaahpcaabaaaacaaaaaapgapbaaaacaaaaaa
egaobaaaadaaaaaadiaaaaaipcaabaaaacaaaaaaegaobaaaacaaaaaaegiocaaa
aaaaaaaaaeaaaaaadcaaaaakhccabaaaaaaaaaaaegacbaaaacaaaaaaagiacaaa
aaaaaaaaafaaaaaaegacbaaaaaaaaaaadcaaaaakiccabaaaaaaaaaaaakaabaaa
abaaaaaadkiacaaaaaaaaaaaacaaaaaadkaabaaaacaaaaaadoaaaaab"
}

SubProgram "gles " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
"!!GLES"
}

SubProgram "glesdesktop " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_OFF" }
"!!GLES"
}

SubProgram "opengl " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
Vector 0 [_SpecColor]
Vector 1 [_Color]
Vector 2 [_ReflectColor]
Float 3 [_ReflectPower]
Float 4 [_Gloss]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_Cube] CUBE
SetTexture 4 [_LightBuffer] 2D
"3.0-!!ARBfp1.0
OPTION ARB_precision_hint_fastest;
# 31 ALU, 5 TEX
PARAM c[6] = { program.local[0..4],
		{ 2, 1 } };
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
DP3 R0.x, fragment.texcoord[2], R1;
DP3 R0.y, R1, fragment.texcoord[3];
DP3 R0.z, R1, fragment.texcoord[4];
MOV R1.x, fragment.texcoord[2].w;
MOV R1.z, fragment.texcoord[4].w;
MOV R1.y, fragment.texcoord[3].w;
DP3 R0.w, R0, R1;
MUL R0.xyz, R0, R0.w;
MAD R0.xyz, -R0, c[5].x, R1;
TEX R2, R0, texture[3], CUBE;
TEX R0, fragment.texcoord[0], texture[0], 2D;
TEX R1.xyz, fragment.texcoord[0], texture[1], 2D;
MUL R3.xyz, R1, c[4].x;
TXP R1, fragment.texcoord[1], texture[4], 2D;
MUL R2, R2, R0.w;
MUL R3.xyz, R1.w, R3;
ADD R1.xyz, R1, fragment.texcoord[5];
MUL R0.w, R3.x, c[0];
MUL R3.yzw, R1.xxyz, R3.xxyz;
MUL R0.xyz, R0, c[1];
MAD R0.xyz, R0, R1, R3.yzww;
MUL R1.xyz, R2, c[2];
MAD result.color.xyz, R1, c[3].x, R0;
MAD result.color.w, R2, c[2], R0;
END
# 31 instructions, 4 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
Vector 0 [_SpecColor]
Vector 1 [_Color]
Vector 2 [_ReflectColor]
Float 3 [_ReflectPower]
Float 4 [_Gloss]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_Cube] CUBE
SetTexture 4 [_LightBuffer] 2D
"ps_3_0
; 26 ALU, 5 TEX
dcl_2d s0
dcl_2d s1
dcl_2d s2
dcl_cube s3
dcl_2d s4
def c5, 2.00000000, -1.00000000, 1.00000000, 0
dcl_texcoord0 v0.xy
dcl_texcoord1 v1
dcl_texcoord2 v2
dcl_texcoord3 v3
dcl_texcoord4 v4
dcl_texcoord5 v5.xyz
texld r0.yw, v0, s2
mad_pp r1.xy, r0.wyzw, c5.x, c5.y
mul_pp r0.x, r1.y, r1.y
mad_pp r0.x, -r1, r1, -r0
add_pp r0.x, r0, c5.z
rsq_pp r0.x, r0.x
rcp_pp r1.z, r0.x
dp3_pp r0.x, v2, r1
dp3_pp r0.y, r1, v3
dp3_pp r0.z, r1, v4
mov r1.x, v2.w
mov r1.z, v4.w
mov r1.y, v3.w
dp3 r0.w, r0, r1
mul r0.xyz, r0, r0.w
mad r0.xyz, -r0, c5.x, r1
texld r2, r0, s3
texld r0, v0, s0
texld r1.xyz, v0, s1
mul_pp r3.xyz, r1, c4.x
texldp r1, v1, s4
mul_pp r2, r2, r0.w
mul_pp r3.xyz, r1.w, r3
add_pp r1.xyz, r1, v5
mul_pp r0.w, r3.x, c0
mul_pp r3.yzw, r1.xxyz, r3.xxyz
mul_pp r0.xyz, r0, c1
mad_pp r0.xyz, r0, r1, r3.yzww
mul_pp r1.xyz, r2, c2
mad_pp oC0.xyz, r1, c3.x, r0
mad_pp oC0.w, r2, c2, r0
"
}

SubProgram "xbox360 " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
Vector 1 [_Color]
Float 4 [_Gloss]
Vector 2 [_ReflectColor]
Float 3 [_ReflectPower]
Vector 0 [_SpecColor]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_BumpMap] 2D
SetTexture 2 [_SpecMap] 2D
SetTexture 3 [_Cube] CUBE
SetTexture 4 [_LightBuffer] 2D
SetTexture 5 [_LightSpecBuffer] 2D
// Shader Timing Estimate, in Cycles/64 pixel vector:
// ALU: 34.67 (26 instructions), vertex: 0, texture: 24,
//   sequencer: 14, interpolator: 24;    8 GPRs, 24 threads,
// Performance (if enough threads): ~34 cycles per vector
// * Texture cycle estimates are assuming an 8bit/component texture with no
//     aniso or trilinear filtering.

"ps_360
backbbaaaaaaacfaaaaaabpmaaaaaaaaaaaaaaceaaaaabpaaaaaacbiaaaaaaaa
aaaaaaaaaaaaabmiaaaaaabmaaaaablkppppadaaaaaaaaalaaaaaabmaaaaaaaa
aaaaabldaaaaaapiaaadaaabaaabaaaaaaaaabaeaaaaaaaaaaaaabbeaaacaaab
aaabaaaaaaaaabbmaaaaaaaaaaaaabcmaaadaaadaaabaaaaaaaaabdeaaaaaaaa
aaaaabeeaaacaaaeaaabaaaaaaaaabemaaaaaaaaaaaaabfmaaadaaaeaaabaaaa
aaaaabaeaaaaaaaaaaaaabgjaaadaaafaaabaaaaaaaaabaeaaaaaaaaaaaaabhk
aaadaaaaaaabaaaaaaaaabaeaaaaaaaaaaaaabidaaacaaacaaabaaaaaaaaabbm
aaaaaaaaaaaaabjbaaacaaadaaabaaaaaaaaabemaaaaaaaaaaaaabjpaaacaaaa
aaabaaaaaaaaabbmaaaaaaaaaaaaabkkaaadaaacaaabaaaaaaaaabaeaaaaaaaa
fpechfgnhaengbhaaaklklklaaaeaaamaaabaaabaaabaaaaaaaaaaaafpedgpgm
gphcaaklaaabaaadaaabaaaeaaabaaaaaaaaaaaafpedhfgcgfaaklklaaaeaaao
aaabaaabaaabaaaaaaaaaaaafpehgmgphdhdaaklaaaaaaadaaabaaabaaabaaaa
aaaaaaaafpemgjghgiheechfgggggfhcaafpemgjghgihefdhagfgdechfgggggf
hcaafpengbgjgofegfhiaafpfcgfgggmgfgdheedgpgmgphcaafpfcgfgggmgfgd
hefagphhgfhcaafpfdhagfgdedgpgmgphcaafpfdhagfgdengbhaaahahdfpddfp
daaadccodacodcdadddfddcodaaaklklaaaaaaaaaaaaaaabaaaaaaaaaaaaaaaa
aaaaaabeabpmaabaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaeaaaaaablm
baaaahaaaaaaaaaeaaaaaaaaaaaafemgaadpaadpaaaaaaabaaaadafaaaaapbfb
aaaapcfcaaaapdfdaaaapefeaaaahfffaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
dpiaaaaaeaaaaaaalpiaaaaadpmaaaaaaaajgaaegaakbcaabcaaaaaaaaaagaba
fabgbcaabcaaabffaaaaaaaagablmeaabcaaaaaaaaaadacbaaaaccaaaaaaaaaa
babaaaabbpbppghpaaaaeaaamiadaaahaamhlbmgilaappppmiaeaaaaaegngngm
nbahahppkaeaahaaaaaaaamgocaaaaiamiabaaagaaloloaapaahacaamiaeaaag
aaloloaapaahadaamiacaaagaaloloaapaahaeaamiaeaaaaaalbblaaobagaeaa
miaeaaaaaamgblmgolagadaamiaeaaaaaagmblmgolagacaaaaeaaaaaaaaaaamg
ocaaaaaamiahaaagaamgmaaaobaaagaaemecaaadaemgblbloaagadabbeamaaaa
abmgkmlbobaaabagaeebadadaegmblbloaagacaemiapaaabaakgmnaapcadadaa
embiacacaablblmgocababibmiagaaacaagbgmblmlabacpplificaabbpbppppi
aaaaeaaaoedibaebbpbppgiiaaaamaaalieidaabbpbppoiiaaaaeaaabacicaab
bpbppeehaaaaeaaabaaiaaabbpbppgecaaaaeaaabecoaaacaaabgmlbkbacaeaa
kibhaeadaamamaebmaadafabkicpaeabaaaablmambabaaabkiehaeabaamamaic
ibabacabbeboaaaaaanbgmblobacacabkiihacabaamagmaaibabadackiihabac
aabfleabmbaaadaamiahaaacaalemaleolaeadacmiapiaaaaaaaaaaaoaacabaa
aaaaaaaaaaaaaaaaaaaaaaaa"
}

SubProgram "ps3 " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
Vector 0 [_SpecColor]
Vector 1 [_Color]
Vector 2 [_ReflectColor]
Float 3 [_ReflectPower]
Float 4 [_Gloss]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_Cube] CUBE
SetTexture 4 [_LightBuffer] 2D
"sce_fp_rsx // 44 instructions using 4 registers
[Configuration]
24
ffffffff000fc020003fffc1000000000000840004000000
[Offsets]
5
_SpecColor 1 0
000001e0
_Color 1 0
000001b0
_ReflectColor 2 0
0000028000000240
_ReflectPower 1 0
000002b0
_Gloss 1 0
00000100
[Microcode]
704
de840140c8011c9dc8000001c8003fe1088e0140ff081c9dc8000001c8000001
94001704c8011c9dc8000001c8003fe106800440ce001c9d00020000aa020000
000040000000bf800000000000000000fe820140c8011c9dc8000001c8003fe1
10840240ab001c9cab000000c80000011e880141c8011c9dc8000001c8003fe1
0880044001001c9e01000000ff0800038e061702c8011c9dc8000001c8003fe1
08800340c9001c9d00020000c800000100003f80000000000000000000000000
08803b40c9003c9d55000001c8000001028a0540c9081c9dc9000001c8000001
be021808c8011c9dc8000001c8003fe1108a0240c8041c9d00020000c8000001
00000000000000000000000000000000048a0540c9001c9dc9040001c8000001
088a0540c9001c9dc9100001c8000001108a0100c9141c9dc8000001c8000001
0e820240c80c1c9dff140001c80000012e800301c8011c9dc8040001c8003fe1
028e0140ff041c9dc8000001c80000010e880240c9001c9dc9040001c8000001
048e0140ff101c9dc8000001c80000019e021700c8011c9dc8000001c8003fe1
1c82024020041c9d20020001c800000100000000000000000000000000000000
0e800440f3041c9dc9000001c9100001048c0140fe021c9dc8000001c8000001
0000000000000000000000000000000004060500c9141c9da51c1001c8000001
0e040400c9141c9faa0c0000a51c00011e041706c8081c9dc8000001c8000001
1e840240c8081c9dfe040001c80000010e8e0240c9081c9dc8020001c8000001
000000000000000000000000000000001086014001041c9cc8000001c8000001
08860140ff081c9dc8000001c8000001028c0140fe021c9dc8000001c8000001
00000000000000000000000000000000108038405d0c1c9dc9180001c8000001
0e810440c91c1c9d00020000c900000100000000000000000000000000000000
"
}

SubProgram "d3d11 " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
ConstBuffer "$Globals" 128 // 92 used size, 10 vars
Vector 32 [_SpecColor] 4
Vector 48 [_Color] 4
Vector 64 [_ReflectColor] 4
Float 80 [_ReflectPower]
Float 88 [_Gloss]
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 0
SetTexture 1 [_SpecMap] 2D 2
SetTexture 2 [_BumpMap] 2D 1
SetTexture 3 [_Cube] CUBE 3
SetTexture 4 [_LightBuffer] 2D 4
// 30 instructions, 4 temp regs, 0 temp arrays:
// ALU 14 float, 0 int, 0 uint
// TEX 5 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0
eefiecedgfoddnbdfhifjebblkickpeioghfakdiabaaaaaaoaafaaaaadaaaaaa
cmaaaaaapmaaaaaadaabaaaaejfdeheomiaaaaaaahaaaaaaaiaaaaaalaaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaalmaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaadadaaaalmaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaa
apalaaaalmaaaaaaacaaaaaaaaaaaaaaadaaaaaaadaaaaaaapapaaaalmaaaaaa
adaaaaaaaaaaaaaaadaaaaaaaeaaaaaaapapaaaalmaaaaaaaeaaaaaaaaaaaaaa
adaaaaaaafaaaaaaapapaaaalmaaaaaaafaaaaaaaaaaaaaaadaaaaaaagaaaaaa
ahahaaaafdfgfpfaepfdejfeejepeoaafeeffiedepepfceeaaklklklepfdeheo
cmaaaaaaabaaaaaaaiaaaaaacaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaaaaaaaaa
apaaaaaafdfgfpfegbhcghgfheaaklklfdeieefckiaeaaaaeaaaaaaackabaaaa
fjaaaaaeegiocaaaaaaaaaaaagaaaaaafkaaaaadaagabaaaaaaaaaaafkaaaaad
aagabaaaabaaaaaafkaaaaadaagabaaaacaaaaaafkaaaaadaagabaaaadaaaaaa
fkaaaaadaagabaaaaeaaaaaafibiaaaeaahabaaaaaaaaaaaffffaaaafibiaaae
aahabaaaabaaaaaaffffaaaafibiaaaeaahabaaaacaaaaaaffffaaaafidaaaae
aahabaaaadaaaaaaffffaaaafibiaaaeaahabaaaaeaaaaaaffffaaaagcbaaaad
dcbabaaaabaaaaaagcbaaaadlcbabaaaacaaaaaagcbaaaadpcbabaaaadaaaaaa
gcbaaaadpcbabaaaaeaaaaaagcbaaaadpcbabaaaafaaaaaagcbaaaadhcbabaaa
agaaaaaagfaaaaadpccabaaaaaaaaaaagiaaaaacaeaaaaaaefaaaaajpcaabaaa
aaaaaaaaegbabaaaabaaaaaaeghobaaaacaaaaaaaagabaaaabaaaaaadcaaaaap
dcaabaaaaaaaaaaahgapbaaaaaaaaaaaaceaaaaaaaaaaaeaaaaaaaeaaaaaaaaa
aaaaaaaaaceaaaaaaaaaialpaaaaialpaaaaaaaaaaaaaaaadcaaaaakicaabaaa
aaaaaaaaakaabaiaebaaaaaaaaaaaaaaakaabaaaaaaaaaaaabeaaaaaaaaaiadp
dcaaaaakicaabaaaaaaaaaaabkaabaiaebaaaaaaaaaaaaaabkaabaaaaaaaaaaa
dkaabaaaaaaaaaaaelaaaaafecaabaaaaaaaaaaadkaabaaaaaaaaaaabaaaaaah
bcaabaaaabaaaaaaegbcbaaaadaaaaaaegacbaaaaaaaaaaabaaaaaahccaabaaa
abaaaaaaegbcbaaaaeaaaaaaegacbaaaaaaaaaaabaaaaaahecaabaaaabaaaaaa
egbcbaaaafaaaaaaegacbaaaaaaaaaaadgaaaaafbcaabaaaaaaaaaaadkbabaaa
adaaaaaadgaaaaafccaabaaaaaaaaaaadkbabaaaaeaaaaaadgaaaaafecaabaaa
aaaaaaaadkbabaaaafaaaaaabaaaaaahicaabaaaaaaaaaaaegacbaaaaaaaaaaa
egacbaaaabaaaaaaaaaaaaahicaabaaaaaaaaaaadkaabaaaaaaaaaaadkaabaaa
aaaaaaaadcaaaaakhcaabaaaaaaaaaaaegacbaaaabaaaaaapgapbaiaebaaaaaa
aaaaaaaaegacbaaaaaaaaaaaefaaaaajpcaabaaaaaaaaaaaegacbaaaaaaaaaaa
eghobaaaadaaaaaaaagabaaaadaaaaaaefaaaaajpcaabaaaabaaaaaaegbabaaa
abaaaaaaeghobaaaaaaaaaaaaagabaaaaaaaaaaadiaaaaahpcaabaaaaaaaaaaa
egaobaaaaaaaaaaapgapbaaaabaaaaaadiaaaaaihcaabaaaabaaaaaaegacbaaa
abaaaaaaegiccaaaaaaaaaaaadaaaaaadiaaaaaipcaabaaaaaaaaaaaegaobaaa
aaaaaaaaegiocaaaaaaaaaaaaeaaaaaaefaaaaajpcaabaaaacaaaaaaegbabaaa
abaaaaaaeghobaaaabaaaaaaaagabaaaacaaaaaadiaaaaaihcaabaaaacaaaaaa
egacbaaaacaaaaaakgikcaaaaaaaaaaaafaaaaaaaoaaaaahdcaabaaaadaaaaaa
egbabaaaacaaaaaapgbpbaaaacaaaaaaefaaaaajpcaabaaaadaaaaaaegaabaaa
adaaaaaaeghobaaaaeaaaaaaaagabaaaaeaaaaaadiaaaaahhcaabaaaacaaaaaa
egacbaaaacaaaaaapgapbaaaadaaaaaaaaaaaaahhcaabaaaadaaaaaaegacbaaa
adaaaaaaegbcbaaaagaaaaaadiaaaaahocaabaaaacaaaaaaagajbaaaacaaaaaa
agajbaaaadaaaaaadcaaaaakiccabaaaaaaaaaaaakaabaaaacaaaaaadkiacaaa
aaaaaaaaacaaaaaadkaabaaaaaaaaaaadcaaaaajhcaabaaaabaaaaaaegacbaaa
abaaaaaaegacbaaaadaaaaaajgahbaaaacaaaaaadcaaaaakhccabaaaaaaaaaaa
egacbaaaaaaaaaaaagiacaaaaaaaaaaaafaaaaaaegacbaaaabaaaaaadoaaaaab
"
}

SubProgram "gles " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
"!!GLES"
}

SubProgram "glesdesktop " {
Keywords { "LIGHTMAP_OFF" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
"!!GLES"
}

SubProgram "opengl " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
Vector 0 [_SpecColor]
Vector 1 [_Color]
Vector 2 [_ReflectColor]
Float 3 [_ReflectPower]
Float 4 [_Gloss]
Vector 5 [unity_LightmapFade]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_Cube] CUBE
SetTexture 4 [_LightBuffer] 2D
SetTexture 5 [unity_Lightmap] 2D
SetTexture 6 [unity_LightmapInd] 2D
"3.0-!!ARBfp1.0
OPTION ARB_precision_hint_fastest;
# 42 ALU, 7 TEX
PARAM c[7] = { program.local[0..5],
		{ 2, 1, 8 } };
TEMP R0;
TEMP R1;
TEMP R2;
TEMP R3;
TEMP R4;
TEX R0.yw, fragment.texcoord[0], texture[2], 2D;
MAD R0.xy, R0.wyzw, c[6].x, -c[6].y;
MUL R0.z, R0.y, R0.y;
MAD R0.z, -R0.x, R0.x, -R0;
TEX R3, fragment.texcoord[0], texture[0], 2D;
ADD R0.z, R0, c[6].y;
RSQ R0.z, R0.z;
RCP R0.z, R0.z;
DP3 R1.x, fragment.texcoord[2], R0;
DP3 R1.y, R0, fragment.texcoord[3];
DP3 R1.z, R0, fragment.texcoord[4];
MOV R0.x, fragment.texcoord[2].w;
MOV R0.z, fragment.texcoord[4].w;
MOV R0.y, fragment.texcoord[3].w;
DP3 R0.w, R1, R0;
MUL R1.xyz, R1, R0.w;
MAD R0.xyz, -R1, c[6].x, R0;
TEX R0, R0, texture[3], CUBE;
MUL R2, R0, R3.w;
TEX R0, fragment.texcoord[5], texture[5], 2D;
MUL R0.xyz, R0.w, R0;
TEX R1, fragment.texcoord[5], texture[6], 2D;
MUL R1.xyz, R1.w, R1;
MUL R1.xyz, R1, c[6].z;
MAD R4.xyz, R0, c[6].z, -R1;
DP4 R0.w, fragment.texcoord[6], fragment.texcoord[6];
RSQ R0.w, R0.w;
RCP R0.w, R0.w;
MAD_SAT R0.w, R0, c[5].z, c[5];
MAD R1.xyz, R0.w, R4, R1;
TEX R0.xyz, fragment.texcoord[0], texture[1], 2D;
MUL R4.xyz, R0, c[4].x;
TXP R0, fragment.texcoord[1], texture[4], 2D;
MUL R4.xyz, R0.w, R4;
ADD R0.xyz, R0, R1;
MUL R0.w, R4.x, c[0];
MUL R1.xyz, R0, R4;
MUL R3.xyz, R3, c[1];
MAD R0.xyz, R3, R0, R1;
MUL R1.xyz, R2, c[2];
MAD result.color.xyz, R1, c[3].x, R0;
MAD result.color.w, R2, c[2], R0;
END
# 42 instructions, 5 R-regs
"
}

SubProgram "d3d9 " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
Vector 0 [_SpecColor]
Vector 1 [_Color]
Vector 2 [_ReflectColor]
Float 3 [_ReflectPower]
Float 4 [_Gloss]
Vector 5 [unity_LightmapFade]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_Cube] CUBE
SetTexture 4 [_LightBuffer] 2D
SetTexture 5 [unity_Lightmap] 2D
SetTexture 6 [unity_LightmapInd] 2D
"ps_3_0
; 35 ALU, 7 TEX
dcl_2d s0
dcl_2d s1
dcl_2d s2
dcl_cube s3
dcl_2d s4
dcl_2d s5
dcl_2d s6
def c6, 2.00000000, -1.00000000, 1.00000000, 8.00000000
dcl_texcoord0 v0.xy
dcl_texcoord1 v1
dcl_texcoord2 v2
dcl_texcoord3 v3
dcl_texcoord4 v4
dcl_texcoord5 v5.xy
dcl_texcoord6 v6
texld r0.yw, v0, s2
mad_pp r0.xy, r0.wyzw, c6.x, c6.y
mul_pp r0.z, r0.y, r0.y
mad_pp r0.z, -r0.x, r0.x, -r0
texld r3, v0, s0
add_pp r0.z, r0, c6
rsq_pp r0.z, r0.z
rcp_pp r0.z, r0.z
dp3_pp r1.x, v2, r0
dp3_pp r1.y, r0, v3
dp3_pp r1.z, r0, v4
mov r0.x, v2.w
mov r0.z, v4.w
mov r0.y, v3.w
dp3 r0.w, r1, r0
mul r1.xyz, r1, r0.w
mad r0.xyz, -r1, c6.x, r0
texld r0, r0, s3
mul_pp r2, r0, r3.w
texld r0, v5, s5
mul_pp r0.xyz, r0.w, r0
texld r1, v5, s6
mul_pp r1.xyz, r1.w, r1
mul_pp r1.xyz, r1, c6.w
mad_pp r4.xyz, r0, c6.w, -r1
dp4 r0.w, v6, v6
rsq r0.w, r0.w
rcp r0.w, r0.w
mad_sat r0.w, r0, c5.z, c5
mad_pp r1.xyz, r0.w, r4, r1
texld r0.xyz, v0, s1
mul_pp r4.xyz, r0, c4.x
texldp r0, v1, s4
mul_pp r4.xyz, r0.w, r4
add_pp r0.xyz, r0, r1
mul_pp r0.w, r4.x, c0
mul_pp r1.xyz, r0, r4
mul_pp r3.xyz, r3, c1
mad_pp r0.xyz, r3, r0, r1
mul_pp r1.xyz, r2, c2
mad_pp oC0.xyz, r1, c3.x, r0
mad_pp oC0.w, r2, c2, r0
"
}

SubProgram "xbox360 " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
Vector 1 [_Color]
Float 4 [_Gloss]
Vector 2 [_ReflectColor]
Float 3 [_ReflectPower]
Vector 0 [_SpecColor]
Vector 5 [unity_LightmapFade]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_BumpMap] 2D
SetTexture 2 [_SpecMap] 2D
SetTexture 3 [_Cube] CUBE
SetTexture 4 [_LightBuffer] 2D
SetTexture 5 [_LightSpecBuffer] 2D
SetTexture 6 [unity_Lightmap] 2D
SetTexture 7 [unity_LightmapInd] 2D
// Shader Timing Estimate, in Cycles/64 pixel vector:
// ALU: 41.33 (31 instructions), vertex: 0, texture: 32,
//   sequencer: 18, interpolator: 28;    8 GPRs, 24 threads,
// Performance (if enough threads): ~41 cycles per vector
// * Texture cycle estimates are assuming an 8bit/component texture with no
//     aniso or trilinear filtering.

"ps_360
backbbaaaaaaacmeaaaaacfmaaaaaaaaaaaaaaceaaaaacgaaaaaaciiaaaaaaaa
aaaaaaaaaaaaacdiaaaaaabmaaaaacckppppadaaaaaaaaaoaaaaaabmaaaaaaaa
aaaaaccdaaaaabdeaaadaaabaaabaaaaaaaaabeaaaaaaaaaaaaaabfaaaacaaab
aaabaaaaaaaaabfiaaaaaaaaaaaaabgiaaadaaadaaabaaaaaaaaabhaaaaaaaaa
aaaaabiaaaacaaaeaaabaaaaaaaaabiiaaaaaaaaaaaaabjiaaadaaaeaaabaaaa
aaaaabeaaaaaaaaaaaaaabkfaaadaaafaaabaaaaaaaaabeaaaaaaaaaaaaaablg
aaadaaaaaaabaaaaaaaaabeaaaaaaaaaaaaaablpaaacaaacaaabaaaaaaaaabfi
aaaaaaaaaaaaabmnaaacaaadaaabaaaaaaaaabiiaaaaaaaaaaaaabnlaaacaaaa
aaabaaaaaaaaabfiaaaaaaaaaaaaabogaaadaaacaaabaaaaaaaaabeaaaaaaaaa
aaaaabopaaadaaagaaabaaaaaaaaabeaaaaaaaaaaaaaabpoaaacaaafaaabaaaa
aaaaabfiaaaaaaaaaaaaacbbaaadaaahaaabaaaaaaaaabeaaaaaaaaafpechfgn
haengbhaaaklklklaaaeaaamaaabaaabaaabaaaaaaaaaaaafpedgpgmgphcaakl
aaabaaadaaabaaaeaaabaaaaaaaaaaaafpedhfgcgfaaklklaaaeaaaoaaabaaab
aaabaaaaaaaaaaaafpehgmgphdhdaaklaaaaaaadaaabaaabaaabaaaaaaaaaaaa
fpemgjghgiheechfgggggfhcaafpemgjghgihefdhagfgdechfgggggfhcaafpen
gbgjgofegfhiaafpfcgfgggmgfgdheedgpgmgphcaafpfcgfgggmgfgdhefagphh
gfhcaafpfdhagfgdedgpgmgphcaafpfdhagfgdengbhaaahfgogjhehjfpemgjgh
gihegngbhaaahfgogjhehjfpemgjghgihegngbhaeggbgegfaahfgogjhehjfpem
gjghgihegngbhaejgogeaahahdfpddfpdaaadccodacodcdadddfddcodaaaklkl
aaaaaaaaaaaaaaabaaaaaaaaaaaaaaaaaaaaaabeabpmaabaaaaaaaaaaaaaaaaa
aaaaaaaaaaaaaaaaaaaaaaeaaaaaacbmbaaaahaaaaaaaaaeaaaaaaaaaaaagaoh
aahpaahpaaaaaaabaaaadafaaaaapbfbaaaapcfcaaaapdfdaaaapefeaaaadfff
aaaapgfgaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaebaaaaaaeaaaaaaalpiaaaaadpmaaaaadpiaaaaaaaaaaaaaaaaaaaaa
aaaaaaaaaffagaafgaalbcaabcaaaaacaaaagabbgabhbcaabcaaaaaaafeagabn
bacdbcaabcaaaaabaaaaaaaagacemeaabcaaaaaaaaaacackaaaaccaaaaaaaaaa
emeeaaahaakhkhblopagagabmiamaaaaaamgkmaaobaaabaalieagaabbpbppoii
aaaaeaaabahabakbbpbppgiiaaaaeaaababahaabbpbpppnjaaaaeaaabagafakb
bpbppgiiaaaaeaaakmiaagaaaaaaaaehmcaaaapomiadaaahaagnlbmgilahpopo
kaeiahahaablgmmgkbabpoihmjaiaaabaamgmgblilahafafmiaeaaahaegngngm
nbahahppkaehahabaablmamgobahabihmiahaaafabblmamaolagafabmiahaaaf
aamablmaolafababmiacaaabaaloloaapaahaeaamiabaaabaaloloaapaahacaa
beaeaaabaalologmpaahadabamiiafagaamgblblobabadacmiapaaafaadedeaa
oaafagaamiaiaaabaalbblblolabaeafaaiaabaaaaaaaablocaaaaabmiahaaab
aablleaaobababaabeacaaabaflbblmgoaabadabaeebababaegmblbloaabacae
miapaaabaakgmnaapcababaaembiacacaablblmgocababibmiagaaacaagbgmbl
mlabacpolificaabbpbppppiaaaaeaaaoedieaebbpbppgiiaaaamaaabaaibaab
bpbppgiiaaaaeaaabaciaaabbpbppeehaaaaeaaakichacadaaleleebibababae
kiepacabaaaablecmbaeabaekiihacabaamamaedibabacaebeboaaaaaanbgmbl
obacacabkiihacabaamagmaaibabadackiihabacaamjleabmbaaafaamiahaaac
aalelemaoladafacmiapiaaaaaaaaaaaoaacabaaaaaaaaaaaaaaaaaaaaaaaaaa
"
}

SubProgram "ps3 " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
Vector 0 [_SpecColor]
Vector 1 [_Color]
Vector 2 [_ReflectColor]
Float 3 [_ReflectPower]
Float 4 [_Gloss]
Vector 5 [unity_LightmapFade]
SetTexture 0 [_MainTex] 2D
SetTexture 1 [_SpecMap] 2D
SetTexture 2 [_BumpMap] 2D
SetTexture 3 [_Cube] CUBE
SetTexture 4 [_LightBuffer] 2D
SetTexture 5 [unity_Lightmap] 2D
SetTexture 6 [unity_LightmapInd] 2D
"sce_fp_rsx // 56 instructions using 4 registers
[Configuration]
24
ffffffff001fc020007fffa1000000000000840004000000
[Offsets]
6
_SpecColor 1 0
000002f0
_Color 1 0
000002a0
_ReflectColor 2 0
00000350000001e0
_ReflectPower 1 0
00000370
_Gloss 1 0
00000250
unity_LightmapFade 2 0
000000f0000000d0
[Microcode]
896
5e020101c8011c9dc8000001c8003fe108020600c8041c9dc8040001c8000001
3e00170dc8011c9dc8000001c8003fe10e800240fe001c9dc8003001c8000001
3e04170bc8011c9dc8000001c8003fe1088a0140fe081c9dc8003001c8000001
08001b0054041c9dc8000001c80000010e04040055141c9dc8080001c9000003
94021704c8011c9dc8000001c8003fe106840440ce041c9d00020000aa020000
000040000000bf80000000000000000010800240ab081c9cab080000c8000001
02063a0054021c9d54000001c800000100000000000000000000000000000000
10008300000c1c9cc8020001c800000100000000000000000000000000000000
02820440c9081c9fc9080001ff000003de860140c8011c9dc8000001c8003fe1
1080034001041c9c00020000c800000100003f80000000000000000000000000
0e800400fe001c9dc8080001c900000108843b40ff003c9dff000001c8000001
02860540c90c1c9dc9080001c8000001fe8c0140c8011c9dc8000001c8003fe1
04860540c9081c9dc9180001c80000011e820141c8011c9dc8000001c8003fe1
08860540c9081c9dc9040001c800000108840140ff041c9dc8000001c8000001
02840140ff0c1c9dc8000001c800000108820140fe021c9dc8000001c8000001
0000000000000000000000000000000004840140ff181c9dc8000001c8000001
be041808c8011c9dc8000001c8003fe10e800340c8081c9dc9000001c8000001
08000500c90c1c9dc9081001c80000010e060400c90c1c9f54000001c9080001
108e0240c8081c9d00020000c800000100000000000000000000000000000000
8e021702c8011c9dc8000001c8003fe10e840240c8041c9dff1c0001c8000001
9e041700c8011c9dc8000001c8003fe11682024048081c9d48020001c8000001
000000000000000000000000000000000e860240c9001c9dc9080001c8000001
1084014001081c9cc8000001c80000010e86044069041c9dc9000001c90c0001
10820140c8021c9dc8000001c800000100000000000000000000000000000000
1e061706c80c1c9dc8000001c80000011e800240c80c1c9dfe080001c8000001
08840140ff001c9dc8000001c8000001108038405d081c9d5d040001c8000001
0e800240c9001c9dc8020001c800000100000000000000000000000000000000
0e810440c9001c9d00020000c90c000100000000000000000000000000000000
"
}

SubProgram "d3d11 " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
ConstBuffer "$Globals" 160 // 144 used size, 12 vars
Vector 32 [_SpecColor] 4
Vector 48 [_Color] 4
Vector 64 [_ReflectColor] 4
Float 80 [_ReflectPower]
Float 88 [_Gloss]
Vector 128 [unity_LightmapFade] 4
BindCB "$Globals" 0
SetTexture 0 [_MainTex] 2D 0
SetTexture 1 [_SpecMap] 2D 2
SetTexture 2 [_BumpMap] 2D 1
SetTexture 3 [_Cube] CUBE 3
SetTexture 4 [_LightBuffer] 2D 4
SetTexture 5 [unity_Lightmap] 2D 5
SetTexture 6 [unity_LightmapInd] 2D 6
// 40 instructions, 4 temp regs, 0 temp arrays:
// ALU 19 float, 0 int, 0 uint
// TEX 7 (0 load, 0 comp, 0 bias, 0 grad)
// FLOW 1 static, 0 dynamic
"ps_4_0
eefiecedgobogfchdnljonbpfjcmcbiafihnnbldabaaaaaaiaahaaaaadaaaaaa
cmaaaaaabeabaaaaeiabaaaaejfdeheooaaaaaaaaiaaaaaaaiaaaaaamiaaaaaa
aaaaaaaaabaaaaaaadaaaaaaaaaaaaaaapaaaaaaneaaaaaaaaaaaaaaaaaaaaaa
adaaaaaaabaaaaaaadadaaaaneaaaaaaafaaaaaaaaaaaaaaadaaaaaaabaaaaaa
amamaaaaneaaaaaaabaaaaaaaaaaaaaaadaaaaaaacaaaaaaapalaaaaneaaaaaa
acaaaaaaaaaaaaaaadaaaaaaadaaaaaaapapaaaaneaaaaaaadaaaaaaaaaaaaaa
adaaaaaaaeaaaaaaapapaaaaneaaaaaaaeaaaaaaaaaaaaaaadaaaaaaafaaaaaa
apapaaaaneaaaaaaagaaaaaaaaaaaaaaadaaaaaaagaaaaaaapapaaaafdfgfpfa
epfdejfeejepeoaafeeffiedepepfceeaaklklklepfdeheocmaaaaaaabaaaaaa
aiaaaaaacaaaaaaaaaaaaaaaaaaaaaaaadaaaaaaaaaaaaaaapaaaaaafdfgfpfe
gbhcghgfheaaklklfdeieefcdaagaaaaeaaaaaaaimabaaaafjaaaaaeegiocaaa
aaaaaaaaajaaaaaafkaaaaadaagabaaaaaaaaaaafkaaaaadaagabaaaabaaaaaa
fkaaaaadaagabaaaacaaaaaafkaaaaadaagabaaaadaaaaaafkaaaaadaagabaaa
aeaaaaaafkaaaaadaagabaaaafaaaaaafkaaaaadaagabaaaagaaaaaafibiaaae
aahabaaaaaaaaaaaffffaaaafibiaaaeaahabaaaabaaaaaaffffaaaafibiaaae
aahabaaaacaaaaaaffffaaaafidaaaaeaahabaaaadaaaaaaffffaaaafibiaaae
aahabaaaaeaaaaaaffffaaaafibiaaaeaahabaaaafaaaaaaffffaaaafibiaaae
aahabaaaagaaaaaaffffaaaagcbaaaaddcbabaaaabaaaaaagcbaaaadmcbabaaa
abaaaaaagcbaaaadlcbabaaaacaaaaaagcbaaaadpcbabaaaadaaaaaagcbaaaad
pcbabaaaaeaaaaaagcbaaaadpcbabaaaafaaaaaagcbaaaadpcbabaaaagaaaaaa
gfaaaaadpccabaaaaaaaaaaagiaaaaacaeaaaaaabbaaaaahbcaabaaaaaaaaaaa
egbobaaaagaaaaaaegbobaaaagaaaaaaelaaaaafbcaabaaaaaaaaaaaakaabaaa
aaaaaaaadccaaaalbcaabaaaaaaaaaaaakaabaaaaaaaaaaackiacaaaaaaaaaaa
aiaaaaaadkiacaaaaaaaaaaaaiaaaaaaefaaaaajpcaabaaaabaaaaaaogbkbaaa
abaaaaaaeghobaaaagaaaaaaaagabaaaagaaaaaadiaaaaahccaabaaaaaaaaaaa
dkaabaaaabaaaaaaabeaaaaaaaaaaaebdiaaaaahocaabaaaaaaaaaaaagajbaaa
abaaaaaafgafbaaaaaaaaaaaefaaaaajpcaabaaaabaaaaaaogbkbaaaabaaaaaa
eghobaaaafaaaaaaaagabaaaafaaaaaadiaaaaahicaabaaaabaaaaaadkaabaaa
abaaaaaaabeaaaaaaaaaaaebdcaaaaakhcaabaaaabaaaaaapgapbaaaabaaaaaa
egacbaaaabaaaaaajgahbaiaebaaaaaaaaaaaaaadcaaaaajhcaabaaaaaaaaaaa
agaabaaaaaaaaaaaegacbaaaabaaaaaajgahbaaaaaaaaaaaaoaaaaahdcaabaaa
abaaaaaaegbabaaaacaaaaaapgbpbaaaacaaaaaaefaaaaajpcaabaaaabaaaaaa
egaabaaaabaaaaaaeghobaaaaeaaaaaaaagabaaaaeaaaaaaaaaaaaahhcaabaaa
aaaaaaaaegacbaaaaaaaaaaaegacbaaaabaaaaaaefaaaaajpcaabaaaacaaaaaa
egbabaaaabaaaaaaeghobaaaabaaaaaaaagabaaaacaaaaaadiaaaaaihcaabaaa
abaaaaaaegacbaaaacaaaaaakgikcaaaaaaaaaaaafaaaaaadiaaaaahhcaabaaa
abaaaaaaegacbaaaabaaaaaapgapbaaaabaaaaaadiaaaaahocaabaaaabaaaaaa
agajbaaaaaaaaaaaagajbaaaabaaaaaaefaaaaajpcaabaaaacaaaaaaegbabaaa
abaaaaaaeghobaaaaaaaaaaaaagabaaaaaaaaaaadiaaaaaihcaabaaaacaaaaaa
egacbaaaacaaaaaaegiccaaaaaaaaaaaadaaaaaadcaaaaajhcaabaaaaaaaaaaa
egacbaaaacaaaaaaegacbaaaaaaaaaaajgahbaaaabaaaaaaefaaaaajpcaabaaa
adaaaaaaegbabaaaabaaaaaaeghobaaaacaaaaaaaagabaaaabaaaaaadcaaaaap
dcaabaaaacaaaaaahgapbaaaadaaaaaaaceaaaaaaaaaaaeaaaaaaaeaaaaaaaaa
aaaaaaaaaceaaaaaaaaaialpaaaaialpaaaaaaaaaaaaaaaadcaaaaakicaabaaa
aaaaaaaaakaabaiaebaaaaaaacaaaaaaakaabaaaacaaaaaaabeaaaaaaaaaiadp
dcaaaaakicaabaaaaaaaaaaabkaabaiaebaaaaaaacaaaaaabkaabaaaacaaaaaa
dkaabaaaaaaaaaaaelaaaaafecaabaaaacaaaaaadkaabaaaaaaaaaaabaaaaaah
bcaabaaaadaaaaaaegbcbaaaadaaaaaaegacbaaaacaaaaaabaaaaaahccaabaaa
adaaaaaaegbcbaaaaeaaaaaaegacbaaaacaaaaaabaaaaaahecaabaaaadaaaaaa
egbcbaaaafaaaaaaegacbaaaacaaaaaadgaaaaafbcaabaaaacaaaaaadkbabaaa
adaaaaaadgaaaaafccaabaaaacaaaaaadkbabaaaaeaaaaaadgaaaaafecaabaaa
acaaaaaadkbabaaaafaaaaaabaaaaaahicaabaaaaaaaaaaaegacbaaaacaaaaaa
egacbaaaadaaaaaaaaaaaaahicaabaaaaaaaaaaadkaabaaaaaaaaaaadkaabaaa
aaaaaaaadcaaaaakocaabaaaabaaaaaaagajbaaaadaaaaaapgapbaiaebaaaaaa
aaaaaaaaagajbaaaacaaaaaaefaaaaajpcaabaaaadaaaaaajgahbaaaabaaaaaa
eghobaaaadaaaaaaaagabaaaadaaaaaadiaaaaahpcaabaaaacaaaaaapgapbaaa
acaaaaaaegaobaaaadaaaaaadiaaaaaipcaabaaaacaaaaaaegaobaaaacaaaaaa
egiocaaaaaaaaaaaaeaaaaaadcaaaaakhccabaaaaaaaaaaaegacbaaaacaaaaaa
agiacaaaaaaaaaaaafaaaaaaegacbaaaaaaaaaaadcaaaaakiccabaaaaaaaaaaa
akaabaaaabaaaaaadkiacaaaaaaaaaaaacaaaaaadkaabaaaacaaaaaadoaaaaab
"
}

SubProgram "gles " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
"!!GLES"
}

SubProgram "glesdesktop " {
Keywords { "LIGHTMAP_ON" "DIRLIGHTMAP_OFF" "HDR_LIGHT_PREPASS_ON" }
"!!GLES"
}

}
	}

#LINE 93

	}
	Fallback "Reflective/Bumped Diffuse"
}
