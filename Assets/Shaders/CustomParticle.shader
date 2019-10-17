// Upgrade NOTE: replaced 'mul(UNITY_MATRIX_MVP,*)' with 'UnityObjectToClipPos(*)'

Shader "Custom/CustomParticle"
{
	Properties
	{
		_MainTex("Base (RGB)", 2D) = "white" {}
	}



	SubShader{
		Tags { "Queue" = "Transparent" "RenderType" = "Transparent" }

		Pass {
			Cull Back
			ZWrite Off
			Blend srcAlpha OneMinusSrcAlpha

			CGPROGRAM
			#pragma vertex vert
			#pragma fragment frag
			#pragma fragmentoption ARB_precision_hint_fastest

			#include "UnityCG.cginc"

			sampler2D _MainTex;
			float4 _MainTex_ST;

			// Struct Input || VertOut
			struct appdata {
				half4 vertex : POSITION;
				half2 texcoord : TEXCOORD0;
				fixed4 color : COLOR;
			};

			//VertIn
			struct v2f {
				half4 pos : POSITION;
				fixed4 color : COLOR;
				half2 texcoord : TEXCOORD0;
			};

			v2f vert(appdata v)
			{
				v2f o;
				o.pos = UnityObjectToClipPos(v.vertex);
				o.color = v.color;
				o.texcoord = TRANSFORM_TEX(v.texcoord, _MainTex);

				return o;
			}


			fixed4 frag(v2f i) : COLOR
			{
				fixed4 col;
				fixed4 tex = tex2D(_MainTex, i.texcoord);

				col.rgb = tex.rgb;
				col.a = i.color.a * tex.a;
				return col;

			}
			ENDCG
		}
	}
	FallBack "Diffuse"
}