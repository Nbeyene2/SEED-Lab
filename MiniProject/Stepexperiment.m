time = [1:1:451];
AngVel = [0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
1
1
2
2
3
3
4
5
6
6
7
8
9
10
11
11
12
13
14
15
16
18
19
20
21
22
24
25
26
27
29
30
31
33
34
36
37
39
40
42
43
45
46
48
50
51
53
55
56
58
59
61
63
65
66
68
70
72
74
75
77
79
81
83
85
87
89
91
93
94
96
98
100
102
104
106
108
110
112
114
116
118
120
122
124
126
128
130
132
134
136
138
141
143
145
147
149
151
153
156
158
160
162
164
166
168
171
173
175
177
179
181
184
186
188
95
96
97
98
99
100
101
102
103
105
106
107
108
109
110
111
112
113
115
116
117
118
119
120
121
123
124
125
126
127
128
129
131
132
133
134
135
136
137
139
140
141
142
143
144
146
147
148
149
150
151
153
154
155
156
157
158
160
161
162
163
164
165
166
168
169
170
171
172
173
175
176
177
178
179
180
181
183
184
185
186
187
188
189
191
192
193
194
195
196
198
199
200
201
202
203
205
206
207
208
209
210
211
213
214
215
216
217
218
219
220
222
223
224
225
226
227
229
230
231
232
155
156
157
158
158
159
160
161
161
162
163
164
164
165
166
167
167
168
169
170
170
171
172
173
173
174
175
176
176
177
178
179
180
180
181
182
183
183
184
185
186
187
187
188
189
190
191
192
192
193
194
195
196
197
197
198
199
200
201
202
203
204
204
205
206
206
207
208
208
209
209
210
210


];
AngVel = AngVel/2;
plot(time, AngVel);
K = 2;
sigma = 1.28;
open_system('PIDMini')
%
% run the simulation
%
out=sim('PIDMini');
hold off;
figure(2);
plot(out.simout);