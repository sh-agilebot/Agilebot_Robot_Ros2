FROM nginx:1.28-alpine

COPY .vitepress/dist /usr/share/nginx/html/docs/ros

COPY <<EOF /etc/nginx/conf.d/default.conf
server {
  listen 80;
  access_log off;
  error_log off;
  root /usr/share/nginx/html;
  index index.html;

  location ~* \.(?:jpg|jpeg|gif|png|ico|webp|svg|js|css|mp4|webm)$ {
    expires 10d;
  }

  location ~* \.txt$ {
    charset utf-8;
  }
}
EOF

EXPOSE 80

CMD ["nginx", "-g", "daemon off;"]
