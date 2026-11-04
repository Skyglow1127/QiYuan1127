"""Service diagnostics shared by capture and saved-report presentation."""
from contextlib import contextmanager
import socket
import urllib.error


def is_network_error(message):
    value=str(message).lower()
    return any(token in value for token in ('urlopen error', 'connection refused',
        'connection reset', 'connection aborted', 'timed out', 'http error'))


class ServiceRequestError(RuntimeError):
    def __init__(self, error, service, url):
        self.technical_error=str(error)
        self.service=service
        self.url=url
        raw=str(error).lower()
        if isinstance(error,urllib.error.HTTPError):
            self.code='service_http_error'
            cause=f'服务返回 HTTP {error.code}，请求未成功'
        elif 'connection refused' in raw or getattr(error,'errno',None)==111:
            self.code='service_connection_refused'
            cause='连接被拒绝，该接口当前未接受连接'
        elif isinstance(error,(TimeoutError,socket.timeout)) or 'timed out' in raw:
            self.code='service_timeout'
            cause='等待响应超时'
        else:
            self.code='service_connection_error'
            cause='网络连接中断或无法建立'
        self.message=f'{service}无法访问（{url}）：{cause}'
        self.action=('请检查 KR260 板卡供电和网络、对应相机服务及本机 SSH 转发，恢复后重新准备'
                     if '相机' in service else '请检查该服务的启动状态和网络连接，恢复后重新准备')
        super().__init__(self.message)

    def blocker(self):
        return dict(code=self.code,message=self.message,action=self.action,
                    service=self.service,url=self.url)


@contextmanager
def service_request(opener,url,service,timeout=1):
    try:
        with opener.open(url,timeout=timeout) as response:
            yield response
    except (urllib.error.URLError,TimeoutError,ConnectionError) as error:
        raise ServiceRequestError(error,service,url) from error


def saved_service_failure(report):
    """Use a saved failed acquisition span, never infer the failing URL from backend defaults."""
    if not isinstance(report,dict) or not is_network_error(report.get('error','')):
        return None
    timing=report.get('timing')
    spans=timing.get('spans',[]) if isinstance(timing,dict) else []
    if isinstance(spans,list) and any(isinstance(span,dict) and span.get('outcome')=='failed'
            and span.get('name')=='wrist_http_status_and_image' for span in spans):
        return ServiceRequestError(report['error'],'腕部相机服务','http://127.0.0.1:18760')
    return None
